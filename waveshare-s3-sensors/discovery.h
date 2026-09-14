// discovery.h — IP-agnostic logger discovery (v1.15.17)
//
// WHY: the old resolvePi() had exactly three ways to find the logger:
//   1. a static piHost typed into /config
//   2. derive 192.168.<NNN>.10 from a "rigNNN" WiFi SSID
//   3. a one-shot mDNS browse for _rig-logger._tcp
// On a flat test network there is no per-rig router, so (2) is dead, and (3)
// only ran at boot / every 5 min / after a failed POST — so a logger that
// booted AFTER the module stayed invisible for up to 5 minutes, and a logger
// whose IP changed (DHCP) took just as long to re-find.
//
// Strategy now, in priority order:
//   1. static piHost — always wins, it's the per-rig override
//   2. legacy rigNNN SSID derivation (still used on the per-rig-router rigs)
//   3. mDNS browse for _rig-logger._tcp, re-run on a short retry interval
//      while unresolved and re-verified on a long interval once resolved
//   4. SUBNET SWEEP fallback: walk the module's own /24 looking for something
//      answering GET /api/status on :8080. Only runs when mDNS is silent, and
//      is rate-limited, so it can't stall the CAN loop.
//
// DEPENDENCY: the sweep uses Arduino's builtin ESP32Ping library (bundled with
// the esp32 core since 2.x — no Library Manager install needed). If your core
// somehow doesn't ship it, comment out ESP32PING_AVAILABLE below and the sweep
// degrades to "not available"; mDNS + static still work.
//
// THREADING: everything here runs from loopTask on core 1 only (same as the
// old resolvePi). webui.h reads the *_disc* values via discoveryMethod(),
// which is a plain String copy — same cross-task pattern already used for
// resolvedPiIp on this firmware.
#ifndef DISCOVERY_H
#define DISCOVERY_H

#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <HTTPClient.h>

#ifndef ESP32PING_AVAILABLE
#define ESP32PING_AVAILABLE 1
#endif
#if ESP32PING_AVAILABLE
#include <ESP32Ping.h>
#endif

static const unsigned long DISCOVER_RETRY_MS  = 20000UL;   // unresolved: retry discovery every 20s
static const unsigned long DISCOVER_VERIFY_MS = 300000UL;  // resolved: re-verify via mDNS every 5 min
static const unsigned long SWEEP_RETRY_MS     = 900000UL;  // don't re-sweep the /24 more than every 15 min
static const int           SWEEP_TIMEOUT_MS   = 120;       // per-host ping timeout
static const uint16_t      LOGGER_PORT        = 8080;

static String        _discResolvedIp  = "";
static String        _discMethod      = "none";
static unsigned long _discLastAttempt = 0;
static unsigned long _discLastSweep   = 0;
static bool          _discSweepExhausted = false;

// Cheap liveness probe: does anything speak our logger API at this IP?
// GET /api/status answers 200 on rig-pi-logger's local server.
static bool probeLogger(const String& ip) {
  HTTPClient http;
  http.setConnectTimeout(250);
  http.setTimeout(250);
  http.begin("http://" + ip + ":" + String(LOGGER_PORT) + "/api/status");
  http.addHeader("Accept", "application/json");
  int code = http.GET();
  bool found = (code == 200);
  http.end();
  return found;
}

// Walk the module's own /24. Returns true if a logger answered.
// The ping gate means empty addresses cost ~120ms each rather than a full HTTP
// connect timeout, so a quiet /24 costs roughly 30s — which is why this only
// runs after mDNS has already failed, and is rate-limited.
static bool sweepSubnet() {
  IPAddress local = WiFi.localIP();
  IPAddress mask  = WiFi.subnetMask();
  if (!local || local == IPAddress(0, 0, 0, 0)) return false;

  // Only /24 sweeps. Anything larger (/16) is far too big to walk from a
  // microcontroller — that case needs a static piHost.
  if (mask != IPAddress(255, 255, 255, 0)) {
    Serial.printf("[Disc] Subnet %s is not /24 — sweep skipped, set piHost manually\n",
                  mask.toString().c_str());
    return false;
  }

#if !ESP32PING_AVAILABLE
  Serial.println("[Disc] ESP32Ping not available — sweep skipped");
  return false;
#else
  uint8_t base0 = local[0], base1 = local[1], base2 = local[2];
  uint8_t selfLast = local[3];
  Serial.printf("[Disc] Sweeping %d.%d.%d.1-254 for a logger on port %d...\n",
                base0, base1, base2, LOGGER_PORT);
  unsigned long t0 = millis();
  int pinged = 0, alive = 0;

  for (int host = 1; host <= 254; host++) {
    if (host == selfLast) continue;  // that's us
    IPAddress cand(base0, base1, base2, (uint8_t)host);
    pinged++;
    if (!Ping.ping(cand, 1, SWEEP_TIMEOUT_MS)) continue;  // skip dead slots fast
    alive++;
    if (probeLogger(cand.toString())) {
      _discResolvedIp = cand.toString();
      _discMethod = "subnet-sweep";
      Serial.printf("[Disc] Logger found by sweep: %s (%d pinged, %d alive, %lums)\n",
                    _discResolvedIp.c_str(), pinged, alive, millis() - t0);
      return true;
    }
  }
  Serial.printf("[Disc] Sweep found nothing (%d pinged, %d alive, %lums)\n",
                pinged, alive, millis() - t0);
  return false;
#endif
}

// Returns the logger IP, or "" if not currently known.
//   staticHost = user's piHost setting (empty = auto)
//   wifiSSID   = current SSID, for the legacy rigNNN derivation
String discoverLogger(const String& staticHost, const String& wifiSSID) {
  unsigned long now = millis();

  // 1. Explicit static always wins, no timers involved.
  if (!staticHost.isEmpty() && staticHost != "__auto__") {
    if (_discMethod != "static") {
      _discMethod = "static";
      Serial.printf("[Disc] Using static piHost: %s\n", staticHost.c_str());
    }
    _discResolvedIp = staticHost;
    return _discResolvedIp;
  }

  // 2. Legacy rigNNN SSID derivation (per-rig routers). Cheap, so it stays.
  if (wifiSSID.length() >= 4 && wifiSSID.startsWith("rig") &&
      wifiSSID.substring(3).toInt() > 0) {
    String derived = "192.168." + wifiSSID.substring(3) + ".10";
    if (_discMethod != "rig-ssid") {
      _discMethod = "rig-ssid";
      Serial.printf("[Disc] Derived from rig SSID \"%s\": %s\n",
                    wifiSSID.c_str(), derived.c_str());
    }
    _discResolvedIp = derived;
    return _discResolvedIp;
  }

  // 3. mDNS browse. Retry fast while unresolved, verify slowly once found.
  bool due = _discResolvedIp.isEmpty()
               ? (now - _discLastAttempt >= DISCOVER_RETRY_MS)
               : (now - _discLastAttempt >= DISCOVER_VERIFY_MS);
  if (due) {
    _discLastAttempt = now;
    int n = MDNS.queryService("_rig-logger", "_tcp");
    if (n > 0) {
      String found = MDNS.address(0).toString();
      if (found != _discResolvedIp) {
        Serial.printf("[Disc] mDNS: %s -> %s\n", _discResolvedIp.c_str(), found.c_str());
      }
      _discResolvedIp = found;
      _discMethod = "mdns";
      _discSweepExhausted = false;
      return _discResolvedIp;
    }
    // mDNS silent. If we already hold an address, keep it — one missed browse
    // isn't evidence the logger went away, and dropping it would cost data.
    if (!_discResolvedIp.isEmpty()) return _discResolvedIp;
  }

  // 4. Subnet sweep fallback (only while still unresolved).
  if (_discResolvedIp.isEmpty() && !_discSweepExhausted &&
      (now - _discLastSweep >= SWEEP_RETRY_MS)) {
    _discLastSweep = now;
    if (sweepSubnet()) {
      _discMethod = "subnet-sweep";
      return _discResolvedIp;
    }
    _discSweepExhausted = true;  // stop hammering the LAN; mDNS retries continue
    Serial.println("[Disc] Sweep exhausted — relying on mDNS retries. "
                   "Set piHost manually if the logger is on a different subnet.");
  }

  return _discResolvedIp;
}

// Called after a POST failure: the address we hold may be stale.
// A static piHost is NOT dropped — if the user pinned an IP, a failed POST
// means the logger is down, not that the IP is wrong, and re-discovering
// could happily post to some unrelated host answering on :8080.
static void discoveryInvalidate(const String& badIp) {
  if (!badIp.isEmpty() && badIp == _discResolvedIp && _discMethod != "static") {
    Serial.printf("[Disc] Dropping %s (post failed, method=%s)\n",
                  badIp.c_str(), _discMethod.c_str());
    _discResolvedIp = "";
    _discMethod = "none";
    _discLastAttempt = 0;  // retry immediately on the next pass
  }
}

static String discoveryMethod() { return _discMethod; }

#endif  // DISCOVERY_H
