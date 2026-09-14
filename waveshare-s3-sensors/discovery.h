// discovery.h — IP-agnostic logger discovery (v1.15.18)
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
//   4. TCP PORT SWEEP fallback: walk the module's own /24 looking for
//      something listening on :8080. Only runs when mDNS is silent, and is
//      rate-limited, so it can't stall the CAN loop.
//
// NO EXTERNAL LIBRARY. v1.15.17 originally used ESP32Ping, which is NOT part
// of the ESP32 Arduino core (it's marian-craciunescu/ESP32Ping, a separate
// Library Manager install) — build failed with "ESP32Ping.h: No such file or
// directory". The sweep now uses plain non-blocking TCP connects instead,
// which is what actually matters here: the logger answers on TCP :8080, and a
// live host with the port closed replies RST immediately, so a closed port and
// a dead host are still distinguishable without ICMP.
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
#include <WiFiClient.h>
#include <HTTPClient.h>

static const unsigned long DISCOVER_RETRY_MS  = 20000UL;   // unresolved: retry discovery every 20s
static const unsigned long DISCOVER_VERIFY_MS = 300000UL;  // resolved: re-verify via mDNS every 5 min
static const unsigned long SWEEP_RETRY_MS     = 900000UL;  // don't re-sweep the /24 more than every 15 min
static const int           SWEEP_PORT_TIMEOUT = 120;       // ms per host waiting for a SYN/ACK
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
  http.setConnectTimeout(300);
  http.setTimeout(300);
  http.begin("http://" + ip + ":" + String(LOGGER_PORT) + "/api/status");
  http.addHeader("Accept", "application/json");
  int code = http.GET();
  bool found = (code == 200);
  http.end();
  return found;
}

// Did a TCP handshake to ip:LOGGER_PORT complete within timeoutMs?
// Non-blocking connect + poll, so a dead host costs timeoutMs and nothing more.
static bool tcpPortOpen(const IPAddress& ip, uint16_t port, int timeoutMs) {
  WiFiClient c;
  if (!c.connect(ip, port, timeoutMs)) return false;
  c.stop();
  return true;
}

// Walk the module's own /24 looking for a listener on LOGGER_PORT.
// Cost: dead slots answer RST or time out at SWEEP_PORT_TIMEOUT, so a quiet /24
// is roughly 25s worst case — which is why this only runs after mDNS has
// already failed, and is rate-limited.
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

  uint8_t base0 = local[0], base1 = local[1], base2 = local[2];
  uint8_t selfLast = local[3];
  Serial.printf("[Disc] Sweeping %d.%d.%d.1-254 for TCP:%d...\n",
                base0, base1, base2, LOGGER_PORT);
  unsigned long t0 = millis();
  int open = 0;

  for (int host = 1; host <= 254; host++) {
    if (host == selfLast) continue;  // that's us
    IPAddress cand(base0, base1, base2, (uint8_t)host);
    if (!tcpPortOpen(cand, LOGGER_PORT, SWEEP_PORT_TIMEOUT)) continue;
    open++;
    // Port is open. Confirm it's actually our logger before committing —
    // anything could listen on 8080 (Grafana, another dev box, a printer).
    if (probeLogger(cand.toString())) {
      _discResolvedIp = cand.toString();
      _discMethod = "subnet-sweep";
      Serial.printf("[Disc] Logger found by sweep: %s (%d ports open, %lums)\n",
                    _discResolvedIp.c_str(), open, millis() - t0);
      return true;
    }
    Serial.printf("[Disc]   %s has :%d open but isn't our logger, continuing\n",
                  cand.toString().c_str(), LOGGER_PORT);
  }
  Serial.printf("[Disc] Sweep found nothing (%d ports open, %lums)\n",
                open, millis() - t0);
  return false;
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

  // 4. TCP port sweep fallback (only while still unresolved).
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
