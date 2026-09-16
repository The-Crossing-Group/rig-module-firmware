// discovery.h — IP-agnostic logger discovery (v1.15.20, v1.15.40/41 sweep fixes)
//
// WHY: the old resolvePi() had exactly three ways to find the logger:
//   1. a static piHost typed into /config
//   2. derive 192.168.<NNN>.10 from a "rigNNN" SSID
//   3. a one-shot mDNS browse for _rig-logger._tcp
// On a flat test network there is no per-rig router, so (2) is dead, and (3)
// only ran at boot / every 5 min / after a failed POST — so a logger that
// booted AFTER the module stayed invisible for up to 5 minutes, and a logger
// whose IP changed (DHCP) took just as long to re-find.
//
// v1.15.40 NOTE — mDNS tier (3) is now UNRELIABLE, not this file's fault:
// v1.15.32 made bringUpWifi() keep WIFI_AP_STA (setup AP + station) active
// permanently, even after a successful WiFi connection, to fix a boot-loop
// bug. Simultaneous AP+STA is a documented Espressif/arduino-esp32
// limitation for mDNS — MDNS.queryService() frequently gets no replies in
// that mode even though the Pi is advertising correctly and everything is
// on the same network (see espressif/arduino-esp32#10613 and similar).
// Before v1.15.32 the module dropped to plain WIFI_STA once connected,
// where mDNS works fine — that's why this "just worked" a few versions ago
// and doesn't now. We are NOT reverting to mode-switching (that's what
// caused the boot-loop this was fixing). Instead tier (4), the TCP
// port-sweep, is now the primary fallback in practice, not a rare backstop
// — see the first-sweep timer fix below.
//
// Strategy now, in priority order:
//   1. static piHost — always wins, it's the per-rig override
//   2. legacy rigNNN SSID derivation (still used on the per-rig-router rigs)
//   3. mDNS browse for _rig-logger._tcp, re-run on a short retry interval
//      while unresolved and re-verified on a long interval once resolved
//      (expect this to often fail silently now — see v1.15.40 note above)
//   4. TCP port-sweep fallback: walk the module's own /24 looking for
//      something listening on :8080. Runs immediately on first attempt
//      (not after a 15-min wait — see _discSweepEverRun below), and is
//      rate-limited afterward so it can't stall the CAN loop.
//
// NO EXTERNAL LIBRARY, AND NO EXTRA INCLUDES EITHER.
// Two earlier attempts at this file broke Sarah's build:
//   v1.15.17 — `#include <ESP32Ping.h>`: not in the core, it's a separate
//              Library Manager install (marian-craciunescu/ESP32Ping).
//   v1.15.18/19 — `#include <WiFiClient.h>` and `<HTTPClient.h>`: those are
//              NOT header names in the esp32 core. The classes live in
//              WiFi.h / NetworkClient.h and the HTTP client is only pulled in
//              by HTTPClient.h's real path, which Arduino's sketch preprocessor
//              already resolves for the .ino. Including them by the wrong name
//              fails exactly like the ping one did.
// So: this file includes ONLY Arduino.h and ESPmDNS.h, and uses WiFiClient +
// HTTPClient purely as types, relying on the .ino having included <WiFi.h> and
// <HTTPClient.h> above it. That is the same contract the other .h files in this
// sketch already use (webui.h uses WebServer& without including WebServer.h).
//
// THREADING: everything here runs from loopTask on core 1 only (same as the
// old resolvePi). webui.h reads discoveryMethod(), a plain String copy — the
// same cross-task pattern already used for resolvedPiIp on this firmware.
#ifndef DISCOVERY_H
#define DISCOVERY_H

#include <Arduino.h>
#include <ESPmDNS.h>
// Requires <WiFi.h> and <HTTPClient.h> already included by the .ino (they are,
// at the top of waveshare-s3-sensors.ino) for WiFiClient / HTTPClient / WiFi.

static const unsigned long DISCOVER_RETRY_MS  = 20000UL;   // unresolved: retry discovery every 20s
static const unsigned long DISCOVER_VERIFY_MS = 300000UL;  // resolved: re-verify via mDNS every 5 min
// v1.15.41 (Sarah, 2026-09-16: "waiting 15 minutes is too long for plug and
// play"): a sweep only costs ~25-30s worst case on a quiet /24 (254 hosts x
// SWEEP_PORT_TIMEOUT), so there was never a real cost reason for a 15-minute
// gap between attempts — that number was just carried over unchanged from
// the original "rare backstop" design intent, before mDNS became unreliable
// in AP+STA mode (see file header) and the sweep became the primary path in
// practice. The realistic failure this gap matters for: module boots faster
// than the Pi (e.g. both powered on together, Pi still in its own boot),
// first sweep finds nothing, and under the old 15-min value the module then
// sat undiscoverable for the rest of that gap even once the Pi came up.
// 90s covers that startup-race case comfortably while still being a small
// fraction of one duty cycle on the LAN (roughly 30s of scanning per 90s,
// i.e. ~33% duty at worst, only while genuinely unresolved — once found,
// discoverLogger() never re-enters this branch again until invalidated).
static const unsigned long SWEEP_RETRY_MS     = 90000UL;   // don't re-sweep the /24 more than every 90s
static const int           SWEEP_PORT_TIMEOUT = 120;       // ms per host waiting for a SYN/ACK
static const uint16_t      LOGGER_PORT        = 8080;

static String        _discResolvedIp  = "";
static String        _discMethod      = "none";
static unsigned long _discLastAttempt = 0;
static unsigned long _discLastSweep   = 0;
// v1.15.40 fix: _discLastAttempt/_discLastSweep both start at 0, and the old
// gating was `now - last >= INTERVAL`, i.e. "has it been INTERVAL since last
// ran" — which on a fresh boot means "wait a full INTERVAL before the FIRST
// run ever happens." For the sweep (SWEEP_RETRY_MS = 15 min) that meant a
// module which can't reach the Pi via mDNS (now the common case, see file
// header) sat completely undiscoverable for 15 minutes after every boot.
// These two flags make the first attempt of each tier run immediately.
static bool           _discMdnsEverRun  = false;
static bool           _discSweepEverRun = false;

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
// A live host with the port closed sends RST and returns immediately; only a
// silent/dead host costs the full timeout.
static bool tcpPortOpen(const IPAddress& ip, uint16_t port, int timeoutMs) {
  WiFiClient c;
  if (!c.connect(ip, port, timeoutMs)) return false;
  c.stop();
  return true;
}

// Walk the module's own /24 looking for a listener on LOGGER_PORT.
// Cost: dead slots cost SWEEP_PORT_TIMEOUT each, so a quiet /24 is roughly 25s
// worst case — which is why this only runs after mDNS has already failed, and
// is rate-limited.
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
  // v1.15.40: !_discMdnsEverRun forces the very first call through
  // immediately instead of waiting DISCOVER_RETRY_MS (20s) after boot.
  bool due = !_discMdnsEverRun || (_discResolvedIp.isEmpty()
               ? (now - _discLastAttempt >= DISCOVER_RETRY_MS)
               : (now - _discLastAttempt >= DISCOVER_VERIFY_MS));
  if (due) {
    _discMdnsEverRun = true;
    _discLastAttempt = now;
    int n = MDNS.queryService("_rig-logger", "_tcp");
    if (n > 0) {
      String found = MDNS.address(0).toString();
      if (found != _discResolvedIp) {
        Serial.printf("[Disc] mDNS: %s -> %s\n", _discResolvedIp.c_str(), found.c_str());
      }
      _discResolvedIp = found;
      _discMethod = "mdns";
      return _discResolvedIp;
    }
    // mDNS silent. If we already hold an address, keep it — one missed browse
    // isn't evidence the logger went away, and dropping it would cost data.
    if (!_discResolvedIp.isEmpty()) return _discResolvedIp;
  }

  // 4. TCP port sweep fallback (only while still unresolved).
  // v1.15.40 fixes (two):
  //   a) !_discSweepEverRun forces the very first sweep to run immediately
  //      instead of waiting SWEEP_RETRY_MS (15 min) after boot — this was
  //      the main bug (see file header).
  //   b) removed the old _discSweepExhausted latch, which PERMANENTLY
  //      stopped retrying the sweep after one failed attempt for the rest
  //      of the boot session, only re-armed by an mDNS success. That was
  //      fine back when mDNS was the reliable primary tier — it isn't
  //      anymore. In practice: Pi not up yet on the module's one sweep
  //      attempt after boot -> sweep gives up for good -> module never
  //      finds the Pi even after it comes online, until rebooted. Now the
  //      sweep just retries every SWEEP_RETRY_MS forever while unresolved,
  //      same pacing as before, no permanent give-up.
  if (_discResolvedIp.isEmpty() &&
      (!_discSweepEverRun || (now - _discLastSweep >= SWEEP_RETRY_MS))) {
    _discSweepEverRun = true;
    _discLastSweep = now;
    if (sweepSubnet()) {
      _discMethod = "subnet-sweep";
      return _discResolvedIp;
    }
    Serial.println("[Disc] Sweep found nothing this pass — will retry in "
                   "~90s (or sooner if mDNS succeeds). Set piHost manually "
                   "if the logger is on a different subnet.");
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
