// FIRMWARE VERSION: rig-module-sensors-1.16.0 (see FW_VERSION in config.h)
// =============================================================================
// waveshare-s3-sensors.ino — Direct-Sensor Rig Module
// Waveshare ESP32-S3-RS485-CAN (isolated, DIN-rail, ESP32-S3)
//
// Unlike waveshare-s3/ (which talks to ONE fixed analog-to-Modbus adapter
// board), this variant is a real Modbus RTU MASTER that polls a
// configurable list of independent RS485 sensors directly — each its own
// Modbus slave with its own register map, data type, and scaling. Works
// with any Modbus RTU sensor (pressure, temp, flow, level...) as long as
// you know its slave ID + register + data type. Includes a live register-
// probe + bus-scan diagnostics tool so you can find that info by trial
// without a laptop or separate Modbus utility.
//
// Also brings up CAN (unused on the sibling waveshare-s3/ variant) via the
// ESP32-S3's native TWAI controller in listen-only mode: a raw frame
// sniffer for looking at unknown bus traffic, plus a configurable list of
// "signals" (byte range + decode rule) once you know what to extract.
//
// POSTs JSON telemetry to Rig Pi Logger — same wire format/endpoint as the
// other rig-module variants (/api/rig/module), so the existing Pi ingest
// and rig-modules.html dashboard need no changes.
//
// Libraries required (install via Arduino Library Manager):
//   ArduinoJson  (Benoit Blanchon) >= 6.x
//   NTPClient    (Fabrice Weinberg)
//   LittleFS / Preferences / ESPmDNS / ArduinoOTA / HTTPClient / WebServer
//     / WiFi (all built-in to the ESP32 Arduino core)
//   driver/twai.h (built-in to the ESP32 Arduino core — no library needed)
//
// Arduino IDE board settings:
//   Board: "ESP32S3 Dev Module"
//   USB CDC On Boot: Enabled
//   Flash Size: 16MB, PSRAM: OPI PSRAM
//   Partition Scheme: 16M Flash (3MB APP/9.9MB FATFS)
//   To enter download mode if flashing ever hangs: hold BOOT, tap RESET,
//   release RESET, then release BOOT.
// =============================================================================

#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <HTTPClient.h>
#include <Update.h>
#include <Preferences.h>
#include <LittleFS.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <NTPClient.h>
#include <time.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <nvs.h>

#include "config.h"
#include "discovery.h"
#include "modbus.h"
#include "can.h"
#include "scaling.h"

#include "webui.h"
#include "cli.h"

// =============================================================================
// PIN DEFINITIONS — Waveshare ESP32-S3-RS485-CAN
// Same pins as the adapter-board variant (waveshare-s3/) — verified
// against Waveshare's schematic + Battery-Emulator / esphome-yambms ports.
//   RS485: TX=GPIO17, RX=GPIO18, DE/RE=GPIO21 (SP3485, HIGH=transmit)
//   CAN:   TX=GPIO15, RX=GPIO16 (native ESP32-S3 TWAI + onboard transceiver)
// =============================================================================
#define RS485_TXD   17
#define RS485_RXD   18
#define RS485_DE    21
#define CAN_TXD     15
#define CAN_RXD     16

// =============================================================================
// GLOBALS
// =============================================================================
Preferences prefs;
WiFiUDP ntpUDP;
NTPClient ntpClient(ntpUDP, "pool.ntp.org", 0, 3600000);

SemaphoreHandle_t stateMutex;     // guards sensorReadings/canReadings
SemaphoreHandle_t modbusBusMutex; // guards Serial2 (poll task vs. web diagnostics)

ModuleConfig cfg;
SensorReading    sensorReadings[MAX_SENSORS];
CanSignalReading canReadings[MAX_CAN_SIGNALS];
ModbusChannelReading modbusReadings[MAX_MODBUS_CHANNELS];
String modbusDetectedType = "unknown";

WebServer webServer(80);

String resolvedPiIp = "";
unsigned long lastPiResolve = 0;
unsigned long lastPostMs = 0;
int lastPostStatus = 0;
bool lastPostOk = false;
int bufferCount = 0;
bool flushNow = false;

// v1.15.32: apModeActive now means "not currently connected to a site
// network" (drives the /config banner + resolvePi() gating) — it is NOT
// the same thing as "is the AP radio broadcasting". The AP radio is ALWAYS
// broadcasting, in every state, from very early in setup() onward (see
// bringUpWifi() below). See that function's header comment for the full
// rationale — this replaces the old startSetupAP()/connectWifi() pair and
// the ensureApAlive()/s_apKeepAlive patch that used to paper over the mode
// churn between them.
bool apModeActive = false;
String apSSID = "";

// Set false only if LittleFS is unusable even after a format (see setup()).
// Every LittleFS user in this sketch checks it first so a dead filesystem
// degrades to "no buffering" instead of repeated failed opens in the loop.
bool fsUsable = true;

// =============================================================================
// QUIET THE USB-CDC PORT WHILE THE RADIO IS BRINGING UP (v1.15.21)
//
// SYMPTOM Sarah reported: the serial monitor constantly disconnects and
// reconnects, and it's worst around a WiFi change.
//
// This board's USB is the ESP32-S3's OWN peripheral (USB-CDC on boot, not an
// FTDI/CH340 chip). The tiny USB task runs on core 0, and the WiFi driver's
// init/calibration is a CPU-bound burst that starves it — the device stops
// answering SETUP for longer than the host allows, Linux drops it, and it
// re-enumerates a second later. That's the disconnect/reconnect cycle, and
// it's why it clusters exactly where WiFi work happens:
//   - setup(): esp_log_level_set("wifi", ESP_LOG_VERBOSE) + Serial.setDebugOutput
//     (true) + 5 connect attempts, each doing WiFi.mode(WIFI_OFF) → scan →
//     WiFi.begin()
//   - the WiFi-save reboot, which runs that whole storm right after boot
//
// FIX: drop the radio-log spam to WARN and turn the Arduino assert channel off
// for the duration of the connect storm, then restore both. Real diagnostics
// survive (our own Serial.printf lines are untouched); only the driver's
// firehose is muted while it's most likely to starve USB.
//
// Set these BEFORE the first call below and they stay in effect for the whole
// boot if you want maximum serial stability during a bring-up you're watching.
static bool _quietUsbActive = false;


// =============================================================================
// PIN DEFINITIONS — Waveshare ESP32-S3-RS485-CAN
// Same pins as the adapter-board variant (waveshare-s3/) — verified
// against Waveshare's schematic + Battery-Emulator / esphome-yambms ports.
//   RS485: TX=GPIO17, RX=GPIO18, DE/RE=GPIO21 (SP3485, HIGH=transmit)
//   CAN:   TX=GPIO15, RX=GPIO16 (native ESP32-S3 TWAI + onboard transceiver)
// =============================================================================
#define RS485_TXD   17
#define RS485_RXD   18
#define RS485_DE    21
#define CAN_TXD     15
#define CAN_RXD     16

static void quietUsbForRadioWork() {
  if (_quietUsbActive) return;
  _quietUsbActive = true;
  esp_log_level_set("wifi", ESP_LOG_WARN);
  esp_log_level_set("wifi_init", ESP_LOG_WARN);
  esp_log_level_set("phy_init", ESP_LOG_WARN);
  esp_log_level_set("phy", ESP_LOG_WARN);
  esp_log_level_set("system_api", ESP_LOG_WARN);
  esp_log_level_set("nvs", ESP_LOG_WARN);
  Serial.setDebugOutput(false);
}
static void restoreVerboseRadioLogs() {
  if (!_quietUsbActive) return;
  _quietUsbActive = false;
  esp_log_level_set("wifi", ESP_LOG_VERBOSE);
  esp_log_level_set("wifi_init", ESP_LOG_VERBOSE);
  esp_log_level_set("phy_init", ESP_LOG_VERBOSE);
  esp_log_level_set("phy", ESP_LOG_VERBOSE);
  esp_log_level_set("system_api", ESP_LOG_VERBOSE);
  esp_log_level_set("nvs", ESP_LOG_VERBOSE);
  Serial.setDebugOutput(true);
}

// =============================================================================
// SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  // REMOVED 2026-09-17 (Sarah: build switches no longer needed in
  // production — see config.h's "BUILD SWITCHES" section header for the
  // full removal note). This used to be three troubleshooting-only
  // blocks here: QUIET_SERIAL_BOOT (muted radio logs for the whole boot),
  // FORGET_WIFI_ONCE (one-shot WiFi-credential wipe), and
  // FACTORY_RESET_ONCE (one-shot full NVS wipe) — each gated by a
  // compile-time #if plus its own NVS "already fired" done-flag so
  // leaving the switch at 1 in a flashed binary wouldn't re-wipe on
  // every subsequent boot. All three needed a reflash to trigger anyway;
  // the equivalent recovery actions are one tap in the web UI with no
  // reflash needed at all: /config's "Forget WiFi" button, and
  // /system's "Factory Reset" button.

  // See modbus/WiFi self-heal rationale below (ensureStaStarted) — these
  // two logging systems both matter for diagnosing WiFi driver failures.
  esp_log_level_set("wifi", ESP_LOG_VERBOSE);
  esp_log_level_set("wifi_init", ESP_LOG_VERBOSE);
  esp_log_level_set("phy_init", ESP_LOG_VERBOSE);
  esp_log_level_set("phy", ESP_LOG_VERBOSE);
  esp_log_level_set("system_api", ESP_LOG_VERBOSE);
  esp_log_level_set("nvs", ESP_LOG_VERBOSE);
  Serial.setDebugOutput(true);

  // Disable WiFi's own flash-persistent config storage FIRST, before any
  // WiFi.mode()/begin()/softAP() call anywhere in this sketch — prevents
  // the WL_STOPPED/scan=-2 NVS-corruption wedge seen on the LilyGo variant
  // (root cause: WiFi.persistent(false) was previously set too late).
  WiFi.persistent(false);

  Serial.println("\n\n========================================");
  Serial.println("[BOOT] Rig Module (Direct Sensors) " FW_VERSION);
  Serial.println("[BOOT] Starting up...");
  Serial.printf("[BOOT] CPU freq: %d MHz, free heap: %d bytes\n", ESP.getCpuFreqMHz(), ESP.getFreeHeap());
  Serial.println("========================================");

  Serial.println("[BOOT] Mounting LittleFS...");
  // v1.15.21: mount WITHOUT auto-format first. LittleFS.begin(true) formats
  // the partition itself when the mount fails, and doing that on top of a
  // filesystem that another task is mid-write to is how /buffer.jsonl got
  // wiped on reboot (see webui.h handleConfigPost — the WiFi-save reboot
  // happens from the webTask while loopTask can be inside appendBufferEntry).
  // One clean retry, then format only as a genuine last resort, and say so
  // loudly because it destroys the outage buffer.
  // Print WHICH partition we were actually given before blaming the
  // filesystem — a missing/renamed "spiffs" partition (wrong Partition Scheme
  // selected in the IDE) fails the mount exactly like corruption does, and the
  // two need completely different fixes.
  {
    const esp_partition_t* fsPart = esp_partition_find_first(
      ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, NULL);
    if (!fsPart) {
      Serial.println("[BOOT] !!! NO storage partition in the running partition table.");
      Serial.println("[BOOT] !!! Partition Scheme is wrong — set it to "
                     "\"16M Flash (3MB APP/9.9MB FATFS)\" and re-flash. "
                     "LittleFS cannot work without it.");
      fsUsable = false;
    } else {
      Serial.printf("[BOOT] Storage partition: \"%s\" @0x%x size=%u bytes\n",
                    fsPart->label, (unsigned)fsPart->address, (unsigned)fsPart->size);
    }
  }

  if (fsUsable && !LittleFS.begin(false)) {
    Serial.println("[BOOT] LittleFS mount failed — retrying once before any format");
    delay(200);
    if (!LittleFS.begin(false)) {
      Serial.println("[BOOT] LittleFS still failing — FORMATTING (buffered data lost)");
      LittleFS.format();
      if (!LittleFS.begin(true)) {
        Serial.println("[BOOT] !!! LittleFS unusable even after format — "
                       "buffering and OTA disabled, rest of firmware continues");
        fsUsable = false;
      }
    }
  }
  if (fsUsable) {
    Serial.printf("[BOOT] LittleFS OK, total=%d used=%d\n", LittleFS.totalBytes(), LittleFS.usedBytes());
  }

  Serial.println("[BOOT] Loading config from NVS...");
  prefs.begin("rigmod", false);
  loadConfig(prefs, cfg);
  prefs.end();

  buildModuleId(cfg);
  Serial.printf("[BOOT] Module ID  : %s\n", cfg.moduleId.c_str());
  Serial.printf("[BOOT] Module name: %s\n", cfg.moduleName.c_str());
  Serial.printf("[BOOT] WiFi SSID: %s\n", cfg.wifiSSID.isEmpty() ? "(none saved)" : cfg.wifiSSID.c_str());
  Serial.printf("[BOOT] Pi host  : %s\n", cfg.piHost.isEmpty() ? "(mDNS auto)" : cfg.piHost.c_str());
  Serial.printf("[BOOT] Poll int : %d s\n", cfg.pollIntervalS);

  int nSensors = 0;
  for (int i = 0; i < MAX_SENSORS; i++) {
    if (cfg.sensors[i].enabled) {
      nSensors++;
      Serial.printf("[BOOT]   slot %d: \"%s\" slaveId=%d fc=%d reg=0x%04X\n",
        i, cfg.sensors[i].name.c_str(), cfg.sensors[i].slaveId,
        cfg.sensors[i].funcCode, cfg.sensors[i].regAddr);
    }
  }
  Serial.printf("[BOOT] Configured sensors: %d / %d slots\n", nSensors, MAX_SENSORS);

  stateMutex = xSemaphoreCreateMutex();
  modbusBusMutex = xSemaphoreCreateMutex();

  Serial.printf("[BOOT] RS485 pins: RX=%d TX=%d DE=%d baud=%ld\n",
    RS485_RXD, RS485_TXD, RS485_DE, cfg.modbusBaud);
  modbusInit(RS485_RXD, RS485_TXD, RS485_DE, cfg.modbusBaud);

  // No automatic bus scan at boot — sensors already saved/enabled in
  // config just start polling normally below. Use the "Auto-Detect &
  // Enable Now" button on /sensors if you ever need to find new ones.

  // BOOT ORDER CHANGE (2026-09-10, after a real field incident): CAN
  // init used to run HERE, before connectWifi() — moved to AFTER WiFi/AP
  // + the web server are up (see below, right after webServer.begin()).
  // Reasoning: CAN is unconditional now (no toggle, see can.h) and
  // canopenBringup() blocks for a full 2 seconds transmitting on the CAN
  // bus during every boot. Whatever WAS or WASN'T the actual cause of
  // that incident, network access (WiFi/AP + web UI) should NEVER be
  // gated behind CAN hardware initializing successfully — if a CAN
  // transceiver is ever miswired, floating, or the driver install/start
  // call ever misbehaves on some board, you should still always be able
  // to reach the web UI to fix it. Sensors/CAN not coming up is a
  // recoverable, visible-on-the-web-UI problem; losing network access
  // entirely is not.

  // v1.15.32: bringUpWifi() replaces the old startSetupAP()/connectWifi() pair.
  // The setup AP is unconditional and permanent now (see that function's header
  // comment for the full story) — no AP_FIRST_NO_WIFI_WAIT switch needed, this
  // is just always how it works.
  bringUpWifi();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("[NTP] Starting NTP client...");
    ntpClient.begin();
    ntpClient.update();
    Serial.printf("[NTP] Time set: %s, epoch: %lu\n",
      ntpClient.isTimeSet() ? "YES" : "NO", ntpClient.getEpochTime());
  } else {
    Serial.println("[NTP] Skipping NTP (no STA connection)");
    ntpClient.begin();
  }

  String hostname = cfg.moduleId;
  hostname.toLowerCase();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[mDNS] Starting as %s.local\n", hostname.c_str());
    MDNS.begin(hostname.c_str());
    setupOTA(hostname);
  }

  Serial.println("[HTTP] Setting up web routes...");
  setupWebRoutes(webServer, cfg, prefs, sensorReadings, canReadings, stateMutex);
  webServer.begin();
  // v1.15.32: the setup AP is unconditional now, so it's ALWAYS a valid way
  // to reach the web UI regardless of STA outcome — print both lines instead
  // of an either/or, so the fallback address is never hidden.
  Serial.printf("[HTTP] Setup AP web server always at http://192.168.4.1/ (connect to \"%s\" / \"modulesetup\")\n", apSSID.c_str());
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[HTTP] Also reachable on site network at http://%s/\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("[HTTP] Not connected to a site network yet — use the setup AP above.");
  }

  // CAN starts HERE, deliberately AFTER WiFi/AP + the web server are
  // already up and reachable — see this function's comment above
  // connectWifi() for why. Unconditional — plug-and-play, no config
  // toggle, no way to turn it off (Sarah 2026-09-10: doesn't want the
  // OPTION to disable it, not just a default). canopenBridge still
  // gates transmit-capable mode vs. safe listen-only tap (see can.h
  // header comment) — that stays a real setting since it changes
  // electrical behavior on someone else's bus, but the CAN controller
  // itself always starts.
  {
    bool bridgeMode = cfg.canopenBridge;
    Serial.printf("[BOOT] CAN pins: TX=%d RX=%d bitrate=%ld mode=%s\n", CAN_TXD, CAN_RXD,
      cfg.canBitrate, bridgeMode ? "CANopen Bridge (TX enabled)" : "listen-only");
    canStart(CAN_TXD, CAN_RXD, cfg.canBitrate, /*listenOnly=*/!bridgeMode);
    if (bridgeMode) {
      canopenBringup(cfg.canopenNodeId, cfg.canopenTargetSpecific);
    }
  }

  bufferCount = countBufferEntries();
  Serial.printf("[BOOT] Buffered entries: %d\n", bufferCount);

  Serial.println("[BOOT] Starting Modbus poll task...");
  xTaskCreatePinnedToCore(pollTask, "poll", 8192, NULL, 1, NULL, 1);

  cliInit();

  Serial.println("========================================");
  Serial.println("[BOOT] Ready! Open the web UI to configure sensors.");
  Serial.println("========================================\n");
}

// =============================================================================
// LOOP — handles OTA, NTP, CAN polling, Pi discovery/posting
// =============================================================================
void loop() {
  // v1.15.29: ~1Hz "still alive" line. bringUpWifi()'s retry loop can run for
  // up to ~2 minutes with nothing else printed (its own printf lines already went
  // out before the loop's internal delays), and the poll task doesn't start
  // until setup() returns — so a board that looks silent is indistinguishable
  // from a board that has crashed. One line a second proves the difference.
  //
  // v1.15.32: apIP is now printed UNCONDITIONALLY, not just when
  // apModeActive. The setup AP is always broadcasting from very early in
  // setup() onward regardless of whether STA ever connects — the old
  // apModeActive-gated version would print "-" here even while the AP was
  // fully up and reachable, which is exactly the wrong information for
  // exactly the case (can't reach 192.168.4.1) this line exists to help with.
  {
    static unsigned long _hbLast = 0;
    unsigned long _hbNow = millis();
    if (_hbNow - _hbLast >= 1000UL) {
      _hbLast = _hbNow;
      Serial.printf("[HB] up=%lus mode=%d ap=%d sta=%d wifi=%d apIP=%s pi=%s\n",
        (unsigned long)(_hbNow / 1000UL), (int)WiFi.getMode(), (int)apModeActive,
        (int)(WiFi.status() == WL_CONNECTED), (int)WiFi.status(),
        WiFi.softAPIP().toString().c_str(),
        resolvedPiIp.isEmpty() ? "-" : resolvedPiIp.c_str());
    }
  }

  // Runtime WiFi reconnect watchdog (see its own header comment above) —
  // called BEFORE the apModeActive check below so a drop noticed THIS tick
  // immediately gates discovery/postToPi the same tick, not one tick late.
  wifiWatchdog();

  ArduinoOTA.handle();
  webServer.handleClient();
  ntpClient.update();
  cliPoll();

  // Drain any pending CAN frames — cheap no-op if CAN isn't enabled.
  canPoll(cfg, canReadings, stateMutex);

  // CANopen Bridge self-heal — internally rate-limited, cheap no-op if
  // canopenBridge is off or traffic is already flowing. See can.h's
  // canopenBringupIfDue() comment.
  if (cfg.canopenBridge) {
    canopenBringupIfDue(cfg.canopenNodeId, cfg.canopenTargetSpecific);
  }

  // BUG CAUGHT + FIXED 2026-09-17 (before ever shipping — found while adding
  // wifiWatchdog() above): this used to be gated on `if (!apModeActive)`.
  // Before wifiWatchdog() existed, apModeActive was ONLY ever set at boot
  // (true if STA failed, false if it connected) and NEVER updated again —
  // so a module that connected fine at boot and then dropped WiFi mid-
  // session in the field kept apModeActive stuck at false, meaning this
  // block kept running the whole time it was actually offline. That was an
  // accident, not a design — but it's exactly what made buffering work
  // during a real outage: postToPi() -> httpPost() fails fast (Pi
  // unreachable), and postToPi()'s OWN failure path calls
  // appendBufferEntry() to save the sample to flash. Once wifiWatchdog()
  // started keeping apModeActive ACCURATE (true the moment a drop is
  // noticed, which is the whole point of the fix), this gate would have
  // started SKIPPING resolvePi()/postToPi() entirely during every outage —
  // silently losing data instead of buffering it, the opposite of what
  // Sarah asked for ("modules in poor WiFi locations"). Removed the gate:
  // resolvePi()/postToPi()/discoverLogger() were already designed to be
  // called unconditionally and degrade gracefully with no connectivity
  // (sweepSubnet() bails out cleanly on no local IP, mDNS just returns 0,
  // httpPost() just fails and gets buffered) — that graceful-failure path
  // IS the buffering mechanism, so it needs to keep running through an
  // outage, not stop the moment one is detected.
  {
    // Discovery is a state machine with its own timers (see discovery.h):
    // calling it every cycle is cheap when resolved and lets it retry
    // promptly when not. resolvedPiIp is core-1-owned, same as before.
    resolvePi();
    static unsigned long lastPostAttempt = 0;
    unsigned long now = millis();
    if (!resolvedPiIp.isEmpty() && (now - lastPostAttempt >= (unsigned long)cfg.pollIntervalS * 1000UL)) {
      lastPostAttempt = now;
      postToPi();
    }
  }

  delay(20); // shorter than other variants' 100ms — keeps CAN frame draining responsive
}

// =============================================================================
// v1.15.32: ensureStaStarted()'s NVS-erase self-heal REMOVED.
//
// THIS WAS THE ACTUAL CAUSE OF SARAH'S "changing WiFi network boot-loops, have
// to erase flash and reflash from the IDE" report (2026-09-15). The old
// function called nvs_flash_erase() — which wipes the ENTIRE NVS partition
// (WiFi driver's own PHY calibration/country-code data included, not just our
// "rigmod" namespace) — as a "self-heal" any time WiFi.mode(WIFI_STA) left the
// interface at WL_STOPPED for 2s. That is squarely radio-driver noise you can
// hit while WiFi is being cycled during a config change; the old
// connectWifi() cycled WIFI_OFF -> WIFI_STA on every one of its 5 retry
// attempts, which is exactly the kind of rapid mode churn that trips this.
// Erasing the WHOLE NVS partition mid-boot, then immediately trying to write
// straight back into it (nvs_flash_init() + saveConfig()) while the WiFi
// driver is simultaneously retrying against a partition that just changed out
// from under it, produced exactly the crash/wedge Sarah described that only
// esptool erase_flash + a clean reflash could get her out of.
//
// The actual WL_STOPPED bug this was chasing (see WiFi.persistent(false) at
// the top of setup()) is already fixed at its real root cause. bringUpWifi()
// below also no longer cycles WIFI_OFF/WIFI_STA at all — mode is set to
// WIFI_AP_STA exactly once and left alone — so the condition this self-heal
// existed to patch around shouldn't recur. If WiFi.begin() ever still fails
// outright, that's now just a normal "couldn't join, stay on the AP" case,
// not a reason to touch NVS.
// =============================================================================

// =============================================================================
// RIG NETWORK AUTO-DISCOVERY — REMOVED IN v1.15.23 AT SARAH'S REQUEST
// =============================================================================
// This used to hold isRigSSID() + tryAutoConnectRigNetwork(): when NVS had no
// saved SSID, it scanned for any SSID matching "rig" + digits and joined it
// using a compiled-in password (RIG_WIFI_PASS "7804991970"), then saved that
// pair to NVS as if the user had chosen it.
//
// Removed because it made the module look like it had a hardcoded network: a
// board with a blank config would find whatever rigNNN AP was in range (e.g.
// the bench test AP on drill-pi-1's router), join it, and PERSIST that choice.
// From then on a network name came back on every boot no matter how many times
// the chip was erased-and-reflashed, because the SSID was being re-derived from
// the live RF environment at first boot rather than left empty.
//
// NOTE ON THE WORDING ABOVE, because it invites the wrong conclusion: nothing
// was ever hardcoded. No SSID string exists anywhere in this repo — grep for
// any network name and the only hits are comments. The persistence was NVS
// storage plus this routine re-deriving a candidate from a live scan.
//
// The WL_STOPPED self-heal in ensureStaStarted() does call nvs_flash_erase()
// and then saveConfig() right after, which LOOKS like it re-writes a stale SSID
// over a clean erase. It can't: loadConfig() runs earlier in setup() and is the
// only thing that populates cfg.wifiSSID, so on a freshly erased chip that field
// is empty and the rebuild writes an empty SSID. Self-heal can only ever
// republish what was already in NVS — it cannot invent a network.
//
// Policy now: the firmware never joins, guesses, or persists a network the
// user didn't explicitly save on /config. Blank config -> setup AP, full stop.
// isRigSSID() is gone too; nothing else referenced it. If the per-rig-router
// convenience is ever wanted back, it must be an explicit opt-in checkbox on
// /config, not a boot-time default.

// Configures + (re)issues the AP radio side only. Does NOT touch WiFi.mode()
// — the caller (bringUpWifi()) sets WIFI_AP_STA exactly once and this never
// runs a second time after that, so there's nothing here to accidentally
// undo a mode change. Safe to think of as "the AP settings", not an action
// with side effects on STA.
static void configureSetupAP() {
  uint8_t mac[6];
  esp_efuse_mac_get_default(mac);
  char suffix[7];
  snprintf(suffix, sizeof(suffix), "%02X%02X%02X", mac[3], mac[4], mac[5]);
  apSSID = "RigModule-" + String(suffix);

  // v1.15.28: pin the AP to channel 1 and force 20MHz.
  //
  // Two separate real-world failures this fixes, both of which look identical
  // from the client side — "joined the AP, browser cannot connect":
  //
  // 1) Without an explicit channel, softAP() lets the ARF algorithm pick, and it
  //    can land on 5GHz-adjacent/DFS-adjacent settings or hop when the STA side
  //    scans. A phone that associated while it was on one channel then sits on a
  //    channel the AP has left. Pinning to 1 removes the variable entirely.
  //
  // 2) Bandwidth: on a BW_SECOND (40MHz) AP, a client that negotiated 20MHz —
  //    common on phones in a crowded office — can associate and then fail to
  //    pass traffic. HTX_MODE_20 is the conservative choice.
  //
  // NOTE: no WiFi.setBandWidth() call — that's an ESP8266-core-only API, does
  // not exist on this core (3.2.0). softAP() already defaults to HT20.
  WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1),
                    IPAddress(255, 255, 255, 0));
  WiFi.softAP(apSSID.c_str(), "modulesetup", 1, 0, 4, true);

  Serial.println("[WiFi] ----------------------------------------");
  Serial.printf("[WiFi] Setup AP: \"%s\" / \"modulesetup\"\n", apSSID.c_str());
  Serial.printf("[WiFi] AP IP: %s  channel=1 bw=20MHz\n",
    WiFi.softAPIP().toString().c_str());
  Serial.println("[WiFi] This AP is ALWAYS on, even once connected to a site network —");
  Serial.println("[WiFi] http://192.168.4.1/ always works as a fallback to fix WiFi settings.");
  Serial.println("[WiFi] ----------------------------------------");
}

// =============================================================================
// v1.15.32: bringUpWifi() replaces the old startSetupAP()/connectWifi() pair.
//
// REAL BUG FIXED (Sarah's field report, 2026-09-15): changing the saved WiFi
// network from /config would boot-loop the board hard enough that only an
// esptool erase_flash + fresh reflash from the IDE recovered it — and even
// on a normal boot with a dead/out-of-range saved network, the setup AP was
// only reachable AFTER a ~40-100s retry storm finished, because the AP
// didn't exist until every STA attempt against the (bad) saved network had
// failed.
//
// ROOT CAUSE, two compounding bugs:
//   1. The old connectWifi() retry loop called WiFi.mode(WIFI_OFF) then
//      WiFi.mode(WIFI_STA) between every one of its 5 connection attempts.
//      Once the AP existed (either from an early-start experiment or the
//      fallback path), those mode calls silently tore its radio state down;
//      a patch called ensureApAlive() tried to flip the mode back to
//      WIFI_AP_STA afterward, but flipping a mode bit does not re-run
//      softAP()/softAPConfig() — the AP could end up "associates fine, HTTP
//      never answers", which is exactly what you were seeing.
//   2. ensureStaStarted() treated 2s of WL_STOPPED after WiFi.mode(WIFI_STA)
//      as a reason to nvs_flash_erase() the ENTIRE NVS partition (not just
//      our config — the WiFi driver's own PHY/cal data too) and immediately
//      write straight back into it while still mid-connect-retry. Rapid
//      mode churn during a WiFi change is exactly the kind of radio noise
//      that trips WL_STOPPED transiently — this is almost certainly what
//      actually crashed/wedged the board hard enough to need a flash erase.
//
// THE FIX: stop cycling the radio mode entirely.
//   - WiFi.mode(WIFI_AP_STA) is set ONCE, right here, and never changed again
//     for the rest of this boot (not WIFI_OFF, not WIFI_STA, not WIFI_AP).
//   - The AP is unconditional and PERMANENT — it comes up before any STA
//     attempt and stays up whether or not a site network is ever joined.
//     http://192.168.4.1/ is now always the answer to "how do I fix WiFi
//     settings", never a race against a retry timer.
//   - WiFi.begin() is called directly for retries — the ESP32 WiFi driver
//     supports re-calling begin() on an already-active STA interface without
//     needing disconnect()/mode(WIFI_OFF) first; that churn was never
//     actually required, just inherited from older example code.
//   - No NVS erase self-heal. If STA genuinely can't connect, that's a
//     normal "stay on the AP, let her fix it from /config" outcome, not a
//     reason to touch flash.
// =============================================================================
void bringUpWifi() {
  WiFi.mode(WIFI_AP_STA);
  configureSetupAP();
  apModeActive = true; // corrected below to false if STA connects

  if (cfg.wifiSSID.isEmpty()) {
    Serial.println("[WiFi] No SSID saved — staying on setup AP only.");
    Serial.println("[WiFi] No network is ever guessed or hardcoded; pick one on /config.");
    return;
  }

  Serial.println("[WiFi] ----------------------------------------");
  Serial.printf("[WiFi] Target SSID: %s\n", cfg.wifiSSID.c_str());
  Serial.println("[WiFi] Scanning for networks (AP stays up throughout — mode never changes)...");

  int found = WiFi.scanNetworks();
  int targetChannel = 0;
  uint8_t targetBSSID[6] = {0};
  if (found <= 0) {
    Serial.println("[WiFi] Scan found no networks — will still try WiFi.begin() blind.");
  } else {
    Serial.printf("[WiFi] Scan found %d network(s):\n", found);
    bool targetFound = false;
    int bestRssi = -1000;
    for (int i = 0; i < found; i++) {
      bool isTarget = (WiFi.SSID(i) == cfg.wifiSSID);
      bool is24GHz = (WiFi.channel(i) >= 1 && WiFi.channel(i) <= 14);
      if (isTarget && is24GHz && WiFi.RSSI(i) > bestRssi) {
        targetFound = true;
        bestRssi = WiFi.RSSI(i);
        targetChannel = WiFi.channel(i);
        memcpy(targetBSSID, WiFi.BSSID(i), 6);
      }
    }
    if (!targetFound) {
      Serial.printf("[WiFi] WARNING: \"%s\" not found in scan (or 5GHz-only, unusable on this chip)!\n", cfg.wifiSSID.c_str());
    } else {
      Serial.printf("[WiFi] Target on channel %d, RSSI %d dBm\n", targetChannel, bestRssi);
    }
  }
  WiFi.scanDelete();

  // Mute the driver's log firehose for the connect storm — see
  // quietUsbForRadioWork() for why (radio init starves the USB-CDC task and
  // the host drops/re-enumerates the serial port). Our own [WiFi] lines
  // still print; only ESP_LOG_VERBOSE radio spam is suppressed.
  quietUsbForRadioWork();
  WiFi.setAutoReconnect(false);
  WiFi.persistent(false);

  // v1.15.37 (Sarah, 2026-09-16: "a shorter connect time would make it more
  // reliable and plug and play"): this loop used to run 5 attempts x up to
  // 20s each + 3s between = up to ~112s worst case (saved-but-unreachable
  // network -- exactly the bench scenario, wrong SSID in range/dead
  // password/router off). That entire time is spent doing real CPU-bound
  // radio work on the same core that services the USB-CDC port, which is
  // the documented starvation mechanism behind this board's flaky
  // auto-reset-to-bootloader (esptool "No serial data received" without a
  // manual BOOT+RESET). Shrinking the retry budget directly shrinks how
  // often an Upload attempt lands inside that bad window.
  // 3 attempts x 8s + 2x2s between = ~28s worst case (plus a few seconds
  // for the scan above), down from ~112s. If a network is actually in
  // range with the right password it almost always associates within the
  // first couple seconds anyway -- this budget cut costs real-world
  // reconnect reliability only in marginal-signal edge cases, and trades
  // that for a MUCH shorter window where the board is busy enough with
  // radio work to risk starving USB.
  const int MAX_ATTEMPTS = 3;
  const int ATTEMPT_TIMEOUT_MS = 8000;   // was ~20000 (tries<40 @ 500ms)
  const int RETRY_GAP_MS = 2000;         // was 3000
  const int pollMs = 250;
  const int maxPolls = ATTEMPT_TIMEOUT_MS / pollMs;
  for (int attempt = 1; attempt <= MAX_ATTEMPTS; attempt++) {
    Serial.printf("[WiFi] Attempt %d/%d — connecting to \"%s\"...\n", attempt, MAX_ATTEMPTS, cfg.wifiSSID.c_str());

    // No disconnect()/mode(WIFI_OFF) here — calling WiFi.begin() again on an
    // already-active AP_STA interface is enough to make the driver retry;
    // the mode churn that used to happen here is exactly what was tearing
    // down the AP (see header comment above).
    bool useHint = (targetChannel > 0) && (attempt < MAX_ATTEMPTS);
    if (useHint) {
      Serial.printf("[WiFi]   Using BSSID hint, channel %d\n", targetChannel);
      WiFi.begin(cfg.wifiSSID.c_str(), cfg.wifiPass.c_str(), targetChannel, targetBSSID);
    } else {
      if (targetChannel > 0) Serial.println("[WiFi]   Last attempt — dropping BSSID/channel hint");
      WiFi.begin(cfg.wifiSSID.c_str(), cfg.wifiPass.c_str());
    }

    int tries = 0;
    while (WiFi.status() != WL_CONNECTED && tries < maxPolls) {
      delay(pollMs);
      tries++;
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("[WiFi] Connected on attempt %d!\n", attempt);
      Serial.printf("[WiFi] IP      : %s\n", WiFi.localIP().toString().c_str());
      Serial.printf("[WiFi] MAC     : %s\n", WiFi.macAddress().c_str());
      Serial.printf("[WiFi] RSSI    : %d dBm\n", WiFi.RSSI());
      apModeActive = false; // connected — banner/discovery gating updates
      break;
    } else {
      Serial.printf("[WiFi] Attempt %d failed (final status=%d)\n", attempt, WiFi.status());
      if (attempt < MAX_ATTEMPTS) {
        Serial.printf("[WiFi] Waiting %ds before retry...\n", RETRY_GAP_MS / 1000);
        delay(RETRY_GAP_MS);
      }
    }
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WiFi] All attempts failed against saved network.");
    Serial.println("[WiFi] Setup AP was never torn down — http://192.168.4.1/ is reachable");
    Serial.println("[WiFi] right now to fix credentials on /config.");
  }
  Serial.println("[WiFi] ----------------------------------------");

  // Bring the radio logs back now that the worst of the CPU burst is over —
  // restored regardless of outcome (connected OR still on AP-only) so
  // /system and the console stay useful either way. (QUIET_SERIAL_BOOT, the
  // old opt-out for a port that couldn't handle the restored verbosity, was
  // removed 2026-09-17 along with the rest of the build switches.)
  restoreVerboseRadioLogs();
}

// =============================================================================
// ADDED 2026-09-17 (Sarah: modules are going into locations with poor WiFi —
// needs to automatically retry the saved network if it drops). bringUpWifi()
// above only ever runs ONCE, from setup() — before this, nothing in loop()
// noticed or reacted to a runtime disconnect at all. WiFi.setAutoReconnect
// (false) was set deliberately in bringUpWifi() (see that function's header
// comment on why the ESP32 driver's own reconnect churn was part of the
// original boot-loop bug), so turning that back on is NOT the fix -- this is
// a separate, deliberately gentler watchdog, called from loop() every tick:
//
//   - Does nothing if no SSID is saved (nothing to reconnect to) or already
//     connected -- near-zero cost on the common path.
//   - On WL_CONNECTED -> anything else, immediately marks apModeActive=true
//     (this was ALSO a latent bug: apModeActive is documented since v1.15.32
//     as meaning "not currently connected to a site network", but nothing
//     ever set it back to true after a runtime drop -- it would stay stuck
//     at false, showing a stale "connected" state on /config's save-message
//     branch and the [HB] heartbeat line, even while genuinely offline).
//   - Retries via a bare WiFi.begin(ssid, pass) call, rate-limited to once
//     every WIFI_RECONNECT_INTERVAL_MS -- same "just call begin() again on
//     the live AP_STA interface, no disconnect()/mode() churn" pattern
//     bringUpWifi() already uses, just spread out over wall-clock time
//     instead of a tight retry loop. WiFi.begin() itself returns
//     immediately (the actual association happens in the background), so
//     this never blocks loop() -- a later tick's cheap WiFi.status() check
//     is what notices success, same as the heartbeat line already does.
//   - No log-muting around the retry call (unlike bringUpWifi()'s quiet-
//     during-connect-storm handling): a single begin() call every 15s is a
//     light, one-shot radio operation, not the sustained multi-attempt CPU
//     burst that motivated muting at boot -- muting here would just
//     suppress useful ongoing diagnostics on a board that's supposed to run
//     unattended for a long time.
//   - Deliberately does NOT touch NVS, WiFi.mode(), or call disconnect() --
//     see bringUpWifi()'s header comment for exactly why each of those was
//     the real root cause of the earlier boot-loop bug this firmware
//     already fixed once.
// =============================================================================
static const unsigned long WIFI_RECONNECT_INTERVAL_MS = 15000UL;

void wifiWatchdog() {
  if (cfg.wifiSSID.isEmpty()) return; // nothing saved to reconnect to
  if (WiFi.status() == WL_CONNECTED) {
    if (apModeActive) {
      Serial.println("[WiFi] Watchdog: reconnected.");
      apModeActive = false;
    }
    return;
  }

  // Not connected right now. Make sure apModeActive reflects that truthfully
  // even on the very first tick this is noticed (a runtime drop, not just
  // the already-failed-at-boot case bringUpWifi() itself already flags).
  apModeActive = true;

  static unsigned long lastAttemptMs = 0;
  unsigned long now = millis();
  if (now - lastAttemptMs < WIFI_RECONNECT_INTERVAL_MS) return;
  lastAttemptMs = now;

  Serial.printf("[WiFi] Watchdog: not connected (status=%d) — retrying \"%s\"...\n",
    (int)WiFi.status(), cfg.wifiSSID.c_str());
  WiFi.begin(cfg.wifiSSID.c_str(), cfg.wifiPass.c_str());
}

// =============================================================================
// Pi DISCOVERY
// Thin wrapper over discovery.h — see that file for the full strategy.
// Kept as resolvePi() so the existing loopTask call site is unchanged.
void resolvePi() {
  lastPiResolve = millis();
  resolvedPiIp = discoverLogger(cfg.piHost, cfg.wifiSSID);
}

// =============================================================================
// MODBUS POLL TASK (core 1) — walks the configured sensor list every cycle,
// polling each enabled sensor at its own slave ID/register/type. Unlike the
// adapter-board variants, there's no single "board" to detect — every
// sensor is independently addressed, so a comms failure on one sensor
// doesn't affect any other (each gets its own status/error tracking).
// =============================================================================
void pollTask(void* param) {
  Serial.println("[Poll] Task started, waiting 2s before first poll...");
  delay(2000);

  int cycleCount = 0;
  // Background periodic auto-detect scan removed 2026-08-18 at Sarah's
  // request — it interrupted regular polling every ~5 min while hunting
  // for new sensors on any free slot. New sensors can still be added via
  // the "Auto-Detect & Enable" button on /sensors (modbusAutoDetectAndEnable,
  // still in modbus.h) — this just stops it running unattended in the
  // background and stealing bus time from live polls.
  for (;;) {
    cycleCount++;
    int okCount = 0, failCount = 0;

    if (cfg.modbusBoard.enabled && xSemaphoreTake(modbusBusMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
      int boardRc = modbusPollBoard(cfg.modbusBoard, modbusReadings, modbusDetectedType);
      xSemaphoreGive(modbusBusMutex);
      if (boardRc != MB_OK) { for (int i=0;i<MAX_MODBUS_CHANNELS;i++) { if (cfg.modbusBoard.channels[i].enabled) modbusReadings[i].status="timeout"; modbusReadings[i].valid=false; } }
    }

    for (int i = 0; i < MAX_SENSORS; i++) {
      SensorConfig& s = cfg.sensors[i];
      if (!s.enabled) continue;

      float raw = 0, value = 0;
      int rc;
      if (xSemaphoreTake(modbusBusMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        // verbose=true: prints every RS485 TX/RX byte for this poll to
        // Serial (see modbusSend()/modbusReceive() in modbus.h) — handy
        // for bench testing over USB without needing WiFi/web UI at all.
        rc = modbusPollSensor(s, raw, value, true);
        xSemaphoreGive(modbusBusMutex);
      } else {
        rc = MB_TIMEOUT;
      }

      if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        SensorReading& r = sensorReadings[i];
        r.lastPollMs = millis();
        r.pollCount++;
        if (rc == MB_OK) {
          r.valid = true;
          r.hasValue = true;
          r.rawValue = raw;
          r.value = round(value * 100.0f) / 100.0f;
          r.status = "ok";
          r.displayStatus = "ok";
          r.consecutiveTimeouts = 0;
          r.lastOkMs = millis();
          okCount++;
        } else {
          r.valid = false;
          r.errorCount++;
          r.status = (rc == MB_TIMEOUT) ? "timeout" : (rc == MB_CRC_ERROR) ? "crc" : "error";
          failCount++;

          if (rc == MB_TIMEOUT) {
            r.consecutiveTimeouts++;
            // Some sensors (slow measurement cycle) only answer on a
            // fraction of polls by design — don't flap the /sensors and
            // /live pages to "timeout" over a short run of these if the
            // sensor has reported a real value before. The raw `status`
            // field always reflects every real timeout regardless of this.
            bool hasReportedBefore = r.hasValue;
            if (!hasReportedBefore || r.consecutiveTimeouts >= TIMEOUT_DISPLAY_THRESHOLD) {
              r.displayStatus = "timeout";
            }
            // else: leave displayStatus as whatever it last was (likely "ok")
          } else {
            // CRC/other errors are real, unexpected failures — always
            // shown immediately, no debouncing.
            r.consecutiveTimeouts = 0;
            r.displayStatus = r.status;
          }
        }
        xSemaphoreGive(stateMutex);
      }

      // Small gap between sensors on the shared bus — lets the line settle
      // and avoids one slow/failing sensor's timeout stacking directly
      // into the next request with zero breathing room.
      vTaskDelay(pdMS_TO_TICKS(20));
    }

    {
      // Summary table printed every cycle (bench-testing over serial —
      // no downside to printing every time; this used to only print for
      // the first 5 cycles then every 20th, which was too sparse for
      // active USB-serial testing without WiFi/web UI).
      Serial.printf("[Poll] Cycle #%d: %d ok, %d failed\n", cycleCount, okCount, failCount);
      if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        for (int i = 0; i < MAX_SENSORS; i++) {
          if (!cfg.sensors[i].enabled) continue;
          SensorConfig& s = cfg.sensors[i];
          SensorReading& r = sensorReadings[i];
          String label = s.name.isEmpty() ? ("Sensor" + String(i)) : s.name;
          if (r.hasValue) {
            Serial.printf("  %-20s slave=%-3d reg=0x%04X  raw=%8.2f  = %8.2f %-6s (%s)\n",
              label.c_str(), s.slaveId, s.regAddr, r.rawValue, r.value, s.unit.c_str(), r.status.c_str());
          } else {
            Serial.printf("  %-20s slave=%-3d reg=0x%04X  -- (%s)\n",
              label.c_str(), s.slaveId, s.regAddr, r.status.c_str());
          }
        }
        // CAN signals (Sarah, 2026-09-17: wanted them in this same [Poll]
        // serial summary alongside RS485 -- previously only RS485 sensors
        // printed here; CAN readings only ever showed up on the web UI's
        // /can or /live pages, invisible over plain USB serial). Same
        // table style/column widths as the RS485 loop above so the two
        // read as one consistent block, just keyed by CAN ID instead of
        // Modbus slave+register -- canReadings[]/canPoll() (can.h) update
        // independently of this task's own poll cycle (driven by whatever
        // frames have actually arrived on the bus), so this only ever
        // prints whatever canReadings currently holds, same read-only
        // spirit as the RS485 print above, not a fresh poll of its own.
        for (int i = 0; i < MAX_CAN_SIGNALS; i++) {
          if (!cfg.canSignals[i].enabled) continue;
          CanSignalConfig& sg = cfg.canSignals[i];
          CanSignalReading& r = canReadings[i];
          String label = sg.name.isEmpty() ? ("CanSig" + String(i)) : sg.name;
          if (r.hasValue) {
            Serial.printf("  %-20s canId=0x%-6X            = %8.2f %-6s (%s)\n",
              label.c_str(), sg.canId, r.value, sg.unit.c_str(), r.status.c_str());
          } else {
            Serial.printf("  %-20s canId=0x%-6X            -- (%s)\n",
              label.c_str(), sg.canId, r.status.c_str());
          }
        }
        xSemaphoreGive(stateMutex);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(cfg.pollIntervalS * 1000));
  }
}

// =============================================================================
// BUILD JSON PAYLOAD
// =============================================================================
// Tracks the newest raw-frame millis() already included in a previous
// POST, so canFrames below only ever sends NEW frames — see can.h's
// canSerializeRecentFrames() comment for why this board's own millis()
// is only used locally for this dedupe, never transmitted as a
// timestamp (the PC times frames by its own receipt time instead).
static unsigned long _lastCanFramesSentMs = 0;

String buildPayload(bool bufferedFlag) {
  // Sized generously: up to MAX_SENSORS (16) + MAX_CAN_SIGNALS (16)
  // entries, each with several fields + optional nested volume object,
  // PLUS up to 40 raw CAN frames (canopenBridge mode) at ~50-60 bytes
  // each once serialized.
  DynamicJsonDocument doc(16384);

  doc["moduleId"] = cfg.moduleId;
  doc["type"]     = cfg.moduleType.isEmpty() ? "generic" : cfg.moduleType;
  doc["name"]     = cfg.moduleName.isEmpty() ? cfg.moduleId : cfg.moduleName;
  doc["moduleName"] = cfg.moduleName;
  if (WiFi.status() == WL_CONNECTED) {
    doc["ip"] = WiFi.localIP().toString();
  }
  doc["fw"]       = FW_VERSION;
  doc["uptimeS"]  = (unsigned long)(millis() / 1000UL);
  doc["rssi"]     = WiFi.RSSI();
  doc["buffered"] = bufferedFlag;
  // CAN is unconditional (no toggle) — always report frame stats. Key kept
  // as "canEnabled" (always true) for backward compat with any existing
  // Pi-side dashboard code that reads it, rather than a wire-format change.
  doc["canEnabled"] = true;
  doc["canFrameRate"] = canGetRecentFrameRate();
  doc["canFrameTotal"] = canGetFrameTotal();

  // Raw CAN frame relay (CANopen Bridge mode only) — decode stays on the
  // PC (ditchwitch-logger's can_listener.py, already handles both
  // CANopen PDO and J1939-shaped frames), this board just forwards
  // what's in the ring buffer since the last successful POST. Skipped
  // entirely (no "canFrames" key at all) when not in bridge mode.
  if (cfg.canopenBridge && !bufferedFlag) {
    JsonArray frames = doc.createNestedArray("canFrames");
    _lastCanFramesSentMs = canSerializeRecentFrames(frames, _lastCanFramesSentMs, 40);
  }

  if (ntpClient.isTimeSet() && ntpClient.getEpochTime() > 1000000000UL) {
    time_t t = ntpClient.getEpochTime();
    struct tm* tm_info = gmtime(&t);
    char buf[32];
    strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", tm_info);
    doc["ts"] = buf;
  } else {
    doc["ts"] = "1970-01-01T00:00:00Z";
  }

  // Sensors reported under "channels" (same key rig-modules.html/Pi ingest
  // already expects from every other rig-module variant) — "ch" here is
  // the sensor's slot index, not a fixed hardware channel number.
  JsonArray chArr = doc.createNestedArray("channels");
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
    for (int i = 0; i < MAX_SENSORS; i++) {
      SensorConfig& s = cfg.sensors[i];
      if (!s.enabled) continue;
      JsonObject c = chArr.createNestedObject();
      c["ch"]   = i;
      c["kind"] = s.kind;
      c["name"] = s.name;
      c["unit"] = s.unit;
      SensorReading& r = sensorReadings[i];
      if (r.hasValue) {
        c["value"] = r.value;
      } else {
        c["value"] = nullptr;
      }
      // "status" = raw per-poll result (unchanged wire format, what the
      // Pi/rig-dashboard has always received). "displayStatus" = the
      // debounced version for this firmware's OWN /live page only —
      // added rather than substituted so the Pi-side payload keeps
      // reporting exactly what actually just happened.
      c["status"] = r.status;
      c["displayStatus"] = r.displayStatus;

      if (s.volumeEnabled) {
        VolumeReading vol;
        computeSensorVolume(s, r, vol);
        JsonObject volObj = c.createNestedObject("volume");
        volObj["value"] = vol.hasValue ? vol.value : (float)0;
        if (!vol.hasValue) volObj["value"] = nullptr;
        volObj["unit"]   = vol.unit;
        volObj["status"] = vol.status;
        // CHANGED 2026-09-17: capacity is no longer a stored config value
        // (s.capacity is gone -- replaced by s.tankLengthM/tankWidthM, see
        // config.h) -- it's now derived geometry, computed once inside
        // computeSensorVolume() alongside the volume itself (same unit
        // conversion, same rounding) and carried back via vol.capacity so
        // this wire field's shape/meaning is unchanged for the Pi side
        // (rig-modules.js's fill-% widget, module-traces.js's axis-max
        // default both still just read `capacity` off this same payload).
        c["capacity"] = vol.capacity;
      }
    }

    if (cfg.modbusBoard.enabled) {
      doc["modbusBoardSlaveId"] = MODBUS_BOARD_SLAVE_ID;
      doc["modbusBoardType"] = modbusDetectedType;
      JsonArray mbArr = doc.createNestedArray("modbusChannels");
      for (int i=0;i<MAX_MODBUS_CHANNELS;i++) {
        if (!cfg.modbusBoard.channels[i].enabled) continue;
        JsonObject c=mbArr.createNestedObject(); c["ch"]=i; c["source"]="modbus";
        c["name"]=cfg.modbusBoard.channels[i].name; c["kind"]=cfg.modbusBoard.channels[i].kind; c["unit"]=cfg.modbusBoard.channels[i].unit;
        c["ma"]=modbusReadings[i].hasValue ? modbusReadings[i].mA : (float)0;
        c["value"]=modbusReadings[i].hasValue ? modbusReadings[i].value : (float)0;
        if (!modbusReadings[i].hasValue) { c["ma"]=nullptr; c["value"]=nullptr; }
        c["status"]=modbusReadings[i].status; c["slaveId"]=MODBUS_BOARD_SLAVE_ID; c["boardType"]=modbusDetectedType;
      }
    }

    // CAN signals reported under their own "canSignals" array — kept
    // separate from "channels" since they're a genuinely different data
    // source (CAN bus, not Modbus/RS485), even though the shape is similar.
    // Unconditional now (CAN always runs), same as canEnabled above.
    {
      JsonArray canArr = doc.createNestedArray("canSignals");
      for (int i = 0; i < MAX_CAN_SIGNALS; i++) {
        CanSignalConfig& sig = cfg.canSignals[i];
        if (!sig.enabled) continue;
        JsonObject c = canArr.createNestedObject();
        c["idx"]  = i;
        c["name"] = sig.name;
        c["kind"] = sig.kind;
        c["unit"] = sig.unit;
        c["canId"] = sig.canId;
        CanSignalReading& r = canReadings[i];
        c["value"] = r.hasValue ? r.value : (float)0;
        if (!r.hasValue) c["value"] = nullptr;
        c["status"] = r.status;
      }
    }

    xSemaphoreGive(stateMutex);
  }

  String out;
  serializeJson(doc, out);
  return out;
}

// =============================================================================
// POST TO PI
// =============================================================================
void postToPi() {
  // BUG FIXED 2026-09-09: this used to call popBufferEntry() — which
  // DESTRUCTIVELY removes the line from disk — BEFORE knowing whether
  // the resend actually succeeded. On failure it took the early
  // `return`, meaning: the buffered entry was already gone from disk,
  // bufferCount was never decremented (so it silently drifted away
  // from what's actually on disk forever), AND this cycle's own live
  // sensor reading was never even attempted/sent, since the function
  // returned before reaching buildPayload() below. During a real
  // multi-cycle Pi outage this meant almost every data point got
  // silently discarded instead of buffered — the "Buffered entries"
  // counter on /system kept climbing while the actual file barely
  // grew. Fixed by peeking (non-destructive) and only removing the
  // entry from disk + decrementing the count once httpPost() actually
  // returns true.
  if (bufferCount > 0 || flushNow) {
    flushNow = false;
    String buffered = peekBufferEntry();
    if (!buffered.isEmpty()) {
      bool ok = httpPost(resolvedPiIp, buffered);
      if (ok) {
        removeBufferHead();
        bufferCount = max(0, bufferCount - 1);
      } else {
        // Address may be stale (logger moved / DHCP change) — let discovery
        // re-resolve instead of silently sitting on a dead IP.
        discoveryInvalidate(resolvedPiIp);
        resolvedPiIp = "";
        return;
      }
    }
  }

  String payload = buildPayload(false);
  bool ok = httpPost(resolvedPiIp, payload);
  lastPostMs = millis();
  lastPostOk = ok;
  lastPostStatus = ok ? 200 : 0;

  if (!ok) {
    appendBufferEntry(payload);
    bufferCount++;
    discoveryInvalidate(resolvedPiIp);
    resolvedPiIp = "";
  }
}

bool httpPost(const String& ip, const String& json) {
  if (ip.isEmpty()) return false;
  String url = "http://" + ip + ":8080/api/rig/module";
  HTTPClient http;
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  if (!cfg.rigToken.isEmpty()) {
    http.addHeader("X-Rig-Token", cfg.rigToken);
  }
  http.setTimeout(5000);
  int code = http.POST(json);
  http.end();
  Serial.printf("[Post] %s → %d\n", url.c_str(), code);
  return (code == 200);
}

// =============================================================================
// BUFFER (LittleFS /buffer.jsonl)
// =============================================================================
int countBufferEntries() {
  if (!fsUsable) return 0;
  File f = LittleFS.open("/buffer.jsonl", "r");
  if (!f) return 0;
  int n = 0;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    if (line.length() > 2) n++;
  }
  f.close();
  return n;
}

void appendBufferEntry(const String& json) {
  if (!fsUsable) return;
  int maxEntries = min(3600, (int)(3 * 3600 / max(1, cfg.pollIntervalS)));
  if (bufferCount >= maxEntries) {
    // BUG FIXED 2026-09-09: this called trimBufferHead() (removes the
    // oldest line from disk) but never decremented bufferCount to
    // match — every trim silently made bufferCount overcount the real
    // number of lines in the file by one, forever (never
    // self-correcting except across a reboot, which recounts from
    // disk via countBufferEntries()). Harmless to the file itself
    // (still correctly capped at maxEntries lines) but made the
    // "Buffered entries" figure on /system meaningless during any
    // outage long enough to fill the buffer.
    removeBufferHead();
    bufferCount = max(0, bufferCount - 1);
  }
  File f = LittleFS.open("/buffer.jsonl", "a");
  if (f) {
    f.println(json);
    f.close();
  }
}

// Returns the OLDEST buffered entry WITHOUT removing it from disk — safe
// to call every poll cycle even if the send that follows fails, unlike
// the old popBufferEntry() which deleted on read regardless of whether
// the resend actually succeeded (see postToPi() comment).
String peekBufferEntry() {
  if (!fsUsable) return "";
  File f = LittleFS.open("/buffer.jsonl", "r");
  if (!f) return "";
  String first = "";
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (!line.isEmpty()) { first = line; break; }
  }
  f.close();
  return first;
}

// Removes just the oldest line from the buffer file, rewriting the rest.
// Only call this once you've confirmed that entry was actually delivered
// (or intentionally discarding it, e.g. trimming a full buffer) — this
// is the one place that destructively shrinks /buffer.jsonl.
void removeBufferHead() {
  if (!fsUsable) return;
  File f = LittleFS.open("/buffer.jsonl", "r");
  if (!f) return;

  String rest = "";
  bool skippedFirst = false;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.isEmpty()) continue;
    if (!skippedFirst) {
      skippedFirst = true;
    } else {
      rest += line + "\n";
    }
  }
  f.close();

  File fw = LittleFS.open("/buffer.jsonl", "w");
  if (fw) {
    fw.print(rest);
    fw.close();
  }
}

// =============================================================================
// OTA SETUP
// =============================================================================
void setupOTA(const String& hostname) {
  ArduinoOTA.setHostname(hostname.c_str());
  ArduinoOTA.onStart([]() { Serial.println("[OTA] Start"); });
  ArduinoOTA.onEnd([]()   { Serial.println("[OTA] End");   });
  ArduinoOTA.onError([](ota_error_t e) {
    Serial.printf("[OTA] Error %u\n", e);
  });
  ArduinoOTA.begin();
  Serial.printf("[OTA] Ready, hostname: %s\n", hostname.c_str());
}