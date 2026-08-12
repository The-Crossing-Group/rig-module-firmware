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
#include "modbus.h"
#include "can.h"
#include "scaling.h"
#include "webui.h"

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

WebServer webServer(80);

String resolvedPiIp = "";
unsigned long lastPiResolve = 0;
unsigned long lastPostMs = 0;
int lastPostStatus = 0;
bool lastPostOk = false;
int bufferCount = 0;
bool flushNow = false;

bool apModeActive = false;
String apSSID = "";

// =============================================================================
// SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(500);

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
  if (!LittleFS.begin(true)) {
    Serial.println("[BOOT] LittleFS mount failed — formatting");
    LittleFS.format();
    LittleFS.begin(true);
  } else {
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
  for (int i = 0; i < MAX_SENSORS; i++) if (cfg.sensors[i].enabled) nSensors++;
  Serial.printf("[BOOT] Configured sensors: %d / %d slots\n", nSensors, MAX_SENSORS);

  stateMutex = xSemaphoreCreateMutex();
  modbusBusMutex = xSemaphoreCreateMutex();

  Serial.printf("[BOOT] RS485 pins: RX=%d TX=%d DE=%d baud=%ld\n",
    RS485_RXD, RS485_TXD, RS485_DE, cfg.modbusBaud);
  modbusInit(RS485_RXD, RS485_TXD, RS485_DE, cfg.modbusBaud);

  // Auto-detect & enable: scan a modest address range (1-16, keeps boot
  // delay reasonable) at the configured baud, and if nothing's configured
  // yet AND nothing answers, also sweep the other standard baud rates
  // (see modbusAutoDetectAndEnable() in modbus.h) — covers "brand new
  // module, sensor's already wired, but at some other baud" without any
  // manual steps. Auto-fills/enables any responding slave that isn't
  // already configured. No mutex contention concern here — nothing else
  // touches the bus yet this early in setup(), poll task hasn't started.
  Serial.println("[BOOT] Auto-detecting RS485 sensors (addresses 1-16, current baud first)...");
  {
    int found = modbusAutoDetectAndEnable(cfg, 16);
    if (found > 0) {
      Serial.printf("[BOOT] Auto-detect found and enabled %d new sensor(s) at %ld baud\n", found, cfg.modbusBaud);
      prefs.begin("rigmod", false);
      saveConfig(prefs, cfg);
      prefs.end();
    } else {
      Serial.println("[BOOT] Auto-detect found no new sensors at any standard baud");
    }
  }

  // CAN only comes up if explicitly enabled on the Config page — listen-
  // only mode (see can.h), so an unconfigured/unused CAN bus is never
  // touched at all unless you ask for it.
  if (cfg.canEnabled) {
    Serial.printf("[BOOT] CAN pins: TX=%d RX=%d bitrate=%ld\n", CAN_TXD, CAN_RXD, cfg.canBitrate);
    canStart(CAN_TXD, CAN_RXD, cfg.canBitrate);
  } else {
    Serial.println("[BOOT] CAN disabled (enable on / to start it)");
  }

  connectWifi();

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
  if (apModeActive) {
    Serial.printf("[HTTP] Setup AP web server at http://192.168.4.1/ (connect to \"%s\")\n", apSSID.c_str());
  } else if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[HTTP] Web server at http://%s/\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("[HTTP] Web server up but not connected to any network yet");
  }

  bufferCount = countBufferEntries();
  Serial.printf("[BOOT] Buffered entries: %d\n", bufferCount);

  Serial.println("[BOOT] Starting Modbus poll task...");
  xTaskCreatePinnedToCore(pollTask, "poll", 8192, NULL, 1, NULL, 1);

  Serial.println("========================================");
  Serial.println("[BOOT] Ready! Open the web UI to configure sensors.");
  Serial.println("========================================\n");
}

// =============================================================================
// LOOP — handles OTA, NTP, CAN polling, Pi discovery/posting
// =============================================================================
void loop() {
  ArduinoOTA.handle();
  webServer.handleClient();
  ntpClient.update();

  // Drain any pending CAN frames — cheap no-op if CAN isn't enabled.
  canPoll(cfg, canReadings, stateMutex);

  if (!apModeActive) {
    if (resolvedPiIp.isEmpty() || (millis() - lastPiResolve > 300000UL)) {
      resolvePi();
    }
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
// Wait for the STA interface to actually come up after WiFi.mode(WIFI_STA).
// Identical self-heal logic to the other rig-module variants — see their
// comments for the full WL_STOPPED/NVS-corruption backstory.
// =============================================================================
static bool _nvsEraseAttempted = false;
static bool ensureStaStarted() {
  for (int retry = 0; retry < 3; retry++) {
    bool modeOk = WiFi.mode(WIFI_STA);
    Serial.printf("[WiFi]   WiFi.mode(WIFI_STA) returned %s, getMode()=%d, freeHeap=%d\n",
      modeOk ? "true" : "FALSE", (int)WiFi.getMode(), ESP.getFreeHeap());
    unsigned long start = millis();
    while (WiFi.status() == WL_STOPPED && (millis() - start) < 2000) {
      delay(50);
    }
    if (WiFi.status() != WL_STOPPED) return true;
    Serial.printf("[WiFi]   STA still WL_STOPPED after mode(WIFI_STA), retry %d...\n", retry + 1);
    WiFi.mode(WIFI_OFF);
    delay(300);
  }
  Serial.println("[WiFi]   WARNING: STA stuck at WL_STOPPED after 3 retries.");

  if (!_nvsEraseAttempted) {
    _nvsEraseAttempted = true;
    Serial.println("[WiFi]   Attempting NVS erase + reinit as a self-heal...");
    esp_err_t erase_err = nvs_flash_erase();
    esp_err_t init_err = nvs_flash_init();
    Serial.printf("[WiFi]   nvs_flash_erase=0x%x nvs_flash_init=0x%x\n", erase_err, init_err);
    Serial.println("[WiFi]   Restoring saved module config into freshly-erased NVS...");
    prefs.begin("rigmod", false);
    saveConfig(prefs, cfg);
    prefs.end();
    WiFi.mode(WIFI_OFF);
    delay(300);
    return ensureStaStarted();
  }

  Serial.println("[WiFi]   Still stuck after NVS erase — proceeding anyway.");
  return false;
}

// =============================================================================
// RIG NETWORK AUTO-DISCOVERY — identical convention to the other variants.
// =============================================================================
#define RIG_WIFI_PASS "7804991970"

static bool isRigSSID(const String& ssid) {
  if (ssid.length() < 4) return false;
  String lower = ssid;
  lower.toLowerCase();
  if (!lower.startsWith("rig")) return false;
  for (size_t i = 3; i < lower.length(); i++) {
    if (!isDigit(lower[i])) return false;
  }
  return true;
}

static bool tryAutoConnectRigNetwork() {
  Serial.println("[WiFi] ----------------------------------------");
  Serial.println("[WiFi] No saved network — scanning for rigXXX networks...");

  ensureStaStarted();
  WiFi.disconnect(true);
  delay(100);

  int found = WiFi.scanNetworks();
  if (found <= 0) {
    Serial.println("[WiFi] Scan found no networks at all.");
    WiFi.scanDelete();
    return false;
  }

  int candidates[32];
  int nCandidates = 0;
  for (int i = 0; i < found && nCandidates < 32; i++) {
    if (isRigSSID(WiFi.SSID(i)) && WiFi.channel(i) >= 1 && WiFi.channel(i) <= 14) {
      candidates[nCandidates++] = i;
    }
  }
  for (int i = 1; i < nCandidates; i++) {
    int key = candidates[i];
    int j = i - 1;
    while (j >= 0 && WiFi.RSSI(candidates[j]) < WiFi.RSSI(key)) {
      candidates[j+1] = candidates[j];
      j--;
    }
    candidates[j+1] = key;
  }

  if (nCandidates == 0) {
    Serial.printf("[WiFi] Scan found %d network(s), none match \"rigNNN\" pattern.\n", found);
    WiFi.scanDelete();
    return false;
  }

  Serial.printf("[WiFi] Found %d rigXXX candidate(s):\n", nCandidates);
  for (int k = 0; k < nCandidates; k++) {
    int i = candidates[k];
    Serial.printf("[WiFi]   %s  RSSI: %d dBm  Ch: %d\n", WiFi.SSID(i).c_str(), WiFi.RSSI(i), WiFi.channel(i));
  }

  bool connected = false;
  for (int k = 0; k < nCandidates && !connected; k++) {
    int i = candidates[k];
    String ssid = WiFi.SSID(i);
    int channel = WiFi.channel(i);
    uint8_t bssid[6];
    memcpy(bssid, WiFi.BSSID(i), 6);

    Serial.printf("[WiFi] Trying \"%s\" with standard rig password...\n", ssid.c_str());
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(300);
    ensureStaStarted();
    delay(200);
    WiFi.begin(ssid.c_str(), RIG_WIFI_PASS, channel, bssid);

    int tries = 0;
    while (WiFi.status() != WL_CONNECTED && tries < 30) {
      delay(500);
      tries++;
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("[WiFi] Connected to \"%s\"!\n", ssid.c_str());
      Serial.printf("[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());
      cfg.wifiSSID = ssid;
      cfg.wifiPass = RIG_WIFI_PASS;
      saveConfig(prefs, cfg);
      Serial.println("[WiFi] Saved to NVS — future boots will connect directly.");
      connected = true;
    } else {
      Serial.printf("[WiFi] Failed to connect to \"%s\" (status=%d)\n", ssid.c_str(), WiFi.status());
    }
  }

  WiFi.scanDelete();
  Serial.println("[WiFi] ----------------------------------------");
  return connected;
}

void startSetupAP() {
  WiFi.mode(WIFI_AP_STA);
  uint8_t mac[6];
  esp_efuse_mac_get_default(mac);
  char suffix[7];
  snprintf(suffix, sizeof(suffix), "%02X%02X%02X", mac[3], mac[4], mac[5]);
  apSSID = "RigModule-" + String(suffix);

  WiFi.softAP(apSSID.c_str(), "modulesetup");
  apModeActive = true;

  Serial.println("[WiFi] ----------------------------------------");
  Serial.printf("[WiFi] Starting setup AP: \"%s\" / \"modulesetup\"\n", apSSID.c_str());
  Serial.printf("[WiFi] AP IP: %s\n", WiFi.softAPIP().toString().c_str());
  Serial.println("[WiFi] Connect to this network, then open http://192.168.4.1/");
  Serial.println("[WiFi] ----------------------------------------");
}

void connectWifi() {
  if (cfg.wifiSSID.isEmpty()) {
    Serial.println("[WiFi] No SSID saved in NVS.");
    if (tryAutoConnectRigNetwork()) return;
    Serial.println("[WiFi] No rigXXX network found/connectable — going to setup AP.");
    startSetupAP();
    return;
  }

  Serial.println("[WiFi] ----------------------------------------");
  Serial.printf("[WiFi] Target SSID: %s\n", cfg.wifiSSID.c_str());

  ensureStaStarted();
  WiFi.disconnect(true);
  delay(100);

  Serial.println("[WiFi] Scanning for networks...");
  int found = WiFi.scanNetworks();
  int targetChannel = 0;
  uint8_t targetBSSID[6] = {0};
  if (found == 0) {
    Serial.println("[WiFi] Scan found NO networks at all");
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
      Serial.printf("[WiFi] WARNING: \"%s\" not found in scan (or only on 5GHz, unusable on this chip)!\n", cfg.wifiSSID.c_str());
    } else {
      Serial.printf("[WiFi] Target on channel %d, RSSI %d dBm\n", targetChannel, bestRssi);
    }
  }
  WiFi.scanDelete();

  Serial.println("[WiFi] Attempting connection...");
  WiFi.setAutoReconnect(false);
  WiFi.persistent(false);

  const int MAX_ATTEMPTS = 5;
  for (int attempt = 1; attempt <= MAX_ATTEMPTS; attempt++) {
    Serial.printf("[WiFi] Attempt %d/%d — connecting to \"%s\"...\n", attempt, MAX_ATTEMPTS, cfg.wifiSSID.c_str());

    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(500);
    ensureStaStarted();
    delay(200);

    bool useHint = (targetChannel > 0) && (attempt < MAX_ATTEMPTS);
    if (useHint) {
      Serial.printf("[WiFi]   Using BSSID hint, channel %d\n", targetChannel);
      WiFi.begin(cfg.wifiSSID.c_str(), cfg.wifiPass.c_str(), targetChannel, targetBSSID);
    } else {
      if (targetChannel > 0) Serial.println("[WiFi]   Last attempt — dropping BSSID/channel hint");
      WiFi.begin(cfg.wifiSSID.c_str(), cfg.wifiPass.c_str());
    }

    int tries = 0;
    while (WiFi.status() != WL_CONNECTED && tries < 40) {
      delay(500);
      tries++;
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("[WiFi] Connected on attempt %d!\n", attempt);
      Serial.printf("[WiFi] IP      : %s\n", WiFi.localIP().toString().c_str());
      Serial.printf("[WiFi] MAC     : %s\n", WiFi.macAddress().c_str());
      Serial.printf("[WiFi] RSSI    : %d dBm\n", WiFi.RSSI());
      break;
    } else {
      Serial.printf("[WiFi] Attempt %d failed (final status=%d)\n", attempt, WiFi.status());
      if (attempt < MAX_ATTEMPTS) {
        Serial.println("[WiFi] Waiting 3s before retry...");
        delay(3000);
      }
    }
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WiFi] All attempts failed against saved network.");
    Serial.println("[WiFi] Falling back to setup AP so credentials can be corrected.");
    startSetupAP();
  }
  Serial.println("[WiFi] ----------------------------------------");
}

// =============================================================================
// Pi DISCOVERY
// =============================================================================
void resolvePi() {
  lastPiResolve = millis();

  if (!cfg.piHost.isEmpty()) {
    resolvedPiIp = cfg.piHost;
    Serial.printf("[Pi] Using static host: %s\n", resolvedPiIp.c_str());
    return;
  }

  if (!cfg.wifiSSID.isEmpty() && isRigSSID(cfg.wifiSSID)) {
    String rigNum = cfg.wifiSSID.substring(3);
    resolvedPiIp = "192.168." + rigNum + ".10";
    Serial.printf("[Pi] Derived from rig SSID \"%s\": %s\n", cfg.wifiSSID.c_str(), resolvedPiIp.c_str());
    return;
  }

  int n = MDNS.queryService("_rig-logger", "_tcp");
  if (n > 0) {
    resolvedPiIp = MDNS.address(0).toString();
    Serial.printf("[Pi] mDNS found: %s\n", resolvedPiIp.c_str());
    return;
  }

  IPAddress ip;
  if (WiFi.hostByName("rig-logger.local", ip)) {
    resolvedPiIp = ip.toString();
    Serial.printf("[Pi] Hostname fallback: %s\n", resolvedPiIp.c_str());
    return;
  }

  Serial.println("[Pi] Could not resolve Pi — will retry in 5 min");
  resolvedPiIp = "";
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
  unsigned long lastAutoDetectMs = millis(); // boot-time scan already ran in setup()
  for (;;) {
    cycleCount++;
    int okCount = 0, failCount = 0;

    // Periodic background auto-detect: every ~5 min, and only if a free
    // sensor slot actually exists (skip the bus-hogging scan entirely
    // once all slots are full — nothing more it could do anyway). Runs
    // between poll cycles rather than interleaved with per-sensor polls,
    // so a slow/empty scan doesn't stall live sensors that are already
    // reporting fine.
    if (millis() - lastAutoDetectMs > 300000UL) {
      lastAutoDetectMs = millis();
      bool hasFreeSlot = false;
      for (int i = 0; i < MAX_SENSORS; i++) {
        if (!cfg.sensors[i].enabled) { hasFreeSlot = true; break; }
      }
      if (hasFreeSlot) {
        if (xSemaphoreTake(modbusBusMutex, pdMS_TO_TICKS(2000)) == pdTRUE) {
          int found = modbusAutoDetectAndEnable(cfg, 16);
          xSemaphoreGive(modbusBusMutex);
          if (found > 0) {
            Serial.printf("[Poll] Background auto-detect found %d new sensor(s)\n", found);
            prefs.begin("rigmod", false);
            saveConfig(prefs, cfg);
            prefs.end();
          }
        }
      }
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
        xSemaphoreGive(stateMutex);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(cfg.pollIntervalS * 1000));
  }
}

// =============================================================================
// BUILD JSON PAYLOAD
// =============================================================================
String buildPayload(bool bufferedFlag) {
  // Sized generously: up to MAX_SENSORS (16) + MAX_CAN_SIGNALS (16)
  // entries, each with several fields + optional nested volume object.
  DynamicJsonDocument doc(8192);

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
  doc["canEnabled"] = cfg.canEnabled;
  if (cfg.canEnabled) {
    doc["canFrameRate"] = canGetRecentFrameRate();
    doc["canFrameTotal"] = canGetFrameTotal();
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
        c["capacity"] = s.capacity;
      }
    }

    // CAN signals reported under their own "canSignals" array — kept
    // separate from "channels" since they're a genuinely different data
    // source (CAN bus, not Modbus/RS485), even though the shape is similar.
    if (cfg.canEnabled) {
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
  if (bufferCount > 0 || flushNow) {
    flushNow = false;
    String buffered = popBufferEntry();
    if (!buffered.isEmpty()) {
      bool ok = httpPost(resolvedPiIp, buffered);
      if (ok) {
        bufferCount = max(0, bufferCount - 1);
      } else {
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
  int maxEntries = min(3600, (int)(3 * 3600 / max(1, cfg.pollIntervalS)));
  if (bufferCount >= maxEntries) {
    trimBufferHead();
  }
  File f = LittleFS.open("/buffer.jsonl", "a");
  if (f) {
    f.println(json);
    f.close();
  }
}

String popBufferEntry() {
  File f = LittleFS.open("/buffer.jsonl", "r");
  if (!f) return "";

  String first = "";
  String rest  = "";
  bool gotFirst = false;

  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.isEmpty()) continue;
    if (!gotFirst) {
      first = line;
      gotFirst = true;
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
  return first;
}

void trimBufferHead() {
  popBufferEntry();
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