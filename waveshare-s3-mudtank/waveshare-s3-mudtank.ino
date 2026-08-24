// =============================================================================
// waveshare-s3-mudtank.ino — Combined Rig Module Firmware
// Waveshare ESP32-S3-RS485-CAN (isolated, DIN-rail, ESP32-S3)
//
// Merges the two previous rig-module firmware approaches into one image:
//   - A fixed analog-to-Modbus adapter board (Waveshare 8AI (B) or
//     Eletechsup AMIDJ14, auto-detected), 8 (or 6) 4-20mA channels +
//     optional AMIDJ14 digital I/O + RPM via DI pulse counting. Up to
//     MAX_EXTRA_BOARDS more of these on the same bus (see /advanced).
//   - Any number of independent RS485 Modbus sensors (pressure, level,
//     temp, radar...), each its own slave/register/type/scale.
//   - CAN bus (listen-only) with configurable signal extraction.
//   - Full config export/import as JSON over the web UI.
// All RS485 devices share ONE Serial2 bus at one baud; CAN is separate.
//
// POSTs JSON telemetry to Rig Pi Logger (/api/rig/module) — same wire
// format/endpoint as every other rig-module variant.
//
// Libraries required (install via Arduino Library Manager):
//   ArduinoJson (>= 6.x), NTPClient, LittleFS/Preferences/ESPmDNS/
//   ArduinoOTA/HTTPClient/WebServer/WiFi (all built-in to ESP32 core)
//   driver/twai.h (built-in, no library needed)
//
// Arduino IDE board settings:
//   Board: "ESP32S3 Dev Module"
//   USB CDC On Boot: Enabled
//   Flash Size: 16MB, PSRAM: OPI PSRAM
//   Partition Scheme: 16M Flash (3MB APP/9.9MB FATFS)
//   To enter download mode if flashing hangs: hold BOOT, tap RESET,
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
#include "pulse.h"
#include "webui.h"

// =============================================================================
// PIN DEFINITIONS — Waveshare ESP32-S3-RS485-CAN
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

SemaphoreHandle_t stateMutex;     // guards readings/sensorReadings/canReadings
SemaphoreHandle_t modbusBusMutex; // guards Serial2 (poll task vs. web diagnostics/pulse task)

ModuleConfig cfg;

// --- Fixed adapter board state ---------------------------------------------
ChannelReading readings[8];        // latest scaled readings, primary board
uint16_t rawModbus[8] = {0};       // latest raw register values, primary board
DigitalReading dinReadings[4];     // AMIDJ14 only
DigitalReading doutReadings[4];    // AMIDJ14 only
bool modbusOk = false;
BoardProfile boardProfile = BOARD_WAVESHARE_8AI; // detected once at poll-task startup

BoardProfile     extraBoardProfile[MAX_EXTRA_BOARDS];
ChannelReading   extraReadings[MAX_EXTRA_BOARDS][8];
DigitalReading   extraDinReadings[MAX_EXTRA_BOARDS][4];
DigitalReading   extraDoutReadings[MAX_EXTRA_BOARDS][4];
static ChannelConfig _extraBoardDefaultCh; // shared default (4-20mA -> 0-1 linear map)

// --- Independent RS485 sensors + CAN state ----------------------------------
SensorReading    sensorReadings[MAX_SENSORS];
CanSignalReading canReadings[MAX_CAN_SIGNALS];

WebServer webServer(80);

// Pi connectivity
String resolvedPiIp = "";
unsigned long lastPiResolve = 0;
unsigned long lastPostMs = 0;
int lastPostStatus = 0;
bool lastPostOk = false;
int bufferCount = 0;
bool flushNow = false;

// WiFi setup-portal mode
bool apModeActive = false;
String apSSID = "";

// =============================================================================
// SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(500); // let serial settle

  // Two separate logging systems both matter for diagnosing WiFi driver
  // failures — see ensureStaStarted() below for the full backstory.
  esp_log_level_set("wifi", ESP_LOG_VERBOSE);
  esp_log_level_set("wifi_init", ESP_LOG_VERBOSE);
  esp_log_level_set("phy_init", ESP_LOG_VERBOSE);
  esp_log_level_set("phy", ESP_LOG_VERBOSE);
  esp_log_level_set("system_api", ESP_LOG_VERBOSE);
  esp_log_level_set("nvs", ESP_LOG_VERBOSE);
  Serial.setDebugOutput(true);

  // Disable WiFi's own flash-persistent config storage as the very first
  // thing, before ANY WiFi.mode()/begin()/softAP() call anywhere in this
  // sketch — prevents the WL_STOPPED/scan=-2 NVS-corruption wedge.
  WiFi.persistent(false);

  Serial.println("\n\n========================================");
  Serial.println("[BOOT] Rig Module (Mud/Tank Combined) " FW_VERSION);
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
  for (int i = 0; i < MAX_SENSORS; i++) {
    if (cfg.sensors[i].enabled) {
      nSensors++;
      Serial.printf("[BOOT]   sensor slot %d: \"%s\" slaveId=%d fc=%d reg=0x%04X\n",
        i, cfg.sensors[i].name.c_str(), cfg.sensors[i].slaveId,
        cfg.sensors[i].funcCode, cfg.sensors[i].regAddr);
    }
  }
  Serial.printf("[BOOT] Configured independent sensors: %d / %d slots\n", nSensors, MAX_SENSORS);

  stateMutex = xSemaphoreCreateMutex();
  modbusBusMutex = xSemaphoreCreateMutex();

  // Init RS485 / Modbus — shared bus for the fixed board AND independent
  // sensors, one baud for everyone.
  Serial.printf("[BOOT] RS485 pins: RX=%d TX=%d DE=%d baud=%ld\n",
    RS485_RXD, RS485_TXD, RS485_DE, cfg.modbusBaud);
  modbusInit(RS485_RXD, RS485_TXD, RS485_DE, cfg.modbusBaud);

  // CAN only comes up if explicitly enabled on the Config page — listen-
  // only mode (can.h), so an unconfigured/unused CAN bus is never touched
  // at all unless asked for.
  if (cfg.canEnabled) {
    Serial.printf("[BOOT] CAN pins: TX=%d RX=%d bitrate=%ld\n", CAN_TXD, CAN_RXD, cfg.canBitrate);
    canStart(CAN_TXD, CAN_RXD, cfg.canBitrate);
  } else {
    Serial.println("[BOOT] CAN disabled (enable on / to start it)");
  }

  connectWifi();
  if (apModeActive) {
    // no LED on this board (see waveshare-s3/ variant comment history) —
    // nothing to set here.
  }

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
  setupWebRoutes(webServer, cfg, prefs, readings, rawModbus, sensorReadings, canReadings, stateMutex);
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
  xTaskCreatePinnedToCore(pollTask, "poll", 10240, NULL, 1, NULL, 1);

  // DI pulse-counter fast-poll task (pulse.h) — shares modbusBusMutex with
  // pollTask, sleeps on its own when no DI has Pulse Counter Mode enabled.
  Serial.println("[BOOT] Starting DI pulse-counter fast-poll task...");
  xTaskCreatePinnedToCore(pulsePollTask, "pulsepoll", 4096, &cfg, 1, NULL, 1);

  Serial.println("========================================");
  Serial.println("[BOOT] Ready! Open the web UI to configure.");
  Serial.println("========================================\n");
}

// =============================================================================
// LOOP — handles OTA, NTP, CAN polling, Pi discovery, posting
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

  delay(20); // shorter than 100ms — keeps CAN frame draining responsive
}

// =============================================================================
// Wait for the STA interface to actually come up after WiFi.mode(WIFI_STA).
// On ESP32 Arduino core 3.x, rapidly cycling WIFI_OFF -> WIFI_STA can leave
// WiFi.status() stuck reporting WL_STOPPED (254) even though the mode call
// itself returned success. Give the driver a moment and re-issue the mode
// call once if it's still stuck; self-heals via one NVS erase+reinit pass
// if it's still stuck after 3 retries (classic fingerprint of corrupted
// WiFi calibration data in flash after a different firmware ran before).
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
    Serial.println("[WiFi]   Attempting NVS erase + reinit as a self-heal (WiFi calibration");
    Serial.println("[WiFi]   data may be corrupted) — this only happens once per boot...");
    esp_err_t erase_err = nvs_flash_erase();
    esp_err_t init_err = nvs_flash_init();
    Serial.printf("[WiFi]   nvs_flash_erase=0x%x nvs_flash_init=0x%x\n", erase_err, init_err);
    Serial.println("[WiFi]   Restoring saved module config into freshly-erased NVS...");
    prefs.begin("rigmod", false);
    saveConfig(prefs, cfg);
    prefs.end();
    WiFi.mode(WIFI_OFF);
    delay(300);
    return ensureStaStarted(); // one recursive retry pass after the erase
  }

  Serial.println("[WiFi]   Still stuck after NVS erase — proceeding anyway.");
  return false;
}

// =============================================================================
// RIG NETWORK AUTO-DISCOVERY — site routers are always named "rigXXX" with
// a fixed password. No manual setup needed for a standard rig router.
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
// MODBUS POLL TASK (core 1) — polls THREE independent things every cycle,
// all sharing the same RS485 bus/mutex:
//   1) The fixed adapter board's analog channels (+ digital I/O if the
//      detected board has it) + any enabled extra boards.
//   2) Every enabled independent RS485 sensor slot.
// CAN is drained separately in loop() (canPoll()), not here.
// =============================================================================
void pollTask(void* param) {
  Serial.println("[Poll] Task started, waiting 2s before Modbus init...");
  delay(2000); // let WiFi settle

  // --- Boot-time baud check + fixed-board detection -----------------------
  {
    uint16_t probe[1];
    bool bootOk = false;
    if (xSemaphoreTake(modbusBusMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
      bootOk = modbusReadRegs(cfg.modbusSlaveId, 0x04, 0x0000, 1, probe);
      if (!bootOk && !cfg.baudManuallySet) {
        Serial.printf("[Poll] No response at configured baud (%ld) — auto-scanning...\n", cfg.modbusBaud);
        long found = modbusAutoDetectBaud(cfg.modbusSlaveId, (uint32_t)cfg.modbusBaud);
        if (found > 0 && found != cfg.modbusBaud) {
          Serial.printf("[Poll] Adopting auto-detected baud %ld (was %ld)\n", found, cfg.modbusBaud);
          cfg.modbusBaud = found;
          prefs.begin("rigmod", false);
          saveConfig(prefs, cfg);
          prefs.end();
        } else if (found <= 0) {
          Serial.println("[Poll] Boot auto-scan found nothing either — check wiring/board power.");
        }
      }

      // Board detection — Product ID register (0x00F7) identifies the
      // fixed adapter board wired up (Waveshare vs. Eletechsup AMIDJ14).
      // cfg.boardOverride lets this be skipped and forced to a known board.
      if (cfg.boardOverride == "waveshare") {
        boardProfile = BOARD_WAVESHARE_8AI;
        Serial.println("[Poll] Board override: forced to Waveshare 8AI (B) — skipping auto-probe");
      } else if (cfg.boardOverride == "amidj14") {
        boardProfile = BOARD_ELETECHSUP_AMIDJ14;
        Serial.println("[Poll] Board override: forced to Eletechsup AMIDJ14 — skipping auto-probe");
      } else {
        boardProfile = modbusDetectBoard(cfg.modbusSlaveId);
      }
      Serial.printf("[Poll] Board in use: %s (%d channels, raw/%.0f = mA, digitalIO=%s)\n",
        boardProfile.name, boardProfile.numChannels, boardProfile.rawDivisor,
        boardProfile.hasDigitalIO ? "yes" : "no");

      for (int i = 0; i < MAX_EXTRA_BOARDS; i++) {
        ExtraBoardConfig& xb = cfg.extraBoards[i];
        if (xb.enabled && xb.slaveId >= 1 && xb.slaveId <= 247) {
          extraBoardProfile[i] = boardProfileForType(xb.boardType);
          Serial.printf("[Poll] Extra board %d (\"%s\"): slave=%d type=%s (%d channels, digitalIO=%s)\n",
            i, xb.name.c_str(), xb.slaveId, extraBoardProfile[i].name,
            extraBoardProfile[i].numChannels, extraBoardProfile[i].hasDigitalIO ? "yes" : "no");
        }
      }

      xSemaphoreGive(modbusBusMutex);
    }
  }

  // Write mode 3 (4-20mA) to all enabled channels on boot — Waveshare-
  // specific holding-register convention. Skipped for boards that don't
  // implement it (e.g. AMIDJ14) to avoid a pointless bus round-trip and a
  // scary-looking warning on every boot for those boards. BOARD_UNKNOWN
  // shares Waveshare's divisor by design but isn't confirmed Waveshare
  // hardware — compare by name, not divisor.
  if (String(boardProfile.name) == BOARD_WAVESHARE_8AI.name) {
    writeChannelModes();
  } else {
    Serial.printf("[Modbus] Skipping mode-3 write — %s doesn't use Waveshare's mode registers\n", boardProfile.name);
  }
  Serial.println("[Poll] Modbus init complete, entering poll loop");

  int pollCount = 0;
  for (;;) {
    // ─── Fixed adapter board: primary ────────────────────────────────────
    uint16_t raw[8] = {0};
    bool ok = false;
    if (xSemaphoreTake(modbusBusMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
      ok = modbusReadAll(cfg.modbusSlaveId, raw, boardProfile.numChannels);
      xSemaphoreGive(modbusBusMutex);
    }
    pollCount++;

    bool din[4] = {false, false, false, false};
    bool dout[4] = {false, false, false, false};
    bool diOk = false, doOk = false;
    if (boardProfile.hasDigitalIO) {
      if (xSemaphoreTake(modbusBusMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        diOk = modbusReadAllDI(cfg.modbusSlaveId, din);
        doOk = modbusReadAllDO(cfg.modbusSlaveId, dout);
        xSemaphoreGive(modbusBusMutex);
      }
    }

    // ─── Fixed adapter board: extra boards (Advanced) ────────────────────
    uint16_t extraRaw[MAX_EXTRA_BOARDS][8] = {{0}};
    bool     extraOk[MAX_EXTRA_BOARDS]     = {false};
    bool     extraDin[MAX_EXTRA_BOARDS][4]  = {{false}};
    bool     extraDout[MAX_EXTRA_BOARDS][4] = {{false}};
    bool     extraDiOk[MAX_EXTRA_BOARDS]    = {false};
    bool     extraDoOk[MAX_EXTRA_BOARDS]    = {false};
    for (int i = 0; i < MAX_EXTRA_BOARDS; i++) {
      ExtraBoardConfig& xb = cfg.extraBoards[i];
      if (!xb.enabled || xb.slaveId < 1 || xb.slaveId > 247) continue;
      if (xSemaphoreTake(modbusBusMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        extraOk[i] = modbusReadAll((uint8_t)xb.slaveId, extraRaw[i], extraBoardProfile[i].numChannels);
        xSemaphoreGive(modbusBusMutex);
      }
      if (extraBoardProfile[i].hasDigitalIO) {
        if (xSemaphoreTake(modbusBusMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
          extraDiOk[i] = modbusReadAllDI((uint8_t)xb.slaveId, extraDin[i]);
          extraDoOk[i] = modbusReadAllDO((uint8_t)xb.slaveId, extraDout[i]);
          xSemaphoreGive(modbusBusMutex);
        }
      }
      if (!extraOk[i]) {
        Serial.printf("[Poll] Extra board %d (\"%s\") FAILED — Modbus error (slave ID=%d)\n",
          i, xb.name.c_str(), xb.slaveId);
      }
    }

    if (!ok) {
      Serial.printf("[Poll] #%d Fixed board FAILED — Modbus error (slave ID=%d)\n", pollCount, cfg.modbusSlaveId);
    }

    // ─── Independent RS485 sensors ────────────────────────────────────────
    int sensOkCount = 0, sensFailCount = 0;
    for (int i = 0; i < MAX_SENSORS; i++) {
      SensorConfig& s = cfg.sensors[i];
      if (!s.enabled) continue;

      float rawS = 0, valueS = 0;
      int rc;
      if (xSemaphoreTake(modbusBusMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        rc = modbusPollSensor(s, rawS, valueS, false);
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
          r.rawValue = rawS;
          r.value = round(valueS * 100.0f) / 100.0f;
          r.status = "ok";
          r.displayStatus = "ok";
          r.consecutiveTimeouts = 0;
          r.lastOkMs = millis();
          sensOkCount++;
        } else {
          r.valid = false;
          r.errorCount++;
          r.status = (rc == MB_TIMEOUT) ? "timeout" : (rc == MB_CRC_ERROR) ? "crc" : "error";
          sensFailCount++;

          if (rc == MB_TIMEOUT) {
            r.consecutiveTimeouts++;
            bool hasReportedBefore = r.hasValue;
            if (!hasReportedBefore || r.consecutiveTimeouts >= TIMEOUT_DISPLAY_THRESHOLD) {
              r.displayStatus = "timeout";
            }
          } else {
            r.consecutiveTimeouts = 0;
            r.displayStatus = r.status;
          }
        }
        xSemaphoreGive(stateMutex);
      }

      // Small gap between sensors on the shared bus.
      vTaskDelay(pdMS_TO_TICKS(20));
    }

    // ─── Commit fixed-board readings ──────────────────────────────────────
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
      modbusOk = ok;
      if (ok) {
        memcpy(rawModbus, raw, sizeof(raw));
        for (int ch = 0; ch < 8; ch++) {
          if (cfg.ch[ch].enabled && ch < boardProfile.numChannels) {
            scaleChannel(ch, raw[ch], cfg, readings[ch], boardProfile.rawDivisor);
          } else {
            readings[ch].valid = false;
          }
        }
      }

      if (boardProfile.hasDigitalIO) {
        for (int i = 0; i < 4; i++) {
          if (diOk) {
            dinReadings[i].valid  = true;
            dinReadings[i].state  = din[i];
            dinReadings[i].status = "ok";
          } else {
            dinReadings[i].valid  = false;
            dinReadings[i].status = "stale";
          }
          if (doOk) {
            doutReadings[i].valid  = true;
            doutReadings[i].state  = dout[i];
            doutReadings[i].status = "ok";
          } else {
            doutReadings[i].valid  = false;
            doutReadings[i].status = "stale";
          }
        }
      }

      for (int b = 0; b < MAX_EXTRA_BOARDS; b++) {
        ExtraBoardConfig& xb = cfg.extraBoards[b];
        if (!xb.enabled || xb.slaveId < 1 || xb.slaveId > 247) continue;
        if (extraOk[b]) {
          for (int ch = 0; ch < 8; ch++) {
            if (ch < extraBoardProfile[b].numChannels) {
              scaleChannelCfg(extraRaw[b][ch], _extraBoardDefaultCh, extraReadings[b][ch], extraBoardProfile[b].rawDivisor);
            } else {
              extraReadings[b][ch].valid = false;
            }
          }
        } else {
          for (int ch = 0; ch < 8; ch++) extraReadings[b][ch].valid = false;
        }
        if (extraBoardProfile[b].hasDigitalIO) {
          for (int i = 0; i < 4; i++) {
            if (extraDiOk[b]) {
              extraDinReadings[b][i].valid  = true;
              extraDinReadings[b][i].state  = extraDin[b][i];
              extraDinReadings[b][i].status = "ok";
            } else {
              extraDinReadings[b][i].valid  = false;
              extraDinReadings[b][i].status = "stale";
            }
            if (extraDoOk[b]) {
              extraDoutReadings[b][i].valid  = true;
              extraDoutReadings[b][i].state  = extraDout[b][i];
              extraDoutReadings[b][i].status = "ok";
            } else {
              extraDoutReadings[b][i].valid  = false;
              extraDoutReadings[b][i].status = "stale";
            }
          }
        }
      }

      // Debug print — fixed board channels
      if (ok) {
        Serial.printf("[Poll] #%d Fixed board OK\n", pollCount);
        for (int ch = 0; ch < 8; ch++) {
          if (!cfg.ch[ch].enabled) continue;
          ChannelReading& r = readings[ch];
          String label = cfg.ch[ch].name.isEmpty() ? ("Ch" + String(ch + 1)) : cfg.ch[ch].name;
          String kind  = cfg.ch[ch].kind.isEmpty() ? "" : (" [" + cfg.ch[ch].kind + "]");
          if (r.valid && r.hasValue) {
            Serial.printf("  %-20s%-10s raw=%5d  %6.2f mA  = %8.2f %-6s (%s)\n",
              label.c_str(), kind.c_str(), raw[ch], r.mA, r.value,
              cfg.ch[ch].unit.c_str(), r.status.c_str());
          } else {
            Serial.printf("  %-20s%-10s raw=%5d  %6.2f mA  -- (%s)\n",
              label.c_str(), kind.c_str(), raw[ch], r.mA, r.status.c_str());
          }
        }
        for (int ch = 0; ch < 8; ch++) {
          if (!cfg.ch[ch].volumeEnabled) continue;
          VolumeReading volDbg;
          computeChannelVolume(ch, cfg, readings, volDbg);
          String label = "Tank Vol Ch" + String(ch + 1);
          if (volDbg.hasValue) {
            Serial.printf("  %-20s%-10s          = %8.2f %-6s (%s)\n",
              label.c_str(), "[derived]", volDbg.value, volDbg.unit.c_str(), volDbg.status.c_str());
          } else {
            Serial.printf("  %-20s%-10s          -- (%s)\n",
              label.c_str(), "[derived]", volDbg.status.c_str());
          }
        }
      }

      if (boardProfile.hasDigitalIO) {
        Serial.println("  --- Digital I/O ---");
        for (int i = 0; i < 4; i++) {
          if (!cfg.din[i].enabled) continue;
          String label = cfg.din[i].name.isEmpty() ? ("DI" + String(i + 1)) : cfg.din[i].name;
          DigitalReading& r = dinReadings[i];
          if (r.valid) {
            Serial.printf("  %-20s%-10s          state=%-4s (%s)\n",
              label.c_str(), "[input]", r.state ? "ON" : "OFF", r.status.c_str());
          } else {
            Serial.printf("  %-20s%-10s          state=--   (%s)\n",
              label.c_str(), "[input]", r.status.c_str());
          }
        }
        for (int i = 0; i < 4; i++) {
          if (!cfg.dout[i].enabled) continue;
          String label = cfg.dout[i].name.isEmpty() ? ("DO" + String(i + 1)) : cfg.dout[i].name;
          DigitalReading& r = doutReadings[i];
          if (r.valid) {
            Serial.printf("  %-20s%-10s          state=%-4s (%s)\n",
              label.c_str(), "[output]", r.state ? "ON" : "OFF", r.status.c_str());
          } else {
            Serial.printf("  %-20s%-10s          state=--   (%s)\n",
              label.c_str(), "[output]", r.status.c_str());
          }
        }
      }

      xSemaphoreGive(stateMutex);
    }

    // Sensor debug summary — printed every cycle (bench testing over
    // serial), separate from the fixed-board debug block above since it's
    // a different mutex-guarded read (stateMutex already released above).
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
      if (sensOkCount > 0 || sensFailCount > 0) {
        Serial.printf("[Poll] Independent sensors: %d ok, %d failed\n", sensOkCount, sensFailCount);
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
      }
      xSemaphoreGive(stateMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(cfg.pollIntervalS * 1000));
  }
}

// =============================================================================
// Write mode 3 (4-20mA) to all holding registers 0x1000-0x1007
// (Waveshare-specific convention; harmlessly skipped for other boards)
// =============================================================================
void writeChannelModes() {
  Serial.println("[Modbus] Writing mode 3 (4-20mA) to all channels...");
  uint16_t modes[8];
  for (int i = 0; i < 8; i++) modes[i] = 3; // 4-20mA for all
  bool ok = modbusWriteMultiple(cfg.modbusSlaveId, 0x1000, 8, modes);
  if (ok) {
    Serial.println("[Modbus] Channel modes set OK");
  } else {
    Serial.println("[Modbus] WARNING: failed to set channel modes — board may read wrong range");
  }
}

// =============================================================================
// BUILD JSON PAYLOAD — merges the fixed board's "channels"/"digitalInputs"/
// "digitalOutputs"/"extraBoards" shape with the independent sensors, which
// are appended to the SAME "channels" array (continuing the index numbering
// after the fixed board's 8 slots) so a single Pi ingest/dashboard consumer
// sees one unified list of readings regardless of which subsystem produced
// them. CAN signals get their own "canSignals" array, same as the
// waveshare-s3-sensors/ variant.
// =============================================================================
String buildPayload(bool bufferedFlag) {
  // Sized generously: 8 fixed channels + up to MAX_EXTRA_BOARDS*8 extra
  // channels + MAX_SENSORS independent sensors + MAX_CAN_SIGNALS CAN
  // signals, each with several fields + optional nested volume object.
  DynamicJsonDocument doc(20480);

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

  JsonArray chArr = doc.createNestedArray("channels");
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
    // --- Fixed board channels (indices 0-7, or fewer on a smaller board) ---
    for (int ch = 0; ch < 8; ch++) {
      if (!cfg.ch[ch].enabled) continue;
      if (ch >= boardProfile.numChannels) continue;
      JsonObject c = chArr.createNestedObject();
      c["ch"]   = ch;
      c["kind"] = cfg.ch[ch].kind;
      c["name"] = cfg.ch[ch].name;
      c["unit"] = cfg.ch[ch].unit;
      ChannelReading& r = readings[ch];
      if (r.valid && r.mA >= 0) {
        c["ma"] = round(r.mA * 100.0f) / 100.0f;
      } else {
        c["ma"] = nullptr;
      }
      if (r.valid && r.hasValue) {
        c["value"] = r.value;
      } else {
        c["value"] = nullptr;
      }
      c["status"] = r.status;

      if (cfg.ch[ch].volumeEnabled) {
        VolumeReading chVol;
        computeChannelVolume(ch, cfg, readings, chVol);
        JsonObject volObj = c.createNestedObject("volume");
        if (chVol.hasValue) {
          volObj["value"] = chVol.value;
        } else {
          volObj["value"] = nullptr;
        }
        volObj["unit"]   = chVol.unit;
        volObj["status"] = chVol.status;
        c["capacity"] = cfg.ch[ch].capacity;
      }
    }

    // --- Independent sensors (indices continue after the fixed board's
    // 8 slots — MAX_SENSORS slots, "ch" here is 8 + slot index) ---
    for (int i = 0; i < MAX_SENSORS; i++) {
      SensorConfig& s = cfg.sensors[i];
      if (!s.enabled) continue;
      JsonObject c = chArr.createNestedObject();
      c["ch"]   = 8 + i;
      c["kind"] = s.kind;
      c["name"] = s.name;
      c["unit"] = s.unit;
      SensorReading& r = sensorReadings[i];
      if (r.hasValue) {
        c["value"] = r.value;
      } else {
        c["value"] = nullptr;
      }
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

    // Back-compat top-level derived.volume + capacity — mirrors whichever
    // FIXED BOARD channel is the first with volumeEnabled (independent
    // sensors' volumes are only ever in the per-channel array above —
    // this back-compat field predates independent sensors entirely).
    VolumeReading vol;
    computeTankVolume(cfg, readings, vol);
    if (vol.status != "disabled") {
      JsonObject derived = doc.createNestedObject("derived");
      JsonObject volObj = derived.createNestedObject("volume");
      if (vol.hasValue) {
        volObj["value"] = vol.value;
      } else {
        volObj["value"] = nullptr;
      }
      volObj["unit"]   = vol.unit;
      volObj["status"] = vol.status;
      for (int i = 0; i < 8; i++) {
        if (cfg.ch[i].volumeEnabled) { doc["capacity"] = cfg.ch[i].capacity; break; }
      }
    }

    if (boardProfile.hasDigitalIO) {
      JsonArray diArr = doc.createNestedArray("digitalInputs");
      for (int i = 0; i < 4; i++) {
        if (!cfg.din[i].enabled) continue;
        JsonObject d = diArr.createNestedObject();
        d["ch"]     = i;
        d["name"]   = cfg.din[i].name;
        if (dinReadings[i].valid) d["state"] = dinReadings[i].state;
        else                      d["state"] = nullptr;
        d["status"] = dinReadings[i].status;

        if (cfg.din[i].pulseModeEnabled) {
          PulseReading pr;
          pulseRpmCompute(i, cfg, pr);
          JsonObject rpmObj = d.createNestedObject("rpm");
          if (pr.valid) rpmObj["value"] = pr.rpm;
          else          rpmObj["value"] = nullptr;
          rpmObj["status"] = pr.status;
        }
      }
      JsonArray doArr = doc.createNestedArray("digitalOutputs");
      for (int i = 0; i < 4; i++) {
        if (!cfg.dout[i].enabled) continue;
        JsonObject d = doArr.createNestedObject();
        d["ch"]     = i;
        d["name"]   = cfg.dout[i].name;
        if (doutReadings[i].valid) d["state"] = doutReadings[i].state;
        else                       d["state"] = nullptr;
        d["status"] = doutReadings[i].status;
      }
    }

    JsonArray extraArr = doc.createNestedArray("extraBoards");
    for (int b = 0; b < MAX_EXTRA_BOARDS; b++) {
      ExtraBoardConfig& xb = cfg.extraBoards[b];
      if (!xb.enabled || xb.slaveId < 1 || xb.slaveId > 247) continue;
      JsonObject bo = extraArr.createNestedObject();
      bo["slaveId"]   = xb.slaveId;
      bo["boardType"] = extraBoardProfile[b].name;
      bo["name"]      = xb.name.isEmpty() ? ("Board " + String(b + 2)) : xb.name;

      JsonArray bch = bo.createNestedArray("channels");
      for (int ch = 0; ch < extraBoardProfile[b].numChannels; ch++) {
        JsonObject c = bch.createNestedObject();
        c["ch"]   = ch;
        c["name"] = xb.name + " Ch " + String(ch + 1);
        ChannelReading& r = extraReadings[b][ch];
        if (r.valid && r.mA >= 0) c["ma"] = round(r.mA * 100.0f) / 100.0f;
        else                      c["ma"] = nullptr;
        if (r.valid && r.hasValue) c["value"] = r.value;
        else                       c["value"] = nullptr;
        c["status"] = r.status;
      }

      if (extraBoardProfile[b].hasDigitalIO) {
        JsonArray bdi = bo.createNestedArray("digitalInputs");
        for (int i = 0; i < 4; i++) {
          JsonObject d = bdi.createNestedObject();
          d["ch"] = i;
          if (extraDinReadings[b][i].valid) d["state"] = extraDinReadings[b][i].state;
          else                              d["state"] = nullptr;
          d["status"] = extraDinReadings[b][i].status;
        }
        JsonArray bdo = bo.createNestedArray("digitalOutputs");
        for (int i = 0; i < 4; i++) {
          JsonObject d = bdo.createNestedObject();
          d["ch"] = i;
          if (extraDoutReadings[b][i].valid) d["state"] = extraDoutReadings[b][i].state;
          else                               d["state"] = nullptr;
          d["status"] = extraDoutReadings[b][i].status;
        }
      }
    }

    // CAN signals — own top-level array, separate data source from
    // Modbus/RS485 even though the shape is similar.
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
