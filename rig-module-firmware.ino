// =============================================================================
// rig-module-firmware.ino — Generic Rig Module Firmware
// LilyGo T-CAN485 / XY-32 CAN+RS485 (ESP32)
// Reads Waveshare Modbus RTU Analog Input 8CH (B) via RS485
// Reports RAW ENGINEERING VALUES per channel — no tank/mud-specific logic.
// Each channel's name/kind/unit/scaling is configured in the web UI.
// POSTs JSON telemetry to Rig Pi Logger
//
// Libraries required (install via Arduino Library Manager):
//   ArduinoJson        (Benoit Blanchon) >= 6.x
//   NTPClient          (Fabrice Weinberg)
//   LittleFS           (built-in ESP32 Arduino core)
//   Preferences        (built-in)
//   ESPmDNS            (built-in)
//   ArduinoOTA         (built-in)
//   HTTPClient         (built-in)
//   WebServer          (built-in)
//   WiFi               (built-in)
//
// NO ESPAsyncWebServer or AsyncTCP needed.
// =============================================================================

#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <HTTPClient.h>
#include <Update.h>          // for HTTP OTA
#include <Preferences.h>
#include <LittleFS.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <NTPClient.h>
#include <time.h>
#include <driver/rmt.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include "config.h"
#include "modbus.h"
#include "scaling.h"
#include "webui.h"

// =============================================================================
// PIN DEFINITIONS — LilyGo T-CAN485 (verified against official LilyGo example)
// https://github.com/Xinyuan-LilyGO/T-CAN485/blob/main/example/Arduino/RS485/config.h
// =============================================================================
#define PIN_5V_EN   16   // 5V booster enable — must be HIGH or RS485 has no power
#define RS485_TXD   22   // Serial2 TX
#define RS485_RXD   21   // Serial2 RX
#define RS485_DE    17   // RS485 DE/RE (driver enable, active HIGH = transmit)
#define RS485_SE    19   // RS485 /SHDN (shutdown pin — must be HIGH to enable chip)
#define WS2812_PIN   4   // onboard WS2812B RGB LED

// =============================================================================
// WS2812 via ESP32 RMT — no external library needed
// =============================================================================
#define RMT_CHANNEL    RMT_CHANNEL_0
#define RMT_CLK_DIV    4                  // 80MHz / 4 = 20MHz → 50ns per tick
// WS2812B timing (in 50ns ticks):
//   T0H=7 (350ns), T0L=16 (800ns)
//   T1H=14 (700ns), T1L=9 (450ns)
//   Reset: >50µs = hold low

static void ws2812Init() {
  rmt_config_t cfg_rmt = {};
  cfg_rmt.rmt_mode      = RMT_MODE_TX;
  cfg_rmt.channel       = RMT_CHANNEL;
  cfg_rmt.gpio_num      = (gpio_num_t)WS2812_PIN;
  cfg_rmt.clk_div       = RMT_CLK_DIV;
  cfg_rmt.mem_block_num = 1;
  cfg_rmt.tx_config.loop_en              = false;
  cfg_rmt.tx_config.carrier_en           = false;
  cfg_rmt.tx_config.idle_output_en       = true;
  cfg_rmt.tx_config.idle_level           = RMT_IDLE_LEVEL_LOW;
  esp_err_t e1 = rmt_config(&cfg_rmt);
  esp_err_t e2 = rmt_driver_install(RMT_CHANNEL, 0, 0);
  if (e1 != ESP_OK || e2 != ESP_OK) {
    Serial.printf("[BOOT] WARNING: RMT init failed - rmt_config=0x%x rmt_driver_install=0x%x\n", e1, e2);
  }
}

// Send one GRB pixel (WS2812 order is G, R, B)
static void ws2812Set(uint8_t r, uint8_t g, uint8_t b) {
  // 24 bits: G7..G0, R7..R0, B7..B0
  uint8_t bytes[3] = { g, r, b };
  rmt_item32_t items[24];
  for (int i = 0; i < 24; i++) {
    int byteIdx = i / 8;
    int bitIdx  = 7 - (i % 8);
    bool bit = (bytes[byteIdx] >> bitIdx) & 1;
    if (bit) {
      items[i].level0    = 1; items[i].duration0 = 14; // T1H 700ns
      items[i].level1    = 0; items[i].duration1 = 9;  // T1L 450ns
    } else {
      items[i].level0    = 1; items[i].duration0 = 7;  // T0H 350ns
      items[i].level1    = 0; items[i].duration1 = 16; // T0L 800ns
    }
  }
  rmt_write_items(RMT_CHANNEL, items, 24, true);
  delayMicroseconds(60); // reset pulse
}

// LED state machine — call ledTick() from loop()
// Use plain #define + uint8_t to avoid Arduino 1.8 ctags enum-in-prototype bug
#define LED_WIFI_CONNECTING 0
#define LED_MODBUS_ERROR    1
#define LED_DATA_OK         2
#define LED_IDLE            3
#define LED_AP_MODE         4
static uint8_t _ledState     = LED_WIFI_CONNECTING;
static unsigned long _ledLastUpdate = 0;
static bool _ledPhase        = false;

void ledSet(uint8_t s) { _ledState = s; _ledPhase = false; }

void ledTick() {
  unsigned long now = millis();
  switch (_ledState) {
    case LED_WIFI_CONNECTING:
      if (now - _ledLastUpdate > 500) {
        _ledPhase = !_ledPhase;
        if (_ledPhase) ws2812Set(0, 0, 40);   // dim blue on
        else           ws2812Set(0, 0, 0);    // off
        _ledLastUpdate = now;
      }
      break;
    case LED_MODBUS_ERROR:
      if (now - _ledLastUpdate > 200) {
        _ledPhase = !_ledPhase;
        if (_ledPhase) ws2812Set(40, 0, 0);   // dim red on
        else           ws2812Set(0, 0, 0);    // off
        _ledLastUpdate = now;
      }
      break;
    case LED_DATA_OK:
      ws2812Set(0, 40, 0);                    // brief green flash
      _ledLastUpdate = now;
      _ledState = LED_IDLE;
      break;
    case LED_IDLE:
      if (now - _ledLastUpdate > 2000) {
        ws2812Set(0, 0, 0);                   // off
        _ledLastUpdate = now;
      }
      break;
    case LED_AP_MODE:
      // Slow amber pulse — "waiting for WiFi setup"
      if (now - _ledLastUpdate > 800) {
        _ledPhase = !_ledPhase;
        if (_ledPhase) ws2812Set(40, 20, 0);  // dim amber on
        else           ws2812Set(0, 0, 0);    // off
        _ledLastUpdate = now;
      }
      break;
  }
}

// =============================================================================
// FIRMWARE VERSION
// =============================================================================
// FW_VERSION defined in config.h

// =============================================================================
// GLOBALS
// =============================================================================
Preferences prefs;
WiFiUDP ntpUDP;
NTPClient ntpClient(ntpUDP, "pool.ntp.org", 0, 3600000); // sync every hour

// Shared state (protected by mutex)
SemaphoreHandle_t stateMutex;
// Guards the RS485/Serial2 bus itself — the poll task (core 1) is
// continuously calling modbusReadAll() in a tight loop, so any other code
// path that also talks to Serial2 directly (e.g. baud auto-detect from the
// web handler, which runs on core 0) MUST hold this first, or the two will
// interleave bytes on the wire and corrupt both requests.
SemaphoreHandle_t modbusBusMutex;
ModuleConfig cfg;
ChannelReading readings[8];  // latest scaled readings
uint16_t rawModbus[8] = {0}; // latest raw register values from 8AI
bool modbusOk = false;
bool modbusInitDone = false; // true after mode-3 write on boot

WebServer webServer(80);

// Pi connectivity
String resolvedPiIp = "";
unsigned long lastPiResolve = 0;
unsigned long lastPostMs = 0;
int lastPostStatus = 0;
bool lastPostOk = false;
int bufferCount = 0;
bool flushNow = false;

// WiFi setup-portal mode (true = we're broadcasting our own AP for config)
bool apModeActive = false;
String apSSID = "";

// =============================================================================
// SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(500); // let serial settle

  // Two separate logging systems are in play here, and we need both live:
  //  1) Arduino-core wrapper code (e.g. WiFiGenericClass::mode() in
  //     WiFiGeneric.cpp) uses log_e()/log_w(), gated at COMPILE TIME by
  //     the Arduino IDE's Tools > Core Debug Level setting. At "None"
  //     (the default) those calls compile down to no-ops — no runtime
  //     call can bring them back. If mode()/scan() failures still show
  //     no Arduino-side error after this build, Core Debug Level needs
  //     to be raised (Verbose/Info) in the IDE and reflashed.
  //  2) The actual precompiled ESP-IDF driver internals (esp_wifi_init,
  //     esp_wifi_start, phy_init, etc) log through IDF's own tag-based
  //     esp_log system, which IS controllable at runtime via
  //     esp_log_level_set() — this is the one that will actually show
  //     *why* esp_wifi_init/esp_wifi_start fails, so we set it here.
  esp_log_level_set("wifi", ESP_LOG_VERBOSE);
  esp_log_level_set("wifi_init", ESP_LOG_VERBOSE);
  esp_log_level_set("phy_init", ESP_LOG_VERBOSE);
  esp_log_level_set("phy", ESP_LOG_VERBOSE);
  esp_log_level_set("system_api", ESP_LOG_VERBOSE);
  esp_log_level_set("nvs", ESP_LOG_VERBOSE);
  Serial.setDebugOutput(true);

  // Disable WiFi's own flash-persistent config storage as the very first
  // thing we do, before ANY WiFi.mode()/begin()/softAP() call anywhere in
  // this sketch (startSetupAP, tryAutoConnectRigNetwork, connectWifi all
  // call WiFi.mode() before this used to run — it was previously set much
  // later, inside connectWifi(), which left every earlier WiFi.mode() call
  // in every boot path free to read/write the WiFi driver's own NVS blob
  // by default). We keep our own SSID/pass in the "rigmod" Preferences
  // namespace already, so we don't need the WiFi driver's separate
  // persistent storage — turning it off everywhere removes a whole class
  // of "WiFi calibration data got corrupted in flash" failure mode, which
  // is our leading theory for the WL_STOPPED/scan=-2 wedge in the field.
  WiFi.persistent(false);

  Serial.println("\n\n========================================");
  Serial.println("[BOOT] Rig Module " FW_VERSION);
  Serial.println("[BOOT] Starting up...");
  Serial.printf( "[BOOT] CPU freq: %d MHz, free heap: %d bytes\n", ESP.getCpuFreqMHz(), ESP.getFreeHeap());
  Serial.println("========================================");

  // Init WS2812 LED via RMT
  ws2812Init();
  ws2812Set(0, 0, 40); // solid dim blue on boot
  ledSet(LED_WIFI_CONNECTING);

  // Init LittleFS
  Serial.println("[BOOT] Mounting LittleFS...");
  if (!LittleFS.begin(true)) {
    Serial.println("[BOOT] LittleFS mount failed — formatting");
    LittleFS.format();
    LittleFS.begin(true);
  } else {
    Serial.printf("[BOOT] LittleFS OK, total=%d used=%d\n", LittleFS.totalBytes(), LittleFS.usedBytes());
  }

  // Load config from NVS
  Serial.println("[BOOT] Loading config from NVS...");
  prefs.begin("rigmod", false);
  loadConfig(prefs, cfg);
  prefs.end();

  // Build module ID
  buildModuleId(cfg);
  Serial.printf("[BOOT] Module ID  : %s\n", cfg.moduleId.c_str());
  Serial.printf("[BOOT] Module name: %s\n", cfg.moduleName.c_str());
  Serial.printf("[BOOT] WiFi SSID: %s\n", cfg.wifiSSID.isEmpty() ? "(none saved)" : cfg.wifiSSID.c_str());
  Serial.printf("[BOOT] Pi host  : %s\n", cfg.piHost.isEmpty() ? "(mDNS auto)" : cfg.piHost.c_str());
  Serial.printf("[BOOT] Poll int : %d s\n", cfg.pollIntervalS);

  // Mutex for shared state
  stateMutex = xSemaphoreCreateMutex();
  modbusBusMutex = xSemaphoreCreateMutex();

  // Enable 5V booster and RS485 chip (required on T-CAN485)
  pinMode(PIN_5V_EN, OUTPUT);
  digitalWrite(PIN_5V_EN, HIGH);   // enable 5V rail for RS485 transceiver
  pinMode(RS485_SE, OUTPUT);
  digitalWrite(RS485_SE, HIGH);    // un-shutdown the MAX13487 RS485 chip
  delay(10);                       // let the chip come up

  // Init RS485 / Modbus
  // Baud is configurable from the webUI (/config, "RS485 Baud Rate") since
  // different analog-to-Modbus boards ship with different factory defaults
  // (Waveshare 8AI (B) = 9600bps, SDSIN SN-3002 clone = 4800bps).
  Serial.printf("[BOOT] RS485 pins: RX=%d TX=%d DE=%d SE=%d 5V_EN=%d baud=%ld\n",
    RS485_RXD, RS485_TXD, RS485_DE, RS485_SE, PIN_5V_EN, cfg.modbusBaud);
  modbusInit(RS485_RXD, RS485_TXD, RS485_DE, cfg.modbusBaud);

  // Connect WiFi (LED slow-blinks blue during this)
  connectWifi();
  if (apModeActive) {
    ledSet(LED_AP_MODE); // slow amber pulse = waiting for setup
  } else if (WiFi.status() == WL_CONNECTED) {
    ws2812Set(0, 40, 0); // solid green = connected
    delay(1000);
    ledSet(LED_IDLE);
  } else {
    ledSet(LED_MODBUS_ERROR); // red blink = no wifi
  }

  // NTP (only if STA connected)
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

  // mDNS + OTA (only useful when connected)
  String hostname = cfg.moduleId;
  hostname.toLowerCase();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[mDNS] Starting as %s.local\n", hostname.c_str());
    MDNS.begin(hostname.c_str());
    setupOTA(hostname);
  }

  // Web server routes
  Serial.println("[HTTP] Setting up web routes...");
  setupWebRoutes(webServer, cfg, prefs, readings, rawModbus, stateMutex);
  webServer.begin();
  if (apModeActive) {
    Serial.printf("[HTTP] Setup AP web server at http://192.168.4.1/wifi (connect to \"%s\")\n", apSSID.c_str());
  } else if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[HTTP] Web server at http://%s/\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("[HTTP] Web server up but not connected to any network yet");
  }

  // Count buffer entries
  bufferCount = countBufferEntries();
  Serial.printf("[BOOT] Buffered entries: %d\n", bufferCount);

  // Start poll task on core 1
  Serial.println("[BOOT] Starting Modbus poll task...");
  xTaskCreatePinnedToCore(pollTask, "poll", 8192, NULL, 1, NULL, 1);

  Serial.println("========================================");
  Serial.println("[BOOT] Ready! Open the web UI to configure.");
  Serial.println("========================================\n");
}

// =============================================================================
// LOOP — handles OTA, NTP, Pi discovery, posting
// =============================================================================
void loop() {
  ArduinoOTA.handle();
  webServer.handleClient();
  ntpClient.update();
  ledTick();

  // Skip Pi discovery/posting entirely while sitting in the setup AP —
  // there's no uplink network to reach a Pi on.
  if (!apModeActive) {
    // Re-resolve Pi every 5 minutes or if blank
    if (resolvedPiIp.isEmpty() || (millis() - lastPiResolve > 300000UL)) {
      resolvePi();
    }

    // Post to Pi at poll interval
    static unsigned long lastPostAttempt = 0;
    unsigned long now = millis();
    if (!resolvedPiIp.isEmpty() && (now - lastPostAttempt >= (unsigned long)cfg.pollIntervalS * 1000UL)) {
      lastPostAttempt = now;
      postToPi();
    }
  }

  delay(100);
}

// =============================================================================
// Wait for the STA interface to actually come up after WiFi.mode(WIFI_STA).
// On ESP32 Arduino core 3.x, rapidly cycling WIFI_OFF -> WIFI_STA (as our
// retry loop does) can leave WiFi.status() stuck reporting WL_STOPPED (254)
// even though the mode call itself returned success — the STA_START event
// that clears WL_STOPPED doesn't always fire before we start polling. If we
// call WiFi.begin() while still stuck at WL_STOPPED, the connection attempt
// never gets anywhere and status stays 254 for the whole attempt, which
// looks identical to a bad password but has nothing to do with it. Give the
// driver a moment and re-issue the mode call once if it's still stuck.
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

  // Self-heal attempt: WiFi.mode() failing outright from the very first
  // boot call (not just racing an event) with scanNetworks() returning -2
  // is the classic fingerprint of a corrupted WiFi calibration/config blob
  // in the NVS partition — very common right after flashing new firmware
  // onto a board that ran something else before. Erase and reinit NVS
  // once, then retry the whole mode sequence before giving up.
  //
  // IMPORTANT: nvs_flash_erase() wipes the ENTIRE default NVS partition,
  // not just the WiFi driver's internal blob — that's the same partition
  // our own "rigmod" Preferences namespace (saved SSID/pass/channel config)
  // lives in. cfg is just an in-memory struct at this point (already
  // loaded in setup() before connectWifi() ran), so it's untouched by the
  // erase — we immediately re-save it via saveConfig() so the module's
  // saved WiFi credentials and channel config survive into the freshly
  // wiped partition instead of being silently lost on next reboot.
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
  Serial.println("[WiFi]   NOTE: if no esp_wifi_init/esp_wifi_start/phy error appeared above,");
  Serial.println("[WiFi]   set Arduino IDE Tools > Core Debug Level to \"Verbose\" and reflash —");
  Serial.println("[WiFi]   the Arduino-side wrapper error is compiled out at the default level.");
  return false;
}

// =============================================================================
// RIG NETWORK AUTO-DISCOVERY
// Site routers are always named "rigXXX" (e.g. rig132) with a fixed
// password. If no WiFi is saved yet, scan for any SSID matching that
// naming convention and try it automatically — no manual setup needed
// for a standard rig router. Falls back to the setup AP only if no
// rigXXX network is found, or it's found but won't connect.
// =============================================================================
#define RIG_WIFI_PASS "7804991970"

// Matches "rig" (any case) followed by one or more digits, e.g. rig132, RIG9
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

// Scans for rigXXX networks, tries the strongest signal first with the
// standard rig password. On success, saves the SSID/pass to NVS so future
// boots skip straight to the normal saved-network path (no rescanning).
// Returns true if connected.
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

  // Collect rigXXX candidates, sorted strongest-RSSI-first.
  // Skip 5GHz entries (channel > 14) — this ESP32 is 2.4GHz-only, so if a
  // rig router broadcasts the same SSID on both bands, trying the 5GHz
  // entry would just waste ~15s failing before falling through.
  int candidates[32];
  int nCandidates = 0;
  for (int i = 0; i < found && nCandidates < 32; i++) {
    if (isRigSSID(WiFi.SSID(i)) && WiFi.channel(i) >= 1 && WiFi.channel(i) <= 14) {
      candidates[nCandidates++] = i;
    }
  }
  // simple insertion sort by RSSI descending
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
    Serial.printf("[WiFi]   %s  RSSI: %d dBm  Ch: %d\n",
      WiFi.SSID(i).c_str(), WiFi.RSSI(i), WiFi.channel(i));
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

      // Save so next boot skips scanning/auto-discovery entirely
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

// =============================================================================
// WIFI SETUP AP — no saved network, no rigXXX network found/reachable.
// Broadcasts "RigModule-XXXXXX" (last 6 MAC hex chars), password
// "modulesetup", serves the /wifi config page at 192.168.4.1.
// =============================================================================
void startSetupAP() {
  // AP_STA (not plain AP) so the /wifi page can still scan for networks
  // while the setup AP is broadcasting — STA stays idle/unconnected,
  // it's just enough for WiFi.scanNetworks() to work.
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
  Serial.println("[WiFi] Connect to this network, then open http://192.168.4.1/wifi");
  Serial.println("[WiFi] ----------------------------------------");
}

// =============================================================================
// WIFI — connects using cfg.wifiSSID / cfg.wifiPass (from NVS, set via the
// web UI, or auto-saved after a successful rigXXX auto-connect). If nothing
// is saved: first tries to auto-discover a "rigNNN" network with the
// standard rig password (tryAutoConnectRigNetwork), and only falls back to
// the setup AP (startSetupAP) if that fails too. No recompile needed to
// move a unit to a different rig or network.
// =============================================================================
void connectWifi() {
  if (cfg.wifiSSID.isEmpty()) {
    Serial.println("[WiFi] No SSID saved in NVS.");
    if (tryAutoConnectRigNetwork()) {
      return; // connected + saved to NVS inside tryAutoConnectRigNetwork()
    }
    Serial.println("[WiFi] No rigXXX network found/connectable — going to setup AP.");
    startSetupAP();
    return;
  }

  Serial.println("[WiFi] ----------------------------------------");
  Serial.printf("[WiFi] Target SSID: %s\n", cfg.wifiSSID.c_str());

  ensureStaStarted();
  WiFi.disconnect(true);
  delay(100);

  // Scan for networks first so we can see what's visible
  Serial.println("[WiFi] Scanning for networks...");
  int found = WiFi.scanNetworks();
  int targetChannel = 0;
  uint8_t targetBSSID[6] = {0};
  if (found == 0) {
    Serial.println("[WiFi] Scan found NO networks at all");
  } else {
    Serial.printf("[WiFi] Scan found %d network(s):\n", found);
    bool targetFound = false;
    int targetAuth = -1;
    int bestRssi = -1000;
    for (int i = 0; i < found; i++) {
      bool isTarget = (WiFi.SSID(i) == cfg.wifiSSID);
      // This ESP32 (T-CAN485 / WROOM-32) is 2.4GHz-only — channels 1-14.
      // If an AP/mesh broadcasts the same SSID on both 2.4GHz and 5GHz,
      // the scan returns both as separate entries. Only ever pin to a
      // 2.4GHz entry, and if there are several (mesh nodes / repeaters),
      // pick the strongest signal — not just whichever appears last.
      bool is24GHz = (WiFi.channel(i) >= 1 && WiFi.channel(i) <= 14);
      if (isTarget && is24GHz && WiFi.RSSI(i) > bestRssi) {
        targetFound = true;
        bestRssi = WiFi.RSSI(i);
        targetChannel = WiFi.channel(i);
        targetAuth = WiFi.encryptionType(i);
        memcpy(targetBSSID, WiFi.BSSID(i), 6);
      }
      Serial.printf("[WiFi]   %2d: %-32s  RSSI: %4d dBm  Ch: %2d  Auth: %d  BSSID: %02X:%02X:%02X:%02X:%02X:%02X  %s\n",
        i+1,
        WiFi.SSID(i).c_str(),
        WiFi.RSSI(i),
        WiFi.channel(i),
        WiFi.encryptionType(i),
        WiFi.BSSID(i)[0], WiFi.BSSID(i)[1], WiFi.BSSID(i)[2],
        WiFi.BSSID(i)[3], WiFi.BSSID(i)[4], WiFi.BSSID(i)[5],
        (isTarget && is24GHz) ? "<-- TARGET (2.4GHz)" : (isTarget ? "<-- target (5GHz, unusable on this chip)" : ""));
    }
    if (!targetFound) {
      Serial.printf("[WiFi] WARNING: \"%s\" not found in scan (or only seen on 5GHz, which this chip can't use)!\n", cfg.wifiSSID.c_str());
    } else {
      Serial.printf("[WiFi] Target on channel %d, RSSI %d dBm, auth type %d (3=WPA2, 4=WPA/WPA2, 8=WPA3)\n",
        targetChannel, bestRssi, targetAuth);
    }
  }
  WiFi.scanDelete();

  Serial.println("[WiFi] ----------------------------------------");
  Serial.println("[WiFi] Attempting connection...");

  WiFi.setAutoReconnect(false); // we handle retries manually
  WiFi.persistent(false); // also set once at the very top of setup() now — harmless to repeat here

  const int MAX_ATTEMPTS = 5;
  for (int attempt = 1; attempt <= MAX_ATTEMPTS; attempt++) {
    Serial.printf("[WiFi] Attempt %d/%d — connecting to \"%s\"...\n", attempt, MAX_ATTEMPTS, cfg.wifiSSID.c_str());

    // Full reset before each attempt. ensureStaStarted() actively confirms
    // the STA netif actually comes back up (leaves WL_STOPPED) before we
    // touch WiFi.begin() — on ESP32 core 3.x, cycling OFF->STA repeatedly
    // like this can otherwise leave status() wedged at WL_STOPPED (254)
    // for the whole attempt, which looks exactly like a connect failure
    // but has nothing to do with SSID/password.
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(500);
    ensureStaStarted();
    delay(200);

    // Connect — pass BSSID+channel if we found the target in the scan.
    // Safety net: on the last attempt, drop the hint entirely and let the
    // driver do its own scan/join. Covers cases where our hint was stale
    // (AP roamed/rebooted between our scan and now) or otherwise bad —
    // without this, a wrong hint fails identically on every retry.
    bool useHint = (targetChannel > 0) && (attempt < MAX_ATTEMPTS);
    if (useHint) {
      Serial.printf("[WiFi]   Using BSSID hint, channel %d\n", targetChannel);
      WiFi.begin(cfg.wifiSSID.c_str(), cfg.wifiPass.c_str(), targetChannel, targetBSSID);
    } else {
      if (targetChannel > 0) Serial.println("[WiFi]   Last attempt — dropping BSSID/channel hint, letting driver scan+join");
      WiFi.begin(cfg.wifiSSID.c_str(), cfg.wifiPass.c_str());
    }

    int tries = 0;
    while (WiFi.status() != WL_CONNECTED && tries < 40) {
      delay(500);
      Serial.printf("[WiFi]   ... %ds (status=%d)\n", tries/2+1, WiFi.status());
      tries++;
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("[WiFi] Connected on attempt %d!\n", attempt);
      Serial.printf("[WiFi] IP      : %s\n", WiFi.localIP().toString().c_str());
      Serial.printf("[WiFi] MAC     : %s\n", WiFi.macAddress().c_str());
      Serial.printf("[WiFi] RSSI    : %d dBm\n", WiFi.RSSI());
      Serial.printf("[WiFi] Gateway : %s\n", WiFi.gatewayIP().toString().c_str());
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
    Serial.println("[WiFi] Status codes: 1=SSID not found, 3=wrong password, 4=connect failed, 6=disconnected");
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

  // Try mDNS _rig-logger._tcp.local
  int n = MDNS.queryService("_rig-logger", "_tcp");
  if (n > 0) {
    resolvedPiIp = MDNS.address(0).toString();
    Serial.printf("[Pi] mDNS found: %s\n", resolvedPiIp.c_str());
    return;
  }

  // Fallback: rig-logger.local
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
// MODBUS POLL TASK (core 1)
// =============================================================================
void pollTask(void* param) {
  Serial.println("[Poll] Task started, waiting 2s before Modbus init...");
  delay(2000); // let WiFi settle

  // Plug-and-play baud check: if the configured/default baud gets no
  // response at all, don't just sit there silently failing forever —
  // auto-scan every standard rate once at boot and adopt whichever one
  // actually gets an answer from the board. Covers swapping in a
  // different analog-to-Modbus board (e.g. Waveshare <-> SDSIN) without
  // anyone needing to know or set its factory-default baud rate first.
  {
    uint16_t probe[1];
    bool bootOk = false;
    if (xSemaphoreTake(modbusBusMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
      bootOk = modbusReadInputRegs(cfg.modbusSlaveId, 0x0000, 1, probe);
      if (!bootOk) {
        Serial.printf("[Poll] No response at configured baud (%ld) — auto-scanning...\n", cfg.modbusBaud);
        long found = modbusAutoDetectBaud(cfg.modbusSlaveId, (uint32_t)cfg.modbusBaud);
        if (found > 0 && found != cfg.modbusBaud) {
          Serial.printf("[Poll] Adopting auto-detected baud %ld (was %ld)\n", found, cfg.modbusBaud);
          cfg.modbusBaud = found;
          prefs.begin("rigmod", false);
          saveConfig(prefs, cfg);
          prefs.end();
          // modbusAutoDetectBaud() already left Serial2 running at the
          // winning rate — no reboot needed, we can just carry on.
        } else if (found <= 0) {
          Serial.println("[Poll] Boot auto-scan found nothing either — check wiring/board power.");
        }
      }
      xSemaphoreGive(modbusBusMutex);
    }
  }

  // Write mode 3 (4-20mA) to all enabled channels on boot
  writeChannelModes();
  modbusInitDone = true;
  Serial.println("[Poll] Modbus init complete, entering poll loop");

  int pollCount = 0;
  for (;;) {
    // Read all 8 input registers. Hold modbusBusMutex for the actual
    // wire transaction so a concurrent baud auto-detect request (web
    // handler, core 0) can't interleave bytes with us on Serial2.
    uint16_t raw[8] = {0};
    bool ok = false;
    if (xSemaphoreTake(modbusBusMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
      ok = modbusReadAll(cfg.modbusSlaveId, raw);
      xSemaphoreGive(modbusBusMutex);
    }
    pollCount++;

    if (ok) {
      ledSet(LED_DATA_OK);
      // Print raw values — first 10 polls always, then every 30
      if (pollCount <= 10 || pollCount % 30 == 0) {
        Serial.printf("[Poll] #%d OK — raw: ", pollCount);
        for (int i = 0; i < 8; i++) Serial.printf("%5d ", raw[i]);
        Serial.printf("(%.2f %.2f %.2f mA)\n",
          raw[0]/1000.0f, raw[1]/1000.0f, raw[2]/1000.0f);
      }
    } else {
      ledSet(LED_MODBUS_ERROR);
      Serial.printf("[Poll] #%d FAILED — Modbus error (slave ID=%d)\n", pollCount, cfg.modbusSlaveId);
    }

    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
      modbusOk = ok;
      if (ok) {
        memcpy(rawModbus, raw, sizeof(raw));
        for (int ch = 0; ch < 8; ch++) {
          if (cfg.ch[ch].enabled) {
            scaleChannel(ch, raw[ch], cfg, readings[ch]);
          } else {
            readings[ch].valid = false;
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
// BUILD JSON PAYLOAD
// =============================================================================
String buildPayload(bool bufferedFlag) {
  DynamicJsonDocument doc(4096);

  doc["moduleId"] = cfg.moduleId;   // primary key the Pi uses
  doc["type"]     = cfg.moduleType.isEmpty() ? "generic" : cfg.moduleType;  // configurable on /config
  doc["name"]     = cfg.moduleName.isEmpty() ? cfg.moduleId : cfg.moduleName;  // rig-modules.html card title
  doc["moduleName"] = cfg.moduleName;
  // Report our IP so rig-modules.html can deep-link to the config page
  if (WiFi.status() == WL_CONNECTED) {
    doc["ip"] = WiFi.localIP().toString();
  }
  doc["fw"]       = FW_VERSION;
  doc["uptimeS"]  = (unsigned long)(millis() / 1000UL);
  doc["rssi"]     = WiFi.RSSI();
  doc["buffered"] = bufferedFlag;

  // Timestamp
  if (ntpClient.isTimeSet() && ntpClient.getEpochTime() > 1000000000UL) {
    time_t t = ntpClient.getEpochTime();
    struct tm* tm_info = gmtime(&t);
    char buf[32];
    strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", tm_info);
    doc["ts"] = buf;
  } else {
    doc["ts"] = "1970-01-01T00:00:00Z";
  }

  // Channels
  JsonArray chArr = doc.createNestedArray("channels");
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
    for (int ch = 0; ch < 8; ch++) {
      if (!cfg.ch[ch].enabled) continue;
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
    }

    // Tank volume (optional derived value) — only emitted once some channel
    // has "Compute Tank Volume" checked on /channels (see computeTankVolume).
    // Computed from that channel's SCALED reading currently in `readings[]`,
    // so it reflects the same cal/fault state the channel itself is showing.
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
      // Top-level capacity — rig-modules.js reads this for the fill-bar %,
      // same shape as the original tank spec.
      for (int i = 0; i < 8; i++) {
        if (cfg.ch[i].volumeEnabled) { doc["capacity"] = cfg.ch[i].capacity; break; }
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
  // First try to flush one buffered entry if we have any
  if (bufferCount > 0 || flushNow) {
    flushNow = false;
    String buffered = popBufferEntry();
    if (!buffered.isEmpty()) {
      bool ok = httpPost(resolvedPiIp, buffered);
      if (ok) {
        bufferCount = max(0, bufferCount - 1);
      } else {
        // Put it back by re-pushing (we already popped it; simplest: just note failure)
        // The entry is lost — acceptable for brief outages; we'll buffer new ones
        resolvedPiIp = ""; // force re-resolve
        return;
      }
    }
  }

  // Now post live data
  String payload = buildPayload(false);
  bool ok = httpPost(resolvedPiIp, payload);
  lastPostMs = millis();
  lastPostOk = ok;
  lastPostStatus = ok ? 200 : 0;

  if (!ok) {
    // Buffer it
    appendBufferEntry(payload);
    bufferCount++;
    resolvedPiIp = ""; // force re-resolve next cycle
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

  // If over cap, remove oldest line
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

  // Rewrite without first line
  File fw = LittleFS.open("/buffer.jsonl", "w");
  if (fw) {
    fw.print(rest);
    fw.close();
  }
  return first;
}

void trimBufferHead() {
  popBufferEntry(); // just discard the oldest
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
