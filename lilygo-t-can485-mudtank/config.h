// =============================================================================
// config.h — Rig Module (Mud/Tank Combined) configuration structures + NVS
//
// This variant merges the two previous rig-module approaches into one
// firmware image:
//   1) A FIXED analog-to-Modbus adapter board (Waveshare 8AI (B) or
//      Eletechsup AMIDJ14, auto-detected via Product ID register 0x00F7),
//      reporting 8 (or 6) 4-20mA channels + optional AMIDJ14 digital I/O —
//      same as the waveshare-s3/ variant. Up to MAX_EXTRA_BOARDS more of
//      these can share the same RS485 bus at their own slave addresses
//      (the "/advanced" power-user feature).
//   2) Any number of INDEPENDENT RS485 Modbus sensors (pressure, level,
//      temp, radar, whatever) each with their own slave ID/register/data
//      type/scale — same as the waveshare-s3-sensors/ variant. Also
//      brings up CAN (listen-only) with a configurable list of signal
//      extractors, same as that variant.
// All of the above coexist on the SAME shared RS485 bus (one baud for
// everyone) plus one CAN bus if enabled.
//
// Also adds: full config export/import as a single JSON blob over the web
// UI (see configToJson()/jsonFromConfig() below) — download a backup of
// every setting on this device, or restore/clone it onto another unit.
// =============================================================================
#pragma once
#include <Arduino.h>

#define FW_VERSION "rig-module-mudtank-1.0.0"

#include <WiFi.h>
#include <Preferences.h>
#include <esp_efuse.h>
#include <esp_mac.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <ArduinoJson.h>

// =============================================================================
// FIXED ADAPTER BOARD — per-channel config (Waveshare 8AI / Eletechsup
// AMIDJ14 / any future board added to modbus.h's BoardProfile table).
// Reports RAW ENGINEERING VALUES ONLY per channel, same convention as the
// original waveshare-s3/ variant.
// =============================================================================
struct ChannelConfig {
  bool   enabled  = true;   // plug-and-play: every channel reports by default
  String name     = "";
  String kind     = "";     // free text: "level","pressure","temp","flow","rpm",...
  String unit     = "";
  float  maMin    = 4.0f;
  float  maMax    = 20.0f;
  float  engMin   = 0.0f;
  float  engMax   = 1.0f;
  int    zeroRaw  = -1;     // -1 = not calibrated
  int    maxRaw   = -1;

  // --- Tank volume (optional derived calc) ---------------------------------
  bool   volumeEnabled = false;
  float  capacity      = 0.0f;
  String capacityUnit  = "m3";   // "m3" or "gal"
  float  volZeroLevel  = 0.0f;
  float  volMaxLevel   = 1.0f;
};

// Digital I/O config — only meaningful on boards with hasDigitalIO=true
// (currently the Eletechsup AMIDJ14: 4 DI + 4 DO).
struct DigitalChannelConfig {
  bool   enabled          = true;
  String name             = "";
  bool   pulseModeEnabled = false; // DI-only: ON/OFF transitions -> RPM (no GPIO, Modbus-polled)
  int    pulsesPerRev     = 1;
  float  timeoutS         = 3.0f;
  int    pulseTimeoutMs   = 60;    // per-read timeout (ms) for the fast-poll loop's FC02 request
};

struct PulseReading {
  bool   valid  = false;
  float  rpm    = 0.0f;
  String status = "stale"; // "ok" | "stopped" | "stale"
};

// =============================================================================
// EXTRA FIXED BOARDS (Advanced / hidden) — up to MAX_EXTRA_BOARDS additional
// analog-to-Modbus boards on the SAME RS485 bus, each at its own slave
// address. No per-channel calibration/tank-volume (generic names + standard
// 4-20mA linear map only) — see /advanced page.
// =============================================================================
#define MAX_EXTRA_BOARDS 3

struct ExtraBoardConfig {
  bool   enabled   = false;
  int    slaveId   = 0;          // 0 = not set; must be unique on the bus
  String boardType = "amidj14";  // "amidj14" or "waveshare" — explicit, no "auto"
  String name      = "";
};

// =============================================================================
// INDEPENDENT RS485 SENSORS — same generic Modbus-RTU-master approach as
// waveshare-s3-sensors/. Each is its own slave on the shared bus, fully
// independent of the fixed adapter board above. "Read this register (or
// pair) from this slave, interpret it as this data type, then scale/offset
// it into an engineering value."
// =============================================================================
#define MAX_SENSORS 16

enum ModbusDataType {
  MB_UINT16 = 0,
  MB_INT16  = 1,
  MB_UINT32 = 2,
  MB_INT32  = 3,
  MB_FLOAT32 = 4,
};

enum ModbusWordOrder {
  MB_WORD_HIGH_FIRST = 0,  // register[0] = high word, register[1] = low word (most common)
  MB_WORD_LOW_FIRST  = 1,  // register[0] = low word,  register[1] = high word
};

struct SensorConfig {
  bool   enabled     = false;   // slots start empty; check to activate
  String name        = "";      // e.g. "Standpipe Pressure"
  String kind        = "";      // free text: pressure, temp, flow, level...
  String unit        = "";      // e.g. psi, degC, gpm
  uint8_t slaveId    = 1;       // Modbus slave address, 1-247
  uint8_t funcCode   = 3;       // 3 = Read Holding Registers, 4 = Read Input Registers
                                 // Default is 3, not 4: most real-world sensors (SM7779
                                 // radar included) only answer FC03 and stay silent on FC04.
  uint16_t regAddr   = 0;       // starting register address
  uint8_t dataType   = MB_UINT16;
  uint8_t wordOrder  = MB_WORD_HIGH_FIRST; // only matters for 32-bit types
  // Some real sensors (confirmed: SM7779 radar level) ignore the requested
  // register address entirely and always reply with the SAME fixed
  // multi-register block starting from their own register 0. respRegOffset
  // is "how many registers into that fixed reply block the value I
  // actually want starts at" — e.g. 0 = distance, 1 = level, 2 = status
  // for SM7779. Defaults to 0 (take the front of the reply).
  uint8_t respRegOffset = 0;
  float  scale       = 1.0f;    // engineering value = raw * scale + offset
  float  offset      = 0.0f;
  int    decimals    = 2;       // rounding for display/report

  // --- Tank volume (optional derived calc) ---------------------------------
  bool   volumeEnabled = false;
  float  capacity      = 0.0f;
  String capacityUnit  = "m3";   // "m3" or "gal"
  float  volZeroLevel  = 0.0f;
  float  volMaxLevel   = 1.0f;
};

// How many consecutive raw timeouts before a sensor that HAS reported
// successfully before is allowed to show "timeout" on the /sensors and
// /live pages — some sensors (slow measurement cycle) only have fresh
// data ready on a fraction of polls by design. The raw `status` field
// always shows every raw timeout regardless of this debounce.
#define TIMEOUT_DISPLAY_THRESHOLD 6

struct SensorReading {
  bool   valid    = false;   // false = comms failure (timeout/CRC/no response)
  bool   hasValue = false;   // true once a real value has been decoded
  float  rawValue = 0.0f;    // decoded raw value BEFORE scale/offset (for diagnostics)
  float  value    = 0.0f;    // final engineering value (raw*scale+offset)
  String status   = "stale"; // RAW per-poll result: "ok" | "timeout" | "crc" | "stale" | "disabled"
  // Debounced version of `status` for the /sensors and /live pages — see
  // TIMEOUT_DISPLAY_THRESHOLD above. CRC/other errors are NOT debounced.
  String displayStatus       = "stale";
  unsigned long consecutiveTimeouts = 0;
  unsigned long lastPollMs   = 0;
  unsigned long lastOkMs     = 0;
  unsigned long pollCount    = 0;
  unsigned long errorCount   = 0;
};

// =============================================================================
// CAN — same as waveshare-s3-sensors/. One independently-configured signal
// per byte range extracted from frames matching a given CAN ID.
// =============================================================================
#define MAX_CAN_SIGNALS 16

struct CanSignalConfig {
  bool     enabled    = false;
  String   name       = "";
  String   kind       = "";
  String   unit       = "";
  uint32_t canId      = 0;      // 11-bit or 29-bit CAN identifier to match
  bool     extended   = false;  // true = 29-bit extended ID, false = 11-bit standard
  uint8_t  byteOffset = 0;      // starting byte within the 8-byte data payload
  uint8_t  byteLen    = 2;      // 1, 2, or 4 bytes
  bool     bigEndian  = true;   // true = most CAN/J1939 traffic; false = little-endian
  bool     signedVal  = false;
  float    scale      = 1.0f;
  float    offset     = 0.0f;
  int      decimals   = 2;
};

struct CanSignalReading {
  bool   hasValue = false;
  float  rawValue = 0.0f;
  float  value    = 0.0f;
  String status   = "stale"; // "ok" | "stale" (no matching frame seen yet)
  unsigned long lastSeenMs = 0;
};

// =============================================================================
// FULL MODULE CONFIG — union of both prior variants' config, plus a shared
// baudManuallySet lock (originally a waveshare-s3-sensors concept, now
// applies bus-wide since fixed-board + independent-sensor auto-detect both
// touch the same shared baud).
// =============================================================================
struct ModuleConfig {
  String moduleId       = "";    // built from MAC: MODULE-ABC123
  String moduleName     = "";
  String moduleType     = "generic";  // free text — sent as payload "type"
  String description    = "";

  // --- Shared RS485 bus ------------------------------------------------
  long   modbusBaud       = 9600;
  // True once the user has explicitly set the baud (on /config or via any
  // "Auto-Detect Baud" button). While true, background auto-detect only
  // probes the CURRENT baud looking for new sensors/boards — it never
  // hunts through other baud rates and silently overwrites this setting.
  // See 2026-08-10 incident (radar+pressure address collision -> garbled
  // bus -> manual baud fix kept reverting) for why this flag exists.
  bool   baudManuallySet  = false;

  // --- Fixed adapter board (primary) ------------------------------------
  int    modbusSlaveId  = 1;
  // Board type — normally left "auto" (probes Product ID register 0x00F7,
  // modbus.h modbusDetectBoard()). Override to "waveshare" or "amidj14" if
  // the auto-probe isn't identifying the connected board correctly.
  String boardOverride = "auto";
  ChannelConfig ch[8];
  DigitalChannelConfig din[4];
  DigitalChannelConfig dout[4];
  // Advanced / hidden — see ExtraBoardConfig above.
  ExtraBoardConfig extraBoards[MAX_EXTRA_BOARDS];

  // --- Independent RS485 sensors + CAN ----------------------------------
  SensorConfig    sensors[MAX_SENSORS];
  bool   canEnabled     = false; // CAN controller only starts if this is on
  long   canBitrate     = 250000; // 250k = most common (J1939/drill CAN)
  CanSignalConfig canSignals[MAX_CAN_SIGNALS];

  // --- Common ------------------------------------------------------------
  int    pollIntervalS  = 7;     // how often to POST to the Pi / poll-task cycle gap
  String piHost         = "";
  String rigToken       = "7804991970"; // shared default across every rig's Pi logger
  String wifiSSID       = "";
  String wifiPass       = "";
};

// Per-channel live reading (fixed board)
struct ChannelReading {
  bool   valid    = false;
  bool   hasValue = false;
  float  mA       = 0.0f;
  float  value    = 0.0f;
  String status   = "stale";
};

// Live digital I/O state — separate from ChannelReading since these are
// simple booleans, not scaled engineering values.
struct DigitalReading {
  bool   valid = false;   // false until the first successful poll
  bool   state = false;   // current on/off
  String status = "stale";
};

// =============================================================================
// NVS partition stats for the "rigmod" namespace's underlying storage —
// used to catch a real, silent failure mode: Preferences::putXxx() does
// NOT throw or block on a full partition, it just returns 0 (failure)
// while everything else carries on as if nothing happened. With 8 fixed
// channels + 8 DI/DO + 3 extra boards + 16 sensor slots x ~16 keys each +
// 16 CAN slots x ~13 keys each, this namespace is large — if the default
// NVS partition fills up, NEW keys silently fail to write while EXISTING
// keys keep updating fine. Exposed on /system so this is visible without
// a serial cable.
// =============================================================================
struct NvsStats {
  size_t usedEntries = 0;
  size_t freeEntries = 0;
  size_t totalEntries = 0;
  bool   ok = false;
};
NvsStats getNvsStats() {
  NvsStats s;
  nvs_stats_t nvsStats;
  if (nvs_get_stats(NULL, &nvsStats) == ESP_OK) {
    s.usedEntries  = nvsStats.used_entries;
    s.freeEntries  = nvsStats.free_entries;
    s.totalEntries = nvsStats.total_entries;
    s.ok = true;
  }
  return s;
}

// Build MODULE-ABC123 from MAC (always, no manual unit-number scheme —
// every module's identity is intrinsic to its hardware).
// Call this AFTER WiFi.mode() so the MAC is valid.
void buildModuleId(ModuleConfig& cfg) {
  uint8_t mac[6];
  esp_efuse_mac_get_default(mac);  // read from eFuse, always valid regardless of WiFi state
  char buf[20];
  snprintf(buf, sizeof(buf), "MODULE-%02X%02X%02X", mac[3], mac[4], mac[5]);
  cfg.moduleId = String(buf);
}

// Load all config from NVS
void loadConfig(Preferences& p, ModuleConfig& c) {
  c.moduleName    = p.getString("modName", "");
  c.moduleType    = p.getString("modType", "generic");
  c.description   = p.getString("desc", "");
  c.modbusBaud      = p.getLong("mbBaud", 9600);
  c.baudManuallySet = p.getBool("mbBaudSet", false);
  c.modbusSlaveId = p.getInt("mbSlave", 1);
  c.boardOverride = p.getString("boardOvr", "auto");
  c.pollIntervalS = p.getInt("pollInt", 7);
  c.piHost        = p.getString("piHost", "");
  c.rigToken      = p.getString("rigToken", "7804991970");
  // Self-heal: p.getString()'s default only applies when the NVS key is
  // completely absent — if it EXISTS but was saved as an empty string,
  // getString() correctly returns "" instead of the default, and
  // X-Rig-Token then goes out blank on every request, silently failing
  // auth against the Pi. Re-apply the shared default here too so any
  // unit that already has a blank token in NVS heals itself on the very
  // next boot with no manual re-entry needed.
  if (c.rigToken.isEmpty()) c.rigToken = "7804991970";
  c.wifiSSID      = p.getString("wifiSSID", "");
  c.wifiPass      = p.getString("wifiPass", "");
  c.canEnabled    = p.getBool("canEn", false);
  c.canBitrate    = p.getLong("canBit", 250000);

  for (int i = 0; i < 8; i++) {
    String pre = "ch" + String(i);
    c.ch[i].enabled = p.getBool((pre + "en").c_str(), true);
    c.ch[i].name    = p.getString((pre + "nm").c_str(), "Ch " + String(i+1));
    c.ch[i].kind    = p.getString((pre + "kd").c_str(), "");
    c.ch[i].unit    = p.getString((pre + "ut").c_str(), "");
    c.ch[i].maMin   = p.getFloat((pre + "maLo").c_str(), 4.0f);
    c.ch[i].maMax   = p.getFloat((pre + "maHi").c_str(), 20.0f);
    c.ch[i].engMin  = p.getFloat((pre + "eLo").c_str(), 0.0f);
    c.ch[i].engMax  = p.getFloat((pre + "eHi").c_str(), 1.0f);
    c.ch[i].zeroRaw = p.getInt((pre + "zRaw").c_str(), -1);
    c.ch[i].maxRaw  = p.getInt((pre + "mRaw").c_str(), -1);

    c.ch[i].volumeEnabled = p.getBool((pre + "volEn").c_str(), false);
    c.ch[i].capacity      = p.getFloat((pre + "cap").c_str(), 0.0f);
    c.ch[i].capacityUnit  = p.getString((pre + "capUt").c_str(), "m3");
    c.ch[i].volZeroLevel  = p.getFloat((pre + "vZLvl").c_str(), 0.0f);
    c.ch[i].volMaxLevel   = p.getFloat((pre + "vMLvl").c_str(), 1.0f);
  }

  for (int i = 0; i < 4; i++) {
    String preIn  = "di" + String(i);
    String preOut = "do" + String(i);
    c.din[i].enabled          = p.getBool((preIn + "en").c_str(), true);
    c.din[i].name             = p.getString((preIn + "nm").c_str(), "DI " + String(i+1));
    c.din[i].pulseModeEnabled = p.getBool((preIn + "pmEn").c_str(), false);
    c.din[i].pulsesPerRev     = p.getInt((preIn + "ppr").c_str(), 1);
    c.din[i].timeoutS         = p.getFloat((preIn + "pto").c_str(), 3.0f);
    c.din[i].pulseTimeoutMs   = p.getInt((preIn + "pms").c_str(), 60);
    c.dout[i].enabled = p.getBool((preOut + "en").c_str(), true);
    c.dout[i].name    = p.getString((preOut + "nm").c_str(), "DO " + String(i+1));
  }

  for (int i = 0; i < MAX_EXTRA_BOARDS; i++) {
    String pre = "xb" + String(i);
    c.extraBoards[i].enabled   = p.getBool((pre + "en").c_str(), false);
    c.extraBoards[i].slaveId   = p.getInt((pre + "sid").c_str(), 0);
    c.extraBoards[i].boardType = p.getString((pre + "bt").c_str(), "amidj14");
    c.extraBoards[i].name      = p.getString((pre + "nm").c_str(), "Board " + String(i+2));
  }

  for (int i = 0; i < MAX_SENSORS; i++) {
    String pre = "s" + String(i) + "_";
    c.sensors[i].enabled     = p.getBool((pre + "en").c_str(), false);
    c.sensors[i].name        = p.getString((pre + "nm").c_str(), "");
    c.sensors[i].kind        = p.getString((pre + "kd").c_str(), "");
    c.sensors[i].unit        = p.getString((pre + "ut").c_str(), "");
    c.sensors[i].slaveId     = (uint8_t)p.getInt((pre + "sid").c_str(), 1);
    // One-time NVS migration: NVS may hold an old fc=4 saved by a
    // pre-fix auto-detect (see modbus.h — some sensors like SM7779 reply
    // FC03 with valid CRC data even when FC04 was requested, so old scans
    // could record foundFc=4 for a sensor that only really answers FC03).
    // New struct default only applies to never-written NVS keys — a slot
    // that already has fc=4 saved needs an explicit rewrite, not just a
    // new default.
    int savedFc = p.getInt((pre + "fc").c_str(), 3);
    c.sensors[i].funcCode    = (uint8_t)(savedFc == 4 ? 3 : savedFc);
    c.sensors[i].regAddr     = (uint16_t)p.getInt((pre + "reg").c_str(), 0);
    c.sensors[i].dataType    = (uint8_t)p.getInt((pre + "dt").c_str(), MB_UINT16);
    c.sensors[i].wordOrder   = (uint8_t)p.getInt((pre + "wo").c_str(), MB_WORD_HIGH_FIRST);
    c.sensors[i].respRegOffset = (uint8_t)p.getInt((pre + "ro").c_str(), 0);
    c.sensors[i].scale       = p.getFloat((pre + "sc").c_str(), 1.0f);
    c.sensors[i].offset      = p.getFloat((pre + "of").c_str(), 0.0f);
    c.sensors[i].decimals    = p.getInt((pre + "dec").c_str(), 2);
    c.sensors[i].volumeEnabled = p.getBool((pre + "vE").c_str(), false);
    c.sensors[i].capacity      = p.getFloat((pre + "cap").c_str(), 0.0f);
    c.sensors[i].capacityUnit  = p.getString((pre + "cu").c_str(), "m3");
    c.sensors[i].volZeroLevel  = p.getFloat((pre + "vz").c_str(), 0.0f);
    c.sensors[i].volMaxLevel   = p.getFloat((pre + "vm").c_str(), 1.0f);
  }

  for (int i = 0; i < MAX_CAN_SIGNALS; i++) {
    String pre = "c" + String(i) + "_";
    c.canSignals[i].enabled    = p.getBool((pre + "en").c_str(), false);
    c.canSignals[i].name       = p.getString((pre + "nm").c_str(), "");
    c.canSignals[i].kind       = p.getString((pre + "kd").c_str(), "");
    c.canSignals[i].unit       = p.getString((pre + "ut").c_str(), "");
    c.canSignals[i].canId      = (uint32_t)p.getLong((pre + "id").c_str(), 0);
    c.canSignals[i].extended   = p.getBool((pre + "ext").c_str(), false);
    c.canSignals[i].byteOffset = (uint8_t)p.getInt((pre + "bo").c_str(), 0);
    c.canSignals[i].byteLen    = (uint8_t)p.getInt((pre + "bl").c_str(), 2);
    c.canSignals[i].bigEndian  = p.getBool((pre + "be").c_str(), true);
    c.canSignals[i].signedVal  = p.getBool((pre + "sv").c_str(), false);
    c.canSignals[i].scale      = p.getFloat((pre + "sc").c_str(), 1.0f);
    c.canSignals[i].offset     = p.getFloat((pre + "of").c_str(), 0.0f);
    c.canSignals[i].decimals   = p.getInt((pre + "dec").c_str(), 2);
  }
}

// Save all config to NVS. Checks the two baud-related keys' actual write
// result and logs loudly if either silently failed (see NvsStats comment
// above) — every other key is best-effort like before, but these two are
// exactly what's been reported stuck in the past, so they get verified.
void saveConfig(Preferences& p, ModuleConfig& c) {
  p.begin("rigmod", false);
  p.putString("modName", c.moduleName);
  p.putString("modType", c.moduleType);
  p.putString("desc", c.description);
  size_t wroteBaud = p.putLong("mbBaud", c.modbusBaud);
  size_t wroteBaudSet = p.putBool("mbBaudSet", c.baudManuallySet);
  if (wroteBaud == 0 || wroteBaudSet == 0) {
    Serial.printf("[Config] WARNING: NVS write FAILED for mbBaud (ret=%u) and/or mbBaudSet (ret=%u) "
      "— partition may be full. baud=%ld manuallySet=%d\n",
      (unsigned)wroteBaud, (unsigned)wroteBaudSet, c.modbusBaud, (int)c.baudManuallySet);
    NvsStats st = getNvsStats();
    if (st.ok) {
      Serial.printf("[Config] NVS entries: used=%u free=%u total=%u\n",
        (unsigned)st.usedEntries, (unsigned)st.freeEntries, (unsigned)st.totalEntries);
    }
  }
  p.putInt("mbSlave", c.modbusSlaveId);
  p.putString("boardOvr", c.boardOverride);
  p.putInt("pollInt", c.pollIntervalS);
  p.putString("piHost", c.piHost);
  p.putString("rigToken", c.rigToken);
  p.putString("wifiSSID", c.wifiSSID);
  p.putString("wifiPass", c.wifiPass);
  p.putBool("canEn", c.canEnabled);
  p.putLong("canBit", c.canBitrate);

  for (int i = 0; i < 8; i++) {
    String pre = "ch" + String(i);
    p.putBool((pre + "en").c_str(), c.ch[i].enabled);
    p.putString((pre + "nm").c_str(), c.ch[i].name);
    p.putString((pre + "kd").c_str(), c.ch[i].kind);
    p.putString((pre + "ut").c_str(), c.ch[i].unit);
    p.putFloat((pre + "maLo").c_str(), c.ch[i].maMin);
    p.putFloat((pre + "maHi").c_str(), c.ch[i].maMax);
    p.putFloat((pre + "eLo").c_str(), c.ch[i].engMin);
    p.putFloat((pre + "eHi").c_str(), c.ch[i].engMax);
    p.putInt((pre + "zRaw").c_str(), c.ch[i].zeroRaw);
    p.putInt((pre + "mRaw").c_str(), c.ch[i].maxRaw);

    p.putBool((pre + "volEn").c_str(), c.ch[i].volumeEnabled);
    p.putFloat((pre + "cap").c_str(), c.ch[i].capacity);
    p.putString((pre + "capUt").c_str(), c.ch[i].capacityUnit);
    p.putFloat((pre + "vZLvl").c_str(), c.ch[i].volZeroLevel);
    p.putFloat((pre + "vMLvl").c_str(), c.ch[i].volMaxLevel);
  }

  for (int i = 0; i < 4; i++) {
    String preIn  = "di" + String(i);
    String preOut = "do" + String(i);
    p.putBool((preIn + "en").c_str(), c.din[i].enabled);
    p.putString((preIn + "nm").c_str(), c.din[i].name);
    p.putBool((preIn + "pmEn").c_str(), c.din[i].pulseModeEnabled);
    p.putInt((preIn + "ppr").c_str(), c.din[i].pulsesPerRev);
    p.putFloat((preIn + "pto").c_str(), c.din[i].timeoutS);
    p.putInt((preIn + "pms").c_str(), c.din[i].pulseTimeoutMs);
    p.putBool((preOut + "en").c_str(), c.dout[i].enabled);
    p.putString((preOut + "nm").c_str(), c.dout[i].name);
  }

  for (int i = 0; i < MAX_EXTRA_BOARDS; i++) {
    String pre = "xb" + String(i);
    p.putBool((pre + "en").c_str(), c.extraBoards[i].enabled);
    p.putInt((pre + "sid").c_str(), c.extraBoards[i].slaveId);
    p.putString((pre + "bt").c_str(), c.extraBoards[i].boardType);
    p.putString((pre + "nm").c_str(), c.extraBoards[i].name);
  }

  for (int i = 0; i < MAX_SENSORS; i++) {
    String pre = "s" + String(i) + "_";
    p.putBool((pre + "en").c_str(), c.sensors[i].enabled);
    p.putString((pre + "nm").c_str(), c.sensors[i].name);
    p.putString((pre + "kd").c_str(), c.sensors[i].kind);
    p.putString((pre + "ut").c_str(), c.sensors[i].unit);
    p.putInt((pre + "sid").c_str(), c.sensors[i].slaveId);
    p.putInt((pre + "fc").c_str(), c.sensors[i].funcCode);
    p.putInt((pre + "reg").c_str(), c.sensors[i].regAddr);
    p.putInt((pre + "dt").c_str(), c.sensors[i].dataType);
    p.putInt((pre + "wo").c_str(), c.sensors[i].wordOrder);
    p.putInt((pre + "ro").c_str(), c.sensors[i].respRegOffset);
    p.putFloat((pre + "sc").c_str(), c.sensors[i].scale);
    p.putFloat((pre + "of").c_str(), c.sensors[i].offset);
    p.putInt((pre + "dec").c_str(), c.sensors[i].decimals);
    p.putBool((pre + "vE").c_str(), c.sensors[i].volumeEnabled);
    p.putFloat((pre + "cap").c_str(), c.sensors[i].capacity);
    p.putString((pre + "cu").c_str(), c.sensors[i].capacityUnit);
    p.putFloat((pre + "vz").c_str(), c.sensors[i].volZeroLevel);
    p.putFloat((pre + "vm").c_str(), c.sensors[i].volMaxLevel);
  }

  for (int i = 0; i < MAX_CAN_SIGNALS; i++) {
    String pre = "c" + String(i) + "_";
    p.putBool((pre + "en").c_str(), c.canSignals[i].enabled);
    p.putString((pre + "nm").c_str(), c.canSignals[i].name);
    p.putString((pre + "kd").c_str(), c.canSignals[i].kind);
    p.putString((pre + "ut").c_str(), c.canSignals[i].unit);
    p.putLong((pre + "id").c_str(), c.canSignals[i].canId);
    p.putBool((pre + "ext").c_str(), c.canSignals[i].extended);
    p.putInt((pre + "bo").c_str(), c.canSignals[i].byteOffset);
    p.putInt((pre + "bl").c_str(), c.canSignals[i].byteLen);
    p.putBool((pre + "be").c_str(), c.canSignals[i].bigEndian);
    p.putBool((pre + "sv").c_str(), c.canSignals[i].signedVal);
    p.putFloat((pre + "sc").c_str(), c.canSignals[i].scale);
    p.putFloat((pre + "of").c_str(), c.canSignals[i].offset);
    p.putInt((pre + "dec").c_str(), c.canSignals[i].decimals);
  }
  p.end();
}

// =============================================================================
// CONFIG EXPORT / IMPORT (Sarah's request, 2026-08-24) — full config as a
// single JSON blob over the web UI. Export downloads everything in this
// struct (fixed board + digital I/O + extra boards + independent sensors +
// CAN signals + module/WiFi/Pi settings) as one file; Import reads that
// same shape back in, overwrites the in-RAM config, saves to NVS, and
// reboots so every subsystem (Serial2 baud, CAN driver, board detection)
// picks up the restored settings cleanly — same "changed settings need a
// clean reboot" convention already used everywhere else in this firmware.
//
// Deliberately NOT included: moduleId (derived from this device's own MAC,
// re-derived fresh on next boot regardless of what's in the file — a
// cloned config imported onto a DIFFERENT physical module should not carry
// over the original module's identity). wifiPass IS included (so a full
// clone-to-a-new-unit workflow doesn't require re-typing it), but note this
// makes the exported file sensitive — same care as writing down any WiFi
// password on paper.
// =============================================================================

// Serializes the full ModuleConfig into a JsonDocument the caller provides
// (sized by the caller — see webui.h's export handler for the size used).
// Kept as a plain function (not returning a String directly) so the caller
// can add extra top-level metadata (e.g. "exportedFw"/"exportedAt") before
// serializing to the actual HTTP response.
void configToJson(ModuleConfig& c, JsonDocument& doc) {
  doc["moduleName"] = c.moduleName;
  doc["moduleType"] = c.moduleType;
  doc["description"] = c.description;

  doc["modbusBaud"] = c.modbusBaud;
  doc["baudManuallySet"] = c.baudManuallySet;
  doc["modbusSlaveId"] = c.modbusSlaveId;
  doc["boardOverride"] = c.boardOverride;

  doc["pollIntervalS"] = c.pollIntervalS;
  doc["piHost"] = c.piHost;
  doc["rigToken"] = c.rigToken;
  doc["wifiSSID"] = c.wifiSSID;
  doc["wifiPass"] = c.wifiPass;

  doc["canEnabled"] = c.canEnabled;
  doc["canBitrate"] = c.canBitrate;

  JsonArray chArr = doc.createNestedArray("channels");
  for (int i = 0; i < 8; i++) {
    JsonObject o = chArr.createNestedObject();
    ChannelConfig& ch = c.ch[i];
    o["enabled"] = ch.enabled;
    o["name"] = ch.name;
    o["kind"] = ch.kind;
    o["unit"] = ch.unit;
    o["maMin"] = ch.maMin;
    o["maMax"] = ch.maMax;
    o["engMin"] = ch.engMin;
    o["engMax"] = ch.engMax;
    o["zeroRaw"] = ch.zeroRaw;
    o["maxRaw"] = ch.maxRaw;
    o["volumeEnabled"] = ch.volumeEnabled;
    o["capacity"] = ch.capacity;
    o["capacityUnit"] = ch.capacityUnit;
    o["volZeroLevel"] = ch.volZeroLevel;
    o["volMaxLevel"] = ch.volMaxLevel;
  }

  JsonArray dinArr = doc.createNestedArray("din");
  for (int i = 0; i < 4; i++) {
    JsonObject o = dinArr.createNestedObject();
    DigitalChannelConfig& d = c.din[i];
    o["enabled"] = d.enabled;
    o["name"] = d.name;
    o["pulseModeEnabled"] = d.pulseModeEnabled;
    o["pulsesPerRev"] = d.pulsesPerRev;
    o["timeoutS"] = d.timeoutS;
    o["pulseTimeoutMs"] = d.pulseTimeoutMs;
  }
  JsonArray doutArr = doc.createNestedArray("dout");
  for (int i = 0; i < 4; i++) {
    JsonObject o = doutArr.createNestedObject();
    DigitalChannelConfig& d = c.dout[i];
    o["enabled"] = d.enabled;
    o["name"] = d.name;
  }

  JsonArray xbArr = doc.createNestedArray("extraBoards");
  for (int i = 0; i < MAX_EXTRA_BOARDS; i++) {
    JsonObject o = xbArr.createNestedObject();
    ExtraBoardConfig& xb = c.extraBoards[i];
    o["enabled"] = xb.enabled;
    o["slaveId"] = xb.slaveId;
    o["boardType"] = xb.boardType;
    o["name"] = xb.name;
  }

  JsonArray sArr = doc.createNestedArray("sensors");
  for (int i = 0; i < MAX_SENSORS; i++) {
    JsonObject o = sArr.createNestedObject();
    SensorConfig& s = c.sensors[i];
    o["enabled"] = s.enabled;
    o["name"] = s.name;
    o["kind"] = s.kind;
    o["unit"] = s.unit;
    o["slaveId"] = s.slaveId;
    o["funcCode"] = s.funcCode;
    o["regAddr"] = s.regAddr;
    o["dataType"] = s.dataType;
    o["wordOrder"] = s.wordOrder;
    o["respRegOffset"] = s.respRegOffset;
    o["scale"] = s.scale;
    o["offset"] = s.offset;
    o["decimals"] = s.decimals;
    o["volumeEnabled"] = s.volumeEnabled;
    o["capacity"] = s.capacity;
    o["capacityUnit"] = s.capacityUnit;
    o["volZeroLevel"] = s.volZeroLevel;
    o["volMaxLevel"] = s.volMaxLevel;
  }

  JsonArray cArr = doc.createNestedArray("canSignals");
  for (int i = 0; i < MAX_CAN_SIGNALS; i++) {
    JsonObject o = cArr.createNestedObject();
    CanSignalConfig& sg = c.canSignals[i];
    o["enabled"] = sg.enabled;
    o["name"] = sg.name;
    o["kind"] = sg.kind;
    o["unit"] = sg.unit;
    o["canId"] = sg.canId;
    o["extended"] = sg.extended;
    o["byteOffset"] = sg.byteOffset;
    o["byteLen"] = sg.byteLen;
    o["bigEndian"] = sg.bigEndian;
    o["signedVal"] = sg.signedVal;
    o["scale"] = sg.scale;
    o["offset"] = sg.offset;
    o["decimals"] = sg.decimals;
  }
}

// Reverse of configToJson() — reads a previously-exported (or hand-edited)
// JSON document back into a ModuleConfig. Every field uses the CURRENT
// value in `c` as its fallback if missing from the JSON, so a partial/
// hand-trimmed file (e.g. someone deletes the sensors array to reset just
// that section) doesn't wipe out fields it didn't mention — only fields
// actually present in the JSON are changed. Returns false only if the
// document doesn't look like a config export at all (completely empty or
// not an object); otherwise always returns true, since a genuinely partial
// file is a valid (if unusual) use case, not an error.
bool configFromJson(JsonDocument& doc, ModuleConfig& c) {
  if (doc.isNull() || !doc.is<JsonObject>()) return false;

  if (!doc["moduleName"].isNull())     c.moduleName    = doc["moduleName"].as<String>();
  if (!doc["moduleType"].isNull())     c.moduleType    = doc["moduleType"].as<String>();
  if (!doc["description"].isNull())    c.description   = doc["description"].as<String>();

  if (!doc["modbusBaud"].isNull())       c.modbusBaud      = doc["modbusBaud"].as<long>();
  if (!doc["baudManuallySet"].isNull())  c.baudManuallySet = doc["baudManuallySet"].as<bool>();
  if (!doc["modbusSlaveId"].isNull())    c.modbusSlaveId   = doc["modbusSlaveId"].as<int>();
  if (!doc["boardOverride"].isNull())    c.boardOverride   = doc["boardOverride"].as<String>();

  if (!doc["pollIntervalS"].isNull()) c.pollIntervalS = constrain(doc["pollIntervalS"].as<int>(), 1, 30);
  if (!doc["piHost"].isNull())        c.piHost        = doc["piHost"].as<String>();
  if (!doc["rigToken"].isNull()) {
    String rt = doc["rigToken"].as<String>();
    c.rigToken = rt.isEmpty() ? "7804991970" : rt;
  }
  if (!doc["wifiSSID"].isNull()) c.wifiSSID = doc["wifiSSID"].as<String>();
  if (!doc["wifiPass"].isNull()) c.wifiPass = doc["wifiPass"].as<String>();

  if (!doc["canEnabled"].isNull()) c.canEnabled = doc["canEnabled"].as<bool>();
  if (!doc["canBitrate"].isNull()) c.canBitrate = doc["canBitrate"].as<long>();

  if (doc["channels"].is<JsonArray>()) {
    JsonArray arr = doc["channels"].as<JsonArray>();
    int i = 0;
    for (JsonObject o : arr) {
      if (i >= 8) break;
      ChannelConfig& ch = c.ch[i];
      if (!o["enabled"].isNull())       ch.enabled       = o["enabled"].as<bool>();
      if (!o["name"].isNull())          ch.name          = o["name"].as<String>();
      if (!o["kind"].isNull())          ch.kind          = o["kind"].as<String>();
      if (!o["unit"].isNull())          ch.unit          = o["unit"].as<String>();
      if (!o["maMin"].isNull())         ch.maMin         = o["maMin"].as<float>();
      if (!o["maMax"].isNull())         ch.maMax         = o["maMax"].as<float>();
      if (!o["engMin"].isNull())        ch.engMin        = o["engMin"].as<float>();
      if (!o["engMax"].isNull())        ch.engMax        = o["engMax"].as<float>();
      if (!o["zeroRaw"].isNull())       ch.zeroRaw       = o["zeroRaw"].as<int>();
      if (!o["maxRaw"].isNull())        ch.maxRaw        = o["maxRaw"].as<int>();
      if (!o["volumeEnabled"].isNull()) ch.volumeEnabled = o["volumeEnabled"].as<bool>();
      if (!o["capacity"].isNull())      ch.capacity      = o["capacity"].as<float>();
      if (!o["capacityUnit"].isNull())  ch.capacityUnit  = o["capacityUnit"].as<String>();
      if (!o["volZeroLevel"].isNull())  ch.volZeroLevel  = o["volZeroLevel"].as<float>();
      if (!o["volMaxLevel"].isNull())   ch.volMaxLevel   = o["volMaxLevel"].as<float>();
      i++;
    }
  }

  if (doc["din"].is<JsonArray>()) {
    JsonArray arr = doc["din"].as<JsonArray>();
    int i = 0;
    for (JsonObject o : arr) {
      if (i >= 4) break;
      DigitalChannelConfig& d = c.din[i];
      if (!o["enabled"].isNull())          d.enabled          = o["enabled"].as<bool>();
      if (!o["name"].isNull())             d.name             = o["name"].as<String>();
      if (!o["pulseModeEnabled"].isNull()) d.pulseModeEnabled = o["pulseModeEnabled"].as<bool>();
      if (!o["pulsesPerRev"].isNull())     d.pulsesPerRev     = max(1, o["pulsesPerRev"].as<int>());
      if (!o["timeoutS"].isNull())         d.timeoutS         = max(0.1f, o["timeoutS"].as<float>());
      if (!o["pulseTimeoutMs"].isNull())   d.pulseTimeoutMs   = constrain(o["pulseTimeoutMs"].as<int>(), 5, 300);
      i++;
    }
  }
  if (doc["dout"].is<JsonArray>()) {
    JsonArray arr = doc["dout"].as<JsonArray>();
    int i = 0;
    for (JsonObject o : arr) {
      if (i >= 4) break;
      DigitalChannelConfig& d = c.dout[i];
      if (!o["enabled"].isNull()) d.enabled = o["enabled"].as<bool>();
      if (!o["name"].isNull())    d.name    = o["name"].as<String>();
      i++;
    }
  }

  if (doc["extraBoards"].is<JsonArray>()) {
    JsonArray arr = doc["extraBoards"].as<JsonArray>();
    int i = 0;
    for (JsonObject o : arr) {
      if (i >= MAX_EXTRA_BOARDS) break;
      ExtraBoardConfig& xb = c.extraBoards[i];
      if (!o["enabled"].isNull())   xb.enabled   = o["enabled"].as<bool>();
      if (!o["slaveId"].isNull())   xb.slaveId   = o["slaveId"].as<int>();
      if (!o["boardType"].isNull()) xb.boardType = o["boardType"].as<String>();
      if (!o["name"].isNull())      xb.name      = o["name"].as<String>();
      i++;
    }
  }

  if (doc["sensors"].is<JsonArray>()) {
    JsonArray arr = doc["sensors"].as<JsonArray>();
    int i = 0;
    for (JsonObject o : arr) {
      if (i >= MAX_SENSORS) break;
      SensorConfig& s = c.sensors[i];
      if (!o["enabled"].isNull())       s.enabled       = o["enabled"].as<bool>();
      if (!o["name"].isNull())          s.name          = o["name"].as<String>();
      if (!o["kind"].isNull())          s.kind          = o["kind"].as<String>();
      if (!o["unit"].isNull())          s.unit          = o["unit"].as<String>();
      if (!o["slaveId"].isNull())       s.slaveId       = (uint8_t)constrain(o["slaveId"].as<int>(), 1, 247);
      if (!o["funcCode"].isNull())      s.funcCode      = (uint8_t)o["funcCode"].as<int>();
      if (!o["regAddr"].isNull())       s.regAddr       = (uint16_t)o["regAddr"].as<int>();
      if (!o["dataType"].isNull())      s.dataType      = (uint8_t)o["dataType"].as<int>();
      if (!o["wordOrder"].isNull())     s.wordOrder     = (uint8_t)o["wordOrder"].as<int>();
      if (!o["respRegOffset"].isNull()) s.respRegOffset = (uint8_t)constrain(o["respRegOffset"].as<int>(), 0, 15);
      if (!o["scale"].isNull())         s.scale         = o["scale"].as<float>();
      if (!o["offset"].isNull())        s.offset        = o["offset"].as<float>();
      if (!o["decimals"].isNull())      s.decimals      = o["decimals"].as<int>();
      if (!o["volumeEnabled"].isNull()) s.volumeEnabled = o["volumeEnabled"].as<bool>();
      if (!o["capacity"].isNull())      s.capacity      = o["capacity"].as<float>();
      if (!o["capacityUnit"].isNull())  s.capacityUnit  = o["capacityUnit"].as<String>();
      if (!o["volZeroLevel"].isNull())  s.volZeroLevel  = o["volZeroLevel"].as<float>();
      if (!o["volMaxLevel"].isNull())   s.volMaxLevel   = o["volMaxLevel"].as<float>();
      i++;
    }
  }

  if (doc["canSignals"].is<JsonArray>()) {
    JsonArray arr = doc["canSignals"].as<JsonArray>();
    int i = 0;
    for (JsonObject o : arr) {
      if (i >= MAX_CAN_SIGNALS) break;
      CanSignalConfig& sg = c.canSignals[i];
      if (!o["enabled"].isNull())    sg.enabled    = o["enabled"].as<bool>();
      if (!o["name"].isNull())       sg.name       = o["name"].as<String>();
      if (!o["kind"].isNull())       sg.kind       = o["kind"].as<String>();
      if (!o["unit"].isNull())       sg.unit       = o["unit"].as<String>();
      if (!o["canId"].isNull())      sg.canId      = o["canId"].as<uint32_t>();
      if (!o["extended"].isNull())   sg.extended   = o["extended"].as<bool>();
      if (!o["byteOffset"].isNull()) sg.byteOffset = (uint8_t)constrain(o["byteOffset"].as<int>(), 0, 7);
      if (!o["byteLen"].isNull())    sg.byteLen    = (uint8_t)o["byteLen"].as<int>();
      if (!o["bigEndian"].isNull())  sg.bigEndian  = o["bigEndian"].as<bool>();
      if (!o["signedVal"].isNull())  sg.signedVal  = o["signedVal"].as<bool>();
      if (!o["scale"].isNull())      sg.scale      = o["scale"].as<float>();
      if (!o["offset"].isNull())     sg.offset     = o["offset"].as<float>();
      if (!o["decimals"].isNull())   sg.decimals   = o["decimals"].as<int>();
      i++;
    }
  }

  return true;
}