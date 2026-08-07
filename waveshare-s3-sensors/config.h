// =============================================================================
// config.h — Rig Module (Direct Sensors) configuration structures + NVS
//
// This variant is a REAL Modbus RTU MASTER: it polls a configurable list of
// independent RS485 sensors directly (each its own Modbus slave with its
// own register map), instead of one fixed analog-to-Modbus adapter board.
// Any Modbus RTU sensor works — pressure, temp, flow, whatever — as long
// as you can tell it slave ID + register + data type + scale. There's also
// a configurable list of CAN "signals" extracted from raw CAN frames, for
// whenever you know what to pull off the bus.
// =============================================================================
#pragma once
#include <Arduino.h>

#define FW_VERSION "rig-module-sensors-1.6.6"

#include <WiFi.h>
#include <Preferences.h>
#include <esp_efuse.h>
#include <esp_mac.h>

// Max number of independently-configured RS485 Modbus sensors on the bus.
// Modbus RTU addressing goes up to 247 slaves, but polling time and NVS
// storage are the real limits here — 16 is generous for a rig module
// (each sensor poll is a few ms) while keeping config/UI manageable.
#define MAX_SENSORS 16

// Max number of independently-configured CAN signals extracted from raw
// frames. Same reasoning as MAX_SENSORS — plenty of headroom, still small
// enough to keep NVS usage and the config page sane.
#define MAX_CAN_SIGNALS 16

// Modbus data types a register (or register pair) can be interpreted as.
// Covers the overwhelming majority of real sensors: plain 16-bit values
// (signed/unsigned), and 32-bit values spanning two registers (signed/
// unsigned/IEEE-754 float), in either register order (some sensors send
// the high word first, some the low word first — "word order" below).
enum ModbusDataType {
  MB_UINT16 = 0,
  MB_INT16  = 1,
  MB_UINT32 = 2,
  MB_INT32  = 3,
  MB_FLOAT32 = 4,
};

// For 32-bit types (2 registers): which register holds the high 16 bits.
enum ModbusWordOrder {
  MB_WORD_HIGH_FIRST = 0,  // register[0] = high word, register[1] = low word (most common)
  MB_WORD_LOW_FIRST  = 1,  // register[0] = low word,  register[1] = high word
};

// One independently-configured RS485 sensor. Each is its own Modbus RTU
// slave on the shared bus — completely generic, no per-brand special
// casing. "Read this register (or pair) from this slave, interpret it as
// this data type, then scale/offset it into an engineering value."
struct SensorConfig {
  bool   enabled     = false;   // slots start empty; check to activate
  String name        = "";      // e.g. "Standpipe Pressure"
  String kind        = "";      // free text: pressure, temp, flow, level...
  String unit        = "";      // e.g. psi, degC, gpm
  uint8_t slaveId    = 1;       // Modbus slave address, 1-247
  uint8_t funcCode   = 4;       // 3 = Read Holding Registers, 4 = Read Input Registers
  uint16_t regAddr   = 0;       // starting register address
  uint8_t dataType   = MB_UINT16;
  uint8_t wordOrder  = MB_WORD_HIGH_FIRST; // only matters for 32-bit types
  float  scale       = 1.0f;    // engineering value = raw * scale + offset
  float  offset      = 0.0f;
  int    decimals    = 2;       // rounding for display/report

  // --- Tank volume (optional derived calc, same convention as other
  // rig-module variants) — this sensor's engineering value IS a level
  // reading; map it linearly to a volume between two reference points.
  bool   volumeEnabled = false;
  float  capacity      = 0.0f;
  String capacityUnit  = "m3";   // "m3" or "gal"
  float  volZeroLevel  = 0.0f;
  float  volMaxLevel   = 1.0f;
};

// Live reading for one sensor.
// How many consecutive raw timeouts (see `status` below) before a sensor
// that HAS reported successfully before is allowed to show "timeout" on
// the /sensors and /live pages. Some sensors (e.g. a radar level unit
// with a slow measurement cycle) only have fresh data ready on 1 out of
// every 2-3 polls by design — that's not a fault, but it looked like one
// on those two pages every single poll cycle. /diag's comms health table
// and raw traffic log always show every raw timeout regardless of this,
// since those pages exist specifically to see what's really happening.
#define TIMEOUT_DISPLAY_THRESHOLD 6

struct SensorReading {
  bool   valid    = false;   // false = comms failure (timeout/CRC/no response)
  bool   hasValue = false;   // true once a real value has been decoded
  float  rawValue = 0.0f;    // decoded raw value BEFORE scale/offset (for diagnostics)
  float  value    = 0.0f;    // final engineering value (raw*scale+offset)
  String status   = "stale"; // RAW per-poll result: "ok" | "timeout" | "crc" | "stale" | "disabled"
  // Debounced version of `status` for the /sensors and /live pages: a
  // lone timeout (or a short run of them, under TIMEOUT_DISPLAY_THRESHOLD)
  // on a sensor that has reported OK before just keeps showing the last
  // real status instead of flapping to "timeout" every time. Only flips
  // to "timeout" once that many consecutive raw timeouts have piled up.
  // CRC/other errors are NOT debounced — those are real, unexpected
  // failures and show immediately. `status` above is untouched by this
  // and always reflects exactly what just happened, for diagnostics.
  String displayStatus       = "stale";
  unsigned long consecutiveTimeouts = 0;
  unsigned long lastPollMs   = 0;
  unsigned long lastOkMs     = 0;
  unsigned long pollCount    = 0;
  unsigned long errorCount   = 0;
};

// One independently-configured CAN signal — a byte range extracted from
// frames matching a given CAN ID, interpreted as a value. Same idea as a
// Modbus sensor but pulling from the CAN bus instead of RS485. Left empty/
// disabled until you've actually looked at raw frames (via the CAN
// diagnostics page) and know what you're extracting.
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

// Full module config
struct ModuleConfig {
  String moduleId       = "";    // built from MAC: MODULE-ABC123
  String moduleName     = "";
  String moduleType     = "generic";
  String description    = "";
  long   modbusBaud     = 9600;
  int    pollIntervalS  = 3;     // how often to POST to the Pi
  String piHost         = "";
  String rigToken       = "7804991970";
  String wifiSSID       = "";
  String wifiPass       = "";

  bool   canEnabled     = false; // CAN controller only starts if this is on
  long   canBitrate     = 250000; // 250k = most common (J1939/drill CAN); 500k also common

  SensorConfig    sensors[MAX_SENSORS];
  CanSignalConfig canSignals[MAX_CAN_SIGNALS];
};

// Build MODULE-ABC123 from MAC (always, no manual unit-number scheme).
// Call this AFTER WiFi.mode() so the MAC is valid.
void buildModuleId(ModuleConfig& cfg) {
  uint8_t mac[6];
  esp_efuse_mac_get_default(mac);
  char buf[20];
  snprintf(buf, sizeof(buf), "MODULE-%02X%02X%02X", mac[3], mac[4], mac[5]);
  cfg.moduleId = String(buf);
}

// Load all config from NVS
void loadConfig(Preferences& p, ModuleConfig& c) {
  c.moduleName    = p.getString("modName", "");
  c.moduleType    = p.getString("modType", "generic");
  c.description   = p.getString("desc", "");
  c.modbusBaud    = p.getLong("mbBaud", 9600);
  c.pollIntervalS = p.getInt("pollInt", 3);
  c.piHost        = p.getString("piHost", "");
  c.rigToken      = p.getString("rigToken", "7804991970");
  if (c.rigToken.isEmpty()) c.rigToken = "7804991970"; // self-heal, see other variants
  c.wifiSSID      = p.getString("wifiSSID", "");
  c.wifiPass      = p.getString("wifiPass", "");
  c.canEnabled    = p.getBool("canEn", false);
  c.canBitrate    = p.getLong("canBit", 250000);

  for (int i = 0; i < MAX_SENSORS; i++) {
    String pre = "s" + String(i) + "_";
    c.sensors[i].enabled     = p.getBool((pre + "en").c_str(), false);
    c.sensors[i].name        = p.getString((pre + "nm").c_str(), "");
    c.sensors[i].kind        = p.getString((pre + "kd").c_str(), "");
    c.sensors[i].unit        = p.getString((pre + "ut").c_str(), "");
    c.sensors[i].slaveId     = (uint8_t)p.getInt((pre + "sid").c_str(), 1);
    c.sensors[i].funcCode    = (uint8_t)p.getInt((pre + "fc").c_str(), 4);
    c.sensors[i].regAddr     = (uint16_t)p.getInt((pre + "reg").c_str(), 0);
    c.sensors[i].dataType    = (uint8_t)p.getInt((pre + "dt").c_str(), MB_UINT16);
    c.sensors[i].wordOrder   = (uint8_t)p.getInt((pre + "wo").c_str(), MB_WORD_HIGH_FIRST);
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

// Save all config to NVS
void saveConfig(Preferences& p, ModuleConfig& c) {
  p.begin("rigmod", false);
  p.putString("modName", c.moduleName);
  p.putString("modType", c.moduleType);
  p.putString("desc", c.description);
  p.putLong("mbBaud", c.modbusBaud);
  p.putInt("pollInt", c.pollIntervalS);
  p.putString("piHost", c.piHost);
  p.putString("rigToken", c.rigToken);
  p.putString("wifiSSID", c.wifiSSID);
  p.putString("wifiPass", c.wifiPass);
  p.putBool("canEn", c.canEnabled);
  p.putLong("canBit", c.canBitrate);

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
