// FIRMWARE VERSION: rig-module-sensors-1.18.0
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
#include <string.h>  // strncpy — used by pack*/unpack* below

#define FW_VERSION "rig-module-sensors-1.18.0"

// =============================================================================
//  ⚙️  BUILD SWITCHES — edit these, nothing else above the code
//  =============================================================================
//  REMOVED 2026-09-17 (Sarah: "we don't need the build switches in
//  production, those were put there for troubleshooting and we don't need
//  them anymore as we got the major bugs and troubles figured out"). This
//  block used to define 16 switches; a "spot sneaky bugs" pass the same day
//  found that 13 of them (NO_WIFI, ENABLE_MODBUS_POLL_TASK, ENABLE_CAN,
//  CANOPEN_ENCODER_PRESENT, CAN_DEBUG, MODBUS_DEBUG, RS485_DEBUG,
//  BUFFER_DEBUG, CANOPEN_DEBUG, CAN_TASK_DEBUG, DEBUG_BOOT_IN_AP, AUTO_POST,
//  WEBUI_ENABLE, DEBUG_PAGE_ENABLED, ENABLE_OTA, ESP32PING_AVAILABLE — that's
//  actually 16, all of them except the 3 below) were dead: defined here,
//  documented in a "quick reference" as if flipping them changed behavior,
//  but never once checked by an #if/if anywhere in the codebase. Deleting
//  them outright rather than leaving harmless dead defines around, since the
//  quick-reference comment actively lied about what they did and that's
//  worse than nothing being there at all.
//
//  The 3 switches that WERE real (FACTORY_RESET_ONCE, FORGET_WIFI_ONCE,
//  QUIET_SERIAL_BOOT) are gone too, per the same instruction — troubleshooting-
//  only, not needed now that the bugs they existed to recover from are fixed.
//  If a genuine field rescue is ever needed again (bricked config, dead WiFi
//  credentials wedging boot), the equivalent web UI actions still exist:
//  /system's "Factory Reset" button, and /config's "Forget WiFi" button —
//  those don't need a reflash at all, unlike these switches ever did.
// =============================================================================

#include <WiFi.h>
#include <Preferences.h>
#include <esp_efuse.h>
#include <esp_mac.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <esp_partition.h>

// Max number of independently-configured RS485 Modbus sensors on the bus.
// Modbus RTU addressing goes up to 247 slaves, but polling time and NVS
// storage are the real limits here — 16 is generous for a rig module
// (each sensor poll is a few ms) while keeping config/UI manageable.
#define MAX_SENSORS 16

// Max number of independently-configured CAN signals extracted from raw
// frames. Same reasoning as MAX_SENSORS — plenty of headroom, still small
// enough to keep NVS usage and the config page sane.
#define MAX_CAN_SIGNALS 16
#define MAX_MODBUS_CHANNELS 8
#define MODBUS_BOARD_SLAVE_ID 1
#define SENSOR_MIN_SLAVE_ID 2

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
  uint8_t slaveId    = SENSOR_MIN_SLAVE_ID; // Direct sensors use 2-247; board address 1 is reserved
  uint8_t funcCode   = 3;       // 3 = Read Holding Registers, 4 = Read Input Registers
                                 // Default is 3, not 4: most real-world sensors we've hit
                                 // (SM7779 radar included) only answer FC03 and stay
                                 // completely silent on FC04 — see 2026-08-18 incident where
                                 // three new sensor slots timed out 100% on FC04 while a bus
                                 // scan showed them replying instantly and cleanly to FC03.
  uint16_t regAddr   = 0;       // starting register address
  uint8_t dataType   = MB_UINT16;
  uint8_t wordOrder  = MB_WORD_HIGH_FIRST; // only matters for 32-bit types
  // Some real sensors (confirmed: SM7779 radar level) ignore the register
  // address in the request entirely and always reply with the SAME fixed
  // multi-register block starting from their own register 0 — e.g. SM7779
  // always sends back [distance, level, status] no matter what you ask
  // for. For a sensor like that, regAddr above is really just "what I
  // asked for" (mostly irrelevant to what comes back); THIS is "how many
  // registers into that fixed reply block the value I actually want
  // starts at" — e.g. 0 = distance, 1 = level, 2 = status for SM7779.
  // Defaults to 0, which reproduces the old always-take-the-front
  // behavior exactly — existing configs are unaffected.
  uint8_t respRegOffset = 0;
  float  scale       = 1.0f;    // engineering value = raw * scale + offset
  float  offset      = 0.0f;
  int    decimals    = 2;       // rounding for display/report

  // --- Tank volume (optional derived calc) -------------------------------
  // CHANGED 2026-09-17 (Sarah/Gerald, second pass): the two-point
  // calibration (Value @ Empty / Value @ Full) is GONE -- Sarah didn't
  // understand it and it wasn't needed for how her radar sensors actually
  // work. This is a top-mounted radar reporting air-gap distance straight
  // down to the water surface (closer = higher water, further = lower --
  // the sensor's own reading IS a distance in meters already, no per-
  // sensor calibration step required). Model is now dead simple: enter
  // the tank's real physical height (tankHeightM, meters, measured empty
  // tank floor to sensor face) ONCE, and water height is always
  // `tankHeightM - reading.value` (see scaling.h computeSensorVolume()).
  // Combined with tankLengthM/tankWidthM (also entered directly, same as
  // before), volume = length x width x height. Rectangular tanks only
  // (Sarah's rigs); a non-rectangular tank would need a different area
  // model, not supported here.
  bool   volumeEnabled = false;
  float  tankLengthM   = 0.0f;   // tank footprint length, meters
  float  tankWidthM    = 0.0f;   // tank footprint width, meters
  float  tankHeightM   = 0.0f;   // tank height, meters (floor to sensor face) -- water height = this minus the sensor's own reading
  String capacityUnit  = "m3";   // "m3" or "gal" -- OUTPUT unit only, computed volume is converted to this for display
};

// Live reading for one sensor.
// How many consecutive raw timeouts (see `status` below) before a sensor
// that HAS reported successfully before is allowed to show "timeout" on
// the /sensors and /live pages. Some sensors (e.g. a radar level unit
// with a slow measurement cycle) only have fresh data ready on 1 out of
// every 2-3 polls by design — that's not a fault, but it looked like one
// on those two pages every single poll cycle. The raw "status" field
// (see handleSensorsLive in webui.h) always shows every raw timeout
// regardless of this debounce.
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

// Fixed 4-20mA channel configuration for the adapter board at slave 1.
// enabled=false by default (2026-09-18, Sarah: match the RS485/CAN
// convention) — slots start empty/unchecked, same as SensorConfig and
// CanSignalConfig above, rather than assuming all 8 are wired up.
struct ModbusChannelConfig { bool enabled=false; String name=""; String kind=""; String unit=""; float engMin=0.0f; float engMax=1.0f; };
// Digital I/O config for the adapter board — only real on the AMIDJ14
// variant (4 DI + 4 DO). The Waveshare 8AI variant has no DI/DO hardware
// at all; gated at poll/render time on the auto-detected board type
// (modbusDetectedType == "amidj14"), same "don't report phantom hardware"
// convention as boardProfile.hasDigitalIO in the analog-board variant.
// enabled=false by default, same reasoning/convention as ModbusChannelConfig
// above — check whichever DI/DO are actually wired up.
struct ModbusDigitalConfig { bool enabled=false; String name=""; };
struct ModbusBoardConfig {
  bool enabled=true;
  String boardType="auto";
  ModbusChannelConfig channels[MAX_MODBUS_CHANNELS];
  ModbusDigitalConfig din[4];
  ModbusDigitalConfig dout[4];
};
struct ModbusChannelReading { bool valid=false; bool hasValue=false; float mA=0.0f; float value=0.0f; String status="stale"; };
struct ModbusDigitalReading { bool valid=false; bool state=false; String status="stale"; };

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

// =============================================================================
// Packed (POD, fixed-size) mirrors of SensorConfig/CanSignalConfig, used
// ONLY for NVS blob storage — see BUG FIXED 2026-09-16 below.
//
// BUG FIXED 2026-09-16 (Sarah: "CAN and RS485 sensor settings don't
// persist across a hard power cycle, only survives a page refresh"):
// saveConfig()/loadConfig() previously stored each sensor/CAN-signal
// field as its OWN individual NVS key — 18 keys x 16 sensor slots + 13
// keys x 16 CAN slots + ~15 top-level keys = ~511 separate NVS entries
// in this one "rigmod" namespace, each written with its own
// nvs_set_xxx()+nvs_commit() call (~511 independent flash commits per
// "Save All Sensors"/"Save All Signals" click). Confirmed live on
// Sarah's board: NVS was sitting at 504/630 entries used, 126 free —
// i.e. this feature alone was consuming almost the ENTIRE default 20K
// partition, leaving only the one mandatory reserved page for garbage
// collection with no real headroom. A refresh only re-renders the
// already-correct in-RAM cfg struct (proves nothing about flash); only
// an actual reboot re-reads through loadConfig() and would have exposed
// any of those ~511 writes that silently failed to land.
//
// Fix: sensors[] and canSignals[] are now serialized as ONE packed
// struct each and stored as a single NVS blob per array ("sensorsBlob"/
// "canBlob") — 2 keys, 2 flash commits total per save, instead of ~340.
// This also frees up the vast majority of that NVS headroom. A one-time
// migration (loadConfig()) reads any OLD per-key data still on flash so
// existing saved configs are not lost by this change, then the next
// save writes the new blob format and the old keys become orphaned
// (harmlessly ignored, same pattern as the old "canEn" key).
// =============================================================================
#define SENSOR_STR_LEN 32     // generous for name/kind/unit/capacityUnit
struct SensorConfigPacked {
  bool     enabled;
  char     name[SENSOR_STR_LEN];
  char     kind[SENSOR_STR_LEN];
  char     unit[SENSOR_STR_LEN];
  uint8_t  slaveId;
  uint8_t  funcCode;
  uint16_t regAddr;
  uint8_t  dataType;
  uint8_t  wordOrder;
  uint8_t  respRegOffset;
  float    scale;
  float    offset;
  int32_t  decimals;
  bool     volumeEnabled;
  float    tankLengthM;
  float    tankWidthM;
  float    tankHeightM;
  char     capacityUnit[SENSOR_STR_LEN];
};

#define MODBUS_STR_LEN 32
struct ModbusChannelPacked { bool enabled; char name[MODBUS_STR_LEN]; char kind[MODBUS_STR_LEN]; char unit[MODBUS_STR_LEN]; float engMin; float engMax; };
struct ModbusDigitalPacked { bool enabled; char name[MODBUS_STR_LEN]; };
struct ModbusBoardPacked {
  bool enabled;
  char boardType[MODBUS_STR_LEN];
  ModbusChannelPacked channels[MAX_MODBUS_CHANNELS];
  ModbusDigitalPacked din[4];
  ModbusDigitalPacked dout[4];
};
static void packModbus(const ModbusBoardConfig& b, ModbusBoardPacked& p) {
  p.enabled=b.enabled; strncpy(p.boardType,b.boardType.c_str(),MODBUS_STR_LEN-1); p.boardType[MODBUS_STR_LEN-1]=0;
  for(int i=0;i<MAX_MODBUS_CHANNELS;i++){const auto& c=b.channels[i];auto& q=p.channels[i];q.enabled=c.enabled;strncpy(q.name,c.name.c_str(),MODBUS_STR_LEN-1);q.name[MODBUS_STR_LEN-1]=0;strncpy(q.kind,c.kind.c_str(),MODBUS_STR_LEN-1);q.kind[MODBUS_STR_LEN-1]=0;strncpy(q.unit,c.unit.c_str(),MODBUS_STR_LEN-1);q.unit[MODBUS_STR_LEN-1]=0;q.engMin=c.engMin;q.engMax=c.engMax;}
  for(int i=0;i<4;i++){p.din[i].enabled=b.din[i].enabled;strncpy(p.din[i].name,b.din[i].name.c_str(),MODBUS_STR_LEN-1);p.din[i].name[MODBUS_STR_LEN-1]=0;}
  for(int i=0;i<4;i++){p.dout[i].enabled=b.dout[i].enabled;strncpy(p.dout[i].name,b.dout[i].name.c_str(),MODBUS_STR_LEN-1);p.dout[i].name[MODBUS_STR_LEN-1]=0;}
}
static void unpackModbus(const ModbusBoardPacked& p, ModbusBoardConfig& b) {
  b.enabled=p.enabled;b.boardType=String(p.boardType);if(b.boardType.isEmpty())b.boardType="auto";
  for(int i=0;i<MAX_MODBUS_CHANNELS;i++){const auto& q=p.channels[i];auto& c=b.channels[i];c.enabled=q.enabled;c.name=String(q.name);c.kind=String(q.kind);c.unit=String(q.unit);c.engMin=q.engMin;c.engMax=q.engMax;}
  for(int i=0;i<4;i++){b.din[i].enabled=p.din[i].enabled;b.din[i].name=String(p.din[i].name);}
  for(int i=0;i<4;i++){b.dout[i].enabled=p.dout[i].enabled;b.dout[i].name=String(p.dout[i].name);}
}

struct CanSignalConfigPacked {
  bool     enabled;
  char     name[SENSOR_STR_LEN];
  char     kind[SENSOR_STR_LEN];
  char     unit[SENSOR_STR_LEN];
  uint32_t canId;
  bool     extended;
  uint8_t  byteOffset;
  uint8_t  byteLen;
  bool     bigEndian;
  bool     signedVal;
  float    scale;
  float    offset;
  int32_t  decimals;
};

static void packSensor(const SensorConfig& s, SensorConfigPacked& p) {
  p.enabled = s.enabled;
  strncpy(p.name, s.name.c_str(), SENSOR_STR_LEN - 1); p.name[SENSOR_STR_LEN - 1] = 0;
  strncpy(p.kind, s.kind.c_str(), SENSOR_STR_LEN - 1); p.kind[SENSOR_STR_LEN - 1] = 0;
  strncpy(p.unit, s.unit.c_str(), SENSOR_STR_LEN - 1); p.unit[SENSOR_STR_LEN - 1] = 0;
  p.slaveId = s.slaveId;
  p.funcCode = s.funcCode;
  p.regAddr = s.regAddr;
  p.dataType = s.dataType;
  p.wordOrder = s.wordOrder;
  p.respRegOffset = s.respRegOffset;
  p.scale = s.scale;
  p.offset = s.offset;
  p.decimals = s.decimals;
  p.volumeEnabled = s.volumeEnabled;
  p.tankLengthM = s.tankLengthM;
  p.tankWidthM = s.tankWidthM;
  p.tankHeightM = s.tankHeightM;
  strncpy(p.capacityUnit, s.capacityUnit.c_str(), SENSOR_STR_LEN - 1); p.capacityUnit[SENSOR_STR_LEN - 1] = 0;
}

static void unpackSensor(const SensorConfigPacked& p, SensorConfig& s) {
  s.enabled = p.enabled;
  s.name = String(p.name);
  s.kind = String(p.kind);
  s.unit = String(p.unit);
  s.slaveId = p.slaveId;
  s.funcCode = p.funcCode;
  s.regAddr = p.regAddr;
  s.dataType = p.dataType;
  s.wordOrder = p.wordOrder;
  s.respRegOffset = p.respRegOffset;
  s.scale = p.scale;
  s.offset = p.offset;
  s.decimals = p.decimals;
  s.volumeEnabled = p.volumeEnabled;
  s.tankLengthM = p.tankLengthM;
  s.tankWidthM = p.tankWidthM;
  s.tankHeightM = p.tankHeightM;
  s.capacityUnit = String(p.capacityUnit);
}

static void packCanSignal(const CanSignalConfig& c, CanSignalConfigPacked& p) {
  p.enabled = c.enabled;
  strncpy(p.name, c.name.c_str(), SENSOR_STR_LEN - 1); p.name[SENSOR_STR_LEN - 1] = 0;
  strncpy(p.kind, c.kind.c_str(), SENSOR_STR_LEN - 1); p.kind[SENSOR_STR_LEN - 1] = 0;
  strncpy(p.unit, c.unit.c_str(), SENSOR_STR_LEN - 1); p.unit[SENSOR_STR_LEN - 1] = 0;
  p.canId = c.canId;
  p.extended = c.extended;
  p.byteOffset = c.byteOffset;
  p.byteLen = c.byteLen;
  p.bigEndian = c.bigEndian;
  p.signedVal = c.signedVal;
  p.scale = c.scale;
  p.offset = c.offset;
  p.decimals = c.decimals;
}

static void unpackCanSignal(const CanSignalConfigPacked& p, CanSignalConfig& c) {
  c.enabled = p.enabled;
  c.name = String(p.name);
  c.kind = String(p.kind);
  c.unit = String(p.unit);
  c.canId = p.canId;
  c.extended = p.extended;
  c.byteOffset = p.byteOffset;
  c.byteLen = p.byteLen;
  c.bigEndian = p.bigEndian;
  c.signedVal = p.signedVal;
  c.scale = p.scale;
  c.offset = p.offset;
  c.decimals = p.decimals;
}

struct CanSignalReading {
  bool   hasValue = false;
  float  rawValue = 0.0f;
  float  value    = 0.0f;
  // "ok" | "stale" -- "stale" means no matching frame seen yet OR (fixed
  // 2026-09-17, see can.h's checkCanSignalStaleness()) it USED to be "ok"
  // but no matching frame has arrived in the last CAN_SIGNAL_STALE_MS.
  // hasValue/value/rawValue are deliberately left at their last real
  // reading even after going stale -- same "show it, don't blank it"
  // convention as the rest of this signal's stale-not-hidden handling on
  // the Modules page.
  String status   = "stale";
  unsigned long lastSeenMs = 0;
};

// Full module config
struct ModuleConfig {
  String moduleId       = "";    // built from MAC: MODULE-ABC123
  String moduleName     = "";
  String moduleType     = "generic";
  String description    = "";
  long   modbusBaud     = 9600;
  // True once the user has explicitly set the baud on /config (or via the
  // per-sensor "Auto-Detect Baud" button). While true, auto-detect (boot-
  // time AND the periodic background scan) will ONLY probe the current
  // baud looking for new sensors — it will never hunt through other baud
  // rates and silently overwrite this setting. Without this flag, a noisy/
  // colliding bus with no sensor slot currently enabled could produce a
  // false-positive CRC match at some other baud during a routine scan and
  // stomp a baud you just deliberately set — that's exactly what happened
  // 2026-08-10 (radar+pressure sensor address collision -> garbled bus ->
  // manual baud fix kept reverting because every reboot re-ran the full
  // baud hunt). Defaults false so a genuinely fresh module still gets the
  // "just wire up a sensor, it's auto-found" convenience.
  bool   baudManuallySet = false;
  int    pollIntervalS  = 7;     // how often to POST to the Pi (also the poll-task cycle gap — 7s default clears radar sensors' ~6s measurement cycle without spamming timeouts)
  // Default blank = auto-derive (rigNNN SSID -> 192.168.NNN.10, then mDNS).
  // Set a static IP here for rigs whose WiFi doesn't follow rigNNN naming
  // (e.g. Rig 6 NORQIN at 192.168.5.194). Type __auto__ to clear it again.
  String piHost         = "";
  String rigToken       = "7804991970";
  String wifiSSID       = "";
  String wifiPass       = "";

  // No canEnabled field -- CAN is unconditional (2026-09-10, Sarah: doesn't
  // want the OPTION to disable it, plug-and-play only). Controller always
  // starts in setup() (waveshare-s3-sensors.ino); canopenBridge below still
  // gates transmit-capable mode vs. a safe listen-only tap.
  long   canBitrate     = 250000; // 250k = most common (J1939/drill CAN); 500k also common

  // --- CANopen Bridge mode (2026-09-08) -------------------------------
  // This board sits on the encoder's own dedicated CAN wire (not shared
  // drill traffic) and has to actively wake it up — see can.h's header
  // comment for the full story. Default true (2026-09-09, was false):
  // Sarah's rig-prototype setup always needs BOTH the RS485 sensors AND
  // this CAN bridge running with zero manual setup, same as RS485
  // already is — plug it in and go, no visiting / to check boxes first.
  // Turning this off falls back to "safe passive listen-only tap" for
  // anyone who ever wires this board onto a bus they're not supposed to
  // transmit on -- CAN itself always runs (no way to fully disable it).
  bool   canopenBridge         = true;
  uint8_t canopenNodeId        = 0x7F;  // EPC CANopen encoder factory default (confirmed)
  bool   canopenTargetSpecific = false; // false = NMT Start targets "all nodes" (confirmed
                                          // working on bench, the safe default)

  // LEGACY (v1.15.32): used to bump every time ensureStaStarted()'s
  // NVS-erase self-heal fired. That whole code path — which wiped the
  // ENTIRE NVS partition (not just our namespace) on transient WL_STOPPED
  // during a WiFi mode change — was REMOVED in v1.15.32: it's the actual
  // root cause identified for the "changing WiFi network boot-loops the
  // board, needs a flash erase to recover" field report (2026-09-15). See
  // bringUpWifi() in the .ino for the fix (mode set once, never cycled).
  // Field kept (not deleted) purely so existing NVS/readback code and old
  // boards' stored counts don't break; it will never increment again on
  // any board running v1.15.32+. A non-zero value here on an OLDER
  // firmware version was the smoking gun for that bug.
  uint32_t nvsEraseSelfHealCount = 0;

  SensorConfig    sensors[MAX_SENSORS];
  ModbusBoardConfig modbusBoard;
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
  c.baudManuallySet = p.getBool("mbBaudSet", false);
  c.pollIntervalS = p.getInt("pollInt", 7);
  c.piHost        = p.getString("piHost", "");
  if (c.piHost == "__auto__") c.piHost = ""; // legacy: explicit auto marker normalises to blank
  c.rigToken      = p.getString("rigToken", "7804991970");
  if (c.rigToken.isEmpty()) c.rigToken = "7804991970"; // self-heal, see other variants
  c.wifiSSID      = p.getString("wifiSSID", "");
  c.wifiPass      = p.getString("wifiPass", "");
  // BUG FIXED 2026-09-09: copBr defaulted to false on a truly fresh module
  // (before the very first save), even though the ModuleConfig struct's
  // own in-RAM default above says canopenBridge=true. On brand-new NVS
  // the "copBr" key doesn't exist yet, so getBool() falls back to
  // whatever's passed here — NOT the struct default — meaning the
  // CANopen bridge that actually wakes the encoder came up silently OFF
  // out of the box, requiring one manual visit to / to check the box and
  // Save before anything on the CAN side worked at all. Sarah wants both
  // RS485 sensors AND CAN to be plug-and-play with zero manual setup, so
  // this fallback now matches the struct default. (2026-09-10: canEnabled
  // removed entirely -- CAN itself is unconditional, no toggle exists.)
  c.canBitrate    = p.getLong("canBit", 250000);
  c.canopenBridge = p.getBool("copBr", true);
  c.canopenNodeId = (uint8_t)p.getInt("copNode", 0x7F);
  c.canopenTargetSpecific = p.getBool("copTgtSp", false);
  c.nvsEraseSelfHealCount = p.getULong("nvsHealCnt", 0);

  // BUG FIXED 2026-09-16 — see SensorConfigPacked comment (above the
  // struct defs) for the full story: sensor/CAN-signal config used to be
  // ~511 individual NVS keys, tight enough against the 20K partition
  // (504/630 entries used, confirmed live) that some writes could
  // silently fail. Now read as one packed blob per array.
  //
  // MIGRATION: a board that already has settings saved under the OLD
  // per-key format won't have "sensorsBlob"/"canBlob" yet on its first
  // boot after this update — getBytesLength() returns 0 for a key that
  // doesn't exist (confirmed in Preferences.cpp, not an error/exception).
  // In that case, fall back to reading the old per-key format so
  // existing saved sensors/CAN signals are NOT lost by this change. The
  // next save (any "Save All Sensors"/"Save All Signals" click) writes
  // the new blob format going forward; the old keys are then orphaned
  // and harmlessly ignored, same pattern as the old "canEn" key before.
  {
    SensorConfigPacked packed[MAX_SENSORS];
    size_t got = p.getBytes("sensorsBlob", packed, sizeof(packed));
    if (got == sizeof(packed)) {
      for (int i = 0; i < MAX_SENSORS; i++) unpackSensor(packed[i], c.sensors[i]);
      Serial.println("[Config] Loaded sensors from sensorsBlob (current format).");
    } else {
      Serial.println("[Config] No sensorsBlob found — migrating from old per-key sensor format (if any).");
      for (int i = 0; i < MAX_SENSORS; i++) {
        String pre = "s" + String(i) + "_";
        c.sensors[i].enabled     = p.getBool((pre + "en").c_str(), false);
        c.sensors[i].name        = p.getString((pre + "nm").c_str(), "");
        c.sensors[i].kind        = p.getString((pre + "kd").c_str(), "");
        c.sensors[i].unit        = p.getString((pre + "ut").c_str(), "");
        c.sensors[i].slaveId     = (uint8_t)p.getInt((pre + "sid").c_str(), SENSOR_MIN_SLAVE_ID);
        c.sensors[i].funcCode    = (uint8_t)p.getInt((pre + "fc").c_str(), 3);
        c.sensors[i].regAddr     = (uint16_t)p.getInt((pre + "reg").c_str(), 0);
        c.sensors[i].dataType    = (uint8_t)p.getInt((pre + "dt").c_str(), MB_UINT16);
        c.sensors[i].wordOrder   = (uint8_t)p.getInt((pre + "wo").c_str(), MB_WORD_HIGH_FIRST);
        c.sensors[i].respRegOffset = (uint8_t)p.getInt((pre + "ro").c_str(), 0);
        c.sensors[i].scale       = p.getFloat((pre + "sc").c_str(), 1.0f);
        c.sensors[i].offset      = p.getFloat((pre + "of").c_str(), 0.0f);
        c.sensors[i].decimals    = p.getInt((pre + "dec").c_str(), 2);
        c.sensors[i].volumeEnabled = p.getBool((pre + "vE").c_str(), false);
        // CHANGED 2026-09-17 (two passes same day):
        //   1st pass: "cap" (single capacity number) -> "len"/"wid" (tank
        //   footprint, meters).
        //   2nd pass: two-point calibration ("vz"/"vm", Value @ Empty /
        //   Value @ Full) removed entirely -- Sarah didn't need/understand
        //   it; replaced by a single "th" (tank height, meters) -- see
        //   SensorConfig's volumeEnabled comment in this file for the
        //   current model. This migration path only ever runs on a board
        //   that saved under the OLD per-key format AND has never saved
        //   since either change (see the blob-vs-per-key migration note
        //   above) -- old "cap"/"vz"/"vm" keys are intentionally NOT read
        //   here, none of them mean anything under the current model;
        //   length/width/height just come back as 0 (volume disabled,
        //   effectively) until re-entered on /sensors.
        c.sensors[i].tankLengthM   = p.getFloat((pre + "len").c_str(), 0.0f);
        c.sensors[i].tankWidthM    = p.getFloat((pre + "wid").c_str(), 0.0f);
        c.sensors[i].tankHeightM   = p.getFloat((pre + "th").c_str(), 0.0f);
        c.sensors[i].capacityUnit  = p.getString((pre + "cu").c_str(), "m3");
      }
    }
  }

  { ModbusBoardPacked packed; size_t got=p.getBytes("modbusBlob",&packed,sizeof(packed)); if(got==sizeof(packed)) unpackModbus(packed,c.modbusBoard); }
  {
    CanSignalConfigPacked packed[MAX_CAN_SIGNALS];
    size_t got = p.getBytes("canBlob", packed, sizeof(packed));
    if (got == sizeof(packed)) {
      for (int i = 0; i < MAX_CAN_SIGNALS; i++) unpackCanSignal(packed[i], c.canSignals[i]);
      Serial.println("[Config] Loaded CAN signals from canBlob (current format).");
    } else {
      Serial.println("[Config] No canBlob found — migrating from old per-key CAN format (if any).");
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
  }
}

// NVS partition stats for the "rigmod" namespace's underlying storage —
// used to catch a real, silent failure mode: Preferences::putXxx() does
// NOT throw or block on a full partition, it just returns 0 (failure)
// while everything else carries on as if nothing happened. With 16
// sensor slots x ~16 keys each + 16 CAN slots x ~13 keys each, this
// namespace alone is ~490 keys — if the default NVS partition (a few KB
// after accounting for entry overhead) fills up, NEW keys silently fail
// to write while EXISTING keys keep updating fine. mbBaudSet is exactly
// this shape: added in v1.7.0, long after modbusBaud already existed —
// if the partition was already full, mbBaudSet could never get created
// at all, so baudManuallySet would read back false on every single boot
// even though the code setting it to true "succeeded" (the in-RAM value
// was true right up until save, it just never actually landed in flash).
// Exposed on /system so this is visible without a serial cable.
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

// Actual on-flash size of the "nvs" partition, read straight from the
// partition TABLE at runtime (esp_partition_find_first) — not derived
// from nvs_get_stats()'s entry counts, which only tell you usage WITHIN
// whatever partition got mapped, not its real byte size. Kept as a
// general-purpose sanity check after any flash. NOTE (2026-09-11): this
// helper originally existed to catch a sketch-folder partitions.csv
// being silently ignored by the IDE — that custom table was REMOVED in
// v1.15.11 after it bricked boards in the field (table/app offset
// mismatch, see README "Partition table incident"). This firmware now
// intentionally runs on the stock 20K NVS partition; the check should
// read 0x5000 (20480). Don't "fix" a 20K reading by re-adding a
// partitions.csv — that's what caused the incident.
size_t getNvsPartitionSizeBytes() {
  const esp_partition_t* p = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_NVS, NULL);
  return p ? p->size : 0;
}

// Save all config to NVS. Checks the two baud-related keys' actual write
// result and logs loudly if either silently failed (see getNvsStats()
// comment above) — every other key is best-effort like before, but these
// two are exactly what's been reported stuck, so they get verified.
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
  p.putInt("pollInt", c.pollIntervalS);
  p.putString("piHost", c.piHost);
  p.putString("rigToken", c.rigToken);
  p.putString("wifiSSID", c.wifiSSID);
  p.putString("wifiPass", c.wifiPass);
  // No "canEn" key anymore -- CAN itself is unconditional (2026-09-10),
  // nothing to persist for it. The now-orphaned "canEn" key from older
  // firmware versions is harmlessly ignored (loadConfig() no longer reads
  // it either); factory reset still clears it like everything else.
  p.putLong("canBit", c.canBitrate);
  p.putBool("copBr", c.canopenBridge);
  p.putInt("copNode", c.canopenNodeId);
  p.putBool("copTgtSp", c.canopenTargetSpecific);
  p.putULong("nvsHealCnt", c.nvsEraseSelfHealCount);

  // BUG FIXED 2026-09-16 — see SensorConfigPacked comment above for the
  // full story. Was: 18 keys x 16 slots = 288 individual NVS writes here
  // (and another 208 for CAN signals below), ~511 total in this function.
  // Now: pack all 16 sensor slots into ONE fixed-size blob, one NVS key,
  // one flash commit — same for CAN signals. Massively less NVS pressure,
  // and each save is now effectively atomic instead of ~500 independent
  // writes any one of which could silently fail on a tight partition.
  {
    SensorConfigPacked packed[MAX_SENSORS];
    for (int i = 0; i < MAX_SENSORS; i++) packSensor(c.sensors[i], packed[i]);
    size_t wrote = p.putBytes("sensorsBlob", packed, sizeof(packed));
    if (wrote != sizeof(packed)) {
      Serial.printf("[Config] WARNING: NVS write FAILED for sensorsBlob (wrote=%u, expected=%u) "
        "— partition may be full/corrupt.\n", (unsigned)wrote, (unsigned)sizeof(packed));
    }
  }
  { ModbusBoardPacked packed; packModbus(c.modbusBoard,packed); p.putBytes("modbusBlob",&packed,sizeof(packed)); }
  {
    CanSignalConfigPacked packed[MAX_CAN_SIGNALS];
    for (int i = 0; i < MAX_CAN_SIGNALS; i++) packCanSignal(c.canSignals[i], packed[i]);
    size_t wrote = p.putBytes("canBlob", packed, sizeof(packed));
    if (wrote != sizeof(packed)) {
      Serial.printf("[Config] WARNING: NVS write FAILED for canBlob (wrote=%u, expected=%u) "
        "— partition may be full/corrupt.\n", (unsigned)wrote, (unsigned)sizeof(packed));
    }
  }

  // Old per-key sensor/CAN-signal writes (18 keys x 16 sensor slots + 13
  // keys x 16 CAN slots) removed here 2026-09-16 — replaced by the two
  // blob writes above. loadConfig() below still knows how to read the
  // OLD per-key format for one-time migration; nothing writes through
  // it anymore, so those old keys become orphaned/harmless once a save
  // happens under the new format (same pattern as the old "canEn" key).
  p.end();
}
