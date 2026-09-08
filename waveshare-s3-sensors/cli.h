// =============================================================================
// cli.h — Serial command-line interface, mirrors the web UI config pages.
//
// Handy for configuring/debugging over USB when WiFi/web isn't reachable
// (or just faster than clicking through pages). Commands are short on
// purpose (Sarah's request, 2026-08-17) — type `h` at any time for the
// full list. Nothing is written to NVS until you run `sv` (save) — set/
// en/dis commands only change the in-RAM cfg struct, same as the web UI's
// "Save" button pattern.
//
// Call cliInit() once from setup() (after Serial.begin), and cliPoll()
// from loop() every iteration — it's non-blocking, just drains whatever's
// waiting in the Serial RX buffer a line at a time.
// =============================================================================
#pragma once
#include <Arduino.h>
#include "config.h"

// Provided by the main .ino
extern ModuleConfig cfg;
extern Preferences prefs;
extern SensorReading sensorReadings[MAX_SENSORS];
extern CanSignalReading canReadings[MAX_CAN_SIGNALS];
extern SemaphoreHandle_t stateMutex;
extern SemaphoreHandle_t modbusBusMutex;
void saveConfig(Preferences& p, ModuleConfig& c); // config.h
// modbusReadRegs(), modbusScanSlaves(), modbusRegCount(), modbusDecodeValue(),
// MB_OK/MB_TIMEOUT/MB_CRC_ERROR/MB_BAD_RESPONSE are declared+defined in
// modbus.h, already included before this file via the main .ino.

static String _cliBuf = "";
static bool _cliDirty = false; // true once something's changed but not yet `sv`'d

// ---- small parsing helpers -------------------------------------------------
static bool _cliBoolVal(const String& s) {
  String v = s; v.toLowerCase();
  return v == "1" || v == "true" || v == "on" || v == "yes" || v == "y";
}

// Splits on spaces into up to maxTok tokens; the LAST token gets the rest
// of the line (so values can contain spaces, e.g. sensor names).
static int _cliTokenize(const String& line, String* out, int maxTok) {
  String s = line;
  s.trim();
  int n = 0;
  while (n < maxTok - 1) {
    int sp = s.indexOf(' ');
    if (sp < 0) break;
    out[n++] = s.substring(0, sp);
    s = s.substring(sp + 1);
    s.trim();
    if (s.isEmpty()) break;
  }
  if (!s.isEmpty() || n == 0) out[n++] = s;
  return n;
}

static void _cliOk(const String& what) {
  Serial.println("[cli] OK: " + what + " (not saved — run `sv` to persist)");
  _cliDirty = true;
}

static void _cliErr(const String& msg) {
  Serial.println("[cli] ERROR: " + msg);
}

// ---- help -------------------------------------------------------------------
static void _cliHelp() {
  Serial.println(F(
    "\n"
    "=== Rig Module CLI ===\n"
    "General:\n"
    "  h              this help\n"
    "  st             status summary\n"
    "  lv             live sensor + CAN values\n"
    "  sv             save current config to NVS\n"
    "  rb             reboot now\n"
    "\n"
    "Module config:       cm ...        (type `cm h`)\n"
    "WiFi config:          cw ...        (type `cw h`)\n"
    "Sensor config:       cs ...        (type `cs h`)\n"
    "CAN signal config:   cc ...        (type `cc h`)\n"
    "RS485 bus diagnostics: bs / pr     (type `bs h` / `pr h`)\n"
  ));
}

static void _cliBsHelp() {
  Serial.println(F(
    "\n"
    "=== bs — RS485 bus scan (live, doesn't touch saved config) ===\n"
    "  bs [maxAddr]   scan addresses 1..maxAddr (default 16, max 247)\n"
    "                 at the CURRENT configured baud, print every hit\n"
    "  e.g. bs         scan 1-16\n"
    "       bs 50      scan 1-50\n"
  ));
}

static void _cliPrHelp() {
  Serial.println(F(
    "\n"
    "=== pr — probe one register right now (live, doesn't touch saved config) ===\n"
    "  pr <slave> <fc> <reg> [type]\n"
    "    fc:   3 or 4, sent exactly as given (no fallback)\n"
    "    type: u16(default)/i16/u32/i32/f32\n"
    "  Prints raw TX/RX hex and the decoded value (or the failure reason).\n"
    "  e.g. pr 2 4 0        probe slave 2, FC04, register 0, u16\n"
    "       pr 2 3 0 f32    probe slave 2, FC03, register 0, float32\n"
  ));
}

static void _cliCmHelp() {
  Serial.println(F(
    "\n"
    "=== cm — module config ===\n"
    "  cm get                   show all module settings\n"
    "  cm set name <text>       module name\n"
    "  cm set type <text>       module type (free text)\n"
    "  cm set desc <text>       description\n"
    "  cm set poll <secs>       poll interval (1-3600)\n"
    "  cm set pihost <host>     Pi host override (blank = auto mDNS)\n"
    "  cm set token <text>      X-Rig-Token\n"
    "  cm set baud <rate>       RS485 baud (also sets baudManuallySet)\n"
    "  cm set canen <0|1>       CAN enable\n"
    "  cm set canbit <rate>     CAN bitrate (125000/250000/500000/1000000)\n"
    "  cm set copbr <0|1>       CANopen Bridge (transmit bring-up, wake encoder)\n"
    "  cm set copnode <hex>     encoder node ID, e.g. 7F\n"
    "  cm set coptgt <0|1>      target specific node (0 = broadcast, confirmed default)\n"
    "  cm bringup               re-run CANopen bring-up right now (no reboot)\n"
  ));
}

static void _cliCwHelp() {
  Serial.println(F(
    "\n"
    "=== cw — WiFi config ===\n"
    "  cw get                   show saved SSID (password hidden)\n"
    "  cw set ssid <text>       WiFi SSID\n"
    "  cw set pass <text>       WiFi password\n"
  ));
}

static void _cliCsHelp() {
  Serial.println(F(
    "\n"
    "=== cs — sensor config (slots 0-15) ===\n"
    "  cs list                       overview of all slots\n"
    "  cs get <n>                    show slot n's full config\n"
    "  cs en <n> / cs dis <n>         enable/disable slot n\n"
    "  cs set <n> <field> <value>    set one field on slot n\n"
    "\n"
    "  fields: name kind unit slave fc reg type wo ro scale offset dec\n"
    "          vol cap capu vz vm\n"
    "    type:  u16 i16 u32 i32 f32\n"
    "    wo:    hi | lo   (word order, only matters for 32-bit types)\n"
    "    vol:   0|1        (tank volume calc enabled)\n"
    "    capu:  m3 | gal\n"
    "\n"
    "  e.g. cs set 0 name Standpipe Pressure\n"
    "       cs set 0 slave 3\n"
    "       cs set 0 reg 1\n"
    "       cs set 0 type f32\n"
  ));
}

static void _cliCcHelp() {
  Serial.println(F(
    "\n"
    "=== cc — CAN signal config (slots 0-15) ===\n"
    "  cc list                       overview of all slots\n"
    "  cc get <n>                    show slot n's full config\n"
    "  cc en <n> / cc dis <n>         enable/disable slot n\n"
    "  cc set <n> <field> <value>    set one field on slot n\n"
    "\n"
    "  fields: name kind unit id ext bo bl be sv scale offset dec\n"
    "    id:    CAN ID, decimal or 0x-hex\n"
    "    ext:   0|1   (29-bit extended vs 11-bit standard)\n"
    "    be:    0|1   (big-endian; most CAN/J1939 = 1)\n"
    "    sv:    0|1   (signed value)\n"
    "\n"
    "  e.g. cc set 0 id 0x18FEF100\n"
    "       cc set 0 bo 2\n"
    "       cc set 0 bl 2\n"
  ));
}

// ---- status / live -----------------------------------------------------
static void _cliStatus() {
  Serial.println("\n=== Status ===");
  Serial.printf("Firmware   : %s\n", FW_VERSION);
  Serial.printf("Module ID  : %s\n", cfg.moduleId.c_str());
  Serial.printf("Module name: %s\n", cfg.moduleName.c_str());
  Serial.printf("WiFi       : %s (%s)\n",
    WiFi.status() == WL_CONNECTED ? "connected" : "not connected",
    WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString().c_str() : cfg.wifiSSID.c_str());
  Serial.printf("RS485 baud : %ld (manually set: %s)\n", cfg.modbusBaud, cfg.baudManuallySet ? "yes" : "no");
  Serial.printf("CAN        : %s @ %ld\n", cfg.canEnabled ? "enabled" : "disabled", cfg.canBitrate);
  Serial.printf("Poll int   : %d s\n", cfg.pollIntervalS);
  int nS = 0; for (int i = 0; i < MAX_SENSORS; i++) if (cfg.sensors[i].enabled) nS++;
  int nC = 0; for (int i = 0; i < MAX_CAN_SIGNALS; i++) if (cfg.canSignals[i].enabled) nC++;
  Serial.printf("Sensors    : %d / %d slots enabled\n", nS, MAX_SENSORS);
  Serial.printf("CAN signals: %d / %d slots enabled\n", nC, MAX_CAN_SIGNALS);
  Serial.printf("Unsaved changes: %s\n", _cliDirty ? "YES — run `sv`" : "no");
}

static void _cliLive() {
  Serial.println("\n=== Live values ===");
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
    for (int i = 0; i < MAX_SENSORS; i++) {
      if (!cfg.sensors[i].enabled) continue;
      SensorReading& r = sensorReadings[i];
      Serial.printf("  s%-2d %-20s = %8.3f %-6s [%s]\n",
        i, cfg.sensors[i].name.c_str(), r.value, cfg.sensors[i].unit.c_str(), r.displayStatus.c_str());
    }
    for (int i = 0; i < MAX_CAN_SIGNALS; i++) {
      if (!cfg.canSignals[i].enabled) continue;
      CanSignalReading& r = canReadings[i];
      Serial.printf("  c%-2d %-20s = %8.3f %-6s [%s]\n",
        i, cfg.canSignals[i].name.c_str(), r.value, cfg.canSignals[i].unit.c_str(), r.status.c_str());
    }
    xSemaphoreGive(stateMutex);
  } else {
    Serial.println("[cli] Couldn't get state lock, try again");
  }
}

// ---- cm: module config --------------------------------------------------
static void _cliCm(String* tok, int n) {
  if (n < 2) { _cliCmHelp(); return; }
  if (tok[1] == "h" || tok[1] == "help") { _cliCmHelp(); return; }
  if (tok[1] == "get") {
    Serial.println("\n=== Module config ===");
    Serial.printf("name   : %s\n", cfg.moduleName.c_str());
    Serial.printf("type   : %s\n", cfg.moduleType.c_str());
    Serial.printf("desc   : %s\n", cfg.description.c_str());
    Serial.printf("poll   : %d\n", cfg.pollIntervalS);
    Serial.printf("pihost : %s\n", cfg.piHost.isEmpty() ? "(auto)" : cfg.piHost.c_str());
    Serial.printf("token  : %s\n", cfg.rigToken.c_str());
    Serial.printf("baud   : %ld (manually set: %s)\n", cfg.modbusBaud, cfg.baudManuallySet ? "yes" : "no");
    Serial.printf("canen  : %s\n", cfg.canEnabled ? "1" : "0");
    Serial.printf("canbit : %ld\n", cfg.canBitrate);
    Serial.printf("copbr  : %s\n", cfg.canopenBridge ? "1" : "0");
    Serial.printf("copnode: 0x%02X\n", cfg.canopenNodeId);
    Serial.printf("coptgt : %s\n", cfg.canopenTargetSpecific ? "1" : "0");
    return;
  }
  if (tok[1] == "bringup") {
    if (!cfg.canEnabled || !cfg.canopenBridge) {
      _cliErr("CAN Bridge not enabled — set canen 1 and copbr 1, save, and reboot first");
      return;
    }
    Serial.println("[cli] Re-running CANopen bring-up now...");
    canopenBringup(cfg.canopenNodeId, cfg.canopenTargetSpecific);
    Serial.println("[cli] Done.");
    return;
  }
  if (tok[1] == "set") {
    if (n < 4) { _cliErr("usage: cm set <field> <value>"); return; }
    String field = tok[2];
    String val = tok[3];
    if (field == "name") { cfg.moduleName = val; _cliOk("module name"); }
    else if (field == "type") { cfg.moduleType = val; _cliOk("module type"); }
    else if (field == "desc") { cfg.description = val; _cliOk("description"); }
    else if (field == "poll") {
      int v = val.toInt();
      if (v < 1 || v > 3600) { _cliErr("poll must be 1-3600"); return; }
      cfg.pollIntervalS = v; _cliOk("poll interval");
    }
    else if (field == "pihost") { cfg.piHost = val; _cliOk("pi host"); }
    else if (field == "token") { cfg.rigToken = val; _cliOk("token"); }
    else if (field == "baud") {
      cfg.modbusBaud = val.toInt();
      cfg.baudManuallySet = true;
      _cliOk("RS485 baud (locked — auto-detect won't override)");
    }
    else if (field == "canen") { cfg.canEnabled = _cliBoolVal(val); _cliOk("CAN enable (reboot to apply)"); }
    else if (field == "canbit") { cfg.canBitrate = val.toInt(); _cliOk("CAN bitrate (reboot to apply)"); }
    else if (field == "copbr") { cfg.canopenBridge = _cliBoolVal(val); _cliOk("CANopen Bridge (reboot to apply)"); }
    else if (field == "copnode") { cfg.canopenNodeId = (uint8_t)strtol(val.c_str(), nullptr, 16); _cliOk("encoder node ID"); }
    else if (field == "coptgt") { cfg.canopenTargetSpecific = _cliBoolVal(val); _cliOk("target specific node"); }
    else { _cliErr("unknown field: " + field); }
    return;
  }
  _cliErr("unknown cm subcommand, try `cm h`");
}

// ---- cw: wifi config ------------------------------------------------------
static void _cliCw(String* tok, int n) {
  if (n < 2) { _cliCwHelp(); return; }
  if (tok[1] == "h" || tok[1] == "help") { _cliCwHelp(); return; }
  if (tok[1] == "get") {
    Serial.printf("ssid: %s\npass: %s\n", cfg.wifiSSID.isEmpty() ? "(none saved)" : cfg.wifiSSID.c_str(),
      cfg.wifiPass.isEmpty() ? "(none)" : "********");
    return;
  }
  if (tok[1] == "set") {
    if (n < 4) { _cliErr("usage: cw set ssid|pass <value>"); return; }
    if (tok[2] == "ssid") { cfg.wifiSSID = tok[3]; _cliOk("WiFi SSID (reboot to apply)"); }
    else if (tok[2] == "pass") { cfg.wifiPass = tok[3]; _cliOk("WiFi password (reboot to apply)"); }
    else { _cliErr("unknown field: " + tok[2]); }
    return;
  }
  _cliErr("unknown cw subcommand, try `cw h`");
}

// ---- cs: sensor config -----------------------------------------------------
static void _cliCsPrint(int i) {
  SensorConfig& s = cfg.sensors[i];
  const char* typeNames[] = {"u16", "i16", "u32", "i32", "f32"};
  Serial.printf("\n=== Sensor slot %d ===\n", i);
  Serial.printf("enabled : %s\n", s.enabled ? "1" : "0");
  Serial.printf("name    : %s\n", s.name.c_str());
  Serial.printf("kind    : %s\n", s.kind.c_str());
  Serial.printf("unit    : %s\n", s.unit.c_str());
  Serial.printf("slave   : %d\n", s.slaveId);
  Serial.printf("fc      : %d\n", s.funcCode);
  Serial.printf("reg     : %d\n", s.regAddr);
  Serial.printf("type    : %s\n", (s.dataType <= 4) ? typeNames[s.dataType] : "?");
  Serial.printf("wo      : %s\n", s.wordOrder == MB_WORD_HIGH_FIRST ? "hi" : "lo");
  Serial.printf("ro      : %d\n", s.respRegOffset);
  Serial.printf("scale   : %g\n", s.scale);
  Serial.printf("offset  : %g\n", s.offset);
  Serial.printf("dec     : %d\n", s.decimals);
  Serial.printf("vol     : %s\n", s.volumeEnabled ? "1" : "0");
  Serial.printf("cap     : %g\n", s.capacity);
  Serial.printf("capu    : %s\n", s.capacityUnit.c_str());
  Serial.printf("vz      : %g\n", s.volZeroLevel);
  Serial.printf("vm      : %g\n", s.volMaxLevel);
}

static void _cliCsList() {
  Serial.println("\n=== Sensor slots ===");
  for (int i = 0; i < MAX_SENSORS; i++) {
    SensorConfig& s = cfg.sensors[i];
    if (!s.enabled && s.name.isEmpty()) continue;
    Serial.printf("  s%-2d [%s] %-20s slave=%d reg=%d\n",
      i, s.enabled ? "on " : "off", s.name.c_str(), s.slaveId, s.regAddr);
  }
}

static bool _cliDataTypeFromStr(const String& v, uint8_t& out) {
  String lv = v; lv.toLowerCase();
  if (lv == "u16") { out = MB_UINT16; return true; }
  if (lv == "i16") { out = MB_INT16; return true; }
  if (lv == "u32") { out = MB_UINT32; return true; }
  if (lv == "i32") { out = MB_INT32; return true; }
  if (lv == "f32") { out = MB_FLOAT32; return true; }
  return false;
}

static void _cliCsSet(int i, const String& field, const String& val) {
  SensorConfig& s = cfg.sensors[i];
  if (field == "name") s.name = val;
  else if (field == "kind") s.kind = val;
  else if (field == "unit") s.unit = val;
  else if (field == "slave") s.slaveId = (uint8_t)constrain(val.toInt(), 1, 247);
  else if (field == "fc") s.funcCode = (uint8_t)val.toInt();
  else if (field == "reg") s.regAddr = (uint16_t)val.toInt();
  else if (field == "type") {
    uint8_t dt;
    if (!_cliDataTypeFromStr(val, dt)) { _cliErr("type must be u16/i16/u32/i32/f32"); return; }
    s.dataType = dt;
  }
  else if (field == "wo") s.wordOrder = (val == "lo") ? MB_WORD_LOW_FIRST : MB_WORD_HIGH_FIRST;
  else if (field == "ro") s.respRegOffset = (uint8_t)constrain(val.toInt(), 0, 15);
  else if (field == "scale") s.scale = val.toFloat();
  else if (field == "offset") s.offset = val.toFloat();
  else if (field == "dec") s.decimals = val.toInt();
  else if (field == "vol") s.volumeEnabled = _cliBoolVal(val);
  else if (field == "cap") s.capacity = val.toFloat();
  else if (field == "capu") s.capacityUnit = val;
  else if (field == "vz") s.volZeroLevel = val.toFloat();
  else if (field == "vm") s.volMaxLevel = val.toFloat();
  else { _cliErr("unknown field: " + field); return; }
  _cliOk("sensor " + String(i) + " " + field);
}

static void _cliCs(String* tok, int n) {
  if (n < 2) { _cliCsHelp(); return; }
  if (tok[1] == "h" || tok[1] == "help") { _cliCsHelp(); return; }
  if (tok[1] == "list") { _cliCsList(); return; }
  if (tok[1] == "get") {
    if (n < 3) { _cliErr("usage: cs get <n>"); return; }
    int i = tok[2].toInt();
    if (i < 0 || i >= MAX_SENSORS) { _cliErr("slot out of range 0-15"); return; }
    _cliCsPrint(i);
    return;
  }
  if (tok[1] == "en" || tok[1] == "dis") {
    if (n < 3) { _cliErr("usage: cs en|dis <n>"); return; }
    int i = tok[2].toInt();
    if (i < 0 || i >= MAX_SENSORS) { _cliErr("slot out of range 0-15"); return; }
    cfg.sensors[i].enabled = (tok[1] == "en");
    _cliOk(String("sensor ") + i + (tok[1] == "en" ? " enabled" : " disabled"));
    return;
  }
  if (tok[1] == "set") {
    if (n < 5) { _cliErr("usage: cs set <n> <field> <value>"); return; }
    int i = tok[2].toInt();
    if (i < 0 || i >= MAX_SENSORS) { _cliErr("slot out of range 0-15"); return; }
    _cliCsSet(i, tok[3], tok[4]);
    return;
  }
  _cliErr("unknown cs subcommand, try `cs h`");
}

// ---- cc: CAN signal config -------------------------------------------------
static void _cliCcPrint(int i) {
  CanSignalConfig& s = cfg.canSignals[i];
  Serial.printf("\n=== CAN signal slot %d ===\n", i);
  Serial.printf("enabled : %s\n", s.enabled ? "1" : "0");
  Serial.printf("name    : %s\n", s.name.c_str());
  Serial.printf("kind    : %s\n", s.kind.c_str());
  Serial.printf("unit    : %s\n", s.unit.c_str());
  Serial.printf("id      : 0x%lX (%lu)\n", (unsigned long)s.canId, (unsigned long)s.canId);
  Serial.printf("ext     : %s\n", s.extended ? "1" : "0");
  Serial.printf("bo      : %d\n", s.byteOffset);
  Serial.printf("bl      : %d\n", s.byteLen);
  Serial.printf("be      : %s\n", s.bigEndian ? "1" : "0");
  Serial.printf("sv      : %s\n", s.signedVal ? "1" : "0");
  Serial.printf("scale   : %g\n", s.scale);
  Serial.printf("offset  : %g\n", s.offset);
  Serial.printf("dec     : %d\n", s.decimals);
}

static void _cliCcList() {
  Serial.println("\n=== CAN signal slots ===");
  for (int i = 0; i < MAX_CAN_SIGNALS; i++) {
    CanSignalConfig& s = cfg.canSignals[i];
    if (!s.enabled && s.name.isEmpty()) continue;
    Serial.printf("  c%-2d [%s] %-20s id=0x%lX\n",
      i, s.enabled ? "on " : "off", s.name.c_str(), (unsigned long)s.canId);
  }
}

static void _cliCcSet(int i, const String& field, const String& val) {
  CanSignalConfig& s = cfg.canSignals[i];
  if (field == "name") s.name = val;
  else if (field == "kind") s.kind = val;
  else if (field == "unit") s.unit = val;
  else if (field == "id") {
    // accepts decimal or 0x-prefixed hex
    s.canId = (uint32_t)strtoul(val.c_str(), nullptr, 0);
  }
  else if (field == "ext") s.extended = _cliBoolVal(val);
  else if (field == "bo") s.byteOffset = (uint8_t)constrain(val.toInt(), 0, 7);
  else if (field == "bl") s.byteLen = (uint8_t)constrain(val.toInt(), 1, 4);
  else if (field == "be") s.bigEndian = _cliBoolVal(val);
  else if (field == "sv") s.signedVal = _cliBoolVal(val);
  else if (field == "scale") s.scale = val.toFloat();
  else if (field == "offset") s.offset = val.toFloat();
  else if (field == "dec") s.decimals = val.toInt();
  else { _cliErr("unknown field: " + field); return; }
  _cliOk("CAN signal " + String(i) + " " + field);
}

static void _cliCc(String* tok, int n) {
  if (n < 2) { _cliCcHelp(); return; }
  if (tok[1] == "h" || tok[1] == "help") { _cliCcHelp(); return; }
  if (tok[1] == "list") { _cliCcList(); return; }
  if (tok[1] == "get") {
    if (n < 3) { _cliErr("usage: cc get <n>"); return; }
    int i = tok[2].toInt();
    if (i < 0 || i >= MAX_CAN_SIGNALS) { _cliErr("slot out of range 0-15"); return; }
    _cliCcPrint(i);
    return;
  }
  if (tok[1] == "en" || tok[1] == "dis") {
    if (n < 3) { _cliErr("usage: cc en|dis <n>"); return; }
    int i = tok[2].toInt();
    if (i < 0 || i >= MAX_CAN_SIGNALS) { _cliErr("slot out of range 0-15"); return; }
    cfg.canSignals[i].enabled = (tok[1] == "en");
    _cliOk(String("CAN signal ") + i + (tok[1] == "en" ? " enabled" : " disabled"));
    return;
  }
  if (tok[1] == "set") {
    if (n < 5) { _cliErr("usage: cc set <n> <field> <value>"); return; }
    int i = tok[2].toInt();
    if (i < 0 || i >= MAX_CAN_SIGNALS) { _cliErr("slot out of range 0-15"); return; }
    _cliCcSet(i, tok[3], tok[4]);
    return;
  }
  _cliErr("unknown cc subcommand, try `cc h`");
}

// ---- bs: live RS485 bus scan ------------------------------------------------
// Scans at whatever baud is currently active in RAM (cfg.modbusBaud, as
// last set/loaded) — does NOT touch saved config, does NOT change baud.
// Blocking for a few hundred ms per address (700ms timeout x up to 2
// retries per modbusScanSlaves()), holds modbusBusMutex for the duration
// so it doesn't collide with the poll task.
static void _cliBs(String* tok, int n) {
  if (n >= 2 && (tok[1] == "h" || tok[1] == "help")) { _cliBsHelp(); return; }
  int maxAddr = 16;
  if (n >= 2) maxAddr = tok[1].toInt();
  if (maxAddr < 1) maxAddr = 1;
  if (maxAddr > 247) maxAddr = 247;

  Serial.printf("[cli] Scanning addresses 1-%d at %ld baud (current config, live bus)...\n",
    maxAddr, cfg.modbusBaud);
  if (xSemaphoreTake(modbusBusMutex, pdMS_TO_TICKS(2000)) != pdTRUE) {
    _cliErr("bus busy, try again");
    return;
  }
  int hits = 0;
  modbusScanSlaves(maxAddr, [&](int addr, int fc) {
    Serial.printf("[cli]   FOUND: slave=%d answered FC%02d\n", addr, fc);
    hits++;
  }, 700, false); // verbose=false — bs/pr's own printf lines are enough; no need for the raw TX/RX dump on top
  xSemaphoreGive(modbusBusMutex);
  Serial.printf("[cli] Scan done: %d found out of %d addresses probed.\n", hits, maxAddr);
}

// ---- pr: probe one register right now --------------------------------------
static void _cliPr(String* tok, int n) {
  if (n >= 2 && (tok[1] == "h" || tok[1] == "help")) { _cliPrHelp(); return; }
  if (n < 4) { _cliErr("usage: pr <slave> <fc> <reg> [type]"); return; }

  uint8_t slaveId = (uint8_t)tok[1].toInt();
  uint8_t funcCode = (uint8_t)tok[2].toInt();
  if (funcCode != 3 && funcCode != 4) { _cliErr("fc must be 3 or 4"); return; }
  uint16_t regAddr = (uint16_t)strtol(tok[3].c_str(), nullptr, 0); // accepts 0x-hex or decimal
  uint8_t dataType = MB_UINT16;
  if (n >= 5 && !_cliDataTypeFromStr(tok[4], dataType)) { _cliErr("type must be u16/i16/u32/i32/f32"); return; }

  uint8_t regCount = modbusRegCount(dataType);
  uint16_t regs[2] = {0, 0};
  uint8_t actualSlaveId = 0;

  Serial.printf("[cli] Probing slave=%d fc=%d reg=%d type=%s (verbose TX/RX below)...\n",
    slaveId, funcCode, regAddr, tok[4].c_str());
  if (xSemaphoreTake(modbusBusMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    _cliErr("bus busy, try again");
    return;
  }
  int rc = modbusReadRegs(slaveId, funcCode, regAddr, regCount, regs, true, 600, &actualSlaveId, 0);
  xSemaphoreGive(modbusBusMutex);

  if (rc == MB_OK) {
    float decoded = modbusDecodeValue(regs, dataType, MB_WORD_HIGH_FIRST);
    Serial.printf("[cli] OK: raw regs=[%u", regs[0]);
    if (regCount > 1) Serial.printf(", %u", regs[1]);
    Serial.printf("] decoded=%g\n", decoded);
  } else {
    const char* why = (rc == MB_TIMEOUT) ? "timeout — no response" :
                      (rc == MB_CRC_ERROR) ? "CRC error" : "bad response";
    Serial.printf("[cli] FAIL: %s\n", why);
    if (rc == MB_BAD_RESPONSE && actualSlaveId != 0 && actualSlaveId != slaveId) {
      Serial.printf("[cli]   (but got a CRC-valid reply FROM address %d instead of %d — sensor is alive, just answering as a different address)\n",
        actualSlaveId, slaveId);
    }
  }
}

// ---- top-level dispatch -----------------------------------------------------
static void _cliDispatch(const String& line) {
  String trimmed = line;
  trimmed.trim();
  if (trimmed.isEmpty()) return;

  String tok[6];
  int n = _cliTokenize(trimmed, tok, 6);
  if (n == 0) return;

  String cmd = tok[0];
  cmd.toLowerCase();

  if (cmd == "h" || cmd == "help" || cmd == "?") { _cliHelp(); }
  else if (cmd == "st" || cmd == "status") { _cliStatus(); }
  else if (cmd == "lv" || cmd == "live") { _cliLive(); }
  else if (cmd == "sv" || cmd == "save") {
    prefs.begin("rigmod", false);
    saveConfig(prefs, cfg);
    prefs.end();
    _cliDirty = false;
    Serial.println("[cli] Saved to NVS.");
  }
  else if (cmd == "rb" || cmd == "reboot") {
    Serial.println("[cli] Rebooting...");
    delay(200);
    ESP.restart();
  }
  else if (cmd == "cm") { _cliCm(tok, n); }
  else if (cmd == "cw") { _cliCw(tok, n); }
  else if (cmd == "cs") { _cliCs(tok, n); }
  else if (cmd == "cc") { _cliCc(tok, n); }
  else if (cmd == "bs") { _cliBs(tok, n); }
  else if (cmd == "pr") { _cliPr(tok, n); }
  else { Serial.println("[cli] Unknown command \"" + cmd + "\" — type `h` for help"); }
}

// ---- public entry points ----------------------------------------------------
void cliInit() {
  Serial.println("[cli] Serial CLI ready — type `h` for commands.");
}

// Call every loop() iteration. Non-blocking — just drains whatever's
// already sitting in the Serial RX buffer, one line at a time.
void cliPoll() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (_cliBuf.length() > 0) {
        _cliDispatch(_cliBuf);
        _cliBuf = "";
      }
    } else if (_cliBuf.length() < 200) {
      _cliBuf += c;
    }
  }
}
