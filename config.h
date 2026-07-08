// =============================================================================
// config.h — Rig Module configuration structures and NVS load/save
// =============================================================================
#pragma once
#include <Arduino.h>

#define FW_VERSION "rig-module-1.3.0"

// =============================================================================
// WIFI — no hardcoded network anymore.
// If no WiFi is saved in NVS, the device boots into its own setup AP
// ("RigModule-XXXXXX" / "modulesetup") and serves a config page at
// http://192.168.4.1/wifi so you can point it at whatever site network
// it's deployed on, without recompiling. Once saved, it reconnects to
// that network on every boot going forward.
// =============================================================================
#include <WiFi.h>
#include <Preferences.h>
#include <esp_efuse.h>
#include <esp_mac.h>

// Per-channel config. This module reports RAW ENGINEERING VALUES ONLY —
// no volume/mud-weight/tank-specific derived math. Any higher-level
// interpretation (tank fill %, mud weight, etc.) happens elsewhere
// (rig UI / Pi side), configured against these raw channel values.
struct ChannelConfig {
  // Plug-and-play: every channel reports by default. Nothing to configure
  // to see all 8 — uncheck a channel on /channels only if you want to
  // hide one that's genuinely not wired up (e.g. to declutter the rig UI).
  bool   enabled  = true;
  String name     = "";
  String kind     = "";        // fully free-text: "level","pressure","temp","flow","rpm",... anything
  String unit     = "";
  float  maMin    = 4.0f;
  float  maMax    = 20.0f;
  float  engMin   = 0.0f;
  float  engMax   = 1.0f;
  int    zeroRaw  = -1;        // -1 = not calibrated
  int    maxRaw   = -1;
};

// Full module config
struct ModuleConfig {
  String moduleId       = "";    // built from MAC: MODULE-ABC123
  String moduleName     = "";
  String moduleType     = "generic";  // free text — sent as payload "type"; rig-modules.html
                                       // special-cases "tank"/"pump" for richer rendering,
                                       // anything else (e.g. "pressure", "drill") just shows
                                       // as a plain badge + generic tile view. Configurable so
                                       // a module isn't stuck reporting as "generic" once it's
                                       // deployed on an actual drill.
  String description    = "";
  int    modbusSlaveId  = 1;
  long   modbusBaud     = 9600;  // Waveshare 8AI (B) default; SDSIN clone default is 4800
  int    pollIntervalS  = 3;
  String piHost         = "";
  // Plug-and-play: every rig's Pi logger uses the same shared token
  // (matches config.json's rig_token on every deployed rig). No manual
  // entry needed out of the box — /config still lets it be overridden
  // per-module if a rig ever needs a different token.
  String rigToken       = "7804991970";
  String wifiSSID       = "";
  String wifiPass       = "";

  // --- Tank volume (optional derived calc) ---------------------------------
  // Linear level -> volume map, per spec-tank-modules.md §4b: pick whichever
  // channel is feeding the level reading, then a straight-line map between
  // "empty" and "full" levels to 0..capacity. Disabled (omitted from the
  // payload) unless capacity > 0 and volMaxLevel > volZeroLevel.
  int    volumeLevelCh  = 0;      // which of the 8 channels (0-7) is the level input
  float  capacity       = 0.0f;   // nominal full-tank volume; 0 = feature disabled
  String capacityUnit   = "m3";   // "m3" or "gal" (US gallons)
  float  volZeroLevel   = 0.0f;   // level reading (in that channel's eng unit) = empty
  float  volMaxLevel    = 1.0f;   // level reading = full (== capacity)

  ChannelConfig ch[8];
};

// Per-channel live reading
struct ChannelReading {
  bool   valid    = false;
  bool   hasValue = false;
  float  mA       = 0.0f;
  float  value    = 0.0f;
  String status   = "stale";
};

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
  c.modbusSlaveId = p.getInt("mbSlave", 1);
  c.modbusBaud    = p.getLong("mbBaud", 9600);
  c.pollIntervalS = p.getInt("pollInt", 3);
  c.piHost        = p.getString("piHost", "");
  c.rigToken      = p.getString("rigToken", "7804991970");
  c.wifiSSID      = p.getString("wifiSSID", "");
  c.wifiPass      = p.getString("wifiPass", "");

  c.volumeLevelCh = p.getInt("volCh", 0);
  c.capacity      = p.getFloat("cap", 0.0f);
  c.capacityUnit  = p.getString("capUnit", "m3");
  c.volZeroLevel  = p.getFloat("volZLvl", 0.0f);
  c.volMaxLevel   = p.getFloat("volMLvl", 1.0f);

  for (int i = 0; i < 8; i++) {
    String pre = "ch" + String(i);
    // Default true (plug-and-play) for channels never explicitly saved —
    // matches ChannelConfig's struct default above.
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
  }
}

// Save all config to NVS
void saveConfig(Preferences& p, ModuleConfig& c) {
  p.begin("rigmod", false);
  p.putString("modName", c.moduleName);
  p.putString("modType", c.moduleType);
  p.putString("desc", c.description);
  p.putInt("mbSlave", c.modbusSlaveId);
  p.putLong("mbBaud", c.modbusBaud);
  p.putInt("pollInt", c.pollIntervalS);
  p.putString("piHost", c.piHost);
  p.putString("rigToken", c.rigToken);
  p.putString("wifiSSID", c.wifiSSID);
  p.putString("wifiPass", c.wifiPass);

  p.putInt("volCh", c.volumeLevelCh);
  p.putFloat("cap", c.capacity);
  p.putString("capUnit", c.capacityUnit);
  p.putFloat("volZLvl", c.volZeroLevel);
  p.putFloat("volMLvl", c.volMaxLevel);

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
  }
  p.end();
}
