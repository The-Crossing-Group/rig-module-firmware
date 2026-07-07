// =============================================================================
// config.h — Rig Module configuration structures and NVS load/save
// =============================================================================
#pragma once
#include <Arduino.h>

#define FW_VERSION "rig-module-1.0.6"

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
  bool   enabled  = false;
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
  String description    = "";
  int    modbusSlaveId  = 1;
  int    pollIntervalS  = 3;
  String piHost         = "";
  String rigToken       = "";
  String wifiSSID       = "";
  String wifiPass       = "";

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
  c.description   = p.getString("desc", "");
  c.modbusSlaveId = p.getInt("mbSlave", 1);
  c.pollIntervalS = p.getInt("pollInt", 3);
  c.piHost        = p.getString("piHost", "");
  c.rigToken      = p.getString("rigToken", "");
  c.wifiSSID      = p.getString("wifiSSID", "");
  c.wifiPass      = p.getString("wifiPass", "");

  for (int i = 0; i < 8; i++) {
    String pre = "ch" + String(i);
    c.ch[i].enabled = p.getBool((pre + "en").c_str(), false);
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
  p.putString("desc", c.description);
  p.putInt("mbSlave", c.modbusSlaveId);
  p.putInt("pollInt", c.pollIntervalS);
  p.putString("piHost", c.piHost);
  p.putString("rigToken", c.rigToken);
  p.putString("wifiSSID", c.wifiSSID);
  p.putString("wifiPass", c.wifiPass);

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
