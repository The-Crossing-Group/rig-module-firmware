// =============================================================================
// scaling.h — Per-channel scaling: raw Modbus → mA → engineering value
// Generic module: no per-"kind" special casing — every channel is just a
// linear mA→engineering mapping, rounded to 2 decimal places.
// =============================================================================
#pragma once
#include "config.h"

// Scale one channel
// raw: 16-bit value from FC04 (in mode 3: 4000-20000 = 4-20 mA in µA units)
void scaleChannel(int ch, uint16_t raw, ModuleConfig& cfg, ChannelReading& out) {
  out.valid = true;

  // Convert raw to mA (mode 3: value is µA, divide by 1000)
  float mA = raw / 1000.0f;
  out.mA = mA;

  // Open circuit check (disconnected loop reads near 0)
  if (mA < 3.5f) {
    out.hasValue = false;
    out.value    = 0.0f;
    out.status   = "open";
    return;
  }

  // Over-range check
  if (mA > 20.5f) {
    out.hasValue = false;
    out.value    = 0.0f;
    out.status   = "over";
    return;
  }

  ChannelConfig& c = cfg.ch[ch];
  float frac;

  // Use captured zero/max cal if available
  if (c.zeroRaw >= 0 && c.maxRaw > c.zeroRaw) {
    frac = (float)((int)raw - c.zeroRaw) / (float)(c.maxRaw - c.zeroRaw);
  } else {
    // Fall back to mA linear map
    float range = c.maMax - c.maMin;
    if (range < 0.001f) range = 16.0f; // safety
    frac = (mA - c.maMin) / range;
  }

  // Clamp fraction to valid display range (don't clamp for fault detection)
  if (frac < 0.0f) frac = 0.0f;
  if (frac > 1.0f) frac = 1.0f;

  float val = c.engMin + frac * (c.engMax - c.engMin);
  val = round(val * 100.0f) / 100.0f;    // 0.01 precision, generic across all kinds

  out.value    = val;
  out.hasValue = true;
  out.status   = "ok";
}

// =============================================================================
// TANK VOLUME (optional derived value, spec-tank-modules.md §4b)
//
// One linear map per module: volZeroLevel (eng units, e.g. m) -> volume 0,
// volMaxLevel -> volume = capacity. Clamped so volume never goes negative or
// over capacity, but the underlying channel status (open/over/stale) still
// wins — a fault on the level channel means the volume is unknown too, not
// a bogus number.
//
// Disabled (out.hasValue=false) unless capacity > 0 and volMaxLevel >
// volZeroLevel — i.e. it only turns on once someone actually configures it
// on /config, so a plain level module never suddenly grows a volume field.
// =============================================================================
struct VolumeReading {
  bool   hasValue = false;
  float  value    = 0.0f;   // in cfg.capacityUnit
  String status   = "disabled";
};

void computeTankVolume(ModuleConfig& cfg, ChannelReading* readings, VolumeReading& out) {
  out.hasValue = false;
  out.value    = 0.0f;
  out.status   = "disabled";

  if (cfg.capacity <= 0.0f) return; // feature not configured
  if (cfg.volMaxLevel <= cfg.volZeroLevel) return; // needs a real range

  int ch = cfg.volumeLevelCh;
  if (ch < 0 || ch > 7) return;
  ChannelReading& lvl = readings[ch];

  // Level channel itself faulted (open/over) or hasn't produced a value yet
  // -> volume is unknown, not zero. Mirrors the tank spec's fault handling.
  if (!lvl.valid || !lvl.hasValue) {
    out.status = (lvl.valid ? lvl.status : "stale");
    return;
  }

  float frac = (lvl.value - cfg.volZeroLevel) / (cfg.volMaxLevel - cfg.volZeroLevel);
  if (frac < 0.0f) frac = 0.0f; // below empty reference -> report empty, not negative
  if (frac > 1.0f) frac = 1.0f; // above full reference -> cap at capacity, don't overshoot

  float vol = cfg.capacity * frac;
  out.value    = round(vol * 100.0f) / 100.0f;
  out.hasValue = true;
  out.status   = "ok";
}
