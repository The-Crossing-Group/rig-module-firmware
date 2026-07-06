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
