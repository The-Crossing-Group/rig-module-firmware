// =============================================================================
// scaling.h — Per-channel scaling: raw Modbus → mA → engineering value
// Generic module: no per-"kind" special casing — every channel is just a
// linear mA→engineering mapping, rounded to 2 decimal places.
// =============================================================================
#pragma once
#include "config.h"

// Scale one channel
// raw: 16-bit value from FC04. Different analog-to-Modbus boards report
// this raw value in different units — Waveshare 8AI (B) in mode 3 reports
// µA (divide by 1000 for mA), while the Eletechsup AMIDJ14 reports
// centi-mA/0.01mA units (divide by 100 for mA). rawDivisor comes from the
// BoardProfile picked by modbusDetectBoard() at boot (modbus.h) so this
// function itself never needs to know which specific board is wired up —
// it just applies whatever divisor was detected.
void scaleChannel(int ch, uint16_t raw, ModuleConfig& cfg, ChannelReading& out, float rawDivisor) {
  out.valid = true;

  float mA = raw / rawDivisor;
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
// Any channel can have "Compute Tank Volume" checked on /channels
// (ChannelConfig.volumeEnabled) — that channel IS the level input for its
// own tank. One linear map per channel: volZeroLevel (eng units, e.g. m) ->
// volume 0, volMaxLevel -> volume = capacity. Clamped so volume never goes
// negative or over capacity, but the underlying channel status (open/over/
// stale) still wins — a fault on the level channel means the volume is
// unknown too, not a bogus number.
//
// Multiple channels can each independently have volumeEnabled — e.g. two
// separate tanks wired to two channels on the same Waveshare/Modbus board.
// computeChannelVolume() computes ONE specific channel's volume; call it
// per-channel (buildPayload() does this to put a "volume" field on every
// channel that has it enabled, and /live shows it as a table column).
// =============================================================================
struct VolumeReading {
  bool   hasValue = false;
  float  value    = 0.0f;   // in this channel's capacityUnit
  String unit     = "m3";
  String status   = "disabled";
};

void computeChannelVolume(int ch, ModuleConfig& cfg, ChannelReading* readings, VolumeReading& out) {
  out.hasValue = false;
  out.value    = 0.0f;
  out.unit     = "m3";
  out.status   = "disabled";

  if (ch < 0 || ch > 7) return;
  ChannelConfig& c = cfg.ch[ch];
  if (!c.volumeEnabled) return;

  out.unit = c.capacityUnit;
  if (c.capacity <= 0.0f) return;               // capacity not set yet
  if (c.volMaxLevel <= c.volZeroLevel) return;   // needs a real range

  ChannelReading& lvl = readings[ch];

  // Level channel itself faulted (open/over) or hasn't produced a value yet
  // -> volume is unknown, not zero. Mirrors the tank spec's fault handling.
  if (!lvl.valid || !lvl.hasValue) {
    out.status = (lvl.valid ? lvl.status : "stale");
    return;
  }

  float frac = (lvl.value - c.volZeroLevel) / (c.volMaxLevel - c.volZeroLevel);
  if (frac < 0.0f) frac = 0.0f; // below empty reference -> report empty, not negative
  if (frac > 1.0f) frac = 1.0f; // above full reference -> cap at capacity, don't overshoot

  float vol = c.capacity * frac;
  out.value    = round(vol * 100.0f) / 100.0f;
  out.hasValue = true;
  out.status   = "ok";
}

// Back-compat wrapper: finds the FIRST channel with volumeEnabled and
// returns its volume. Still used for the single top-level derived.volume +
// capacity fields in buildPayload() (kept for any single-tank consumer
// that reads those instead of the per-channel table) — with multiple tanks
// configured, only the lowest-numbered one appears here; the rest show up
// per-channel via computeChannelVolume() (see the "channels" array/table).
void computeTankVolume(ModuleConfig& cfg, ChannelReading* readings, VolumeReading& out) {
  int ch = -1;
  for (int i = 0; i < 8; i++) {
    if (cfg.ch[i].volumeEnabled) { ch = i; break; }
  }
  if (ch < 0) {
    out.hasValue = false;
    out.value    = 0.0f;
    out.unit     = "m3";
    out.status   = "disabled";
    return;
  }
  computeChannelVolume(ch, cfg, readings, out);
}
