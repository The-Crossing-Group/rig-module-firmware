// =============================================================================
// scaling.h — Merged scaling logic for the mudtank variant.
//
// Two independent scaling paths coexist here:
//   1) Fixed adapter board: raw Modbus register -> mA -> engineering value
//      (scaleChannelCfg/scaleChannel), same as waveshare-s3/. Different
//      boards report the raw value in different units (Waveshare 8AI in
//      µA, Eletechsup AMIDJ14 in 0.01mA) — rawDivisor comes from the
//      BoardProfile picked by modbusDetectBoard() at boot (modbus.h).
//   2) Independent RS485 sensors: raw->engineering scaling happens right
//      in modbus.h (modbusPollSensor: raw*scale+offset) since every
//      sensor already reports real engineering units directly (no mA
//      current-loop layer to convert through) — this file only adds the
//      optional tank-volume derived math for sensors (computeSensorVolume).
//
// Tank volume itself is offered on BOTH paths independently — a fixed
// board channel and an independent sensor can each separately be
// configured as "this is a tank level" and get its own derived volume.
// =============================================================================
#pragma once
#include "config.h"

// ─── Fixed adapter board: raw -> mA -> engineering value ────────────────────

// Shared scaling core — takes a ChannelConfig directly rather than
// indexing into a ModuleConfig, so it works both for the primary board's
// per-channel-configured cfg.ch[] AND for extra/advanced boards (config.h
// ExtraBoardConfig) which don't carry full per-channel calibration — those
// callers pass a shared default-constructed ChannelConfig (standard
// 4-20mA -> 0-1 linear map) instead.
void scaleChannelCfg(uint16_t raw, ChannelConfig& c, ChannelReading& out, float rawDivisor) {
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

// Scale one channel of the PRIMARY board (indexes cfg.ch[ch]).
void scaleChannel(int ch, uint16_t raw, ModuleConfig& cfg, ChannelReading& out, float rawDivisor) {
  scaleChannelCfg(raw, cfg.ch[ch], out, rawDivisor);
}

// =============================================================================
// TANK VOLUME (optional derived value) — shared VolumeReading shape used
// by both the fixed board's channels AND independent sensors below.
// =============================================================================
struct VolumeReading {
  bool   hasValue = false;
  float  value    = 0.0f;   // in this channel/sensor's capacityUnit
  String unit     = "m3";
  String status   = "disabled";
};

// Any fixed-board channel can have "Compute Tank Volume" checked —
// ChannelConfig.volumeEnabled. One linear map per channel: volZeroLevel
// (eng units) -> volume 0, volMaxLevel -> volume = capacity. Multiple
// channels can each independently have volumeEnabled — e.g. two tanks on
// two channels of the same board.
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

  // Level channel itself faulted (open/over) or hasn't produced a value
  // yet -> volume is unknown, not zero.
  if (!lvl.valid || !lvl.hasValue) {
    out.status = (lvl.valid ? lvl.status : "stale");
    return;
  }

  float frac = (lvl.value - c.volZeroLevel) / (c.volMaxLevel - c.volZeroLevel);
  if (frac < 0.0f) frac = 0.0f;
  if (frac > 1.0f) frac = 1.0f;

  float vol = c.capacity * frac;
  out.value    = round(vol * 100.0f) / 100.0f;
  out.hasValue = true;
  out.status   = "ok";
}

// Back-compat wrapper: finds the FIRST fixed-board channel with
// volumeEnabled and returns its volume — used for the single top-level
// derived.volume + capacity fields in buildPayload().
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

// ─── Independent RS485 sensors: tank volume only ────────────────────────────
//
// level: this sensor's OWN latest reading (must be a level-type value for
// this to make sense — that's on the person configuring it).
void computeSensorVolume(SensorConfig& s, SensorReading& reading, VolumeReading& out) {
  out.hasValue = false;
  out.value    = 0.0f;
  out.unit     = s.capacityUnit;
  out.status   = "disabled";

  if (!s.volumeEnabled) return;
  if (s.capacity <= 0.0f) return;
  // Only reject an exact match (divide-by-zero) — volMaxLevel is allowed
  // to be LESS than volZeroLevel on purpose. Distance-based level sensors
  // (e.g. a radar unit reporting air-gap-to-surface, like the SM7779)
  // report a SMALLER raw value as the tank fills, so the natural config
  // is volZeroLevel=large (empty) / volMaxLevel=small (full). The frac
  // formula below is a symmetric ratio and produces the right 0..1 curve
  // either way.
  if (s.volMaxLevel == s.volZeroLevel) return;

  // Use the debounced displayStatus, not raw valid/status — a sensor with
  // a slow measurement cycle can fail a poll without anything actually
  // being wrong, and the volume gauge next to it shouldn't flap to
  // "stale" every time that happens while the sensor's own status stays
  // showing "ok".
  if (!reading.hasValue) {
    out.status = "stale";
    return;
  }
  if (reading.displayStatus != "ok") {
    out.status = reading.displayStatus;
    return;
  }

  float frac = (reading.value - s.volZeroLevel) / (s.volMaxLevel - s.volZeroLevel);
  if (frac < 0.0f) frac = 0.0f;
  if (frac > 1.0f) frac = 1.0f;

  float vol = s.capacity * frac;
  out.value    = round(vol * 100.0f) / 100.0f;
  out.hasValue = true;
  out.status   = "ok";
}
