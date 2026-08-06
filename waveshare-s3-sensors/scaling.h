// =============================================================================
// scaling.h — Tank volume derived calc (optional, per-sensor)
//
// Unlike the adapter-board variants, raw->engineering scaling itself now
// happens right in modbus.h (modbusPollSensor: raw*scale+offset) since
// every sensor already reports real engineering units directly (no mA
// current-loop layer to convert through). This file only keeps the
// tank-volume derived math, which is still a useful optional feature: any
// sensor whose value is a level reading can have Compute Tank Volume
// checked to also report a derived volume.
// =============================================================================
#pragma once
#include "config.h"

struct VolumeReading {
  bool   hasValue = false;
  float  value    = 0.0f;
  String unit     = "m3";
  String status   = "disabled";
};

// level: this sensor's OWN latest reading (must be a level-type value for
// this to make sense — that's on the person configuring it, same as every
// other rig-module variant).
void computeSensorVolume(SensorConfig& s, SensorReading& reading, VolumeReading& out) {
  out.hasValue = false;
  out.value    = 0.0f;
  out.unit     = s.capacityUnit;
  out.status   = "disabled";

  if (!s.volumeEnabled) return;
  if (s.capacity <= 0.0f) return;
  if (s.volMaxLevel <= s.volZeroLevel) return;

  // Use the debounced displayStatus, not raw valid/status, for the same
  // reason the sensor's own /sensors and /live display debounces a lone
  // timeout: a sensor with a slow measurement cycle can fail a poll
  // without anything actually being wrong, and the volume gauge next to
  // it shouldn't flap to "stale" every time that happens while the
  // sensor's own status stays showing "ok".
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
