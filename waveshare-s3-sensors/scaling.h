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
//
// CHANGED 2026-09-17 (Sarah/Gerald): volume used to be a straight
// frac * (a single, pre-computed capacity number). Now it's real
// rectangular-tank geometry: frac -> a water HEIGHT (meters), then
// length x width x height -> volume (m3), then converted to the
// configured display unit. This is the standard tank-gauging approach
// for a rectangular tank with a top-mounted level sensor -- volume scales
// linearly with height for a constant cross-section, so frac-of-range
// applied to the tank's total usable height gives the true water height
// directly; no lookup table or non-linear correction needed (that would
// only be a concern for a tank with a non-constant cross-section, e.g. a
// cylinder lying on its side -- not the case here, Sarah's rigs use
// rectangular tanks).
// =============================================================================
#pragma once
#include "config.h"

// US gallon, matching the "gal" convention already used by capacityUnit on
// every other rig-module variant (see their capacityUnit comments: "m3" or
// "gal (US gallons)"). 1 m3 = 264.172052358148 US gal (exact, since a US
// gallon is defined as exactly 231 cubic inches).
static const float M3_TO_US_GALLONS = 264.172052358148f;

struct VolumeReading {
  bool   hasValue = false;
  float  value    = 0.0f;
  String unit     = "m3";
  String status   = "disabled";
  // Full-tank volume (length x width x usable height range), in the SAME
  // unit as `value`/`unit` above. Added 2026-09-17 alongside the
  // capacity->geometry change: the Pi-side dashboard (rig-modules.js's
  // tank fill-% hero widget, module-traces.js's axis-max default) expects
  // a "capacity" number in the wire payload to compute fill % / scale the
  // trends axis against -- that used to be the single stored capacity
  // config value verbatim. Now that it's derived from geometry instead of
  // typed in directly, it's computed here (once, alongside `value`) so
  // waveshare-s3-sensors.ino's buildPayload() can still send a `capacity`
  // field without duplicating the geometry math itself. 0 whenever volume
  // is disabled/misconfigured, same as `value`.
  float  capacity  = 0.0f;
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
  // Both tank footprint dimensions must be real, positive numbers — a
  // zero-area footprint can't sensibly produce a volume (and would
  // silently report 0 forever, indistinguishable from "genuinely empty",
  // if we let it through instead of bailing out to "disabled" here).
  if (s.tankLengthM <= 0.0f || s.tankWidthM <= 0.0f) return;
  // Only reject an exact match (divide-by-zero) — volMaxLevel is allowed
  // to be LESS than volZeroLevel on purpose. Distance-based level sensors
  // (e.g. a radar unit reporting air-gap-to-surface, like the SM7779)
  // report a SMALLER raw value as the tank fills, so the natural config
  // is volZeroLevel=large (empty) / volMaxLevel=small (full). The frac
  // formula below is a symmetric ratio and produces the right 0..1 curve
  // either way — this used to require volMaxLevel > volZeroLevel, which
  // silently disabled Tank Volume for every inverted-direction sensor.
  if (s.volMaxLevel == s.volZeroLevel) return;

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

  // frac-of-range -> water height (meters). volZeroLevel/volMaxLevel are
  // the sensor's OWN reading (e.g. radar air-gap distance) at empty/full,
  // in the same units as reading.value — by existing convention (unchanged
  // from before this fix) that's meters, so their absolute difference IS
  // the tank's usable height range directly, no unit conversion needed.
  float heightRangeM = fabs(s.volMaxLevel - s.volZeroLevel);
  float waterHeightM = frac * heightRangeM;

  // Rectangular tank: volume = footprint area x water height.
  float volM3 = s.tankLengthM * s.tankWidthM * waterHeightM;
  float capM3 = s.tankLengthM * s.tankWidthM * heightRangeM; // full-tank volume (height range, not just current)
  bool  toGal = (s.capacityUnit == "gal");
  float vol = toGal ? (volM3 * M3_TO_US_GALLONS) : volM3;
  float cap = toGal ? (capM3 * M3_TO_US_GALLONS) : capM3;

  out.value    = round(vol * 100.0f) / 100.0f;
  out.capacity = round(cap * 100.0f) / 100.0f;
  out.hasValue = true;
  out.status   = "ok";
}
