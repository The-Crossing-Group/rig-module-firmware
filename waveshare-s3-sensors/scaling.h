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
// CHANGED 2026-09-17 (Sarah/Gerald, two passes same day):
//   1st pass: volume used to be a straight frac * (a single, pre-computed
//   capacity number) -> replaced with real rectangular-tank geometry
//   (length x width x height).
//   2nd pass: the two-point calibration (frac-of-(Value @ Empty..Value @
//   Full)) that 1st pass still used to derive water height is GONE.
//   Sarah's actual words: "i dont really understand the volume @ empty
//   options... remember this is for the tank volume" -- she doesn't want
//   a calibration step at all. Her radar sensors report a plain distance
//   in meters, straight down from the sensor to the water surface
//   (closer = more water). Water height is now computed DIRECTLY:
//   `waterHeightM = tankHeightM - reading.value`, no frac, no
//   calibration, no min/max readings to configure. tankHeightM is a
//   single physical measurement (tape-measure the empty tank, floor to
//   sensor face) entered once, same as length/width.
// This is still the standard tank-gauging approach for a rectangular tank
// with a top-mounted level sensor -- volume scales linearly with height
// for a constant cross-section (not a concern here; that only matters for
// a non-constant cross-section, e.g. a cylinder lying on its side --
// Sarah's rigs use rectangular tanks).
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
// other rig-module variant). Assumed to be a distance in meters, straight
// down from a top-mounted sensor to the water surface (radar air-gap
// convention) — see this file's header comment for the current model.
void computeSensorVolume(SensorConfig& s, SensorReading& reading, VolumeReading& out) {
  out.hasValue = false;
  out.value    = 0.0f;
  out.unit     = s.capacityUnit;
  out.status   = "disabled";

  if (!s.volumeEnabled) return;
  // All three tank dimensions must be real, positive numbers — a
  // zero-area footprint or zero height can't sensibly produce a volume
  // (and would silently report 0 forever, indistinguishable from
  // "genuinely empty", if we let it through instead of bailing out to
  // "disabled" here).
  if (s.tankLengthM <= 0.0f || s.tankWidthM <= 0.0f || s.tankHeightM <= 0.0f) return;

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

  // Direct calc, no calibration step: water height = tank height minus
  // the sensor's own (air-gap distance) reading. Clamped to the tank's
  // physical range — a reading beyond either end (sensor noise, or the
  // tank genuinely empty/overfull) shouldn't produce a negative or
  // over-100%-full volume.
  float waterHeightM = s.tankHeightM - reading.value;
  if (waterHeightM < 0.0f) waterHeightM = 0.0f;
  if (waterHeightM > s.tankHeightM) waterHeightM = s.tankHeightM;

  // Rectangular tank: volume = footprint area x water height.
  float volM3 = s.tankLengthM * s.tankWidthM * waterHeightM;
  float capM3 = s.tankLengthM * s.tankWidthM * s.tankHeightM; // full-tank volume
  bool  toGal = (s.capacityUnit == "gal");
  float vol = toGal ? (volM3 * M3_TO_US_GALLONS) : volM3;
  float cap = toGal ? (capM3 * M3_TO_US_GALLONS) : capM3;

  out.value    = round(vol * 100.0f) / 100.0f;
  out.capacity = round(cap * 100.0f) / 100.0f;
  out.hasValue = true;
  out.status   = "ok";
}
