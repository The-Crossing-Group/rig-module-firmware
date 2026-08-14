// =============================================================================
// rpm.h — GPIO interrupt-based pulse counter / RPM measurement
//
// Why this exists as its own file, separate from Modbus entirely: RPM/
// rotation speed cannot be measured reliably by polling a register over
// RS485 — a poll loop has dead time between polls where pulses just get
// missed, and there's no polling rate that's fast enough to be safe across
// "anything from a slow drill string to a fast-spinning shaft" (aliasing —
// you can get a plausible-looking but WRONG number with no indication it's
// wrong). Hardware interrupt counting on a GPIO pin is the only approach
// that's genuinely rate-independent — same code, same accuracy, whether
// the shaft turns at 2 RPM or 20,000 RPM.
//
// Method: measure the TIME BETWEEN consecutive pulses (period), not a
// pulse count over a fixed window. RPM = 60 / (period_seconds *
// pulsesPerRev). This updates on every single pulse (so it's immediately
// responsive at low RPM, unlike a 1-second counting window which would be
// slow to update and jumpy at low speed) and naturally has no upper speed
// limit other than the debounce floor. If no pulse arrives for
// `timeoutS`, we report 0 RPM / "stopped" rather than freezing on
// whatever the last reading was — a real stopped shaft should read 0, not
// hold a stale nonzero number forever.
//
// Two independent channels, one per available GPIO (this board breaks out
// GPIO1 + GPIO2 on its screw terminal specifically for general-purpose
// sensor input — not used by RS485 or CAN).
// =============================================================================
#pragma once
#include <Arduino.h>
#include "config.h"

// Pins — defined here rather than pulled from the .ino's RS485/CAN pin
// block since these are logically a separate subsystem. See board
// schematic / Waveshare wiki "Interfaces" table: GPIO1 and GPIO2 are the
// two pins broken out alongside the CAN terminal for exactly this kind of
// general external sensor wiring.
#define RPM1_PIN 1
#define RPM2_PIN 2

// Per-channel ISR state. Everything the ISR touches is `volatile` and
// kept as plain fixed-size types (no String, no dynamic alloc) — ISRs on
// ESP32 run with interrupts still able to preempt other code, and doing
// anything heap-related or blocking in one is a fast route to a crash or
// a wedged bus.
struct RpmIsrState {
  volatile uint32_t lastPulseMicros   = 0;   // micros() timestamp of the last accepted pulse
  volatile uint32_t lastPeriodMicros  = 0;   // time between the two most recent accepted pulses
  volatile bool     havePulse         = false; // true once at least one pulse has been accepted
  int               debounceMicros    = 2000;  // set from cfg.rpm[i].debounceMs at init/config-save time
};

static RpmIsrState _rpmIsr[2];

// ISR — kept as short as physically possible: read micros(), debounce
// check, store period, done. No Serial, no NVS, no String work.
void IRAM_ATTR _rpmIsr0() {
  uint32_t now = micros();
  uint32_t last = _rpmIsr[0].lastPulseMicros;
  uint32_t delta = now - last; // unsigned subtraction handles micros() rollover correctly
  if (_rpmIsr[0].havePulse && delta < (uint32_t)(_rpmIsr[0].debounceMicros)) return; // debounce/contact-bounce reject
  if (_rpmIsr[0].havePulse) _rpmIsr[0].lastPeriodMicros = delta;
  _rpmIsr[0].lastPulseMicros = now;
  _rpmIsr[0].havePulse = true;
}
void IRAM_ATTR _rpmIsr1() {
  uint32_t now = micros();
  uint32_t last = _rpmIsr[1].lastPulseMicros;
  uint32_t delta = now - last;
  if (_rpmIsr[1].havePulse && delta < (uint32_t)(_rpmIsr[1].debounceMicros)) return;
  if (_rpmIsr[1].havePulse) _rpmIsr[1].lastPeriodMicros = delta;
  _rpmIsr[1].lastPulseMicros = now;
  _rpmIsr[1].havePulse = true;
}

static const int _rpmPins[2] = { RPM1_PIN, RPM2_PIN };

// Call once from setup(), AFTER cfg has been loaded from NVS. Safe to call
// again after a config save to pick up new pulsesPerRev/debounce/enabled
// without a reboot — detaches+reattaches as needed.
void rpmInit(ModuleConfig& cfg) {
  for (int i = 0; i < 2; i++) {
    detachInterrupt(_rpmPins[i]); // no-op if nothing was attached yet
    _rpmIsr[i].debounceMicros = max(0, cfg.rpm[i].debounceMs) * 1000;
    if (!cfg.rpm[i].enabled) continue;
    // Internal pull-up: the opto-isolator output side (recommended
    // wiring — see README) pulls the pin LOW when triggered, floating/
    // high otherwise. Using the ESP32's internal pull-up means no
    // external pull-up resistor is required for the isolator's output.
    pinMode(_rpmPins[i], INPUT_PULLUP);
    _rpmIsr[i].havePulse = false;
    _rpmIsr[i].lastPeriodMicros = 0;
    attachInterrupt(digitalPinToInterrupt(_rpmPins[i]), i == 0 ? _rpmIsr0 : _rpmIsr1, FALLING);
  }
}

// Call from the poll loop (or any regular cadence) to turn the latest ISR
// state into an RPM reading. Cheap — just reads a few volatiles and does
// float math, no bus I/O, safe to call as often as you like.
void rpmCompute(int i, ModuleConfig& cfg, RpmReading& out) {
  if (!cfg.rpm[i].enabled) {
    out.valid = false;
    out.rpm = 0.0f;
    out.status = "stale";
    return;
  }

  // Snapshot volatiles once (they can change under us between reads,
  // this keeps the math internally consistent for this call).
  bool have = _rpmIsr[i].havePulse;
  uint32_t lastPulse = _rpmIsr[i].lastPulseMicros;
  uint32_t lastPeriod = _rpmIsr[i].lastPeriodMicros;

  if (!have) {
    out.valid = false;
    out.rpm = 0.0f;
    out.status = "stale"; // never seen a pulse since enable/boot — not the same as "confirmed stopped"
    return;
  }

  uint32_t sinceLastPulse = micros() - lastPulse; // unsigned subtraction, rollover-safe
  float timeoutMicros = cfg.rpm[i].timeoutS * 1000000.0f;
  if ((float)sinceLastPulse > timeoutMicros) {
    // No pulse for longer than the configured timeout — genuinely stopped,
    // report 0 rather than holding onto the last nonzero RPM forever.
    out.valid = true;
    out.rpm = 0.0f;
    out.status = "stopped";
    return;
  }

  if (lastPeriod == 0) {
    // Only one pulse seen so far — not enough to compute a period yet.
    out.valid = false;
    out.rpm = 0.0f;
    out.status = "stale";
    return;
  }

  int ppr = max(1, cfg.rpm[i].pulsesPerRev);
  float periodS = lastPeriod / 1000000.0f;
  out.rpm = 60.0f / (periodS * ppr);
  out.valid = true;
  out.status = "ok";
}
