// =============================================================================
// pulse.h — "Pulse Counter Mode" for the AMIDJ14's digital inputs.
//
// Turns a DI's ON/OFF transitions over time into an RPM reading, using
// the SAME Modbus DI wiring already available — no GPIO, no extra
// hardware. A dedicated background task hammers ONE digital input as
// fast as the RS485 bus allows (much faster than the normal
// cfg.pollIntervalS cycle, which is far too slow to catch every rotation
// reliably) and times the gap between rising edges (OFF->ON transitions)
// to compute RPM = 60 / (period_seconds * pulsesPerRev) — same math as
// any period-based tach, just fed from Modbus reads instead of a GPIO
// interrupt.
//
// Honest limitation, by design of using Modbus instead of a hardware
// interrupt: accuracy is capped by how fast this device can round-trip a
// single-bit FC02 request over RS485, NOT open-ended like a real
// interrupt would be. At typical baud rates that's roughly tens of
// milliseconds per read, i.e. reliable up to roughly the "few hundred
// RPM" range, degrading (and eventually aliasing — a real but WRONG
// reading, not just a slow one) beyond that. _pulsePollHz below is
// measured live and surfaced on the /digital page so it's obvious in
// practice rather than a guess baked into this comment.
// =============================================================================
#pragma once
#include <Arduino.h>
#include "config.h"
#include "modbus.h"

// Defined in waveshare-s3.ino — guards the RS485/Serial2 bus itself so
// this fast-poll task and the main poll task never interleave bytes on
// the wire (same mutex both already coordinate through).
extern SemaphoreHandle_t modbusBusMutex;

struct PulseChState {
  volatile bool     haveLastState    = false;
  volatile bool     lastState        = false;
  volatile bool     havePulse        = false; // seen at least one rising edge since enable
  volatile uint32_t lastRiseMicros   = 0;
  volatile uint32_t lastPeriodMicros = 0;      // time between the two most recent rising edges
};

static PulseChState _pulseState[4];

// Measured actual read rate of the fast-poll loop (Hz) — an EMA, updated
// every successful read, shared across whichever DI(s) are enabled (if
// more than one DI has Pulse Counter Mode on, they round-robin the same
// bus and each one's effective rate is roughly this divided by however
// many are enabled). Surfaced on /digital so the accuracy ceiling is a
// measured fact, not a guess.
static volatile float _pulsePollHz = 0.0f;

// Call after any config change that touches din[].pulseModeEnabled — resets
// the edge-timing state for channels no longer in pulse mode so a stale
// period/timestamp from before can't leak into a reading after
// re-enabling later.
void pulseConfigChanged(ModuleConfig& cfg) {
  for (int i = 0; i < 4; i++) {
    if (!cfg.din[i].pulseModeEnabled) {
      _pulseState[i].haveLastState = false;
      _pulseState[i].havePulse = false;
      _pulseState[i].lastPeriodMicros = 0;
    }
  }
}

// Turn a channel's latest edge-timing state into an RPM reading. Cheap —
// just reads a few volatiles and does float math, no bus I/O — safe to
// call from a web handler or buildPayload() on every request.
void pulseRpmCompute(int i, ModuleConfig& cfg, PulseReading& out) {
  if (!cfg.din[i].pulseModeEnabled) {
    out.valid = false; out.rpm = 0.0f; out.status = "stale";
    return;
  }
  PulseChState& ps = _pulseState[i];
  if (!ps.havePulse) {
    out.valid = false; out.rpm = 0.0f; out.status = "stale"; // never seen a rising edge yet
    return;
  }
  uint32_t sinceLastRise = micros() - ps.lastRiseMicros; // unsigned subtraction, micros() rollover-safe
  float timeoutMicros = cfg.din[i].timeoutS * 1000000.0f;
  if ((float)sinceLastRise > timeoutMicros) {
    out.valid = true; out.rpm = 0.0f; out.status = "stopped";
    return;
  }
  if (ps.lastPeriodMicros == 0) {
    out.valid = false; out.rpm = 0.0f; out.status = "stale"; // only one edge so far, no period yet
    return;
  }
  int ppr = max(1, cfg.din[i].pulsesPerRev);
  float periodS = ps.lastPeriodMicros / 1000000.0f;
  out.rpm = 60.0f / (periodS * ppr);
  out.valid = true;
  out.status = "ok";
}

// Background fast-poll task — round-robins whichever DI channel(s) have
// pulseModeEnabled, reading each as a single-bit FC02 request (fastest
// possible round trip) as quickly as the bus + modbusBusMutex contention
// with the normal poll task allow. Sleeps (checks back every 500ms) when
// nothing has pulse mode on, so this costs nothing on a normal setup.
void pulsePollTask(void* param) {
  ModuleConfig* cfg = (ModuleConfig*)param;
  uint32_t lastLoopMicros = 0;
  for (;;) {
    bool anyEnabled = false;
    for (int i = 0; i < 4; i++) if (cfg->din[i].pulseModeEnabled) anyEnabled = true;
    if (!anyEnabled) {
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }

    for (int i = 0; i < 4; i++) {
      if (!cfg->din[i].pulseModeEnabled) continue;

      bool state = false;
      bool ok = false;
      // Short mutex wait (50ms) rather than blocking indefinitely — if the
      // main poll task is mid-cycle this just skips a sample and tries
      // again next loop, rather than stalling the whole fast-poll rate on
      // a slow-owning task.
      if (xSemaphoreTake(modbusBusMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        ok = modbusReadOneDI(cfg->modbusSlaveId, i, &state, /*quiet=*/true,
                              /*timeoutMs=*/cfg->din[i].pulseTimeoutMs);
        xSemaphoreGive(modbusBusMutex);
      }

      uint32_t now = micros();
      if (lastLoopMicros != 0) {
        float dtS = (now - lastLoopMicros) / 1000000.0f;
        if (dtS > 0.0001f) {
          float hz = 1.0f / dtS;
          // EMA smoothing so the displayed rate doesn't jitter wildly
          // read-to-read (bus contention with the main poll task makes
          // individual reads uneven).
          _pulsePollHz = (_pulsePollHz <= 0.0f) ? hz : (_pulsePollHz * 0.9f + hz * 0.1f);
        }
      }
      lastLoopMicros = now;

      if (ok) {
        PulseChState& ps = _pulseState[i];
        if (ps.haveLastState && !ps.lastState && state) {
          // Rising edge (OFF->ON) — counts as one trigger point passing
          // the sensor. Only rising edges are counted (not both edges) so
          // a normally-open proximity sensor's brief ON pulse per pass
          // registers as exactly one event, matching pulsesPerRev's
          // "trigger points per revolution" meaning.
          if (ps.havePulse) ps.lastPeriodMicros = now - ps.lastRiseMicros;
          ps.lastRiseMicros = now;
          ps.havePulse = true;
        }
        ps.lastState = state;
        ps.haveLastState = true;
      }
    }
    // No fixed delay beyond whatever the Modbus round-trip itself takes —
    // that IS the rate limit for this method (see file header). A short
    // yield keeps this from starving lower-priority tasks on the same
    // core without meaningfully throttling the achievable poll rate.
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// Returns the measured actual fast-poll rate (Hz) — surfaced on /digital
// so the accuracy ceiling is a live measurement, not a guess.
float pulsePollRateHz() {
  return _pulsePollHz;
}
