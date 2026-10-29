// =============================================================================
// pulse.h -- DI stroke counter (v1.19.33), waveshare-s3-sensors variant.
//
// Counts pump strokes (driveshaft passes) from an AMIDJ14 digital input over
// the existing RS485 Modbus wiring -- no GPIO. A background task reads the
// enabled DI(s) with single-bit FC02 requests as fast as the bus allows and
// counts rising edges (OFF->ON). strokes = edges / pulsesPerStroke.
//
// Accuracy limit (inherent to counting over Modbus): a pulse is only seen if
// the input is ON during a read. Reads happen roughly every 20-40ms, so a
// pulse shorter than that can be missed. The measured read rate is exposed
// (strokePollHz) so this can be checked on the real pump, not guessed.
// Counts live in RAM: they reset on reboot or via the Reset button.
// =============================================================================
#pragma once
#include <Arduino.h>
#include "config.h"
#include "modbus.h"

extern SemaphoreHandle_t modbusBusMutex;
extern String modbusDetectedType;

#define STROKE_DEBOUNCE_US 30000UL   // ignore a 2nd rising edge within 30ms
#define STROKE_MIN_TIMEOUT_US 3000000UL

struct StrokeState {
  volatile bool     haveLast     = false;
  volatile bool     last         = false;
  volatile uint32_t edges        = 0;
  volatile uint32_t lastEdgeUs   = 0;
  volatile uint32_t lastStrokeUs = 0;
  volatile uint32_t periodUs     = 0;   // time between the two most recent strokes
  volatile bool     haveStroke   = false;
};
static StrokeState _stroke[4];
static volatile float _strokePollHz = 0.0f;

static void strokeReset(int i) {
  if (i < 0 || i > 3) return;
  StrokeState& s = _stroke[i];
  s.edges = 0; s.periodUs = 0; s.haveStroke = false; s.haveLast = false;
}

// Call after saving config -- clears state for channels no longer counting.
static void strokeConfigChanged(ModuleConfig& cfg) {
  for (int i = 0; i < 4; i++)
    if (!(cfg.modbusBoard.din[i].enabled && cfg.modbusBoard.din[i].strokeEnabled)) strokeReset(i);
}

static uint32_t strokeCount(int i, ModuleConfig& cfg) {
  int pps = max(1, cfg.modbusBoard.din[i].pulsesPerStroke);
  return _stroke[i].edges / (uint32_t)pps;
}

// Strokes per minute from the last stroke interval. 0 + "stopped" if no stroke
// for max(3s, 2.5x the last interval); "stale" until two strokes were seen.
static float strokeSpm(int i, const char** status) {
  StrokeState& s = _stroke[i];
  if (!s.haveStroke || s.periodUs == 0) { *status = "stale"; return 0.0f; }
  uint32_t since = micros() - s.lastStrokeUs;
  uint32_t lim = (uint32_t)(s.periodUs * 2.5f);
  if (lim < STROKE_MIN_TIMEOUT_US) lim = STROKE_MIN_TIMEOUT_US;
  if (since > lim) { *status = "stopped"; return 0.0f; }
  *status = "ok";
  return 60000000.0f / (float)s.periodUs;
}

static float strokePollRateHz() { return _strokePollHz; }

void strokeTask(void* param) {
  ModuleConfig* cfg = (ModuleConfig*)param;
  uint32_t lastLoopUs = 0;
  for (;;) {
    bool any = false;
    for (int i = 0; i < 4; i++)
      if (cfg->modbusBoard.din[i].enabled && cfg->modbusBoard.din[i].strokeEnabled) any = true;
    // No TX while bus-listen is on; DI only exists on the AMIDJ14.
    if (!any || modbusListenOn || !cfg->modbusBoard.enabled || !modbusDetectedType.equals("amidj14")) {
      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
    }
    for (int i = 0; i < 4; i++) {
      if (!(cfg->modbusBoard.din[i].enabled && cfg->modbusBoard.din[i].strokeEnabled)) continue;
      bool state = false, ok = false;
      if (xSemaphoreTake(modbusBusMutex, pdMS_TO_TICKS(150)) == pdTRUE) {
        ok = modbusReadOneDI(i, &state, 40);
        xSemaphoreGive(modbusBusMutex);
      }
      uint32_t now = micros();
      if (lastLoopUs) {
        float dt = (now - lastLoopUs) / 1000000.0f;
        if (dt > 0.0001f) { float hz = 1.0f / dt; _strokePollHz = (_strokePollHz <= 0) ? hz : (_strokePollHz * 0.9f + hz * 0.1f); }
      }
      lastLoopUs = now;
      if (!ok) continue;   // keep previous state; don't invent edges from a failed read
      StrokeState& s = _stroke[i];
      if (s.haveLast && !s.last && state && (now - s.lastEdgeUs) >= STROKE_DEBOUNCE_US) {
        s.lastEdgeUs = now;
        s.edges = s.edges + 1;
        int pps = max(1, cfg->modbusBoard.din[i].pulsesPerStroke);
        if (s.edges % (uint32_t)pps == 0) {
          if (s.haveStroke) s.periodUs = now - s.lastStrokeUs;
          s.lastStrokeUs = now;
          s.haveStroke = true;
        }
      }
      s.last = state; s.haveLast = true;
    }
    vTaskDelay(1);
  }
}
