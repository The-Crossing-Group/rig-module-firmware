# Rig Module Firmware — Waveshare ESP32-S3-RS485-CAN

**Version:** rig-module-1.10.0
**Board:** Waveshare ESP32-S3-RS485-CAN (isolated, DIN-rail, screw terminal)
**Target:** Waveshare Modbus RTU Analog Input 8CH (B) or Eletechsup AMIDJ14 — auto-detected, same as LilyGo variant.

## RPM / Pulse Counter Mode (v1.10.0+) — AMIDJ14 Digital Inputs only

New per-DI checkbox on `/digital`: "Pulse Counter Mode". Turns a digital
input's ON/OFF transitions into an RPM reading — **over the existing
Modbus DI wiring, no GPIO, no extra hardware.** Uses whatever sensor is
already wired to that DI (e.g. a proximity sensor watching a bolt/target
pass once per revolution).

- A dedicated background task (`pulse.h`) hammers just that one DI with
  single-bit FC02 reads as fast as the RS485 bus allows — much faster
  than the normal poll cycle, since RPM needs to catch every transition,
  not a periodic snapshot. Shares the same bus mutex as the main poll
  task so the two never collide on the wire.
- **Math:** time between rising edges (OFF→ON), not a fixed counting
  window — `RPM = 60 / (period_seconds × pulsesPerRev)`. Updates on every
  transition, responsive at low RPM, no pulse for the configured timeout
  → reports 0 (stopped) instead of freezing on a stale value.
- **Per-DI config:** enable, pulses-per-revolution (1 for a single
  trigger point on the shaft; higher for e.g. gear teeth), stopped
  timeout (s).
- **Honest limitation — read this before relying on it at speed:**
  accuracy is capped by how fast this device can round-trip a single-bit
  Modbus read over RS485, not open-ended like a hardware interrupt would
  be. Roughly the "few hundred RPM" range at typical baud rates, and it
  gets less reliable (aliasing — a plausible-looking but silently WRONG
  number, not just a slow one) past that. The actual measured fast-poll
  rate is shown live on `/digital` so this is a fact you can check, not a
  guess — if you need faster/higher-RPM sensing than that ceiling, it
  needs a different measurement method (dedicated hardware pulse counter/
  interrupt), which is a bigger scope change than this feature.
- Reported per-DI in the JSON payload (`digitalInputs[].rpm = {value,
  status}`, only present when that DI has pulse mode on) and on `/live`.

## Advanced: Multi-Board RS485 (v1.9.0+)

Hidden power-user page at `/advanced` (linked quietly from the bottom of
`/system` — not in the main nav). Lets you wire up to 3 **additional**
analog-to-Modbus boards on the same RS485 bus as the primary board, each
at its own Modbus slave address — e.g. several Eletechsup AMIDJ14 units
daisy-chained together for more digital I/O than one board provides.

- Each extra board: enable checkbox, name, Slave ID (1-247, must be
  unique across the whole bus — primary board included), Board Type
  (AMIDJ14 or Waveshare 8AI). No auto-detect for these — set the type
  explicitly.
- Extra boards get **generic channel names** ("Board Ch 1", etc.) and the
  standard 4-20mA linear map only — no per-channel calibration or tank
  volume, unlike the primary board's `/channels` page. That's a
  deliberate scope cut to keep this simple; the primary board is still
  the one to use for anything needing real calibration.
- Reported in the JSON payload as a separate top-level `extraBoards[]`
  array (each entry has its own `channels`/`digitalInputs`/
  `digitalOutputs`) — the primary board's existing `channels`/
  `digitalInputs`/`digitalOutputs` shape is completely unchanged, so
  nothing breaks for a Pi/dashboard that doesn't know about this feature.
- **Slave ID Bus Scan** tool also lives on `/advanced` — probes the bus
  for what's actually responding at each address (with Product ID if it
  identifies as a known board). Use it to catch address typos/collisions
  before they turn into a mysteriously silent board.
- Changing a slave ID or board type reboots the module to apply.

This is a **hardware port** of `rig-module-firmware.ino` (the LilyGo
T-CAN485 sketch, in the sibling `rig-module-firmware/` folder) onto the
Waveshare ESP32-S3-RS485-CAN board. All feature docs (multi-tank volume,
board auto-detection, baud auto-detection, WiFi auto-discovery, etc.) in
that repo's README apply here unchanged — only the hardware layer differs.

**We're moving to this board going forward for new modules.** LilyGo units
already in the field / in stock keep using `rig-module-firmware/` — that
firmware is unmodified and still fully supported, not being retired.

## What's different from the LilyGo variant

- **`config.h`, `modbus.h`, `scaling.h`, `webui.h` are byte-for-byte
  identical** to the LilyGo variant — copy over, don't hand-edit, when
  porting a fix from one to the other. Only the `.ino` differs.
- **Pins remapped** for this board's fixed hardware (screw-terminal
  RS485, not GPIO-selectable):
  - RS485: TX=GPIO17, RX=GPIO18, DE/RE=GPIO21 (SP3485 transceiver,
    HIGH=transmit / LOW=receive — same DE-toggle logic as LilyGo, just a
    different pin. **Not automatic** — despite Waveshare's marketing copy
    calling RS485 direction "automatic", it isn't; verified against two
    independent open-source ports (Battery-Emulator, esphome-yambms)).
  - CAN: TX=GPIO15, RX=GPIO16 — wired but **not used by this firmware
    yet**. Available for future CAN work (e.g. the wake-on-CAN mud sensor
    idea).
  - No 5V-booster-enable pin, no RS485-shutdown pin — this board's
    isolated RS485 section is always powered/enabled once the board has
    power, so those LilyGo-specific boot steps are simply omitted.
- **No programmable status LED.** The LilyGo has a WS2812 RGB LED on
  GPIO04 that this firmware blinks for WiFi/Modbus/data-OK states. The
  Waveshare board only has fixed PWR/RS485-TX/RS485-RX/CAN indicator LEDs
  wired directly to hardware — not GPIO-controllable. `ws2812Init()`/
  `ws2812Set()` are stubbed as no-ops here so the rest of the sketch
  (poll task, WiFi state machine, etc.) needed zero changes. All state is
  still visible over Serial/USB and on the web UI as before.
- **ESP32-S3** instead of plain ESP32 — 16MB flash + 8MB PSRAM vs the
  LilyGo's 4MB flash, no large PSRAM. Plenty of extra headroom; compiled
  sketch is ~39% of program space on the larger partition scheme vs ~95%
  on the LilyGo's 4MB default.
- **USB-C direct to the ESP32-S3's native USB** (no separate UART-to-USB
  chip) — Serial requires `USB CDC On Boot: Enabled` in board settings, or
  nothing will print/appear as a COM port.

## Arduino IDE Board Settings

- **Board:** "ESP32S3 Dev Module" (esp32:esp32:esp32s3)
- **USB CDC On Boot:** Enabled
- **Flash Size:** 16MB (128Mb)
- **Partition Scheme:** "16M Flash (3MB APP/9.9MB FATFS)" (`app3M_fat9M_16MB`) —
  gives plenty of room; any 16MB scheme with a large-enough APP partition works
- **PSRAM:** OPI PSRAM (this board is ESP32-S3R8: 8MB Octal/OPI PSRAM)

Verified compiling clean with `arduino-cli` against these settings —
39% program storage, 16% dynamic memory used, zero warnings.

## Entering Download Mode (if flashing hangs)

Hold **BOOT**, tap **RESET**, release **RESET**, then release **BOOT**.

## Wiring

- RS485 A+/B- → screw terminal on the board (labeled on the case)
- CAN H/L → screw terminal, unused for now
- Power → 7–36V DC screw terminal (matches rig 12/24V systems directly,
  no USB power-bank hack needed like the LilyGo deployments)

## Libraries Required

Same as the LilyGo variant — see `rig-module-firmware/README.md`:
ArduinoJson (>=6.x), NTPClient, LittleFS/Preferences/ESPmDNS/ArduinoOTA/
HTTPClient/WebServer/WiFi (all built-in to the ESP32 Arduino core).
