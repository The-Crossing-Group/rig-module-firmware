# Rig Module Firmware — Waveshare ESP32-S3-RS485-CAN

**Version:** rig-module-1.7.1 (ported, same version string/behavior as the LilyGo variant)
**Board:** Waveshare ESP32-S3-RS485-CAN (isolated, DIN-rail, screw terminal)
**Target:** Waveshare Modbus RTU Analog Input 8CH (B) or Eletechsup AMIDJ14 — auto-detected, same as LilyGo variant.

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
