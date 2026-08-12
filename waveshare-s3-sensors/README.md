# Rig Module Firmware — Waveshare ESP32-S3-RS485-CAN (Direct Sensors)

**Version:** rig-module-sensors-1.0.0
**Board:** Waveshare ESP32-S3-RS485-CAN (isolated, DIN-rail, screw terminal)
**Target:** Any Modbus RTU sensor wired directly to RS485 — pressure, temp,
flow, level, whatever. Also brings up CAN (unused on the sibling
`waveshare-s3/` variant).

## How this differs from `waveshare-s3/`

The sibling `waveshare-s3/` folder talks to **one fixed analog-to-Modbus
adapter board** (Waveshare 8AI or Eletechsup AMIDJ14) that converts 4-20mA
current-loop signals into 8 fixed Modbus registers.

This variant is a genuine **Modbus RTU master**: instead of one adapter
board, you configure a list of up to **16 independent RS485 sensors**, each
its own Modbus slave with its own slave ID, register address, function
code, data type (uint16/int16/uint32/int32/float32), word order, and
scale/offset. Any Modbus RTU sensor works as long as you know those four
things (check the sensor's datasheet, or use the per-sensor "Probe Now"
button on /sensors to try a register/type combo before saving).

**CAN is also enabled here** (wired but unused on `waveshare-s3/`) — the
ESP32-S3's native TWAI controller runs in **listen-only mode** (this module
never transmits on the CAN bus, only reads):
- A configurable list of up to 16 "CAN signals" — byte range + decode rule
  extracted from frames matching a given CAN ID, same idea as an RS485
  sensor but sourced from CAN.

**No diagnostics/debugging page in this build.** There is no register
write, raw-traffic log, bus-scan report, or CAN frame sniffer here — only
read-only sensor setup helpers (per-sensor Probe Now / Auto-Detect Baud,
bus-wide Auto-Detect & Enable). This is deliberate: writing to a sensor's
own config registers is what corrupted an SM7779 radar sensor's internal
state during earlier field debugging. For deep debugging (raw sniffing,
register writes, framing experiments) use the LilyGo `sensor-debug` tool
in isolation, on one sensor at a time, before wiring it onto a shared
production bus.

Use `waveshare-s3/` for adapter-board deployments already in the field;
use this (`waveshare-s3-sensors/`) for new direct-Modbus-sensor + CAN work.

## Pages

- **/** — Module info, RS485 baud (shared bus-wide), CAN enable/bitrate, WiFi
- **/sensors** — Add/edit/remove RS485 Modbus sensors (16 slots)
- **/can** — Add/edit/remove CAN signals (16 slots)
- **/live** — Live values table (sensors + CAN signals + system status)
- **/system** — Firmware info, OTA, buffer, reboot/factory-reset

## Workflow for a new sensor

1. Wire it to the RS485 bus (A+/B-, same terminals as always). If it ships
   with a default slave address that collides with a sensor already on
   the bus, give it a unique address **in isolation** first (e.g. via the
   LilyGo `sensor-debug` tool's register-write support) before wiring it
   onto the shared bus.
2. Go to **/sensors**, click **&#9889; Auto-Detect & Enable Now** to find it
   and fill in a starter slot, or add it manually with its slave ID.
3. Use **Probe Now** on that sensor's card to try reading its value
   register as different data types until you get a sane-looking number.
   Use **Auto-Detect Baud** if you're not sure what baud rate it's using.
4. Fill in the register / data type / scale/offset, name it, save.
5. Check **/live** or the sensor's own live readout on **/sensors** to
   confirm it's reporting correctly.

## Workflow for CAN

1. Enable CAN on **/** (pick the right bitrate — 250 kbit/s is the most
   common default for drill/J1939-style CAN).
2. Go to **/can**, add a signal with the CAN ID/byte offset/length you
   already know (identify unknown frames with the LilyGo `sensor-debug`
   tool's sniffer if needed), set endianness/signedness/scale, save.
3. Confirm on **/live** or the signal's own live readout on **/can**.

## Wire format (Pi ingest)

Same endpoint and shape as every other rig-module variant
(`POST /api/rig/module`), so **no Pi-side or rig-modules.html changes are
needed**:
- Each enabled sensor appears under `channels[]` (keyed by its slot index,
  not a fixed hardware channel number) with `name`/`kind`/`unit`/`value`/
  `status`, plus an optional nested `volume` object if tank-volume is
  enabled on that sensor.
- CAN signals appear under a separate `canSignals[]` array, plus top-level
  `canEnabled`/`canFrameRate`/`canFrameTotal`.

## Hardware

Same pins as `waveshare-s3/`:
- RS485: TX=GPIO17, RX=GPIO18, DE/RE=GPIO21 (SP3485, HIGH=transmit).
  **Not automatic** despite Waveshare's marketing — DE toggle logic is
  required, same as the adapter-board variant.
- CAN: TX=GPIO15, RX=GPIO16 (native ESP32-S3 TWAI + onboard transceiver).
- No 5V-booster-enable or RS485-shutdown pins — isolated RS485 section is
  always powered/enabled once the board has power.
- No programmable status LED (fixed hardware LEDs only) — same as
  `waveshare-s3/`.

## Arduino IDE Board Settings

Same as `waveshare-s3/`:
- **Board:** "ESP32S3 Dev Module" (esp32:esp32:esp32s3)
- **USB CDC On Boot:** Enabled
- **Flash Size:** 16MB (128Mb)
- **Partition Scheme:** "16M Flash (3MB APP/9.9MB FATFS)" (`app3M_fat9M_16MB`)
- **PSRAM:** OPI PSRAM

Verified compiling clean with `arduino-cli` against these settings — 40%
program storage, 17% dynamic memory, zero warnings.

## Entering Download Mode (if flashing hangs)

Hold **BOOT**, tap **RESET**, release **RESET**, then release **BOOT**.

## Libraries Required

Same as the other variants: ArduinoJson (>=6.x), NTPClient, plus everything
built into the ESP32 Arduino core (LittleFS, Preferences, ESPmDNS,
ArduinoOTA, HTTPClient, WebServer, WiFi, `driver/twai.h` for CAN — no
extra CAN library needed).
