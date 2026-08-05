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
scale/offset. Any Modbus RTU sensor works as long as you know (or can
figure out via the Diagnostics page) those four things.

**CAN is also enabled here** (wired but unused on `waveshare-s3/`) — the
ESP32-S3's native TWAI controller runs in **listen-only mode** (this module
never transmits on the CAN bus, only reads) with:
- A raw frame sniffer (Diagnostics page) — see actual bus traffic before
  you know what's on it.
- A configurable list of up to 16 "CAN signals" — byte range + decode rule
  extracted from frames matching a given CAN ID, same idea as an RS485
  sensor but sourced from CAN.

Use `waveshare-s3/` for adapter-board deployments already in the field;
use this (`waveshare-s3-sensors/`) for new direct-Modbus-sensor + CAN work.

## Pages

- **/** — Module info, RS485 baud (shared bus-wide), CAN enable/bitrate, WiFi
- **/sensors** — Add/edit/remove RS485 Modbus sensors (16 slots)
- **/can** — Add/edit/remove CAN signals (16 slots)
- **/live** — Live values table (sensors + CAN signals + system status)
- **/diag** — Debugging diagnostics:
  - RS485 bus scan (probes every slave address, reports which respond)
  - Register probe (read any slave/register/type combo right now, no save)
  - Sensor comms health (poll/error counters per sensor)
  - CAN raw frame sniffer (live table of most recent frames)
- **/system** — Firmware info, OTA, buffer, reboot/factory-reset

## Workflow for a new sensor

1. Wire it to the RS485 bus (A+/B-, same terminals as always).
2. Go to **/diag**, run a **Bus Scan** to find its slave ID (or check the
   sensor's own datasheet/DIP switches if it has them).
3. Use **Register Probe** to try reading register 0 (or whatever the
   datasheet says) as different data types until you get a sane-looking
   number.
4. Go to **/sensors**, fill in that slave ID / register / data type, set
   scale/offset if the raw value isn't already in the right engineering
   units, name it, save.
5. Check **/live** or the sensor's own live readout on **/sensors** to
   confirm it's reporting correctly.

## Workflow for CAN

1. Enable CAN on **/** (pick the right bitrate — 250 kbit/s is the most
   common default for drill/J1939-style CAN).
2. Go to **/diag** and watch the raw frame sniffer. If nothing shows up,
   check bitrate and wiring (CAN H/L) before assuming there's no traffic.
3. Once you spot a consistent pattern (a specific CAN ID whose bytes
   correlate with something you can independently verify, e.g. RPM or
   pressure), note the ID, byte offset, and length.
4. Go to **/can**, add a signal with that ID/offset/length, set
   endianness/signedness/scale as needed, save.
5. Confirm on **/live** or the signal's own live readout on **/can**.

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
