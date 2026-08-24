# waveshare-s3-mudtank

Combined Rig Module firmware for the Waveshare ESP32-S3-RS485-CAN board.
Merges the two previous approaches into one image:

1. **Fixed adapter board** — a Waveshare 8AI (B) or Eletechsup AMIDJ14
   analog-to-Modbus board, auto-detected via its Product ID register
   (0x00F7). Reports 8 (or 6) 4-20mA channels, plus AMIDJ14 digital I/O
   (4 DI + 4 DO) and RPM via DI pulse counting. Up to 3 more of these
   boards can share the same RS485 bus (see the hidden `/advanced` page).
2. **Independent RS485 sensors** — any number of standalone Modbus RTU
   sensors (pressure, level, temp, radar...), each fully generic: slave
   ID, register, function code, data type, scale/offset — configure by
   hand or use the bus-wide Auto-Detect & Enable.
3. **CAN bus** (listen-only) with configurable signal extraction.

All RS485 devices — the fixed board(s) and every independent sensor —
share ONE Serial2 bus at ONE baud rate. CAN is a separate physical bus.

## New in this variant: Config Export / Import

`/system` page → Config Export / Import card. Downloads every setting
on the device as a single JSON file (fixed board channels, digital I/O,
extra boards, independent sensors, CAN signals, module/Pi/WiFi settings).
Upload that same file back to restore it, or onto a *different* unit to
clone the configuration. Module ID is never included/imported — it's
always derived from each device's own MAC address on boot.

**The exported file includes the WiFi password in plain text** — handle
it with the same care as any written-down password.

## Web UI Pages

- `/` — Config: module info, shared RS485 baud, fixed-board slave
  ID/board override, CAN enable/bitrate, WiFi
- `/channels` — Fixed adapter board's per-channel config (name, kind,
  unit, mA/eng scaling, calibration, tank volume)
- `/digital` — Fixed adapter board's AMIDJ14 digital I/O + Pulse Counter
  Mode (RPM via Modbus, no GPIO)
- `/sensors` — Independent RS485 sensor list: add/edit/remove, Probe Now,
  per-sensor Auto-Detect Baud, bus-wide Auto-Detect & Enable
- `/can` — CAN signal list
- `/advanced` — Hidden power-user page: extra fixed boards (multi-board
  RS485) + Slave ID Bus Scan (only linked quietly from `/system`)
- `/live` — Live values: fixed board channels + independent sensors +
  digital I/O + extra boards + CAN signals, all in one place
- `/system` — Firmware info, board re-detect, NVS diagnostics, baud
  persistence check, **Config Export/Import**, OTA, buffer, reboot/
  factory-reset

## Hard Rules (carried over from sibling variants — see MEMORY.md)

- **No GPIO for RPM.** Pulse Counter Mode reads a DI over Modbus as fast
  as the bus allows — never a GPIO interrupt. This was tried once on the
  sibling `waveshare-s3` variant and explicitly reverted.
- **No FC06 (write single register) anywhere in this firmware.** Writing
  to a sensor's own config registers corrupted an SM7779 radar sensor's
  internal state during 2026-08-10/11 debugging. FC16 (write multiple
  holding registers) IS retained, but only for the Waveshare board's
  documented mode-3/4-20mA channel-mode convention — not a generic write
  path to arbitrary sensor config space.

## Pins (Waveshare ESP32-S3-RS485-CAN)

- RS485: TX=GPIO17, RX=GPIO18, DE/RE=GPIO21 (SP3485, HIGH=transmit)
- CAN: TX=GPIO15, RX=GPIO16 (native ESP32-S3 TWAI + onboard transceiver)

## Arduino IDE Board Settings

- Board: "ESP32S3 Dev Module"
- USB CDC On Boot: Enabled
- Flash Size: 16MB, PSRAM: OPI PSRAM
- Partition Scheme: 16M Flash (3MB APP/9.9MB FATFS)

## Libraries Required

ArduinoJson (>= 6.x), NTPClient, LittleFS/Preferences/ESPmDNS/ArduinoOTA/
HTTPClient/WebServer/WiFi (all built-in to ESP32 Arduino core),
`driver/twai.h` (built-in, no library needed).

## Relationship to Sibling Variants

This is a NEW, THIRD variant — `waveshare-s3/` (fixed board only) and
`waveshare-s3-sensors/` (independent sensors + CAN only) are both kept
as-is and continue to receive their own updates independently. This
variant does not replace either; pick whichever fits a given module's
job, or use this one if a module genuinely needs both approaches at once
(e.g. a mud tank job with both a fixed 8AI board AND a standalone radar
sensor on the same bus).
