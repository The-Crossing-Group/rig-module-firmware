# Rig Module Firmware

Field instrumentation firmware for roaming/portable rig modules — tank
levels, pump pressures, temperatures, flow, RPM, CAN signals, or anything
else that can be read over RS485 (Modbus RTU) or CAN. Every module POSTs
JSON telemetry to the Rig Pi Logger so it shows up on the rig UI with no
per-sensor-kind code on either end.

**This repo currently holds six sketches.** They are independently
versioned and maintained — pick the one that matches your hardware and
job, there is no single "the" firmware here.

---

## ⚠️ Start Here — Which Folder Do I Want?

| Folder | Board | What it talks to | Status |
|---|---|---|---|
| **`waveshare-s3/`** | Waveshare ESP32-S3-RS485-CAN | One fixed analog-to-Modbus adapter board (Waveshare 8AI (B) or Eletechsup AMIDJ14), auto-detected | **Production** — current target for new fixed-adapter-board modules |
| **`lilygo-t-can485/`** | LilyGo T-CAN485 (plain ESP32) | Same as `waveshare-s3/` | **Production**, bench-testing hardware port of `waveshare-s3/`. Same feature set/version, different pins. |
| **`waveshare-s3-sensors/`** | Waveshare ESP32-S3-RS485-CAN | Up to 16 independent RS485 Modbus sensors directly (no fixed adapter board) + CAN (always on, listen-only or CANopen Bridge) | **Production**, most actively developed variant |
| **`waveshare-s3-mudtank/`** | Waveshare ESP32-S3-RS485-CAN | Merges both approaches: fixed adapter board(s) **and** independent RS485 sensors **and** CAN, all at once | **Production** — use when one module genuinely needs both a fixed board and standalone sensors on the same bus |
| **`lilygo-t-can485-mudtank/`** | LilyGo T-CAN485 (plain ESP32) | Same as `waveshare-s3-mudtank/` | **Production**, bench-testing hardware port of `waveshare-s3-mudtank/` |
| **`sensor-debug/`** / **`sensor-debug-lilygo/`** | Waveshare ESP32-S3-RS485-CAN / LilyGo T-CAN485 | Nothing fixed — standalone RS485/Modbus debugging tool (bus scan, raw read/write, sniffer, bitscope) | **Debug tool**, not a telemetry firmware. Flash temporarily to dig into a misbehaving sensor, then flash the real firmware back. |
| **`rig-module-firmware.ino`** (repo root) | LilyGo T-CAN485 | One fixed Waveshare 8AI board only | **Deprecated.** Stuck at v1.7.1, predates board auto-detect, AMIDJ14 digital I/O, RPM pulse counting, and every variant above. Superseded by `lilygo-t-can485/`. Kept only for reference against units already in the field running this exact image — do not build new modules on it. |

**New module build? Start with `waveshare-s3/`** (fixed adapter board
only) or **`waveshare-s3-sensors/`** (direct Modbus sensors + CAN), or
**`waveshare-s3-mudtank/`** if a job needs both at once. Use the matching
`lilygo-t-can485*` port only for bench testing when the Waveshare board
isn't on hand — they share the same `config.h`/`modbus.h`/`scaling.h`/
`webui.h` byte-for-byte with their Waveshare sibling; only pins and the
onboard WS2812 LED differ.

Each folder has its own `README.md` with full detail (features, pin
maps, Arduino IDE board settings, API/payload shape). This top-level
README only covers what's shared across all of them and the current
state of the repo as a whole.

---

## Shared Design Principles

- **Generic, not sensor-specific.** No tank/mud-weight/kind-specific
  logic lives on any of these firmwares. Each channel or sensor slot is
  independently configured (name, free-text kind, unit, scaling,
  calibration) and reported as a raw engineering value + status.
  Higher-level interpretation (volume from level, mud weight from level
  + pressure, etc.) happens on the Pi/rig-UI side against these raw
  values — this is what lets one firmware image serve any sensor
  combination without recompiling.
- **Plug-and-play WiFi.** On boot, if no WiFi is saved, each module first
  auto-scans for a standard rig router (SSID `rigNNN`, e.g. `rig132`)
  using the shared rig password. If none is found, it falls back to its
  own setup AP (`RigModule-XXXXXX` / password `modulesetup`) for manual
  config via `http://192.168.4.1/`.
- **Pi auto-discovery, no typing required.** Modules find the Rig Pi
  Logger in priority order: static `piHost` override on `/config` (if
  set) → `rigNNN` SSID-derived IP (`192.168.NNN.10`) → mDNS
  (`_rig-logger._tcp.local`) → `rig-logger.local` → (on the
  `waveshare-s3-sensors` line) a bounded /24 subnet sweep as a last
  resort. `/system` shows which method is currently active.
- **Buffer, don't drop.** If the Pi is unreachable, samples buffer to
  LittleFS (`/buffer.jsonl`, up to ~3 hours) and flush oldest-first on
  reconnect.
- **Module identity is fixed.** Every unit's `MODULE-ABC123` ID is
  derived from its MAC address, never user-editable — so a unit stays
  identifiable regardless of what it's measuring or where it's deployed.
- **No GPIO for RPM.** Pulse Counter Mode (where supported) reads a
  digital input over Modbus as fast as the bus allows — never a GPIO
  interrupt. This was tried once and explicitly reverted; it is a hard
  rule across every variant now.
- **No FC06 (write single register) in production firmware.** Writing to
  a sensor's own config registers corrupted an SM7779 radar sensor's
  internal state during field debugging (2026-08-10/11). FC16 (write
  multiple holding registers) is retained only for the Waveshare 8AI
  board's documented mode-3/4-20mA channel-mode convention — never a
  generic write path to arbitrary sensor config space. Register writes
  for debugging/recovery belong in the `sensor-debug*` tools only, used
  in isolation on one sensor at a time before it goes back on a shared
  bus.

---

## Communications Protocols

### RS485 / Modbus RTU (bus side)

- Manual Modbus RTU implemented directly over `HardwareSerial` with
  DE/RE pin toggling (`modbus.h`) — no external Modbus library, avoids
  DE-timing issues some libraries have.
- Standard CRC16 Modbus framing, 8N1.
- **Baud is never hardcoded.** Different adapter boards/sensors ship with
  different factory defaults (Waveshare 8AI (B) = 9600bps, others vary).
  On boot, if the configured baud gets no response, firmware probes every
  standard rate (9600, 4800, 19200, 2400, 38400, 1200, 57600, 115200) and
  adopts whichever answers. Re-runnable on demand from `/config`.
- Tolerant reply parsing: accepts whatever function code/register count
  a sensor actually answers with, and treats slave addresses 0/250 as
  broadcast — a valid reply from an unexpected address is shown as data,
  not rejected as an error.
- Fixed-adapter-board variants auto-detect which board is wired up by
  reading its Product ID register (0x00F7) at boot — no jumper/dropdown
  needed (Waveshare 8AI (B) = ID 2308, 8ch, µA raw; Eletechsup AMIDJ14 =
  ID 2814, 6ch + 4DI/4DO, 0.01mA raw). Unrecognized/failed reads fall
  back to the Waveshare profile.
- Direct-sensor variants (`waveshare-s3-sensors/`, `waveshare-s3-mudtank/`
  and their LilyGo ports) are genuine Modbus RTU masters: each configured
  sensor slot carries its own slave ID, register address, function code
  (FC03/FC04 read), data type (uint16/int16/uint32/int32/float32), word
  order, and scale/offset.
- All RS485 devices on a given module — fixed board(s) and any
  independent sensors — share **one** Serial2 bus at **one** baud rate.

### CAN (where present)

- Native ESP32/ESP32-S3 TWAI controller, no third-party CAN library.
- Defaults to **listen-only** mode — these modules never transmit on the
  CAN bus by default, purely a passive tap, so a mis-wired/mis-timed
  module can't disrupt drill CAN traffic.
- Raw frame capture ring buffer for live diagnostics, plus configurable
  signal extraction (byte range + decode rule per CAN ID), same idea as
  an RS485 sensor slot but sourced from CAN.
- **CANopen Bridge mode** (`waveshare-s3-sensors/` only, v1.15+): switches
  the controller to transmit-capable (`TWAI_MODE_NORMAL`) to send a real
  CANopen bring-up sequence (SYNC/baud-detect + NMT Start) for
  factory-silent devices like the ditchwitch-logger project's EPC
  carriage-position encoder. Self-heals/retries every 30s if no traffic
  is seen. Decode of raw frames happens on the Pi, never on the module.
- Boot order matters: CAN initializes **after** WiFi/AP and the web
  server are already up — network access is never gated behind CAN
  hardware coming up successfully.

### Wire format — Pi ingest

All production variants POST to the same endpoint and shape:

```
POST http://<pi-host>:8080/api/rig/module
Content-Type: application/json
```

Common top-level fields: `moduleId`, `type` (defaults `"generic"`),
`name`, `moduleName`, `ip`, `fw` (firmware version string), `uptimeS`,
`rssi`, `buffered`, `ts`. Per-channel/sensor data lives under
`channels[]` (`name`/`kind`/`unit`/`value`/`status`, plus an optional
nested `volume` object when tank-volume is enabled on that channel).
Variant-specific additions layer on top without changing this base
shape, so the Pi's ingest and rig UI need no per-variant code:

- `extraBoards[]` — additional fixed adapter boards on `/advanced`
  (own `channels`/`digitalInputs`/`digitalOutputs`).
- `digitalInputs[]` / `digitalOutputs[]` — AMIDJ14 digital I/O, with
  `.rpm = {value, status}` on any DI running Pulse Counter Mode.
- `canEnabled`, `canFrameRate`, `canFrameTotal`, `canSignals[]` — CAN
  telemetry on variants with CAN brought up.
- `canFrames[]` — raw captured CAN frames (`id`/`extended`/`dlc`/
  `data_hex`/`ms`), present only in CANopen Bridge mode. A receiver that
  doesn't know this key can ignore it.

A consumer that only understands the base shape (`channels[]` +
top-level fields) works unmodified against every variant; the extra
arrays are purely additive.

### Config Export / Import

The merged variants (`waveshare-s3-mudtank/`, `lilygo-t-can485-mudtank/`)
add a `/system` page JSON export/import of the full on-device config
(channels, digital I/O, extra boards, independent sensors, CAN signals,
module/Pi/WiFi settings) — for backup or cloning a config onto a
different unit. **The exported file includes the WiFi password in plain
text**; handle it like any other saved password. Module ID is never
exported/imported — always derived per-device from its own MAC address.

---

## Current State / Recent Work (see each folder's own README for detail)

- **`waveshare-s3-sensors/`** is the most actively developed line right
  now (v1.15.46 as of this writing) — recent work: direct tank-height
  measurement replacing capacity-based calibration, a WiFi reconnect
  watchdog, real tank geometry (length × width) instead of a typed-in
  capacity number, and a serial CLI (`h` for help) mirroring every web
  UI setting for USB-only configuration/debugging.
- **`waveshare-s3/`** and **`lilygo-t-can485/`** are at v1.11.1 — fixed a
  bug where digital I/O kept reporting stale ON/OFF state forever after
  an RS485/Modbus dropout instead of going invalid.
- **`waveshare-s3-mudtank/`** and **`lilygo-t-can485-mudtank/`** are the
  newest variants (merged fixed-board + independent-sensor + CAN
  support in one image) with Config Export/Import.
- **WiFi auto-join was intentionally stripped from every variant** —
  modules never invent or hardcode an SSID; nothing connects until a
  network is explicitly saved (or the `rigNNN` auto-scan/standard-AP
  fallback above applies).
- **`rig-module-firmware.ino`** at the repo root has not been touched
  since v1.7.1 (2026-07) and is superseded by `lilygo-t-can485/` for any
  new work — see the deprecation note in the table above.

---

## Repo Layout

```
rig-module-firmware.ino, config.h, modbus.h, scaling.h, webui.h   — DEPRECATED root sketch (v1.7.1), see table above
waveshare-s3/              — fixed adapter board, Waveshare ESP32-S3-RS485-CAN   [production]
lilygo-t-can485/           — same, LilyGo T-CAN485 bench-test port              [production]
waveshare-s3-sensors/      — direct RS485 sensors + CAN, Waveshare ESP32-S3     [production, most active]
waveshare-s3-mudtank/      — fixed board + direct sensors + CAN merged, Waveshare [production]
lilygo-t-can485-mudtank/   — same, LilyGo T-CAN485 bench-test port              [production]
sensor-debug/              — standalone RS485/Modbus debug tool, Waveshare S3   [debug tool]
sensor-debug-lilygo/       — same, LilyGo T-CAN485 (separate, not kept in sync) [debug tool]
```

## Required Libraries (all variants)

- **ArduinoJson** >= 6.x (Benoit Blanchon)
- **NTPClient** (Fabrice Weinberg)
- Built-in to the ESP32 Arduino core: `WiFi`, `WebServer`, `Preferences`,
  `LittleFS`, `ESPmDNS`, `ArduinoOTA`, `HTTPClient`, `Update`, and
  `driver/twai.h` for CAN (no separate CAN library needed).
- No ESPAsyncWebServer / AsyncTCP anywhere — every variant uses the
  built-in `WebServer`.

Each folder's own README has the exact Arduino IDE board settings (board
type, flash size, partition scheme, PSRAM) for its hardware — these
differ between the plain-ESP32 LilyGo boards and the ESP32-S3 Waveshare
boards, so check the right one before flashing.
