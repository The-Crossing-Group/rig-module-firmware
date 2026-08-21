# Rig Module Firmware — LilyGo T-CAN485 (Bench Testing)

**Version:** rig-module-1.11.0
**Board:** LilyGo T-CAN485 / XY-32 CAN+RS485 (plain ESP32, WROOM-32)
**Target:** Waveshare Modbus RTU Analog Input 8CH (B) or Eletechsup AMIDJ14 — auto-detected.

This is the **LilyGo hardware port of `waveshare-s3/`** — same firmware
logic, same feature set, same version, just running on the T-CAN485
board instead of the Waveshare ESP32-S3-RS485-CAN. Brought over for bench
testing since the LilyGo is the board actually on the bench.

`config.h` / `modbus.h` / `scaling.h` / `pulse.h` / `webui.h` are
**unchanged and shared byte-for-byte** with `waveshare-s3/` — only
`lilygo-t-can485.ino` differs (pins + WS2812 RGB LED handling, which the
Waveshare board doesn't have). Keep both variants in sync manually when
touching shared logic; there's no build system tying them together.

This **supersedes** the older `rig-module-firmware.ino` at the repo
root, which is stuck at v1.7.1 and predates board auto-detect, AMIDJ14
digital I/O, and RPM Pulse Counter Mode entirely. Use this folder for
any new LilyGo work.

## Hardware pins (LilyGo T-CAN485)

- 5V booster enable: GPIO16 (must be HIGH or RS485 has no power)
- RS485 TX: GPIO22, RX: GPIO21, DE/RE: GPIO17 (active HIGH = transmit)
- RS485 /SHDN: GPIO19 (must be HIGH to enable the MAX13487 chip)
- Onboard WS2812 RGB LED: GPIO4 — used for status (see below)

## Arduino IDE board settings

- Board: **"ESP32 Dev Module"** (plain ESP32/WROOM-32, NOT ESP32-S3 —
  that setting is for the `waveshare-s3/` variant only)
- Flash Size: 4MB (standard T-CAN485)
- Upload Speed: 921600 (lower if flashing is unreliable)

## Status LED (WS2812, onboard)

- Slow dim-blue blink: connecting to WiFi
- Solid green (briefly) → idle off: WiFi connected
- Fast dim-red blink: Modbus read failed / no WiFi
- Slow dim-amber pulse: sitting in setup AP mode, waiting for config
- Brief green flash: a Modbus poll just succeeded

## Feature set (same as waveshare-s3 v1.11.0)

- **Board auto-detection** — Waveshare 8AI (B) or Eletechsup AMIDJ14,
  via Product ID register 0x00F7, no jumper/dropdown needed. Manual
  override available on `/config` if auto-detect picks wrong.
- **RPM / Pulse Counter Mode** (AMIDJ14 digital inputs only) — see the
  `/digital` page. Per-DI: enable, pulses-per-revolution, stopped
  timeout, and fast-poll read timeout (ms) for tuning accuracy at
  higher RPM once RS485 baud is raised (AMIDJ14 supports up to
  115200bps). See `waveshare-s3/README.md` for the full writeup — same
  feature, same tuning advice, applies here unchanged.
- **Advanced / hidden multi-board RS485** (`/advanced`) — wire multiple
  AMIDJ14/Waveshare boards on the same bus, each at its own slave ID.
  Includes a Slave ID Bus Scan tool.
- **RS485 baud auto-detect** — probes every standard rate and adopts
  whichever gets a real response; also runs once automatically at boot
  if the configured baud gets no response at all.
- Plug-and-play WiFi: auto-scans for `rigNNN` site networks with the
  shared rig password if nothing is saved yet; falls back to its own
  setup AP (`RigModule-XXXXXX` / `modulesetup`) otherwise.
- Self-heals from a corrupted WiFi NVS blob (the classic
  WL_STOPPED/scan=-2 wedge) via a one-time NVS erase + config restore.

See `waveshare-s3/README.md` for full detail on every feature above —
the logic is identical, this file only covers what's LilyGo-specific
(pins, board setting, LED).
