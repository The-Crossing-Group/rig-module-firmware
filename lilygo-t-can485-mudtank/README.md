# lilygo-t-can485-mudtank

Hardware port of `waveshare-s3-mudtank/` onto the LilyGo T-CAN485 / XY-32
CAN+RS485 board (plain ESP32/WROOM-32, bench-testing hardware).

`config.h` / `modbus.h` / `can.h` / `scaling.h` / `pulse.h` / `webui.h` are
**UNCHANGED and shared byte-for-byte** with `waveshare-s3-mudtank/` — only
this `.ino` (pins + WS2812 LED handling + CAN transceiver enable pin)
differs. Keep both in sync manually when touching shared logic; there's no
build system tying them together.

Same merged feature set as `waveshare-s3-mudtank/`:

1. **Fixed adapter board** (Waveshare 8AI (B) or Eletechsup AMIDJ14,
   auto-detected) — 8/6 channels + AMIDJ14 digital I/O + RPM pulse
   counting, up to 3 more boards on `/advanced`.
2. **Independent RS485 sensors** — any number, fully generic.
3. **CAN bus** (listen-only) with signal extraction — this board's
   onboard SN65HVD230 transceiver, brought up here for the first time on
   this hardware (the sibling `lilygo-t-can485/` port never used CAN).
4. **Config Export/Import** — `/system` page, full JSON backup/restore.

## Pins (LilyGo T-CAN485)

Verified against LilyGo's own factory examples (two separate example
folders, each with its own `config.h` — this port merges both pin sets):
- RS485 example: `example/Arduino/RS485/config.h`
- CAN example: `example/Arduino/CAN/config.h`

| Signal | Pin | Notes |
|---|---|---|
| PIN_5V_EN | GPIO16 | 5V booster enable — must be HIGH or RS485 has no power |
| RS485_TXD | GPIO22 | Serial2 TX |
| RS485_RXD | GPIO21 | Serial2 RX |
| RS485_DE | GPIO17 | RS485 DE/RE (driver enable, active HIGH = transmit) |
| RS485_SE | GPIO19 | RS485 /SHDN — must be HIGH to enable the MAX13487 chip |
| WS2812_PIN | GPIO4 | Onboard WS2812B RGB LED |
| CAN_TXD | GPIO27 | Native ESP32 TWAI controller |
| CAN_RXD | GPIO26 | Native ESP32 TWAI controller |
| CAN_SE_PIN | GPIO23 | SN65HVD230 transceiver standby/enable — driven LOW at boot (matches LilyGo's own factory CAN example) |

**Note on CAN pin naming:** LilyGo's own `config.h` for the CAN example
defines `CAN_TX_PIN 26` / `CAN_RX_PIN 27` in the header, but the actual
`CAN_cfg.tx_pin_id`/`rx_pin_id` assignment in their `CAN.ino` example uses
`GPIO_NUM_27` for TX and `GPIO_NUM_26` for RX — i.e. the real wiring is
TX=27/RX=26, opposite of what the header `#define` names suggest. This
port follows the actual pin assignment used in their working example
(TX=27, RX=26), not the possibly-mislabeled header constant names.

## Arduino IDE Board Settings

- Board: "ESP32 Dev Module" (plain ESP32/WROOM-32, **NOT** ESP32-S3)
- Flash Size: 4MB (standard T-CAN485)
- Upload Speed: 921600 (or lower if flashing is unreliable over the
  onboard CH9102/CP2102 USB-UART chip)
- To enter download mode if flashing ever hangs: hold BOOT, tap RESET,
  release RESET, then release BOOT.

## Hard Rules (carried over — see MEMORY.md)

Same as `waveshare-s3-mudtank/`: no GPIO for RPM (Modbus DI pulse
counting only), no FC06 anywhere in this firmware (FC16 retained only
for the Waveshare board's mode-3 channel convention).

## Relationship to Other Variants

- `waveshare-s3-mudtank/` — same firmware, Waveshare ESP32-S3-RS485-CAN
  hardware. This is the "production" hardware target; this LilyGo port
  exists for bench testing without needing the Waveshare board on hand.
- `lilygo-t-can485/` — the OLDER LilyGo port, hardware-ports only the
  fixed-board variant (`waveshare-s3/`), no independent sensors, no CAN.
  Kept as-is, not replaced by this one.
- `waveshare-s3/` and `waveshare-s3-sensors/` — the two original
  un-merged variants, both kept as-is.
