# Rig Module Firmware

**Version:** rig-module-1.0.0
**Board:** LilyGo T-CAN485 / XY-32 CAN+RS485 (ESP32)
**Target:** Waveshare Modbus RTU Analog Input 8CH **(B version)**

---

## What This Is

A **generic** field module for roaming/portable rig instrumentation — tank levels,
pump pressures, temperatures, flow, RPM, or anything else on an 8-channel 4-20mA
analog input board. Each unit shows up on the rig UI as a **generic device**
(`RigModule` card, not a tank-specific one), identified by a unique
`MODULE-ABC123` ID derived from its MAC address.

**No tank/mud-weight logic lives on this firmware.** Each of the 8 channels is
independently configured (name, kind, unit, mA-to-engineering scaling,
calibration) and reported as a raw engineering value + status. Any
higher-level interpretation — volume from level, mud weight from level +
pressure, whatever — happens elsewhere (Pi / rig UI side), against these raw
values. This keeps one firmware image usable for any sensor combination
without recompiling.

---

## Files

| File | Purpose |
|------|---------|
| `rig-module-firmware.ino` | Main sketch — WiFi, Pi posting, buffer, OTA |
| `config.h` | Config structs, NVS load/save |
| `modbus.h` | Manual Modbus RTU over HardwareSerial (RS485) |
| `scaling.h` | Raw → mA → engineering value per channel (generic, no kind-specific rounding) |
| `webui.h` | 4-page WebServer UI + REST API |

---

## Required Libraries (install via Arduino Library Manager)

- **ArduinoJson** >= 6.x (Benoit Blanchon)
- **NTPClient** (Fabrice Weinberg)
- Built-in: `WiFi`, `WebServer`, `Preferences`, `LittleFS`, `ESPmDNS`, `ArduinoOTA`, `HTTPClient`, `Update`

No ESPAsyncWebServer / AsyncTCP needed — uses the built-in `WebServer`.

Board: **ESP32 Arduino core** (espressif32) — install via Boards Manager.

---

## Pin Configuration (LilyGo T-CAN485, verified against official example)

```cpp
#define PIN_5V_EN   16   // 5V booster enable — must be HIGH or RS485 has no power
#define RS485_TXD   22   // Serial2 TX
#define RS485_RXD   21   // Serial2 RX
#define RS485_DE    17   // RS485 DE/RE (driver enable, active HIGH = transmit)
#define RS485_SE    19   // RS485 /SHDN (shutdown pin — must be HIGH to enable chip)
```

**Check your specific board schematic** — these vary between LilyGo revisions.

---

## First Setup

1. Flash the firmware.
2. On boot, if no WiFi is saved, the unit first auto-scans for a standard rig
   router (SSID `rigNNN`, e.g. `rig132`) using the shared rig password. If
   none is found/connectable, it creates a setup AP:
   **RigModule-XXXXXX** / password: `modulesetup`
3. Connect to it, browse to **http://192.168.4.1/wifi** and set your network
   (only needed for non-standard networks). Tap **🔍 Scan for Networks** to
   list nearby SSIDs with signal strength — tap one to fill it in, then just
   type the password.
4. Go to **⚙ Config** to set a friendly Module Name, Pi host (blank = mDNS
   auto-discovery), and X-Rig-Token (must match Pi's `rig_token`).
5. Go to **📐 Channels** to configure each of the 8 analog channels you're
   using.

The Module ID (`MODULE-ABC123`) is fixed — derived from the MAC address, not
editable — so every unit is uniquely and permanently identifiable regardless
of what it's measuring.

---

## Waveshare 8AI (B) — Important

The (B) version **defaults to voltage mode**. The firmware writes **mode 3
(4–20mA)** to all 8 channel config registers (0x1000–0x1007) on every boot
before reading.

**You must also set the internal jumpers** inside the board case:
- Jumper **connected** = current mode ✓
- Jumper **disconnected** = voltage mode

The (B) board ships with jumpers **disconnected** (voltage mode). Connect
them for 4–20mA operation.

---

## Channel Configuration

Go to **📐 Channels**. For each channel you're using:

1. Check **Enabled**
2. Set **Name** (e.g. "Suction Pressure") — shown on the rig UI card
3. Set **Kind** — fully free text, no fixed list. Use whatever makes sense:
   `level`, `pressure`, `temp`, `flow`, `rpm`, anything.
4. Set **Unit** (e.g. `psi`, `m`, `degC`, `rpm`) — free text, shown next to the value
5. Set the mA→engineering mapping: either
   - **mA Min/Max + Eng Min/Max** (simple linear map), or
   - **Set Zero / Set Max** with the sensor at known real-world states — this
     captures the actual raw ADC counts and takes priority over the mA map
     when both zeroRaw and maxRaw are set.

The firmware reports the raw mA reading, the scaled engineering value, and a
per-channel status (`ok` / `open` / `over`) for every enabled channel — no
extra processing.

---

## API Reference

| Method | Path | Description |
|--------|------|--------------|
| GET | `/api/status` | Full JSON status (same as Pi payload + system info) |
| GET | `/api/channel-raw` | Raw Modbus values + mA for all 8 channels |
| GET | `/api/wifi/scan` | Scan nearby WiFi networks (SSID, RSSI, secure) for the /wifi page |
| POST | `/api/config` | Save config fields (form or JSON) |
| POST | `/api/cal/zero?ch=N` | Capture zero cal for channel N |
| POST | `/api/cal/max?ch=N` | Capture max cal for channel N |
| POST | `/api/buffer/flush` | Flush backlog immediately |
| POST | `/api/buffer/clear` | Delete all buffered data |
| POST | `/api/reboot` | Reboot |
| POST | `/api/factory-reset` | Clear NVS + LittleFS, reboot |
| GET | `/api/ota?url=...` | HTTP OTA from URL |

---

## Pi Integration

The firmware POSTs to `http://<pi>:8080/api/rig/module` every poll interval,
with `type: "generic"` and `moduleId: "MODULE-ABC123"`. The Pi's field-module
ingest (`upsert_module`) is fully self-describing/generic — new module kinds
need no Pi-side code change, and the rig UI's `renderGeneric()` card shows
whatever channels/derived values are reported.

Pi discovery order:
1. mDNS: `_rig-logger._tcp.local` (Pi advertises this)
2. Hostname: `rig-logger.local:8080`
3. Static IP override on Config page

On Pi unreachable: samples buffer to `/buffer.jsonl` (LittleFS), max 3 hours.
Flushed oldest-first on reconnect.
