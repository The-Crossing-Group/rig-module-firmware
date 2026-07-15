# Rig Module Firmware

**Version:** rig-module-1.7.0
**Board:** LilyGo T-CAN485 / XY-32 CAN+RS485 (ESP32)
**Target:** Waveshare Modbus RTU Analog Input 8CH **(B version)** or Eletechsup AMIDJ14 (6AI-4DO-4DI) — auto-detected, see below.

---

## Pi Host Auto-Discovery from Rig SSID (v1.7.0+)

Standard rig sites follow a fixed naming convention: WiFi SSID `rigNNN`
(e.g. `rig132`) with the Pi/host PC always living at `192.168.NNN.10` on
that rig's own subnet. With **Pi Host** left blank on `/config`, the
module now derives that IP directly from whatever `rigNNN` SSID it's
connected to — no typing, no mDNS round-trip needed, and it re-derives
automatically if the same unit is later moved to a different rig.

If the connected SSID doesn't match the `rigNNN` pattern (a non-standard
network), this step is skipped entirely and Pi discovery falls through to
the existing mDNS (`_rig-logger._tcp.local`) then `rig-logger.local`
lookups, same as before. Setting Pi Host manually on `/config` always
takes priority over all auto-discovery, standard or not.

---

## Multi-Board Support (v1.5.0+)

This firmware now auto-detects which analog-to-Modbus board is wired up —
**no dropdown, jumper, or manual config needed.** At boot, it reads the
board's "Product ID" special-function register (0x00F7) over Modbus and
picks the matching channel count + raw-value scale:

| Board | Product ID | Channels | Raw units |
|---|---|---|---|
| Waveshare Modbus RTU Analog Input 8CH (B) | 2308 | 8 | µA (÷1000 = mA) |
| Eletechsup AMIDJ14 (6AI-4DO-4DI) | 2814 | 6 | 0.01mA (÷100 = mA) |

If the Product ID read fails or returns an ID we don't recognize, the
firmware falls back to the Waveshare profile (the original default before
this detection existed), so nothing regresses for existing deployments.

Swap boards, reboot, done — same firmware image, same config UI, correct
scaling either way. The detected board name + channel count is shown on
the `/channels` and `/system` pages. Channels beyond a board's real count
(e.g. ch 7-8 on the 6-channel AMIDJ14) are greyed out on `/channels` and
omitted from the JSON payload sent to the Pi.

Adding another board: add its Product ID / channel count / raw divisor to
the `BoardProfile` table in `modbus.h` (`modbusDetectBoard()`) — that's the
only place board-specific behavior lives.

---

## Multi-Tank Support (v1.6.0+)

Any number of channels on the same module can independently have "Compute
Tank Volume" checked on `/channels` — e.g. two separate tanks wired to two
channels of the same Waveshare/Modbus board. Each one gets its own linear
level-to-volume map (`computeChannelVolume()` in `scaling.h`).

Every channel with volume enabled carries its own `volume` (`{value, unit,
status}`) + `capacity` field right on its entry in the JSON payload's
`channels[]` array, and the `/live` page shows a **Volume column on the
Channels table** (instead of a single tank card) so all configured tanks
are visible at once, not just the first one found.

The old top-level `derived.volume` + `capacity` fields are still sent
(mirroring whichever channel is the first one with volume enabled) for any
existing consumer that only expects a single tank per module — but with
multiple tanks configured, the per-channel `volume` field is the complete,
authoritative source.

---

## X-Rig-Token Self-Heal (v1.6.1)

`X-Rig-Token` (the header the Pi logger checks for auth) defaults to the
shared rig password (`7804991970`) out of the box, but a unit that had it
saved as an empty string in NVS (e.g. from a very early config save, or an
accidental clear on `/config`) would load that blank value forever and
silently fail to auth against the Pi — no error on the module side, the Pi
would just reject/ignore its posts.

Fixed at both ends: `loadConfig()` now re-applies the shared default if the
saved value is empty (self-heals any unit already stuck with a blank
token, no manual re-entry needed), and `/config`'s save handler refuses to
persist an empty token going forward. Still fully editable to any other
value on `/config` for a rig that needs a non-default token.

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
| `webui.h` | 4-page WebServer UI + REST API (Config/Channels/Live/System — no separate WiFi page, that lives on Config) |

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
3. Connect to it, browse to **http://192.168.4.1/** and set your network in
   the WiFi section of the Config page (only needed for non-standard
   networks). Tap **🔍 Scan for Networks** to list nearby SSIDs with signal
   strength — tap one to fill it in, then just type the password.
4. That's it for connectivity — the module is already talking to the Pi.
   X-Rig-Token defaults to the standard shared rig token (`7804991970`,
   same as every rig's `config.json`), and Pi host defaults to mDNS
   auto-discovery (`_rig-logger._tcp.local`). Nothing to type. Go to
   **⚙ Config** only if you want a friendly Module Name, or need to
   override the token/host for a non-standard rig.
5. Go to **📐 Channels** to configure each of the 8 analog channels you're
   using (all 8 report out of the box — only needed to rename/label them
   or hide ones that aren't wired up).

The Module ID (`MODULE-ABC123`) is fixed — derived from the MAC address, not
editable — so every unit is uniquely and permanently identifiable regardless
of what it's measuring.

---

## Tank Volume (optional derived value)

Turn a level channel into a computed volume — matches `spec-tank-modules.md`
§4b's linear level→volume map, computed on the module itself and sent up as
`derived.volume` (plus top-level `capacity`) in the payload:

```
frac   = (level - valueAtEmpty) / (valueAtFull - valueAtEmpty)   # clamped 0..1
volume = capacity * frac
```

Lives **on the channel itself** — go to **📐 Channels**, find whichever
channel is your level sensor, and check **"Compute Tank Volume from this
channel"**. Its fields appear right there:
- **Capacity** — the tank's full volume. Leave at `0` to leave the feature
  off entirely (no `derived.volume` in the payload at all).
- **Capacity Unit** — `m³` or `gal`.
- **Value @ Empty / Value @ Full** — this channel's already-scaled
  engineering reading (set its Eng Min/Max or zero/max cal above first) at
  0% and 100% full. Doesn't have to be literally 0/capacity — lets a sensor
  mounted partway up the tank, or one that doesn't reach true empty, still
  map correctly.

Only one channel should have this checked at a time (it's per-module, not
per-channel — the first one found wins if more than one somehow is).

If the level channel is faulted (`open`/`over`) or hasn't reported yet,
`derived.volume.status` reflects that instead of showing a fabricated number.
Volume is clamped to `[0, capacity]` — a level reading below "Empty" or above
"Full" reports 0% / 100% rather than a negative or over-capacity number.

The rig UI (`rig-modules.html`) already expects this exact shape
(`derived.volume` + top-level `capacity`) for the tank fill-bar card — no
Pi/UI changes needed, this firmware update alone lights it up.

---

## RS485 Baud Rate — Auto-Detect

No need to know or set the connected board's factory-default baud rate:

- **On boot**, if the configured baud gets no response at all, the firmware
  automatically probes every standard rate (9600, 4800, 19200, 2400, 38400,
  1200, 57600, 115200) and adopts whichever one actually gets an answer.
- **Anytime**, hit **🔍 Auto-Detect Baud Rate** on the **⚙ Config** page to
  re-run the same probe on demand (useful after physically swapping to a
  different board without power-cycling).

Detection works by sending a real Modbus read at each rate and checking for
a CRC-valid response — not a guess. If nothing answers at any rate, check
wiring/DE pin/board power before assuming the baud is the problem.

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
| GET | `/api/wifi/scan` | Scan nearby WiFi networks (SSID, RSSI, secure) for the Config page's WiFi section |
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
