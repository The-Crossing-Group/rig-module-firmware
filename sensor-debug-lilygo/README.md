# sensor-debug-lilygo — Standalone RS485/Modbus Debugging Tool (LilyGo T-CAN485)

LilyGo T-CAN485 hardware port of `sensor-debug/` (the Waveshare
ESP32-S3-RS485-CAN variant). **Separate, independently-maintained
sketch — not kept in sync with `sensor-debug/`.** Don't assume a change
made in one is (or should be) mirrored in the other.

Built after the SM7779 radar sensor got stuck outputting garbage
following experimental register writes on the "real" firmware. A
different sketch you flash to the same board (or a spare one) when you
need low-level RS485/Modbus visibility, then flash the real firmware
back when done — doesn't touch or depend on any production firmware in
this repo.

## Flashing

- Board: **"ESP32 Dev Module"** (plain ESP32/WROOM-32, **NOT** ESP32-S3)
- Default partition scheme is fine — no PSRAM option (this is a plain
  ESP32, not ESP32-S3)
- To enter download mode if flashing hangs: hold **BOOT**, tap **RESET**,
  release **RESET**, then release **BOOT**

## Hardware pins (LilyGo T-CAN485)

Verified against LilyGo's own factory RS485 example:

- RS485: TX=GPIO22, RX=GPIO21, DE/RE=GPIO17 (MAX13487, HIGH=transmit)
- `PIN_5V_EN`=GPIO16 — 5V booster enable, **must be HIGH or RS485 has no
  power at all** (this board has no onboard 3.3V RS485 supply like the
  Waveshare does)
- `RS485_SE`=GPIO19 — MAX13487 `/SHDN` pin, must be HIGH to un-shutdown
  the transceiver (also unique to this board vs the Waveshare variant)
- Onboard WS2812 RGB LED: GPIO4 — used for status (see below)

## Status LED (WS2812, onboard) — not present on `sensor-debug/`

The Waveshare variant has no programmable LED; this LilyGo port adds one
since the board has it:

- Slow dim-blue blink: alive/idle heartbeat — **stops blinking means the
  firmware hung or crashed**, visible without a Serial Monitor open
- Two amber blinks: boot
- Brief green flash: last Modbus read/write/raw send got a valid reply
- Brief red flash: last action failed or timed out

## Connecting — no login screen

On boot the board broadcasts its own open WiFi network:

```cpp
#define AP_SSID "sensor-debug-lilygo"
#define AP_PASS ""   // empty = open network, no password
```

Connect your phone/laptop to **"sensor-debug-lilygo"** like any open
WiFi network — no captive portal, no credentials to enter on the ESP32
side. Then open **http://192.168.4.1/** in a browser (the standard
ESP32 softAP default IP) and the tools are right there.

If you want a password on the AP, set `AP_PASS` to something 8+
characters and re-flash (`WiFi.softAP()` silently ignores anything
shorter and stays open). No runtime WiFi config screen by design — this
tool needs zero setup steps every time you grab it off the bench.

Every action taken from the web page also prints its result to the
**Serial Monitor** with a `[Web]` prefix, so you get the same output
whether you click it in the browser or type it over USB.

If WiFi somehow doesn't come up, every feature still works over **USB
Serial** (115200 baud, line ending "Newline"):

```
help                          show the command list
baud <n>                      set RS485 baud, e.g. baud 9600
parity <n|e|o>                set parity: none/even/odd
stop <1|2>                    set stop bits
status                        show current serial config
scan [maxAddr]                scan slave addresses 1..maxAddr (default 20)
read <slaveId> <fc> <reg> [count]
                               FC03/FC04 read, e.g.: read 1 3 0x0000 2
write <slaveId> <reg> <value> FC06 write (asks "type y to confirm")
write! <slaveId> <reg> <value> FC06 write, no confirmation prompt
raw <hex bytes>               e.g.: raw 01 03 00 00 00 01 84 0A
log [n]                       last n traffic log entries
sniff <seconds>               passive listen, no TX (catches auto-report sensors)
autosniff <on|off>            toggle continuous background traffic capture
bitscope [ms]                 raw electrical capture, bypasses UART framing entirely
recover                       SM7779 recovery sweep (see below)
sweep <slaveId> <fc> <reg> [count]
                               baud sweep across every standard rate
qdw90a [slaveId]               labeled read of all 7 QDW90A/QDY30A registers
```

## What's on the web page

Same tool set as `sensor-debug/` (see that README for full detail on
each — the two sketches share the underlying debug logic in
`modbus_debug.h`/`bitscope.h`, even though the `.ino` wrapper and pins
differ):

- **Serial config** — baud/parity/stop-bits dropdowns + Apply
- **Bus scan** — FC03/FC04 probe across a range of addresses
- **Auto traffic capture** — always running, logs unsolicited bytes with
  an `[Auto]` prefix; Pause/Resume button
- **Passive sniff** — listen N seconds with zero transmission
- **Bitscope** — raw electrical capture bypassing UART framing entirely;
  measures real bit period off the wire and brute-forces every
  byte-alignment against Modbus CRC16
- **Baud Sweep** — tries a register read at 8N1 across every standard
  baud, stops on the first clean reply
- **SM7779 Recovery Sweep** — cycles all 6 parity/stop-bit combos at
  9600 baud, sending **unicast** FC06 resets (`1 -> 0x0068`,
  `1 -> 0x0069`) to slave 1 and checking for a real ack/exception (not a
  broadcast "ok", which can't prove the framing actually matched). Falls
  back to a best-effort broadcast pass if nothing acks, then auto-runs a
  10s passive sniff.
- **QDW90A / QDY30A Pressure Sensor Probe** — one-click labeled read of
  all 7 registers for this sensor family. Needs genuine 24V power.
- **Read registers** — FC03/FC04, any slave/register/count
- **Write register** — FC06, any slave/register/value (confirm dialog)
- **Raw hex send** — bypasses Modbus framing entirely
- **Traffic log** (`/log`) — last 80 TX/RX pairs, newest first, live-updating

No action writes anything to the sensor without you clicking it — Write
asks for a confirm dialog; Raw send sends exactly what you type, nothing
more.

## Using it on the SM7779 recovery

Same working theory and suggested order of attack as `sensor-debug/`:
confirm 9600 8N1 first, Bus Scan; if nothing, sweep baud
(2400/4800/19200/38400/57600/115200) and scan again at each; if still
nothing, sweep parity/stop bits at each baud (or just run **Recovery
Sweep**, which automates that exact combination); if everything comes
back garbled, run **Bitscope** during a known auto-report burst instead
of continuing to guess framing combos. See `sensor-debug/README.md`'s
"Using it on the SM7779 recovery" section for the full written-out
walkthrough — it applies here unchanged.
