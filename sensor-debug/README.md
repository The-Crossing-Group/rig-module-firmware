# sensor-debug — Standalone RS485/Modbus Debugging Tool

A separate, minimal sketch for digging into a misbehaving RS485/Modbus
sensor — built after the SM7779 radar sensor got stuck outputting garbage
following some experimental register writes on the "real" firmware
(`waveshare-s3-sensors/`).

**No setup page, no login screen, no captive portal.** It connects
straight to the bench-test hotspot with hardcoded credentials and serves
a plain web page at its IP — open the IP in a browser, click/type,
done. Everything is also available over USB Serial if you'd rather not
use WiFi at all.

**This does not touch or replace the production firmware.** It's a
different sketch you flash to the same board (or a spare one) when you
need low-level visibility, then flash the real firmware back when done.

## Flashing

Same hardware/board settings as `waveshare-s3-sensors/`:
- Board: "ESP32S3 Dev Module"
- USB CDC On Boot: Enabled
- Flash Size: 16MB, PSRAM: OPI PSRAM
- Partition Scheme: 16M Flash (3MB APP/9.9MB FATFS)
- To enter download mode if flashing hangs: hold BOOT, tap RESET, release
  RESET, then release BOOT.

RS485 wiring is identical: TX=GPIO17, RX=GPIO18, DE/RE=GPIO21.

## Connecting — no setup needed

WiFi credentials are hardcoded at the top of `sensor-debug.ino`:

```cpp
#define WIFI_SSID "rig-test-ap"
#define WIFI_PASS "7804991970"
```

That's the bench-test hotspot on drill-pi-1 (10.42.0.x). On boot it
connects automatically — no setup AP, no login page, nothing to
configure. Watch the Serial Monitor at boot (115200 baud) for the line:

```
[WiFi] Connected. Open http://10.42.0.xxx/ in a browser — no login needed.
```

Open that IP in any browser on the same network. The page has no
authentication at all — it's meant for quick bench debugging, not
anything exposed beyond the test hotspot.

If you need to point it at a different network (not rig-test-ap), edit
those two `#define` lines and re-flash. There's no runtime WiFi config
screen by design — this tool is meant to need zero setup steps every
time you grab it off the bench.

If WiFi can't connect at all, the web page just won't come up — but
every feature still works over **USB Serial** (open Serial Monitor,
115200 baud, line ending "Newline"):

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
sniff <seconds>                passive listen, no TX (catches auto-report sensors)
```

## What's on the web page

- **Serial config** — baud/parity/stop-bits dropdowns + Apply. The
  production firmware only ever varies baud; if an undocumented
  register write changed the sensor's parity/stop-bit framing instead
  of (or in addition to) baud, this is the only way to find that.
- **Bus scan** — FC03/FC04 probe across a range of addresses.
- **Passive sniff** — listens for N seconds with zero transmission, in
  case the sensor is spontaneously streaming data (some have an
  "auto-report" mode) rather than waiting to be polled.
- **Read registers** — FC03/FC04, any slave/register/count.
- **Write register** — FC06, any slave/register/value (confirm dialog
  before it sends).
- **Raw hex send** — bypasses Modbus framing entirely. Type in exact
  bytes, they go out exactly as typed with DE toggled around the
  transmission, whatever comes back is shown as hex.
- **Traffic log** — last 80 TX/RX pairs, newest first.

Same tolerant reply parsing as the production firmware's v1.8.1 fix:
accepts whatever function code/register count a sensor actually answers
with, and treats slave addresses 0/250 as broadcast — if a valid reply
comes back from a different address than expected, it's shown as data,
not rejected as an error.

## Using it on the SM7779 recovery

The working theory going into this tool: the earlier writes to undocumented
registers 0x0068 (comm mode) and 0x0069 (protocol type) may have changed
something about the sensor's own serial framing, not just its baud —
which would explain why standard 8N1 reads at every baud still come back
as garbage/zeros.

Suggested order of attack:
1. Confirm you're at 9600 8N1 (the sensor's documented default), then
   run a **Bus scan** — see if anything answers at all.
2. If nothing, change baud (2400/4800/19200/38400/57600/115200) and
   scan again at each.
3. If still nothing, sweep parity (Even/Odd) and stop bits (2) at each
   baud — this is the framing possibility the production firmware can't
   test at all.
4. Run **Passive sniff** at any point to check if the sensor is
   spontaneously streaming data instead of waiting to be polled.
5. Once *anything* answers to a scan, **Read** register 0x0000 (liquid
   level) and 0x0064 with count 6 (model code through protocol type) to
   see what state the sensor is actually in.
6. If a valid reply comes back from a different slave address than
   expected, that's fine — the tool shows it and the data anyway, same
   as the production firmware's v1.8.1 broadcast fix.

No action writes anything to the sensor without you clicking it —
Write asks for a confirm dialog; Raw send sends exactly what you type,
nothing more.
