# sensor-debug — Standalone RS485/Modbus Debugging Tool (USB serial only)

A separate, minimal sketch for digging into a misbehaving RS485/Modbus
sensor — built after the SM7779 radar sensor got stuck outputting garbage
following some experimental register writes on the "real" firmware
(`waveshare-s3-sensors/`).

**No WiFi. No web page. No setup steps.** Plug the board into USB, open
the Arduino Serial Monitor at 115200 baud (line ending: Newline), type
commands, read results.

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

After flashing, open **Tools → Serial Monitor**, set baud to **115200**
and line ending to **Newline**. You'll see a `> ` prompt.

## Commands

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
raw <hex bytes>               send exact bytes, e.g.: raw 01 03 00 00 00 01 84 0A
log [n]                       show last n traffic log entries (default 15)
sniff <seconds>               listen passively for unsolicited bus traffic (no TX)
```

## What it gives you (that the production firmware doesn't)

- **Serial framing control** — baud, parity (N/E/O), stop bits (1/2),
  changeable live with `baud`/`parity`/`stop` commands. The production
  firmware only ever varies baud; if an undocumented register write
  changed the sensor's parity/stop-bit framing instead of (or in
  addition to) baud, this is the only way to find that from software
  alone.
- **Live traffic log** — every TX/RX, raw or framed, 80 entries. `log`
  prints them straight to Serial Monitor.
- **Passive sniff** — `sniff 10` listens for 10 seconds without
  transmitting anything, in case the sensor is spontaneously streaming
  data (some sensors have an "auto-report" mode) rather than waiting to
  be polled.
- **Bus scan** — FC03/FC04 probe across a range of addresses.
- **Register read** (FC03/FC04, any slave/register/count) and **register
  write** (FC06, any slave/register/value) — same tolerant reply parsing
  as the production firmware (accepts whatever function code/register
  count a sensor actually answers with, treats 0/250 as broadcast
  addresses whose reply legitimately comes back from a different
  address).
- **Raw hex send** — bypasses Modbus framing entirely. Type in exact
  bytes, they go out exactly as typed with DE toggled around the
  transmission, whatever comes back is shown as hex. Useful if you
  suspect the sensor isn't speaking standard Modbus RTU at all anymore.

## Using it on the SM7779 recovery

The working theory going into this tool: the earlier writes to undocumented
registers 0x0068 (comm mode) and 0x0069 (protocol type) may have changed
something about the sensor's own serial framing, not just its baud —
which would explain why standard 8N1 reads at every baud still come back
as garbage/zeros.

Suggested order of attack:
1. `status` to confirm you're starting at 9600 8N1 (the sensor's
   documented default), then `scan` — see if anything answers at all.
2. If nothing, `baud 4800` (or 2400/19200/38400/57600/115200) and `scan`
   again at each.
3. If still nothing, sweep parity (`parity e`, `parity o`) and stop bits
   (`stop 2`) at each baud — this is the framing possibility the
   production firmware can't test at all.
4. `sniff 10` at any point to check if the sensor is spontaneously
   streaming data instead of waiting to be polled.
5. Once *anything* answers to a `scan`, use `read <addr> 3 0x0000 1`
   (liquid level) and `read <addr> 3 0x0064 6` (model code through
   protocol type) to see what state the sensor is actually in.
6. If a valid reply comes back from a DIFFERENT slave address than
   expected, that's fine — the tool reports it and shows the data
   anyway, same as the production firmware's v1.8.1 broadcast fix.

No command writes anything to the sensor without you typing it —
`write` asks you to confirm with `y`; `write!` skips the prompt only if
you use it explicitly. `raw` sends exactly what you type, nothing more.
