# sensor-debug — Standalone RS485/Modbus Debugging Tool

A separate, minimal sketch for digging into a misbehaving RS485/Modbus
sensor — built after the SM7779 radar sensor got stuck outputting garbage
following some experimental register writes on the "real" firmware
(`waveshare-s3-sensors/`).

**This does not touch or replace that firmware.** It's a different sketch
you flash to the same board (or a spare one) when you need low-level
visibility, then flash the real firmware back when you're done.

## What it gives you (that the production firmware doesn't)

- **Serial framing control** — baud rate, parity (N/E/O), stop bits (1/2),
  changeable live from the web page. The production firmware only ever
  varies baud; if an undocumented register write changed the sensor's
  parity/stop-bit framing instead of (or in addition to) baud, this is
  the only way to find that from software alone.
- **Live traffic log** — every TX/RX, raw or framed, 80 entries, newest
  first, auto-refreshing on the page. No need to watch Serial Monitor.
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

## Flashing

Same hardware/board settings as `waveshare-s3-sensors/`:
- Board: "ESP32S3 Dev Module"
- USB CDC On Boot: Enabled
- Flash Size: 16MB, PSRAM: OPI PSRAM
- Partition Scheme: 16M Flash (3MB APP/9.9MB FATFS)
- To enter download mode if flashing hangs: hold BOOT, tap RESET, release
  RESET, then release BOOT.

RS485 wiring is identical: TX=GPIO17, RX=GPIO18, DE/RE=GPIO21.

## First boot / WiFi

If it can't connect to a saved WiFi network (or none is saved yet), it
starts a setup AP:
- **SSID:** `sensor-debug-setup`
- **Password:** `debug1234`
- Connect and go to `192.168.4.1`, enter your real WiFi SSID/password,
  save. It reboots and connects.

Once online, check your router/AP client list (or the Pi's `rig-test-ap`
hotspot if that's what you're using — see TOOLS.md) for its IP, or watch
the Serial Monitor at boot — it prints its IP once connected.

To reconfigure WiFi later without re-flashing, visit `/wifi` on the
running tool.

## Using it on the SM7779 recovery

The working theory going into this tool: the earlier writes to undocumented
registers 0x0068 (comm mode) and 0x0069 (protocol type) may have changed
something about the sensor's own serial framing, not just its baud —
which would explain why standard 8N1 reads at every baud still come back
as garbage/zeros.

Suggested order of attack:
1. **Bus scan** at 9600 8N1 (the sensor's documented default) — see if
   anything answers at all.
2. If nothing, sweep baud (2400/4800/9600/19200/38400/57600/115200) at
   8N1 via the Serial Config panel + Bus Scan again at each.
3. If still nothing, sweep parity (8E1, 8O1) and stop bits (8N2, 8E2,
   8O2) at each baud — this is the framing possibility the production
   firmware can't test at all.
4. Once *anything* answers to a scan, use Register Read against register
   0x0000 (liquid level) and 0x0064-0x0069 (model code through protocol
   type) to see what state the sensor is actually in.
5. If a valid reply comes back from a DIFFERENT slave address than
   expected (the "actualSlaveId" shown in the read result), that's fine
   — broadcast/mismatched replies are handled, same as the production
   firmware's v1.8.1 fix.

No feature of this tool writes anything to the sensor automatically —
every write (register write, raw hex) is a manual, confirmed action.
