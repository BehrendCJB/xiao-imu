# XIAO IMU project

Two Seeed XIAO nRF52840 Sense boards stream accelerometer + gyroscope data
(104 Hz) to an Intel NUC running Ubuntu, over USB serial or Bluetooth LE.
The NUC fuses each board's data into roll / pitch / yaw and shows the angle
between the two boards. Intended use: motion capture while swimming.

Full background, data formats and troubleshooting history:
https://claude.ai/artifact/BC72MCkQk4YKway6YJwZbE ("XIAO IMU Project: Linux Code Reference").
Keep that doc in step when behaviour, formats or setup change.

## Files

| Path | What it is |
|---|---|
| `accel_reader.c` | C11 reader: polls up to 2 serial ports, Madgwick filter, status line, optional `--csv` log |
| `ble_bridge.py` | Python 3 + `bleak`: connects to `XIAO-ACCEL-1/2`, exposes them as pseudo serial ports `/tmp/accel1`, `/tmp/accel2`; `--simulate` for no hardware |
| `ui/ui_server.py` | Python 3 + `aiohttp` web control panel on port 8080: starts/stops the bridge and `accel_reader --json`, streams live angles over a websocket, saves CSVs to `recordings/` |
| `ui/index.html` | The phone-friendly page (plain HTML/JS, no CDN: pool Wi-Fi may have no internet) |
| `firmware/xiao_imu/xiao_imu.ino` | Current board firmware (accel + gyro, 104 Hz, BLE + USB, ~40 s buffer) |
| `firmware/xiao_accel/xiao_accel.ino` | Old USB-only accel firmware (`x,y,z` lines); reader still accepts it |
| `.project`, `.cproject` | Eclipse CDT **Managed Build** project named `accelerometer_1` (not CMake) |
| `analysis/` | Offline motion analysis of recorded CSVs (Python 3 + numpy, scipy, matplotlib), see below |

Each `.ino` must stay alone in a folder of the same name (Arduino IDE rule).

## Motion analysis (analysis/)

Boards are worn on the backs of the hands (board 1 left, board 2 right by
default), flat, component side up, +x toward the fingers (`--finger-axis`
if not). Runs on recorded CSVs, not live:
`python3 analysis/analyze.py swim|shot <csv> [--hands left,right] [--sync start|tap]`
writes `<csv stem>_analysis/` (summary.txt, summary.json, one PNG per hand).
Deps (agreed 2026-11-16): `pip3 install --user --break-system-packages numpy scipy matplotlib`.

- `imu.py`: load CSV per board (board_ms time, split at gaps > 0.25 s,
  resample to 104 Hz), orientation = gyro integration + tilt-drift removal
  (20 s window mean of world accel = gravity). Not Madgwick: hand
  acceleration fooled it by 12-22 deg on synthetic strokes; this gives ~1-2.
  Hand angles: finger pitch, hand roll, palm direction; world linear accel.
- `swim.py`: cycle period from gyro autocorrelation; cycles between the
  hand's highest points; per cycle rate, finger pitch, palm angle to
  backward, hand path relative to body (velocity mean removed, loop
  closed); `arm_timing` = phase between hands (~50 % alternating).
- `shot.py`: shot = gyro peak > 400 deg/s with a rest before it; path
  integrated rest-to-rest with zero velocity at both ends; release = peak
  wrist snap; release angle, speed, rise, dip, finger pitch.
- `synth.py`: synthetic swim/shot CSVs with known truth. After changing the
  analysis, check: `python3 analysis/synth.py swim /tmp/s.csv` then
  `analyze.py swim /tmp/s.csv` (expect ~10 cycles, period 1.4 s, depth and
  fore-aft ~1.2 m, palm angle ~29 deg, timing ~50 %); same for `shot`
  (release angle ~58 deg, speed ~1.5 m/s, rise ~0.58 m).

## Data format (contract between firmware, bridge and reader)

Serial line, one per sample: `t_ms,ax,ay,az,gx,gy,gz,backlog`
(accel in g, gyro in deg/s with offset removed, backlog = samples queued on board).
BLE notify characteristic `8f1c0003-4b6e-4a8e-9f3a-2d6c5e7a1b00`, 18 bytes,
`struct` format `<I6hH` (ms, milli-g x3, 0.1 deg/s x3, backlog).
Battery: firmware measures every 10 s (P0.14 held LOW, divider on P0.31,
`ADC_FULL_SCALE_V` = 3.3 (verified with a meter), per-board `VBAT_CAL` chosen by
`BOARD_ID` (1.005 / 1.013, meter ÷ panel)). USB sends a `BAT <mV>` line;
BLE has characteristic `8f1c0004-...-2d6c5e7a1b00` (uint16 mV, read + notify),
which the bridge turns into the same `BAT <mV>` line. The reader adds
`bat_mv` to its JSON; the UI converts mV to % with a LiPo curve, stretched above 3.80 V so the
measured full charge (4.08 V, `FULL_MV` in index.html) reads 100%.
USB board identity: sending `I` to a board's USB port gets `ID <BOARD_ID>`
back. The UI asks every `/dev/serial/by-id/*` port before starting the
reader and passes the ports in board order (Linux numbers ttyACM* in
plug-in order); over BLE the names `XIAO-ACCEL-<id>` already fix the order.
Change all three sides together if the format changes.

`accel_reader --json` prints one JSON object per 100 ms on stdout
(`{"sensors":[{"id":1,"state":"ok"|"waiting"|"absent","roll",...}],"between":x|null}`);
other stdout lines are messages. The UI depends on this; keep it compatible.
`--csv-wait` (with `--csv`) opens the file but writes no rows until SIGUSR1;
the UI starts the reader at Start (boards connect during the start delay)
and sends SIGUSR1 when collecting begins (later of: delay over, first data).
Zero: SIGUSR2 makes each board's current orientation its reference `q0`
(reader prints `Zeroed: board 1 board 2`). Displayed / JSON angles are then
`conj(q0)*q`, and "between" is the change in board 2's orientation seen from
board 1 since the Zero. CSV keeps absolute roll/pitch/yaw and appends
`roll0_deg,pitch0_deg,yaw0_deg` (zeroed). A gap (> 0.25 s of lost data)
re-levels roll/pitch from gravity but keeps yaw. UI: `/api/zero` + Zero button.

## UI

Run from the project folder: `python3 ui/ui_server.py` (deps:
`pip3 install --user --break-system-packages aiohttp`). It runs Eclipse's
`Debug/accelerometer_1`, else `./accel_reader` (override with `--reader`).
New features = a control on the page + an `/api/...` handler in the server.
Test here with `--simulate` mode via the API or headless Chrome.

## Build and check

- Eclipse (on the NUC): Managed Build; the math library `m` is set under
  C/C++ Build → Settings → GCC C Linker → Libraries. Output goes to `Debug/`.
- Command line check: `gcc -std=c11 -Wall -Wextra -O2 -o /tmp/accel_reader accel_reader.c -lm`
- Bridge syntax check: `python3 -m py_compile ble_bridge.py`
- Test without hardware: `python3 ble_bridge.py --simulate`, then
  `./accel_reader /tmp/accel1 /tmp/accel2`.
- Firmware is built and uploaded from the Arduino IDE on the NUC; it can't be
  compiled in a cloud session.

## Rules

- Everything on the NUC runs as the normal user (`cjb22`); never suggest `sudo`
  except for `apt install` and one-time system setup.
- Changes go straight to `main`. After a push the user runs `git pull` in the
  project folder, then in Eclipse: Refresh → Clean Project → Build.
- Keep code plain C11 / Python 3 with no new dependencies unless agreed.
- Never print or log GitHub tokens.
