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

Each `.ino` must stay alone in a folder of the same name (Arduino IDE rule).

## Data format (contract between firmware, bridge and reader)

Serial line, one per sample: `t_ms,ax,ay,az,gx,gy,gz,backlog`
(accel in g, gyro in deg/s with offset removed, backlog = samples queued on board).
BLE notify characteristic `8f1c0003-4b6e-4a8e-9f3a-2d6c5e7a1b00`, 18 bytes,
`struct` format `<I6hH` (ms, milli-g x3, 0.1 deg/s x3, backlog).
Change all three sides together if the format changes.

`accel_reader --json` prints one JSON object per 100 ms on stdout
(`{"sensors":[{"id":1,"state":"ok"|"waiting"|"absent","roll",...}],"between":x|null}`);
other stdout lines are messages. The UI depends on this; keep it compatible.

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
