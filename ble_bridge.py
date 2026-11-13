#!/usr/bin/env python3
"""
ble_bridge.py - connect to the XIAO accelerometer boards over Bluetooth
and make each one look like a serial port, so accel_reader (the Eclipse
program) can read them exactly as it does over USB.

    XIAO-ACCEL-1  ->  /tmp/accel1
    XIAO-ACCEL-2  ->  /tmp/accel2

Setup (once):
    pip3 install --user --break-system-packages bleak

Run:
    python3 ble_bridge.py              # leave this running
    then run accel_reader with the arguments:  /tmp/accel1 /tmp/accel2

Requires the boards to run xiao_imu.ino.

Test without any boards (fake data):
    python3 ble_bridge.py --simulate
"""
import asyncio
import math
import os
import pty
import struct
import sys
import time
import tty

IMU_DATA_UUID = "8f1c0003-4b6e-4a8e-9f3a-2d6c5e7a1b00"
BATTERY_UUID = "8f1c0004-4b6e-4a8e-9f3a-2d6c5e7a1b00"  # uint16 millivolts
# 18-byte packet: uint32 t_ms, int16 ax,ay,az (milli-g),
#                 int16 gx,gy,gz (0.1 deg/s), uint16 backlog
PACKET = struct.Struct("<I6hH")

DEVICES = {
    "XIAO-ACCEL-1": "/tmp/accel1",
    "XIAO-ACCEL-2": "/tmp/accel2",
}


class PtyPort:
    """A pseudo serial port: we write lines in, accel_reader reads them out."""

    def __init__(self, link_path):
        self.link_path = link_path
        self.master, self.slave = pty.openpty()
        tty.setraw(self.slave)               # no echo / line processing
        os.set_blocking(self.master, False)  # never stall the bridge
        if os.path.lexists(link_path):
            os.unlink(link_path)
        os.symlink(os.ttyname(self.slave), link_path)

    def write_line(self, line):
        try:
            os.write(self.master, line.encode())
        except BlockingIOError:
            pass  # nobody reading right now; drop this sample

    def close(self):
        if os.path.islink(self.link_path):
            os.unlink(self.link_path)
        os.close(self.master)
        os.close(self.slave)


def log(name, msg):
    print(f"[{name}] {msg}", flush=True)


async def run_device(name, port, scan_lock):
    from bleak import BleakClient, BleakScanner

    while True:
        try:
            async with scan_lock:  # BlueZ dislikes overlapping scans
                device = await BleakScanner.find_device_by_name(name, timeout=10.0)
            if device is None:
                log(name, "not found yet - is it powered on? retrying...")
                continue

            disconnected = asyncio.Event()

            def on_data(_char, data: bytearray):
                if len(data) >= PACKET.size:
                    t, ax, ay, az, gx, gy, gz, backlog = PACKET.unpack(
                        bytes(data[:PACKET.size]))
                    port.write_line(
                        f"{t},{ax/1000:.3f},{ay/1000:.3f},{az/1000:.3f},"
                        f"{gx/10:.1f},{gy/10:.1f},{gz/10:.1f},{backlog}\n")

            async with BleakClient(
                device, disconnected_callback=lambda _c: disconnected.set()
            ) as client:
                await client.start_notify(IMU_DATA_UUID, on_data)
                await start_battery(client, name, port)
                log(name, f"connected -> {port.link_path}")
                await disconnected.wait()
            log(name, "disconnected, reconnecting...")
        except asyncio.CancelledError:
            raise
        except Exception as e:  # keep going on any Bluetooth hiccup
            log(name, f"error: {e}; retrying in 2 s")
            await asyncio.sleep(2)


async def start_battery(client, name, port):
    """Pass the battery level on as a "BAT <mV>" line. Older firmware has no
    battery characteristic; that's fine, the data still flows."""
    def on_battery(_char, data: bytearray):
        if len(data) >= 2:
            port.write_line(f"BAT {int.from_bytes(data[:2], 'little')}\n")
    try:
        await client.start_notify(BATTERY_UUID, on_battery)
        on_battery(None, await client.read_gatt_char(BATTERY_UUID))
    except Exception:
        log(name, "no battery reading (update the firmware to get one)")


async def simulate(name, port, index):
    """Fake board: sensor 1 turns slowly around the vertical axis (45 deg/s),
    sensor 2 sits still, tilted 30 degrees."""
    log(name, f"simulating -> {port.link_path}")
    t0 = time.monotonic()
    next_bat = 2.0
    tilt = math.radians(30) if index == 1 else 0.0
    gz = 45.0 if index == 0 else 0.0
    while True:
        t_ms = int((time.monotonic() - t0) * 1000)
        ay, az = math.sin(tilt), math.cos(tilt)
        port.write_line(f"{t_ms},0.000,{ay:.3f},{az:.3f},0.0,0.0,{gz:.1f},0\n")
        if t_ms / 1000 >= next_bat:  # fake battery, slowly draining
            next_bat += 5
            port.write_line(f"BAT {int(4100 - 120 * index - t_ms / 1000)}\n")
        await asyncio.sleep(1 / 104)


async def main():
    sim = "--simulate" in sys.argv
    if not sim:
        try:
            import bleak  # noqa: F401
        except ImportError:
            print("ERROR: the Bluetooth library 'bleak' is not installed. Run:\n"
                  "    pip3 install --user --break-system-packages bleak", flush=True)
            sys.exit(1)
    ports = {name: PtyPort(path) for name, path in DEVICES.items()}
    scan_lock = asyncio.Lock()
    print("Bridge running. Run accel_reader with: "
          + " ".join(DEVICES.values()) + "   (Ctrl+C to stop)", flush=True)
    try:
        tasks = [
            simulate(n, p, i) if sim else run_device(n, p, scan_lock)
            for i, (n, p) in enumerate(ports.items())
        ]
        await asyncio.gather(*tasks)
    finally:
        for p in ports.values():
            p.close()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nBridge stopped.")
