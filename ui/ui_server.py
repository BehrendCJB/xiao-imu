#!/usr/bin/env python3
"""
ui_server.py - web control panel for the XIAO IMU project.

Serves a page you can open on the NUC or on any phone / laptop on the same
network. From the page you can start and stop data collection over USB,
Bluetooth (starts ble_bridge.py for you) or simulated data, optionally
record a CSV, watch live angles, and download recordings.

Under the hood it runs the existing programs:
    accel_reader --json [ports] [--csv recordings/<file>.csv]
    ble_bridge.py [--simulate]          (Bluetooth / simulated modes only)

Setup (once):
    pip3 install --user --break-system-packages aiohttp

Run (from the project folder):
    python3 ui/ui_server.py
then open the address it prints, e.g. http://192.168.1.20:8080

Options:
    --port N        web port (default 8080)
    --reader PATH   accel_reader program to run (default: Eclipse's
                    Debug/accelerometer_1, else ./accel_reader)
"""
import argparse
import asyncio
import collections
import json
import os
import re
import signal
import socket
import sys
import time
from datetime import datetime
from pathlib import Path

from aiohttp import WSMsgType, web

ROOT = Path(__file__).resolve().parent.parent      # project folder
UI_DIR = Path(__file__).resolve().parent
DATA_DIR = ROOT / "recordings"
BRIDGE = ROOT / "ble_bridge.py"
BRIDGE_PORTS = ["/tmp/accel1", "/tmp/accel2"]
READER_CANDIDATES = [ROOT / "Debug" / "accelerometer_1", ROOT / "accel_reader"]
MODES = ("usb", "ble", "sim")


def find_reader(explicit):
    if explicit:
        return Path(explicit).resolve()
    for p in READER_CANDIDATES:
        if p.is_file() and os.access(p, os.X_OK):
            return p
    return None


def lan_addresses():
    """Best guess at the addresses other devices can reach us on."""
    addrs = set()
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("10.255.255.255", 1))  # no packet is sent
        addrs.add(s.getsockname()[0])
        s.close()
    except OSError:
        pass
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            addrs.add(info[4][0])
    except OSError:
        pass
    addrs.discard("127.0.0.1")
    addrs.discard("127.0.1.1")
    return sorted(addrs) or ["localhost"]


class Session:
    """Owns the running bridge / reader processes and the shared state."""

    def __init__(self, reader_arg):
        self.reader_arg = reader_arg
        self.reader = None          # asyncio subprocess
        self.bridge = None
        self.running = False
        self.busy = False           # starting or stopping
        self.mode = None
        self.started_at = None
        self.csv_name = None
        self.status = None          # last JSON status from the reader
        self.log = collections.deque(maxlen=300)
        self.clients = set()
        self.tasks = set()

    # ---------- broadcasting ----------

    def state(self):
        return {
            "type": "state",
            "running": self.running,
            "busy": self.busy,
            "mode": self.mode,
            "started_at": self.started_at,
            "csv": self.csv_name,
            "server_time": time.time(),
        }

    async def send_all(self, msg):
        data = json.dumps(msg)
        for ws in list(self.clients):
            try:
                await ws.send_str(data)
            except (ConnectionError, RuntimeError):
                self.clients.discard(ws)

    async def add_log(self, text):
        line = f"{datetime.now():%H:%M:%S} {text}"
        self.log.append(line)
        print(line, flush=True)
        await self.send_all({"type": "log", "line": line})

    async def push_state(self):
        await self.send_all(self.state())

    def spawn(self, coro):
        t = asyncio.create_task(coro)
        self.tasks.add(t)
        t.add_done_callback(self.tasks.discard)
        return t

    # ---------- process output ----------

    async def pump(self, stream, prefix, parse_json=False):
        while True:
            raw = await stream.readline()
            if not raw:
                return
            line = raw.decode(errors="replace").rstrip()
            if not line:
                continue
            if parse_json and line.startswith("{"):
                try:
                    self.status = json.loads(line)
                except ValueError:
                    continue
                await self.send_all({"type": "status", **self.status})
            else:
                await self.add_log(f"{prefix} {line}")

    async def watch_reader(self, proc):
        out = self.spawn(self.pump(proc.stdout, "[reader]", parse_json=True))
        err = self.spawn(self.pump(proc.stderr, "[reader]"))
        code = await proc.wait()
        await asyncio.gather(out, err, return_exceptions=True)
        if proc is self.reader:  # ended on its own (not via Stop)
            await self.add_log(f"[ui] reader exited (code {code})")
            await self.stop(reader_done=True)

    # ---------- start / stop ----------

    async def start(self, mode, record, name):
        if self.running or self.busy:
            raise web.HTTPConflict(text="already running")
        if mode not in MODES:
            raise web.HTTPBadRequest(text="mode must be usb, ble or sim")
        reader_path = find_reader(self.reader_arg)
        if reader_path is None or not reader_path.is_file():
            raise web.HTTPBadRequest(
                text="accel_reader not found: build the project in Eclipse "
                     "(Debug/accelerometer_1) or run: gcc -O2 -o accel_reader "
                     "accel_reader.c -lm")

        self.busy = True
        await self.push_state()
        try:
            ports = []
            if mode in ("ble", "sim"):
                ports = BRIDGE_PORTS
                t0 = time.time()
                args = [sys.executable, "-u", str(BRIDGE)]
                if mode == "sim":
                    args.append("--simulate")
                self.bridge = await asyncio.create_subprocess_exec(
                    *args, stdout=asyncio.subprocess.PIPE,
                    stderr=asyncio.subprocess.STDOUT, cwd=str(ROOT))
                self.spawn(self.pump(self.bridge.stdout, "[bridge]"))
                await self.add_log(f"[ui] started bridge ({mode})")
                if not await self.wait_for_ports(ports, t0):
                    await self.add_log("[ui] bridge did not create its ports")
                    await self.stop_bridge()
                    return

            args = [str(reader_path), *ports, "--json"]
            self.csv_name = None
            if record:
                DATA_DIR.mkdir(exist_ok=True)
                safe = re.sub(r"[^A-Za-z0-9_-]+", "-", name or "").strip("-")
                stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
                self.csv_name = f"{stamp}_{safe}.csv" if safe else f"{stamp}.csv"
                args += ["--csv", str(DATA_DIR / self.csv_name)]

            self.reader = await asyncio.create_subprocess_exec(
                *args, stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE, cwd=str(ROOT))
            self.status = None
            self.mode = mode
            self.running = True
            self.started_at = time.time()
            await self.add_log(f"[ui] started {reader_path.name} ({mode})"
                               + (f", recording {self.csv_name}" if record else ""))
            self.spawn(self.watch_reader(self.reader))
        finally:
            self.busy = False
            await self.push_state()

    async def wait_for_ports(self, ports, since, timeout=5.0):
        end = time.time() + timeout
        while time.time() < end:
            if self.bridge.returncode is not None:
                return False
            try:
                if all(os.lstat(p).st_mtime >= since - 1 for p in ports):
                    return True
            except FileNotFoundError:
                pass
            await asyncio.sleep(0.1)
        return False

    @staticmethod
    async def end_process(proc, wait=3.0):
        if proc is None or proc.returncode is not None:
            return
        try:
            proc.send_signal(signal.SIGINT)   # lets it close files cleanly
            await asyncio.wait_for(proc.wait(), wait)
        except asyncio.TimeoutError:
            proc.kill()
            await proc.wait()
        except ProcessLookupError:
            pass

    async def stop_bridge(self):
        bridge, self.bridge = self.bridge, None
        if bridge:
            await self.end_process(bridge)
            await self.add_log("[ui] bridge stopped")

    async def stop(self, reader_done=False):
        if not self.running and not self.bridge:
            return
        self.busy = True
        await self.push_state()
        reader, self.reader = self.reader, None
        if not reader_done:
            await self.end_process(reader)
        await self.stop_bridge()
        self.running = False
        self.busy = False
        if self.csv_name:
            await self.add_log(f"[ui] saved recordings/{self.csv_name}")
        await self.add_log("[ui] stopped")
        await self.push_state()
        await self.send_all({"type": "recordings", "files": list_recordings()})


def list_recordings():
    if not DATA_DIR.is_dir():
        return []
    files = sorted(DATA_DIR.glob("*.csv"), key=lambda p: p.stat().st_mtime, reverse=True)
    return [{"name": p.name, "bytes": p.stat().st_size,
             "modified": p.stat().st_mtime} for p in files[:50]]


# ---------- web handlers ----------

async def index(_request):
    return web.FileResponse(UI_DIR / "index.html")


async def ws_handler(request):
    session = request.app["session"]
    ws = web.WebSocketResponse(heartbeat=20)
    await ws.prepare(request)
    session.clients.add(ws)
    await ws.send_str(json.dumps(session.state()))
    await ws.send_str(json.dumps({"type": "loghistory", "lines": list(session.log)}))
    await ws.send_str(json.dumps({"type": "recordings", "files": list_recordings()}))
    if session.status:
        await ws.send_str(json.dumps({"type": "status", **session.status}))
    try:
        async for msg in ws:
            if msg.type == WSMsgType.ERROR:
                break
    finally:
        session.clients.discard(ws)
    return ws


async def api_start(request):
    body = await request.json() if request.can_read_body else {}
    await request.app["session"].start(
        body.get("mode", "usb"), bool(body.get("record")), body.get("name", ""))
    return web.json_response(request.app["session"].state())


async def api_stop(request):
    await request.app["session"].stop()
    return web.json_response(request.app["session"].state())


async def api_recordings(_request):
    return web.json_response(list_recordings())


async def download(request):
    name = request.match_info["name"]
    path = (DATA_DIR / name).resolve()
    if path.parent != DATA_DIR.resolve() or path.suffix != ".csv" or not path.is_file():
        raise web.HTTPNotFound()
    return web.FileResponse(path, headers={
        "Content-Disposition": f'attachment; filename="{path.name}"'})


async def on_shutdown(app):
    await app["session"].stop()


def main():
    ap = argparse.ArgumentParser(description="XIAO IMU web control panel")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--reader", help="path to the accel_reader program")
    opts = ap.parse_args()

    app = web.Application()
    app["session"] = Session(opts.reader)
    app.router.add_get("/", index)
    app.router.add_get("/ws", ws_handler)
    app.router.add_post("/api/start", api_start)
    app.router.add_post("/api/stop", api_stop)
    app.router.add_get("/api/recordings", api_recordings)
    app.router.add_get("/recordings/{name}", download)
    app.on_shutdown.append(on_shutdown)

    reader = find_reader(opts.reader)
    print(f"Reader: {reader or 'NOT FOUND - build the project first'}")
    print("Open one of these in a browser (Ctrl+C to stop):")
    for a in lan_addresses():
        print(f"    http://{a}:{opts.port}")
    web.run_app(app, host="0.0.0.0", port=opts.port, print=None)


if __name__ == "__main__":
    main()
