#!/usr/bin/env python3
"""
ui_server.py - web control panel for the XIAO IMU project.

Serves a page you can open on the NUC or on any phone / laptop on the same
network. From the page you can start and stop data collection over USB,
Bluetooth (starts ble_bridge.py for you) or simulated data, with an optional
start delay and collect time, optionally record a CSV, watch live angles, and
download recordings.

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
import glob
import select
import socket
import sys
import termios
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


def is_program(p):
    return p.is_file() and os.access(p, os.X_OK) and p.suffix in ("", ".elf", ".out")


def find_reader(explicit):
    if explicit:
        return Path(explicit).resolve()
    # Eclipse names the program after the project, which may have been
    # renamed, and an old hand-built ./accel_reader may be lying around:
    # take the most recently built of all of them.
    found = [p for p in READER_CANDIDATES if is_program(p)]
    found += [p for d in ("Debug", "Release") if (ROOT / d).is_dir()
              for p in (ROOT / d).iterdir() if is_program(p)]
    return max(found, key=lambda p: p.stat().st_mtime) if found else None


def ask_board_id(path, timeout=1.5):
    """Send 'I' to a board on a USB serial port; it answers "ID <n>".
    Returns n, or None (old firmware, or no answer)."""
    try:
        fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError:
        return None
    try:
        attrs = termios.tcgetattr(fd)
        attrs[0] = attrs[1] = attrs[3] = 0                       # raw
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[4] = attrs[5] = termios.B115200
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIFLUSH)
        buf, end, next_ask = b"", time.time() + timeout, 0.0
        while time.time() < end:
            if time.time() >= next_ask:          # ask again in case it was busy
                os.write(fd, b"I")
                next_ask = time.time() + 0.3
            if select.select([fd], [], [], 0.05)[0]:
                buf += os.read(fd, 4096)
                m = re.search(rb"(?:^|[\r\n])ID (\d+)\r?\n", buf)
                if m:
                    return int(m.group(1))
                buf = buf[-200:]
        return None
    except OSError:
        return None
    finally:
        os.close(fd)


def usb_ports_by_board():
    """USB serial ports ordered so slot 1 holds board 1 and slot 2 board 2.
    Returns (ports, notes) where notes are log messages."""
    found = sorted(glob.glob("/dev/serial/by-id/*")) or sorted(glob.glob("/dev/ttyACM*"))
    slots, unknown, notes = {}, [], []
    for path in found:
        bid = ask_board_id(path)
        short = os.path.basename(os.path.realpath(path))
        if bid in (1, 2) and bid not in slots:
            slots[bid] = path
            notes.append(f"board {bid} is on {short}")
        else:
            unknown.append(path)
            notes.append(f"{short} did not say which board it is (old firmware?)"
                         if bid is None else f"{short} also says board {bid}: check BOARD_ID")
    for path in unknown:                 # fill the gaps in port order
        for slot in (1, 2):
            if slot not in slots:
                slots[slot] = path
                break
    if not slots:
        return [], notes
    # The reader takes ports by position; a missing board gets a placeholder
    # it will report as not available.
    return [slots.get(1, "(no-board-1)"), slots.get(2, "(no-board-2)")], notes


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
    """Owns the running bridge / reader processes and the shared state.

    Start launches the bridge and reader straight away, so the boards connect
    during the start delay. Phases:
        delay       countdown running (boards connecting or already sending)
        connecting  countdown over, still no data
        collecting  CSV rows being written, collect timer running
    Collecting begins at whichever comes later: the end of the delay or the
    first data. The reader runs with --csv-wait and gets SIGUSR1 at that
    moment, so nothing from the delay is recorded. Duration 0 = until Stop.
    """

    def __init__(self, reader_arg):
        self.reader_arg = reader_arg
        self.reader = None          # asyncio subprocess
        self.reader_path = None
        self.bridge = None
        self.phase = "idle"
        self.busy = False           # starting or stopping processes
        self.mode = None
        self.delay_until = None     # end of the countdown (epoch seconds)
        self.collect_started = None
        self.duration = 0           # seconds, 0 = until Stop
        self.delay_done = False
        self.have_data = False      # a board has sent data this run
        self.csv_name = None
        self.status = None          # last JSON status from the reader
        self.timer = None           # countdown / auto-stop task
        self.log = collections.deque(maxlen=300)
        self.clients = set()
        self.tasks = set()

    @property
    def running(self):
        return self.phase != "idle"

    # ---------- broadcasting ----------

    def state(self):
        return {
            "type": "state",
            "running": self.running,
            "busy": self.busy,
            "phase": self.phase,
            "have_data": self.have_data,
            "mode": self.mode,
            "delay_until": self.delay_until,
            "started_at": self.collect_started,
            "duration": self.duration,
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
                if not self.have_data and self.running and any(
                        s.get("state") == "ok" for s in self.status.get("sensors", [])):
                    self.have_data = True
                    await self.push_state()
                    await self.maybe_begin()
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

    async def start(self, mode, record, name, delay=0.0, duration=0.0):
        if self.running or self.busy:
            raise web.HTTPConflict(text="already running")
        if mode not in MODES:
            raise web.HTTPBadRequest(text="mode must be usb, ble or sim")
        if not (0 <= delay <= 3600):
            raise web.HTTPBadRequest(text="delay must be 0 to 3600 seconds")
        if not (0 <= duration <= 86400):
            raise web.HTTPBadRequest(text="collect time must be 0 to 86400 seconds")
        reader_path = find_reader(self.reader_arg)
        if reader_path is None or not reader_path.is_file():
            raise web.HTTPBadRequest(
                text=f"accel_reader not found in {ROOT}/Debug or {ROOT}: build "
                     "the project in Eclipse, or run: gcc -O2 -o accel_reader "
                     "accel_reader.c -lm")
        if record and b"--csv-wait" not in reader_path.read_bytes():
            raise web.HTTPBadRequest(
                text=f"{reader_path.name} is an old build: in Eclipse do "
                     "Refresh, Clean Project, Build, then press Start again")

        self.mode = mode
        self.duration = duration
        self.csv_name = None
        self.collect_started = None
        self.status = None
        self.have_data = False
        self.delay_done = delay == 0
        self.phase = "delay" if delay else "connecting"
        self.delay_until = time.time() + delay if delay else None
        if delay:
            await self.add_log(f"[ui] connecting; collecting starts in {delay:g} s "
                               "or when data arrives, whichever is later")
        await self.push_state()
        self.timer = self.spawn(self.run(reader_path, mode, record, name))

    async def run(self, reader_path, mode, record, name):
        if not await self.launch(reader_path, mode, record, name):
            await self.stop()
            return
        if self.delay_until:
            await asyncio.sleep(max(0.0, self.delay_until - time.time()))
        self.delay_done = True
        self.delay_until = None
        if self.phase == "delay":
            self.phase = "connecting"
            await self.push_state()
        await self.maybe_begin()

    async def maybe_begin(self):
        if self.phase in ("delay", "connecting") and self.delay_done and self.have_data:
            await self.begin_collecting()

    async def launch(self, reader_path, mode, record, name):
        """Start bridge (if needed) and reader. Returns False on failure."""
        self.busy = True
        await self.push_state()
        try:
            ports = []
            if mode == "usb":
                ports, notes = await asyncio.get_running_loop().run_in_executor(
                    None, usb_ports_by_board)
                for n in notes:
                    await self.add_log(f"[ui] USB: {n}")
                if not ports:
                    await self.add_log("[ui] USB: no boards found")
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
                    return False

            args = [str(reader_path), *ports, "--json"]
            if record:
                DATA_DIR.mkdir(exist_ok=True)
                safe = re.sub(r"[^A-Za-z0-9_-]+", "-", name or "").strip("-")
                stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
                self.csv_name = f"{stamp}_{safe}.csv" if safe else f"{stamp}.csv"
                args += ["--csv", str(DATA_DIR / self.csv_name), "--csv-wait"]

            self.reader_path = reader_path
            self.reader = await asyncio.create_subprocess_exec(
                *args, stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE, cwd=str(ROOT))
            await self.add_log(f"[ui] started {reader_path.name} ({mode})"
                               + (f", recording {self.csv_name}" if record else ""))
            self.spawn(self.watch_reader(self.reader))
            return True
        finally:
            self.busy = False
            await self.push_state()

    async def begin_collecting(self):
        if self.csv_name and self.reader and self.reader.returncode is None:
            self.reader.send_signal(signal.SIGUSR1)   # start writing CSV rows
        self.phase = "collecting"
        self.collect_started = time.time()
        await self.add_log("[ui] collecting"
                           + (f" for {self.duration:g} s" if self.duration else " until Stop"))
        await self.push_state()
        if self.duration:
            self.timer = self.spawn(self.auto_stop(self.duration))

    async def auto_stop(self, seconds):
        await asyncio.sleep(seconds)
        await self.add_log(f"[ui] collect time of {seconds:g} s reached")
        await self.stop()

    async def zero(self):
        """Make the boards' current pose the reference (angles read 0)."""
        if not self.reader or self.reader.returncode is not None:
            raise web.HTTPConflict(text="Press Start first: Zero works while the boards are running")
        if b"Zeroed:" not in self.reader_path.read_bytes():
            raise web.HTTPBadRequest(
                text=f"{self.reader_path.name} is an old build: in Eclipse do Refresh, "
                     "Clean Project, Build, then restart the run")
        self.reader.send_signal(signal.SIGUSR2)   # reader replies "Zeroed: ..."

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
        # Cancel a pending countdown / auto-stop (unless that's who called us).
        timer, self.timer = self.timer, None
        if timer and timer is not asyncio.current_task() and not timer.done():
            timer.cancel()
            await asyncio.gather(timer, return_exceptions=True)
        cancelled = self.phase == "delay"
        self.busy = True
        await self.push_state()
        reader, self.reader = self.reader, None
        if not reader_done:
            await self.end_process(reader)
        await self.stop_bridge()
        self.phase = "idle"
        self.delay_until = None
        self.busy = False
        if self.csv_name and self.collect_started is None:
            # never got to collecting: drop the empty (header-only) file
            (DATA_DIR / self.csv_name).unlink(missing_ok=True)
            self.csv_name = None
        if self.csv_name:
            await self.add_log(f"[ui] saved recordings/{self.csv_name}")
        await self.add_log("[ui] cancelled" if cancelled else "[ui] stopped")
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
    # no-store: phones otherwise keep showing an old page after an update
    return web.FileResponse(UI_DIR / "index.html",
                            headers={"Cache-Control": "no-store"})


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
    try:
        delay = float(body.get("delay", 0) or 0)
        duration = float(body.get("duration", 0) or 0)
    except (TypeError, ValueError):
        raise web.HTTPBadRequest(text="delay and collect time must be numbers")
    await request.app["session"].start(
        body.get("mode", "usb"), bool(body.get("record")), body.get("name", ""),
        delay, duration)
    return web.json_response(request.app["session"].state())


async def api_stop(request):
    await request.app["session"].stop()
    return web.json_response(request.app["session"].state())


async def api_zero(request):
    await request.app["session"].zero()
    return web.json_response({"ok": True})


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
    app.router.add_post("/api/zero", api_zero)
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
