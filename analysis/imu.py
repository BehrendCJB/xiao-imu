"""
imu.py - shared building blocks for motion analysis of accel_reader CSVs.

    load_csv(path)          -> {board_id: Board}
    Board.segments          -> continuous pieces (split at data gaps), each
                               resampled to a uniform 104 Hz grid with
                               orientation, hand angles and world-frame
                               linear acceleration already computed.

Assumed mounting (change with --finger-axis): each board lies flat on the
back of a hand, component side up (board +z points out of the back of the
hand), board +x points toward the fingers.

Hand angles (degrees, no yaw involved, so they do not drift):
    finger_pitch  fingers above (+) or below (-) horizontal
    hand_roll     rotation about the finger axis: 0 = palm down,
                  +90 = thumb side up, +-180 = palm up (meaningless when
                  the fingers point nearly straight up or down)
    palm          direction the palm faces (world vector); swim.py turns it
                  into the angle between the palm and "backward"
World frame: z up; x/y horizontal with an arbitrary heading that drifts
slowly (no magnetometer), so heading is only used within one stroke/shot.
"""
import math
from dataclasses import dataclass, field

import numpy as np
from scipy.spatial.transform import Rotation

G = 9.81          # m/s^2
FS = 104.0        # Hz, board sample rate
MAX_GAP_S = 0.25  # same as accel_reader: bigger gap = lost data

AXES = {"+x": (0, 1), "-x": (0, -1), "+y": (1, 1), "-y": (1, -1)}


@dataclass
class Segment:
    t: np.ndarray            # s, board time from the start of the recording
    acc: np.ndarray          # (N,3) g, board frame
    gyr: np.ndarray          # (N,3) deg/s, board frame
    rot: Rotation = None     # board -> world, per sample
    lin: np.ndarray = None   # (N,3) m/s^2, world frame, gravity removed
    finger: np.ndarray = None  # (N,3) unit finger axis in world frame
    palm: np.ndarray = None    # (N,3) unit vector out of the palm, world frame
    finger_pitch: np.ndarray = None
    hand_roll: np.ndarray = None
    gyr_mag: np.ndarray = None


@dataclass
class Board:
    id: int
    label: str
    segments: list = field(default_factory=list)
    samples: int = 0
    gaps: int = 0


# ---------------------------------------------------------------- loading

def load_csv(path, finger_axis="+x", labels=None, mirror=None):
    """Read an accel_reader CSV. labels: {id: name}. mirror: set of board
    ids worn on the left hand (hand_roll sign flipped so + is thumb up)."""
    import csv

    rows = {}
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            if not r.get("gx_dps"):      # old accel-only firmware rows
                continue
            try:
                rows.setdefault(int(r["sensor"]), []).append((
                    int(r["board_ms"]),
                    float(r["ax_g"]), float(r["ay_g"]), float(r["az_g"]),
                    float(r["gx_dps"]), float(r["gy_dps"]), float(r["gz_dps"])))
            except (ValueError, KeyError):
                continue
    if not rows:
        raise SystemExit(f"{path}: no accel + gyro rows found")

    labels = labels or {}
    mirror = mirror or set()
    boards = {}
    for bid, data in sorted(rows.items()):
        a = np.array(data, dtype=float)
        a = a[np.argsort(a[:, 0], kind="stable")]
        a = a[np.concatenate(([True], np.diff(a[:, 0]) > 0))]  # drop repeats
        b = Board(bid, labels.get(bid, f"board {bid}"), samples=len(a))
        t = (a[:, 0] - a[0, 0]) / 1000.0
        cuts = np.where(np.diff(t) > MAX_GAP_S)[0] + 1
        b.gaps = len(cuts)
        for idx in np.split(np.arange(len(t)), cuts):
            if len(idx) < int(FS):          # ignore scraps under 1 s
                continue
            seg = _resample(t[idx], a[idx, 1:4], a[idx, 4:7])
            orient(seg, finger_axis, left=bid in mirror)
            b.segments.append(seg)
        boards[bid] = b
    return boards


def _resample(t, acc, gyr):
    tu = np.arange(t[0], t[-1], 1.0 / FS)
    ai = np.column_stack([np.interp(tu, t, acc[:, i]) for i in range(3)])
    gi = np.column_stack([np.interp(tu, t, gyr[:, i]) for i in range(3)])
    return Segment(tu, ai, gi)


def align_boards(boards, method="start"):
    """Shift board time axes so the boards line up.

    The two boards' clocks are independent. "start" (default) assumes both
    boards' first CSV rows were recorded at the same moment, which holds to
    within a sample or two when collection starts with live data (USB, or
    Bluetooth with no backlog). "tap" finds the first sharp spike (> 3 g)
    in each board within the first 20 s - tap the two hands together once
    at the start of the recording - and lines those up."""
    if method != "tap" or len(boards) < 2:
        return None
    spikes = {}
    for bid, b in boards.items():
        for s in b.segments:
            m = np.linalg.norm(s.acc, axis=1)
            hit = np.where((m > 3.0) & (s.t < 20.0))[0]
            if len(hit):
                spikes[bid] = s.t[hit[0]]
                break
    if len(spikes) < len(boards):
        return "tap not found in both boards in the first 20 s; using start"
    ref = min(spikes.values())
    for bid, b in boards.items():
        for s in b.segments:
            s.t = s.t - (spikes[bid] - ref)
    return "aligned on tap at " + ", ".join(
        f"board {k} {v:.2f} s" for k, v in spikes.items())


# ---------------------------------------------------------------- orientation
#
# Why not the Madgwick filter used live in accel_reader: during a stroke the
# hand's own acceleration (several g, mostly centripetal) is mistaken for
# gravity, which tilted the estimate by 12-22 degrees on synthetic strokes.
# Offline we can do better: integrate the gyro (accurate over seconds), then
# remove its slow drift using the fact that, averaged over ~20 s, a hand's
# measured acceleration in the world frame is just gravity (its velocity
# stays bounded). On the same synthetic strokes this gives ~1 degree.

DRIFT_WINDOW_S = 20.0


def level_from_accel(a):
    """Board -> world rotation with yaw 0 from a gravity reading."""
    roll = math.atan2(a[1], a[2])
    pitch = math.atan2(-a[0], math.hypot(a[1], a[2]))
    return Rotation.from_euler("ZYX", [0.0, pitch, roll])


def integrate_gyro(acc, gyr, dt=1.0 / FS):
    """Orientation (board -> world) from the gyro alone, levelled from the
    first 10 samples' gravity. Midpoint rule, exact rotation per step."""
    n = len(acc)
    out = np.empty((n, 4))
    out[0] = level_from_accel(acc[: min(n, 10)].mean(axis=0)).as_quat()
    w = np.radians(gyr)
    steps = Rotation.from_rotvec((w[1:] + w[:-1]) * 0.5 * dt).as_quat()
    x, y, z, s = out[0]
    for i in range(n - 1):
        bx, by, bz, bw = steps[i]
        x, y, z, s = (s*bx + x*bw + y*bz - z*by,
                      s*by - x*bz + y*bw + z*bx,
                      s*bz + x*by - y*bx + z*bw,
                      s*bw - x*bx - y*by - z*bz)
        out[i + 1] = (x, y, z, s)
    return Rotation.from_quat(out)


def remove_tilt_drift(rot, acc, window_s=DRIFT_WINDOW_S, passes=2):
    """Rotate each sample so the window-averaged world acceleration points
    straight up. Corrects roll/pitch drift; heading drift is unobservable."""
    from scipy.ndimage import uniform_filter1d
    k = max(3, min(len(acc), int(window_s * FS)))
    for _ in range(passes):
        m = uniform_filter1d(rot.apply(acc), k, axis=0, mode="nearest")
        m /= np.linalg.norm(m, axis=1, keepdims=True)
        axis = np.cross(m, [0.0, 0.0, 1.0])
        s = np.linalg.norm(axis, axis=1, keepdims=True)
        ang = np.arctan2(s, m[:, 2:3])
        rv = np.where(s > 1e-12, axis / np.maximum(s, 1e-12) * ang, 0.0)
        rot = Rotation.from_rotvec(rv) * rot
    return rot


def orient(seg, finger_axis="+x", left=False):
    seg.rot = remove_tilt_drift(integrate_gyro(seg.acc, seg.gyr), seg.acc)
    seg.lin = seg.rot.apply(seg.acc * G) - np.array([0.0, 0.0, G])
    i, sign = AXES[finger_axis]
    f_b = np.zeros(3); f_b[i] = sign
    n_b = np.array([0.0, 0.0, 1.0])                    # back of hand
    l_b = np.cross(n_b, f_b)                           # thumb side (right hand)
    f, l, n = (seg.rot.apply(v) for v in (f_b, l_b, n_b))
    seg.finger = f
    seg.palm = -n
    seg.finger_pitch = np.degrees(np.arcsin(np.clip(f[:, 2], -1, 1)))
    roll = np.degrees(np.arctan2(l[:, 2], n[:, 2]))
    seg.hand_roll = -roll if left else roll
    seg.gyr_mag = np.linalg.norm(seg.gyr, axis=1)


# ---------------------------------------------------------------- helpers

def integrate(x, dt=1.0 / FS):
    """Cumulative trapezoid integral, starting at 0."""
    out = np.zeros_like(x)
    out[1:] = np.cumsum((x[1:] + x[:-1]) * 0.5 * dt, axis=0)
    return out


def detrend(x):
    """Remove a least-squares straight line from each column."""
    from scipy.signal import detrend as _d
    return _d(x, axis=0, type="linear")


def stills(seg, gyro_dps=30.0, acc_g=0.1, min_s=0.1):
    """Boolean mask of samples where the hand is at rest."""
    m = (seg.gyr_mag < gyro_dps) & (np.abs(np.linalg.norm(seg.acc, axis=1) - 1) < acc_g)
    k = max(1, int(min_s * FS))
    # keep only runs of at least k samples
    out = np.zeros_like(m)
    run = 0
    for i, v in enumerate(m):
        run = run + 1 if v else 0
        if run >= k:
            out[i - run + 1: i + 1] = True
    return out
