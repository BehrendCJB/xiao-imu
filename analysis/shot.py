"""
shot.py - basketball shots from a hand-mounted board.

A shot is a burst of fast wrist rotation (> 400 deg/s). For each shot:
  - motion start: the last moment the hand was at rest before the burst
  - release: the peak of the wrist snap (fastest rotation)
  - end: the first rest after the release (follow-through held, or the
    hand back down), if found within 2.5 s

Between start and end the hand's acceleration is integrated twice, with
its velocity forced to zero at both rests, which gives the hand's path to
within a few centimetres for a motion this short. If no rest follows the
shot, only the start rest is used and the path is less reliable ("approx").

Per shot: dip depth, rise to release, hand speed and direction (release
angle, up from horizontal) at release, finger pitch at set and at release,
wrist snap speed, time from motion start to release.
"""
import numpy as np
from scipy.signal import find_peaks

from imu import FS, integrate, stills

SNAP_DPS = 400.0
MAX_BEFORE_S = 3.0
MAX_AFTER_S = 2.5


def analyze(board):
    res = {"board": board.id, "label": board.label, "shots": [],
           "curves": {"t": [], "up": [], "fwd": [], "finger_pitch": []}}
    for seg in board.segments:
        rest = stills(seg)
        pk, _ = find_peaks(seg.gyr_mag, height=SNAP_DPS, distance=int(1.5 * FS))
        for r in pk:
            lo = max(0, r - int(MAX_BEFORE_S * FS))
            before = np.where(rest[lo:r])[0]
            if not len(before):
                continue                     # no rest before: not a clean shot
            i0 = lo + before[-1]
            hi = min(len(rest), r + int(MAX_AFTER_S * FS))
            after = np.where(rest[r:hi])[0]
            i1 = r + after[0] if len(after) else hi - 1
            zupt_end = bool(len(after))

            lin = seg.lin[i0: i1 + 1]
            v = integrate(lin)
            if zupt_end:
                v -= np.linspace(0, 1, len(v))[:, None] * v[-1]
            p = integrate(v)
            k = r - i0                        # release index in the window
            vr = v[k]
            fwd = np.array([vr[0], vr[1]])
            fwd = fwd / np.linalg.norm(fwd) if np.linalg.norm(fwd) > 1e-6 else np.array([1.0, 0])
            ph = p[:, :2] @ fwd
            up = p[:, 2]
            fp = seg.finger_pitch[i0: i1 + 1]
            vh = float(np.hypot(vr[0], vr[1]))
            res["shots"].append({
                "release_s": round(float(seg.t[r]), 2),
                "start_to_release_s": round(k / FS, 3),
                "dip_m": round(float(-up[: k + 1].min()), 3),
                "rise_to_release_m": round(float(up[k]), 3),
                "speed_at_release_mps": round(float(np.linalg.norm(vr)), 2),
                "release_angle_deg": round(float(np.degrees(np.arctan2(vr[2], vh))), 1),
                "finger_pitch_set_deg": round(float(fp[: k + 1].max()), 1),
                "finger_pitch_release_deg": round(float(fp[k]), 1),
                "wrist_snap_peak_dps": round(float(seg.gyr_mag[r]), 0),
                "end": "rest" if zupt_end else "approx",
            })
            res["curves"]["t"].append((np.arange(len(up)) - k) / FS)
            res["curves"]["up"].append(up)
            res["curves"]["fwd"].append(ph - ph[0])
            res["curves"]["finger_pitch"].append(fp)
    s = res["shots"]
    if s:
        keys = [k for k in s[0] if k not in ("release_s", "end")]
        res["summary"] = {"shots": len(s), **{
            k: {"mean": round(float(np.mean([x[k] for x in s])), 3),
                "sd": round(float(np.std([x[k] for x in s])), 3)} for k in keys}}
    else:
        res["summary"] = {"shots": 0}
    return res
