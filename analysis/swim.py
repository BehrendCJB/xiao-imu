"""
swim.py - stroke cycles from a hand-mounted board.

A cycle runs from one highest point of the hand to the next (mid-recovery
in front crawl). For each cycle: duration, stroke rate, finger pitch, the
palm angle (angle between the palm and straight backward: 0 = pushing
water straight back), and an approximate hand path.

Hand path: the hand's acceleration is integrated twice over one cycle. The
swimmer's forward speed and slow sensor errors are removed by assuming the
hand returns to the same place relative to the body each cycle, so the path
is relative to the body (moving at its average speed), not to the pool.
Expect errors of several centimetres; shapes and stroke-to-stroke changes
are more trustworthy than absolute sizes.
"""
import numpy as np
from scipy.signal import butter, filtfilt, find_peaks

from imu import FS, integrate

ACTIVE_DPS = 60.0       # smoothed rotation rate above this = swimming
PERIOD_RANGE = (0.5, 4.0)


def _bp(x, lo, hi):
    b, a = butter(2, [lo / (FS / 2), min(hi, 0.45 * FS) / (FS / 2)], "band")
    return filtfilt(b, a, x, axis=0)


def _hp(x, lo):
    b, a = butter(2, lo / (FS / 2), "high")
    return filtfilt(b, a, x, axis=0)


def stroke_period(seg, active):
    """Dominant cycle period (s) from the autocorrelation of the gyro."""
    g = seg.gyr - seg.gyr[active].mean(axis=0)
    g[~active] = 0
    n = len(g)
    if active.sum() < 3 * FS:
        return None
    nfft = 1 << (2 * n - 1).bit_length()
    ac = sum(np.fft.irfft(np.abs(np.fft.rfft(g[:, i], nfft)) ** 2)[:n] for i in range(3))
    ac /= ac[0]
    lo, hi = int(PERIOD_RANGE[0] * FS), min(int(PERIOD_RANGE[1] * FS), n - 1)
    pk, _ = find_peaks(ac[lo:hi])
    if not len(pk):
        return None
    h = ac[lo:hi][pk]
    best = pk[np.argmax(h >= 0.8 * h.max())]    # first strong peak, not 2T
    return (best + lo) / FS


def find_cycles(seg):
    sm = np.convolve(seg.gyr_mag, np.ones(int(FS)) / FS, mode="same")
    active = sm > ACTIVE_DPS
    T = stroke_period(seg, active)
    if T is None:
        return None, []
    f0 = 1.0 / T
    z = _bp(seg.lin[:, 2], 0.4 * f0, 4 * f0)
    z = _hp(integrate(_hp(integrate(z), 0.4 * f0)), 0.4 * f0)
    pk, _ = find_peaks(z, distance=int(0.65 * T * FS), prominence=0.03)
    cycles = [(i0, i1) for i0, i1 in zip(pk[:-1], pk[1:])
              if 0.6 * T <= (i1 - i0) / FS <= 1.6 * T and active[i0:i1].all()]
    if cycles:   # drop odd ones (first stroke from rest, turns, stops)
        med = np.median([i1 - i0 for i0, i1 in cycles])
        cycles = [c for c in cycles if 0.8 * med <= c[1] - c[0] <= 1.25 * med]
    return T, cycles


def cycle_path(seg, i0, i1):
    """Hand path over one cycle, relative to the body, in metres.
    Returns (N,3) columns forward, side (+ = left), up, and the forward
    direction as a world vector."""
    lin = seg.lin[i0: i1 + 1]
    v = integrate(lin)
    v -= v.mean(axis=0)                        # swimmer's speed + accel bias
    p = integrate(v)
    frac = np.linspace(0, 1, len(p))[:, None]
    p -= frac * (p[-1] - p[0])                 # close the loop
    p -= p.mean(axis=0)
    h = p[:, :2]
    w, vecs = np.linalg.eigh(h.T @ h)
    fwd = vecs[:, np.argmax(w)]
    vf = np.gradient(h @ fwd)
    if np.sum(p[:, 2] * vf) < 0:               # hand moves forward when high
        fwd = -fwd
    side = np.array([-fwd[1], fwd[0]])
    return (np.column_stack([h @ fwd, h @ side, p[:, 2]]),
            np.array([fwd[0], fwd[1], 0.0]))


def norm_cycle(x, n=101):
    s = np.linspace(0, len(x) - 1, n)
    return np.interp(s, np.arange(len(x)), x)


def analyze(board):
    """Returns a result dict for one board (one hand)."""
    res = {"board": board.id, "label": board.label, "cycles": [],
           "curves": {"finger_pitch": [], "palm_back": [], "path": []},
           "starts_s": []}
    periods = []
    for seg in board.segments:
        T, cycles = find_cycles(seg)
        if T:
            periods.append(T)
        for i0, i1 in cycles:
            path, fwd = cycle_path(seg, i0, i1)
            fp = seg.finger_pitch[i0: i1 + 1]
            # 0 deg = palm pushing straight back, 90 = palm sideways/up/down
            pb = np.degrees(np.arccos(np.clip(seg.palm[i0: i1 + 1] @ -fwd, -1, 1)))
            d = (i1 - i0) / FS
            res["starts_s"].append(float(seg.t[i0]))
            res["cycles"].append({
                "start_s": round(float(seg.t[i0]), 2),
                "duration_s": round(d, 3),
                "rate_spm": round(60 / d, 1),
                "finger_pitch_min": round(float(fp.min()), 1),
                "finger_pitch_max": round(float(fp.max()), 1),
                "palm_back_min": round(float(pb.min()), 1),
                "depth_m": round(float(np.ptp(path[:, 2])), 3),
                "fore_aft_m": round(float(np.ptp(path[:, 0])), 3),
                "side_m": round(float(np.ptp(path[:, 1])), 3),
            })
            res["curves"]["finger_pitch"].append(norm_cycle(fp))
            res["curves"]["palm_back"].append(norm_cycle(pb))
            res["curves"]["path"].append(np.column_stack(
                [norm_cycle(path[:, k]) for k in range(3)]))
    c = res["cycles"]
    if c:
        def stat(key):
            v = np.array([x[key] for x in c])
            return {"mean": round(float(v.mean()), 3), "sd": round(float(v.std()), 3)}
        res["summary"] = {
            "cycles": len(c),
            "period_s_autocorr": round(float(np.median(periods)), 3) if periods else None,
            **{k: stat(k) for k in ("duration_s", "rate_spm", "depth_m",
                                    "fore_aft_m", "side_m", "finger_pitch_min",
                                    "finger_pitch_max", "palm_back_min")},
        }
    else:
        res["summary"] = {"cycles": 0}
    return res


def arm_timing(r1, r2):
    """Phase of hand 2's cycle start within hand 1's cycle (0-100 %).
    About 50 % = alternating arms (front crawl, backstroke); near 0 or
    100 % = both arms together (breaststroke, butterfly). Needs the two
    boards' clocks lined up (see align_boards)."""
    s1, s2 = np.array(r1["starts_s"]), np.array(r2["starts_s"])
    if len(s1) < 3 or len(s2) < 3:
        return None
    ph = []
    for t in s2:
        k = np.searchsorted(s1, t) - 1
        if 0 <= k < len(s1) - 1:
            ph.append((t - s1[k]) / (s1[k + 1] - s1[k]))
    if not ph:
        return None
    ang = np.angle(np.mean(np.exp(2j * np.pi * np.array(ph)))) / (2 * np.pi)
    pct = round(float(ang % 1.0) * 100, 1)
    kind = ("alternating (front crawl / backstroke)" if 30 <= pct <= 70 else
            "together (breaststroke / butterfly)" if pct <= 15 or pct >= 85 else
            "unclear")
    return {"phase_pct": pct, "pattern": kind}
