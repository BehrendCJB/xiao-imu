#!/usr/bin/env python3
"""
synth.py - make fake recordings with a known motion, to test the analysis
without hardware. Writes a CSV in accel_reader's format and prints the true
values to compare with what analyze.py reports.

    python3 analysis/synth.py swim /tmp/swim.csv   [--strokes 10 --period 1.4]
    python3 analysis/synth.py shot /tmp/shot.csv   [--shots 5]

Board 1 = left hand, board 2 = right hand, both mounted flat on the back of
the hand with +x toward the fingers. Noise and quantisation match the CSV.
"""
import argparse
import math

import numpy as np
from scipy.interpolate import PchipInterpolator
from scipy.spatial.transform import Rotation

G = 9.81
TRUE = {}     # filled by sensors(), in call order
HI = 1040.0   # internal rate; every 10th sample is kept (104 Hz)


def smoothstep(x):
    x = np.clip(x, 0, 1)
    return x * x * x * (x * (6 * x - 15) + 10)


def sensors(t, pos, rot, rng):
    """True board-frame accel (g) and gyro (deg/s) from a world trajectory."""
    dt = 1.0 / HI
    vel = np.gradient(pos, dt, axis=0)
    acc_w = np.gradient(vel, dt, axis=0) + np.array([0, 0, G])
    acc_b = rot.inv().apply(acc_w) / G
    rel = rot[:-1].inv() * rot[1:]
    gyr = np.degrees(rel.as_rotvec()) / dt
    gyr = np.vstack([gyr, gyr[-1:]])
    k = slice(None, None, 10)
    a = acc_b[k] + rng.normal(0, 0.004, acc_b[k].shape)
    g = gyr[k] + rng.normal(0, 0.15, gyr[k].shape) + np.array([0.2, -0.1, 0.15])
    TRUE[len(TRUE)] = (rot[k], pos[k])   # for tests: true orientation, position
    return t[k], a, g


def write_csv(path, boards):
    with open(path, "w") as f:
        f.write("sensor,board_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,"
                "roll_deg,pitch_deg,yaw_deg,roll0_deg,pitch0_deg,yaw0_deg\n")
        rows = []
        for bid, (t, a, g, t0) in boards.items():
            for i in range(len(t)):
                rows.append((t[i], bid, int(t0 + round(t[i] * 1000)), a[i], g[i]))
        rows.sort(key=lambda r: (r[0], r[1]))
        for _, bid, ms, a, g in rows:
            f.write(f"{bid},{ms},{a[0]:.3f},{a[1]:.3f},{a[2]:.3f},"
                    f"{g[0]:.1f},{g[1]:.1f},{g[2]:.1f},0,0,0,0,0,0\n")


# ---------------------------------------------------------------- swim

def swim(n, period, rng):
    """Front-crawl-like arm circle (radius 0.6 m, straight arm) while the
    body moves forward at 1 m/s. World x = swim direction, z = up."""
    ramp, pre, post = 0.5, 3.0, 2.0
    active = n * period + ramp
    t = np.arange(0, pre + active + post, 1 / HI)
    u = t - pre
    env = smoothstep(u / ramp) * smoothstep((active - u) / ramp)
    phase = np.cumsum(env) / HI / period            # cycles done
    boards, truth = {}, {}
    for bid, (shift, side) in {1: (0.5, -1), 2: (0.0, 1)}.items():
        ph = phase + shift                           # left hand half a cycle on
        w = 2 * math.pi * ph
        th = w - 0.5 * np.sin(w)                     # slow pull, fast recovery
        r = 0.6
        body = 1.0 * np.cumsum(env) / HI
        pos = np.column_stack([r * np.cos(th) + body,
                               side * (0.25 + 0.08 * np.sin(2 * th)),
                               -r * np.sin(th)])
        flex = np.radians(15) * np.sin(th) ** 2      # wrist bends during pull
        roll = np.radians(30) * np.sin(th)           # about the finger axis
        head = Rotation.from_euler("z", 0.7)          # any pool heading
        pos = head.apply(pos)
        rot = (head
               * Rotation.from_euler("y", (th + flex)[:, None])
               * Rotation.from_euler("x", (side * roll)[:, None]))
        tt, a, g = sensors(t, pos, rot, rng)
        boards[bid] = (tt, a, g, 50000 + 73123 * bid)
        truth[bid] = {"cycles": n, "period_s": period,
                      "depth_m (vertical range)": 2 * r, "fore_aft_m": 2 * r}
    return boards, truth


# ---------------------------------------------------------------- shot

def shot(n, rng):
    """Jump-shot-like hand motion. Board 2 (right) shoots: dip, rise, wrist
    snap at release, hold the follow-through, return. Board 1 (left) is the
    guide hand: rises, no snap."""
    T = 5.0
    keys_t = [0, 1.0, 1.3, 1.75, 1.85, 1.95, 2.5, 3.5, 5.0]
    z = [0, 0, -0.12, 0.50, 0.62, 0.65, 0.65, 0, 0]
    x = [0, 0, -0.02, 0.15, 0.22, 0.25, 0.25, 0, 0]
    al = [30, 30, 70, 100, 30, -20, -20, 30, 30]       # finger pitch, deg
    t = np.arange(0, 1.5 + n * T, 1 / HI)
    boards, truth = {}, {"shots": n}
    for bid in (1, 2):
        sc = 1.0 if bid == 2 else 0.7
        a_keys = al if bid == 2 else [10, 10, 20, 40, 40, 40, 40, 10, 10]
        fz = PchipInterpolator(keys_t, np.array(z) * sc)
        fx = PchipInterpolator(keys_t, np.array(x) * sc)
        fa = PchipInterpolator(keys_t, np.radians(a_keys))
        tl = np.mod(t - 1.5, T)
        tl[t < 1.5] = 0
        head = Rotation.from_euler("z", -1.2)         # any court heading
        pos = head.apply(np.column_stack(
            [fx(tl), np.full_like(t, 0.3 * (bid - 1.5)), fz(tl)]))
        rot = head * Rotation.from_euler("y", -fa(tl)[:, None])
        tt, a, g = sensors(t, pos, rot, rng)
        boards[bid] = (tt, a, g, 900000 + 4321 * bid)
        if bid == 2:
            dense = np.linspace(1.70, 1.95, 2501)
            rate = np.gradient(fa(dense), dense)
            rel = dense[np.argmin(rate)]
            vx, vz = fx(rel, 1), fz(rel, 1)
            truth.update({
                "release_s_after_motion_start": round(rel - 1.0, 3),
                "rise_to_release_m": round(float(fz(rel)), 3),
                "speed_at_release_mps": round(math.hypot(vx, vz), 2),
                "release_angle_deg": round(math.degrees(math.atan2(vz, vx)), 1),
                "finger_pitch_at_release_deg": round(math.degrees(
                    math.asin(math.sin(fa(rel)))), 1),
                "wrist_snap_peak_dps": round(float(-np.degrees(rate.min())), 0),
            })
    return boards, truth


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("kind", choices=["swim", "shot"])
    ap.add_argument("out")
    ap.add_argument("--strokes", type=int, default=10)
    ap.add_argument("--period", type=float, default=1.4)
    ap.add_argument("--shots", type=int, default=5)
    ap.add_argument("--seed", type=int, default=1)
    args = ap.parse_args()
    rng = np.random.default_rng(args.seed)
    if args.kind == "swim":
        boards, truth = swim(args.strokes, args.period, rng)
    else:
        boards, truth = shot(args.shots, rng)
    write_csv(args.out, boards)
    print(f"wrote {args.out}")
    print("truth:", truth)


if __name__ == "__main__":
    main()
