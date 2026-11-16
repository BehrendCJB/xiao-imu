#!/usr/bin/env python3
"""
analyze.py - analyse a recording made by the control panel (or accel_reader
--csv) with the boards on the backs of the hands.

    python3 analysis/analyze.py swim recordings/2026-11-16_10-00-00_crawl.csv
    python3 analysis/analyze.py shot recordings/2026-11-16_11-00-00_shots.csv

Writes <recording>_analysis/ next to the CSV: summary.txt, summary.json and
one PNG of plots per hand. Prints the summary.

Recording tips: start with both hands still for ~2 s (palms down if you
can); for shots, hold the hand still for a moment before each shot.

Options:
    --hands left,right   what board 1 and board 2 are worn on (default
                         left,right). Used for labels and to make "thumb up"
                         positive on both hands.
    --finger-axis +x     board axis that points to the fingers (+x -x +y -y)
    --sync start|tap     how to line up the two boards' clocks (see imu.py)
    --out DIR            output folder
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import imu   # noqa: E402
import shot  # noqa: E402
import swim  # noqa: E402


def fmt_stat(d, unit=""):
    return f"{d['mean']:g}{unit} (sd {d['sd']:g})"


def swim_text(r):
    s = r["summary"]
    out = [f"{r['label']}: {s['cycles']} stroke cycles"]
    if s["cycles"]:
        out += [
            f"  stroke rate        {fmt_stat(s['rate_spm'], ' /min')}",
            f"  cycle time         {fmt_stat(s['duration_s'], ' s')}",
            f"  hand depth range   {fmt_stat(s['depth_m'], ' m')}",
            f"  hand fore-aft      {fmt_stat(s['fore_aft_m'], ' m')}",
            f"  hand side-to-side  {fmt_stat(s['side_m'], ' m')}",
            f"  finger pitch       {s['finger_pitch_min']['mean']:g} to "
            f"{s['finger_pitch_max']['mean']:g} deg",
            f"  best palm angle    {fmt_stat(s['palm_back_min'], ' deg')}"
            "  (0 = palm pushing straight back)",
        ]
    return out


def shot_text(r):
    s = r["summary"]
    out = [f"{r['label']}: {s['shots']} shots"]
    if s["shots"]:
        out += [
            f"  start to release   {fmt_stat(s['start_to_release_s'], ' s')}",
            f"  dip                {fmt_stat(s['dip_m'], ' m')}",
            f"  rise to release    {fmt_stat(s['rise_to_release_m'], ' m')}",
            f"  hand speed         {fmt_stat(s['speed_at_release_mps'], ' m/s')}",
            f"  release angle      {fmt_stat(s['release_angle_deg'], ' deg')}",
            f"  finger pitch set   {fmt_stat(s['finger_pitch_set_deg'], ' deg')}",
            f"  finger pitch rel.  {fmt_stat(s['finger_pitch_release_deg'], ' deg')}",
            f"  wrist snap         {fmt_stat(s['wrist_snap_peak_dps'], ' deg/s')}",
            "  per shot (release s, angle deg, speed m/s, end):",
        ]
        out += [f"    {x['release_s']:8.2f}  {x['release_angle_deg']:6.1f}  "
                f"{x['speed_at_release_mps']:5.2f}  {x['end']}" for x in r["shots"]]
    return out


def _thin_and_mean(ax, xs, ys, color):
    for x, y in zip(xs, ys):
        ax.plot(x, y, color=color, alpha=0.25, lw=0.8)


def plot_swim(r, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    c = r["curves"]
    pct = np.linspace(0, 100, 101)
    fig, ax = plt.subplots(2, 2, figsize=(11, 8))
    fig.suptitle(f"{r['label']} - {r['summary']['cycles']} stroke cycles "
                 "(cycle starts at the hand's highest point)")
    for a, key, title in ((ax[0, 0], "finger_pitch", "Finger pitch (deg, + = fingers up)"),
                          (ax[0, 1], "palm_back", "Palm angle (deg, 0 = palm pushing straight back)")):
        ys = c[key]
        _thin_and_mean(a, [pct] * len(ys), ys, "tab:blue")
        a.plot(pct, np.mean(ys, axis=0), color="tab:blue", lw=2.5)
        a.set_title(title); a.set_xlabel("% of cycle"); a.grid(alpha=0.3)
    a = ax[1, 0]
    ps = c["path"]
    _thin_and_mean(a, [p[:, 0] for p in ps], [p[:, 2] for p in ps], "tab:orange")
    m = np.mean(ps, axis=0)
    a.plot(m[:, 0], m[:, 2], color="tab:orange", lw=2.5)
    a.plot(m[0, 0], m[0, 2], "ko", label="cycle start")
    a.set_aspect("equal", adjustable="datalim")
    a.set_title("Hand path, side view, relative to body (m, approx.)")
    a.set_xlabel("forward"); a.set_ylabel("up"); a.grid(alpha=0.3); a.legend()
    a = ax[1, 1]
    a.plot([x["start_s"] for x in r["cycles"]], [x["rate_spm"] for x in r["cycles"]], "o-")
    a.set_title("Stroke rate (cycles/min)"); a.set_xlabel("time in recording (s)")
    a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def plot_shot(r, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    c = r["curves"]
    fig, ax = plt.subplots(2, 2, figsize=(11, 8))
    fig.suptitle(f"{r['label']} - {r['summary']['shots']} shots (time 0 = release)")
    colors = plt.cm.viridis(np.linspace(0, 0.9, len(c["t"])))
    for i, col in enumerate(colors):
        lab = f"shot {i + 1}"
        ax[0, 0].plot(c["t"][i], c["up"][i], color=col, label=lab)
        ax[0, 1].plot(c["fwd"][i], c["up"][i], color=col, label=lab)
        ax[1, 0].plot(c["t"][i], c["finger_pitch"][i], color=col, label=lab)
    for a in (ax[0, 0], ax[1, 0]):
        a.axvline(0, color="k", lw=0.8)
        a.set_xlabel("s from release")
    ax[0, 0].set_title("Hand height from start position (m)")
    ax[0, 1].set_title("Hand path, side view (m)")
    ax[0, 1].set_xlabel("toward the shot"); ax[0, 1].set_ylabel("up")
    ax[0, 1].set_aspect("equal", adjustable="datalim")
    ax[1, 0].set_title("Finger pitch (deg, + = fingers up)")
    a = ax[1, 1]
    sh = r["shots"]
    n = np.arange(1, len(sh) + 1)
    a.plot(n, [x["release_angle_deg"] for x in sh], "o-", color="tab:blue")
    a.set_ylabel("release angle (deg)", color="tab:blue")
    a2 = a.twinx()
    a2.plot(n, [x["speed_at_release_mps"] for x in sh], "s--", color="tab:red")
    a2.set_ylabel("hand speed at release (m/s)", color="tab:red")
    a.set_xlabel("shot"); a.set_xticks(n); a.set_title("Release, shot by shot")
    for a in ax.flat:
        a.grid(alpha=0.3)
    if len(sh) <= 10:
        ax[0, 1].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def jsonable(r):
    return {k: v for k, v in r.items() if k not in ("curves", "starts_s")}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("kind", choices=["swim", "shot"])
    ap.add_argument("csv")
    ap.add_argument("--hands", default="left,right")
    ap.add_argument("--finger-axis", default="+x", choices=list(imu.AXES))
    ap.add_argument("--sync", default="start", choices=["start", "tap"])
    ap.add_argument("--out")
    args = ap.parse_args()

    hands = [h.strip() for h in args.hands.split(",")]
    labels = {i + 1: f"{h} hand (board {i + 1})" for i, h in enumerate(hands)}
    mirror = {i + 1 for i, h in enumerate(hands) if h.lower().startswith("l")}
    boards = imu.load_csv(args.csv, args.finger_axis, labels, mirror)
    sync_msg = imu.align_boards(boards, args.sync)

    src = Path(args.csv)
    out = Path(args.out) if args.out else src.with_name(src.stem + "_analysis")
    out.mkdir(parents=True, exist_ok=True)

    lines = [f"{args.kind} analysis of {src.name}"]
    for b in boards.values():
        dur = sum(s.t[-1] - s.t[0] for s in b.segments)
        lines.append(f"  {b.label}: {b.samples} samples, {dur:.1f} s, "
                     f"{b.gaps} gap(s)")
    if sync_msg:
        lines.append(f"  sync: {sync_msg}")
    lines.append("")

    results = {}
    for bid, b in boards.items():
        if args.kind == "swim":
            r = swim.analyze(b)
            lines += swim_text(r)
            if r["summary"]["cycles"]:
                plot_swim(r, out / f"swim_board{bid}.png")
        else:
            r = shot.analyze(b)
            lines += shot_text(r)
            if r["summary"]["shots"]:
                plot_shot(r, out / f"shot_board{bid}.png")
        lines.append("")
        results[bid] = r

    summary = {"kind": args.kind, "csv": src.name,
               "boards": {k: jsonable(v) for k, v in results.items()}}
    if args.kind == "swim" and len(results) == 2:
        t = swim.arm_timing(results[1], results[2])
        summary["arm_timing"] = t
        if t:
            lines.append(f"Arm timing: board 2 starts its cycle {t['phase_pct']}% "
                         f"into board 1's cycle -> {t['pattern']}")
            if args.sync == "start":
                lines.append("  (boards lined up by their first samples; use "
                             "--sync tap for a more exact timing)")
    (out / "summary.json").write_text(json.dumps(summary, indent=1, default=float))
    text = "\n".join(lines).rstrip() + "\n"
    (out / "summary.txt").write_text(text)
    print(text + f"\nResults in {out}/")


if __name__ == "__main__":
    main()
