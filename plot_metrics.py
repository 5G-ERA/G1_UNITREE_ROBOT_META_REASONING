#!/usr/bin/env python3
"""
plot_metrics.py — visualize the two metrics (clearance + progression) of a run.

"A meaningful visualization of the robot's perception and performance, rather
than blind testing" (Renxi). Reads a dataset run JSON written by g1_goto.py and
produces a figure:
  - top:    clearance(t) and progression(t) together, with collisions marked.
  - bottom: the robot path, coloured by clearance, with collisions marked.

Usage:
  python plot_metrics.py dataset/20260626_200501_ours_B.json [out.png]
  python plot_metrics.py            # uses the newest dataset/*.json
"""
import sys, os, glob, json
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def newest():
    fs = [f for f in glob.glob("dataset/*.json")
          if not f.endswith("_end.json") and "_col" not in f]
    return max(fs, key=os.path.getmtime) if fs else None


def get(s, *keys):
    for k in keys:
        if k in s and s[k] is not None:
            return s[k]
    return None


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else newest()
    if not path or not os.path.exists(path):
        print("No run JSON. Pass one: python plot_metrics.py dataset/<run>.json"); return
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.splitext(path)[0] + "_metrics.png"
    d = json.load(open(path)); s = d["samples"]
    t = [r["t"] for r in s]
    # clearance/progression: use logged values; fall back to deriving from c0/d if missing
    clr = [get(r, "clearance") for r in s]
    prog = [get(r, "progression") for r in s]
    if any(c is None for c in clr):                 # fallback: normalise c0 by 1.5 m
        clr = [min(1.0, (r.get("c0", 0) or 0) / 1.5) for r in s]
    if any(p is None for p in prog):                # fallback: progress rate from d
        prog = []
        for i, r in enumerate(s):
            j = max(0, i - 10)
            dt = (r["t"] - s[j]["t"]) or 1e-3
            prog.append(max(0.0, min(1.0, ((s[j]["d"] - r["d"]) / dt) / 0.30)))
    cols = [(e["t"], e.get("x"), e.get("y")) for e in d.get("events", []) if e.get("kind") == "collision"]

    fig, (a1, a2) = plt.subplots(2, 1, figsize=(12, 9), gridspec_kw={"height_ratios": [1, 1.3]})

    rel = [get(r, "reliability") for r in s]
    if any(v is None for v in rel):                 # fallback: use loc_match if reliability not logged
        rel = [(r.get("loc_match") if r.get("loc_match") is not None else float("nan")) for r in s]

    # ---- top: the metrics together ----
    a1.plot(t, clr, "-", c="#1565c0", lw=1.8, label="clearance (perception)")
    a1.plot(t, prog, "-", c="#e67e22", lw=1.8, label="progression (performance)")
    a1.plot(t, rel, "-", c="#2ca02c", lw=1.6, label="sensing reliability (self-capacity)")
    a1.fill_between(t, clr, alpha=0.08, color="#1565c0")
    for (ct, _, _) in cols:
        a1.axvline(ct, color="#c0392b", lw=0.8, alpha=0.5)
    a1.set_ylim(-0.02, 1.05); a1.set_xlabel("time (s)"); a1.set_ylabel("metric (0..1)")
    sm = d.get("summary", {})
    a1.set_title(f"{d.get('mode','')} → {d.get('label','')}   "
                 f"clearance & progression   (result: {d.get('result','?')}, "
                 f"collisions={sm.get('collisions','?')}, time={sm.get('time_s','?')}s)")
    a1.grid(True, alpha=0.3); a1.legend(loc="upper right")
    a1.text(0.01, 0.02, "red lines = collisions", transform=a1.transAxes, fontsize=8, color="#c0392b")

    # ---- bottom: path coloured by clearance ----
    xs = [r["x"] for r in s]; ys = [r["y"] for r in s]
    sc = a2.scatter(xs, ys, c=clr, cmap="RdYlGn", s=14, vmin=0, vmax=1)
    plt.colorbar(sc, ax=a2, label="clearance (red=blocked, green=open)", shrink=0.8)
    for (_, cx, cy) in cols:
        if cx is not None:
            a2.plot(cx, cy, "x", c="black", ms=10, mew=2)
    gx = d.get("goal", {})
    if isinstance(gx, dict) and "x" in gx:
        a2.plot(gx["x"], gx["y"], "*", c="gold", ms=24, mec="k", label="goal")
    a2.plot(xs[0], ys[0], "o", c="#1a9850", ms=10, label="start")
    a2.set_aspect("equal"); a2.grid(True, alpha=0.3)
    a2.set_xlabel("x (m)"); a2.set_ylabel("y (m)")
    a2.set_title("path coloured by clearance  (black x = collision)")
    a2.legend(loc="best", fontsize=9)

    fig.tight_layout()
    fig.savefig(out, dpi=100, bbox_inches="tight")
    print("saved", out)
    # quick numeric summary
    n = len(s) or 1
    rel_ok = [v for v in rel if v == v]             # drop NaN
    relm = (sum(rel_ok) / len(rel_ok)) if rel_ok else float("nan")
    print(f"  mean clearance={sum(clr)/n:.2f}  mean progression={sum(prog)/n:.2f}  "
          f"mean reliability={relm:.2f}  "
          f"ticks blocked(clear<0.2)={sum(1 for c in clr if c<0.2)}  "
          f"ticks stalled(prog<0.1)={sum(1 for p in prog if p<0.1)}")


if __name__ == "__main__":
    main()
