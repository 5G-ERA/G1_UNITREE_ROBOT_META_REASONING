#!/usr/bin/env python3
"""
plot_metrics.py — easy-to-read picture of a run's clearance, progression and sensing reliability.

Each metric gets its OWN band (no overlapping spaghetti), smoothed, with a plain-language label
and a colour (green = good, red = bad). Grey vertical bands mark where the robot PAUSED (progression
near zero). The bottom shows the path coloured by clearance. A plain summary is printed and drawn.

Usage:
  python plot_metrics.py dataset/<run>.json [out.png]
  python plot_metrics.py            # newest dataset/*.json
"""
import sys, os, glob, json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection


def newest():
    fs = [f for f in glob.glob("dataset/*.json")
          if not f.endswith("_end.json") and "_col" not in f and "_noise" not in f]
    return max(fs, key=os.path.getmtime) if fs else None


def get(s, *keys):
    for k in keys:
        if k in s and s[k] is not None:
            return s[k]
    return None


def smooth(y, w=5):
    y = np.array([np.nan if v is None else v for v in y], float)
    if len(y) < 3:
        return y
    # nan-aware moving average
    out = np.copy(y)
    for i in range(len(y)):
        a = max(0, i - w); b = min(len(y), i + w + 1)
        seg = y[a:b]; seg = seg[~np.isnan(seg)]
        out[i] = seg.mean() if len(seg) else np.nan
    return out


def band(ax, t, y, color, label, hint):
    """One metric as a filled 0..1 band with a plain label."""
    ax.fill_between(t, 0, y, color=color, alpha=0.25)
    ax.plot(t, y, color=color, lw=2)
    ax.set_ylim(0, 1.03); ax.set_yticks([0, 0.5, 1])
    ax.set_ylabel(label, fontsize=10, fontweight="bold")
    ax.text(0.005, 0.04, hint, transform=ax.transAxes, fontsize=8, color="#444", va="bottom")
    ax.grid(True, axis="x", alpha=0.25)
    ax.margins(x=0)


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else newest()
    if not path or not os.path.exists(path):
        print("No run JSON. Pass one: python plot_metrics.py dataset/<run>.json"); return
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.splitext(path)[0] + "_metrics.png"
    d = json.load(open(path)); s = d["samples"]
    t = [r["t"] for r in s]

    clr = [get(r, "clearance") for r in s]
    if any(c is None for c in clr):
        clr = [min(1.0, (r.get("c0", 0) or 0) / 1.5) for r in s]
    prog = [get(r, "progression") for r in s]
    if any(p is None for p in prog):
        prog = []
        for i, r in enumerate(s):
            j = max(0, i - 10); dt = (r["t"] - s[j]["t"]) or 1e-3
            prog.append(max(0.0, min(1.0, ((s[j]["d"] - r["d"]) / dt) / 0.30)))
    rel = [get(r, "reliability") for r in s]
    if any(v is None for v in rel):
        rel = [(r.get("loc_match") if r.get("loc_match") is not None else np.nan) for r in s]

    clrS, progS, relS = smooth(clr), smooth(prog), smooth(rel)
    cols = [(e["t"], e.get("x"), e.get("y")) for e in d.get("events", []) if e.get("kind") == "collision"]
    sm = d.get("summary", {})
    reached = d.get("result") == "reached"

    # pause intervals: progression near zero for a stretch
    paused = progS < 0.12
    spans = []
    i = 0
    while i < len(paused):
        if paused[i]:
            j = i
            while j < len(paused) and paused[j]:
                j += 1
            if t[j-1] - t[i] >= 1.5:               # only mark pauses >= 1.5 s
                spans.append((t[i], t[j-1]))
            i = j
        else:
            i += 1

    fig = plt.figure(figsize=(12, 10))
    gs = fig.add_gridspec(4, 1, height_ratios=[1, 1, 1, 2.4], hspace=0.55)
    a_c = fig.add_subplot(gs[0]); a_p = fig.add_subplot(gs[1], sharex=a_c)
    a_r = fig.add_subplot(gs[2], sharex=a_c); a_path = fig.add_subplot(gs[3])

    band(a_c, t, clrS, "#1565c0", "CLEARANCE", "free space ahead — high = open, low = something blocking")
    band(a_p, t, progS, "#e67e22", "PROGRESSION", "getting to the goal — high = moving to B, 0 = stopped")
    band(a_r, t, relS, "#2ca02c", "SENSING\nRELIABILITY", "trust in its own sensors — low = noisy/unsure")

    # cautious -> aggressive switch (capability switch)
    agg_t = next((r["t"] for r in s if r.get("aggressive")), None)
    if agg_t is None:
        agg_t = next((e["t"] for e in d.get("events", []) if e.get("kind") == "mode_switch"), None)
    for ax in (a_c, a_p, a_r):
        for (t0, t1) in spans:
            ax.axvspan(t0, t1, color="#888888", alpha=0.16, lw=0)
        for (ct, _, _) in cols:
            ax.axvline(ct, color="#c0392b", lw=1.2)
        if agg_t is not None:
            ax.axvline(agg_t, color="#6a0dad", lw=1.6, ls="--")
        for e in d.get("events", []):                # human-marked spills
            if e.get("kind") == "spill":
                ax.axvline(e["t"], color="#0aa3c2", lw=1.4, ls=":")
    if spans:
        a_c.text(spans[0][0], 1.06, "grey = robot paused", fontsize=8, color="#555")
    if agg_t is not None:
        a_c.text(agg_t, 1.06, "→ aggressive mode", fontsize=8, color="#6a0dad")
    a_r.set_xlabel("time (s)")

    # ---- path coloured by clearance, thicker line + points ----
    xs = np.array([r["x"] for r in s]); ys = np.array([r["y"] for r in s])
    cc = np.array([0 if (c is None or c != c) else c for c in clrS])
    pts = np.array([xs, ys]).T.reshape(-1, 1, 2)
    segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
    lc = LineCollection(segs, cmap="RdYlGn", norm=plt.Normalize(0, 1), linewidth=5)
    lc.set_array(cc[:-1]); a_path.add_collection(lc)
    cb = plt.colorbar(lc, ax=a_path, shrink=0.85)
    cb.set_label("clearance  (red = blocked, green = open)")
    for (_, cx, cy) in cols:
        if cx is not None:
            a_path.plot(cx, cy, "x", c="black", ms=12, mew=3)
    gx = d.get("goal", {})
    if isinstance(gx, dict) and "x" in gx:
        a_path.plot(gx["x"], gx["y"], "*", c="gold", ms=30, mec="k", mew=1.2, label="goal B")
    a_path.plot(xs[0], ys[0], "o", c="#1a9850", ms=13, label="start A")
    a_path.set_aspect("equal"); a_path.grid(True, alpha=0.3)
    a_path.margins(0.1)
    a_path.set_xlabel("x (m)"); a_path.set_ylabel("y (m)")
    a_path.set_title("where it went  (line colour = how much free space it saw)", fontsize=11)
    a_path.legend(loc="best", fontsize=10)

    # ---- plain-language headline + summary ----
    n = len(s) or 1
    pc_clear = 100 * sum(1 for c in clr if c is not None and c > 0.5) / n
    pc_move = 100 * sum(1 for p in prog if p is not None and p > 0.3) / n
    rel_ok = [v for v in rel if v == v]
    relm = (sum(rel_ok) / len(rel_ok)) if rel_ok else float("nan")
    head = ("REACHED B" if reached else "did NOT reach B") + \
           f" — {sm.get('time_s','?')}s, {sm.get('collisions','?')} collisions"
    fig.suptitle(head, fontsize=15, fontweight="bold",
                 color=("#1a7d3c" if reached and sm.get("collisions", 1) == 0 else "#b00000"))
    sub = (f"saw open space {pc_clear:.0f}% of the time   ·   "
           f"was advancing {pc_move:.0f}% of the time   ·   "
           f"mean sensing trust {relm:.2f}" + (f"   ·   paused {len(spans)}x" if spans else ""))
    fig.text(0.5, 0.945, sub, ha="center", fontsize=10, color="#333")

    fig.savefig(out, dpi=100, bbox_inches="tight")
    print("saved", out)
    print(" ", head)
    print(" ", sub)


if __name__ == "__main__":
    main()
