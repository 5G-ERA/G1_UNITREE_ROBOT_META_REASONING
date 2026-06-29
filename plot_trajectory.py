#!/usr/bin/env python3
"""
plot_trajectory.py — reconstruct the whole traversal: the path + EVERYTHING the robot saw.

Overlays, in the map frame:
  - the path, coloured by clearance (green = open, red = blocked);
  - every LiDAR point the robot saw along the way (faint grey, from laser_snapshots);
  - the GPU-vision detections (YOLO: table / person / door ...) placed at their world position
    from the robot pose + detection bearing/range, clustered so repeats become one marker;
  - collisions (black x), start (A) and goal (B).

Usage:
  python plot_trajectory.py dataset/<run>.json [out.png]
  python plot_trajectory.py            # newest run

Note: detections only appear if the run used the GPU perception server (G1_PERC set). Older runs
still reconstruct the path + LiDAR.
"""
import sys, os, glob, json, math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection


def newest():
    fs = [f for f in glob.glob("dataset/*.json")
          if not f.endswith("_end.json") and "_col" not in f and "_noise" not in f]
    return max(fs, key=os.path.getmtime) if fs else None


DET_STYLE = {  # label substring -> (colour, marker)
    "person": ("#e41a1c", "*"), "table": ("#ff7f00", "s"), "diningtable": ("#ff7f00", "s"),
    "chair": ("#984ea3", "^"), "couch": ("#a65628", "P"), "door": ("#377eb8", "D"),
    "refrigerator": ("#999999", "h"),
}


def det_style(label):
    for k, v in DET_STYLE.items():
        if k in (label or "").lower():
            return v
    return ("#444444", "o")


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else newest()
    if not path or not os.path.exists(path):
        print("No run JSON. Pass one: python plot_trajectory.py dataset/<run>.json"); return
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.splitext(path)[0] + "_trajectory.png"
    d = json.load(open(path)); s = d["samples"]
    xs = np.array([r["x"] for r in s]); ys = np.array([r["y"] for r in s])
    clr = [r.get("clearance") for r in s]
    if any(c is None for c in clr):
        clr = [min(1.0, (r.get("c0", 0) or 0) / 1.5) for r in s]
    clr = np.array([0 if (c is None or c != c) else c for c in clr])

    fig, ax = plt.subplots(figsize=(12, 10))

    # --- everything the LiDAR saw (faint grey) ---
    laser_pts = []
    for snap in d.get("laser_snapshots", []):
        laser_pts += snap.get("pts", [])
    if laser_pts:
        L = np.array(laser_pts)
        ax.scatter(L[:, 0], L[:, 1], s=16, c="#5a6675", marker="o", linewidths=0, alpha=0.6,
                   label=f"LiDAR seen ({len(L)} pts)", zorder=1)

    # --- detected objects (project to world from pose + bearing/range), cluster repeats ---
    snaps = sorted(d.get("laser_snapshots", []), key=lambda z: z.get("t", 0))

    def lidar_range(t, x, y, aim_deg):
        """estimate distance to the nearest LiDAR point in the detection's bearing (for runs whose
        detections were logged without a range, e.g. uncalibrated depth)."""
        if not snaps:
            return None
        snap = min(snaps, key=lambda z: abs(z.get("t", 0) - t))
        best = None
        for px, py in snap.get("pts", []):
            dx = px - x; dy = py - y; dist = math.hypot(dx, dy)
            if dist < 0.1 or dist > 4.0:
                continue
            ang = abs((math.degrees(math.atan2(dy, dx)) - aim_deg + 180) % 360 - 180)
            if ang < 12 and (best is None or dist < best):
                best = dist
        return best

    raw = []; approx = 0
    for r in s:
        for dct in (r.get("dets") or []):
            lab, conf, bearing, rng = (dct + [None] * 4)[:4]
            if bearing is None:
                continue
            if rng is None:                      # no range logged -> estimate from LiDAR in that bearing
                rng = lidar_range(r["t"], r["x"], r["y"], r["yaw"] + bearing); approx += 1
            if rng is None:
                continue
            a = math.radians(r["yaw"] + bearing)
            wx = r["x"] + rng * math.cos(a); wy = r["y"] + rng * math.sin(a)
            raw.append((lab, conf or 0, wx, wy))
    clusters = []                          # (label, conf_max, x, y, n)  merge same-label within 0.7 m
    for lab, conf, wx, wy in raw:
        for c in clusters:
            if c[0] == lab and math.hypot(c[2] - wx, c[3] - wy) < 0.7:
                c[1] = max(c[1], conf); c[2] = (c[2] * c[4] + wx) / (c[4] + 1)
                c[3] = (c[3] * c[4] + wy) / (c[4] + 1); c[4] += 1
                break
        else:
            clusters.append([lab, conf, wx, wy, 1])
    seen_labels = set()
    for lab, conf, wx, wy, n in clusters:
        col, mk = det_style(lab)
        lbl = None
        if lab not in seen_labels:
            lbl = f"{lab} (YOLO)"; seen_labels.add(lab)
        ax.scatter([wx], [wy], s=240, c=col, marker=mk, edgecolors="k", linewidths=1.2,
                   alpha=0.9, zorder=5, label=lbl)
        ax.annotate(f"{lab} {conf:.2f}", (wx, wy), fontsize=8, color=col,
                    xytext=(6, 6), textcoords="offset points", zorder=6)

    # --- path coloured by clearance ---
    pts = np.array([xs, ys]).T.reshape(-1, 1, 2)
    segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
    lc = LineCollection(segs, cmap="RdYlGn", norm=plt.Normalize(0, 1), linewidth=5, zorder=3)
    lc.set_array(clr[:-1]); ax.add_collection(lc)
    cb = plt.colorbar(lc, ax=ax, shrink=0.8); cb.set_label("clearance (red = blocked, green = open)")

    for e in d.get("events", []):
        if e.get("kind") == "collision" and e.get("x") is not None:
            ax.plot(e["x"], e["y"], "x", c="black", ms=13, mew=3, zorder=7)
    gx = d.get("goal", {})
    if isinstance(gx, dict) and "x" in gx:
        ax.plot(gx["x"], gx["y"], "*", c="gold", ms=30, mec="k", mew=1.2, label="goal B", zorder=8)
    ax.plot(xs[0], ys[0], "o", c="#1a9850", ms=13, label="start A", zorder=8)

    ax.set_aspect("equal"); ax.grid(True, alpha=0.3); ax.margins(0.1)
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    sm = d.get("summary", {})
    nper = sm.get("perc_queries", 0)
    approx_note = "  (object range est. from LiDAR — calibrate camera for exact)" if approx and clusters else ""
    ax.set_title(f"{d.get('mode','')} → {d.get('label','')}: full traversal reconstruction\n"
                 f"{'REACHED' if d.get('result')=='reached' else d.get('result','?')}  ·  "
                 f"{sm.get('time_s','?')}s  ·  {sm.get('collisions','?')} collisions  ·  "
                 f"{len(clusters)} objects detected (YOLO)  ·  vision queries={nper}{approx_note}", fontsize=10.5)
    ax.legend(loc="best", fontsize=9)
    fig.savefig(out, dpi=110, bbox_inches="tight")
    print("saved", out)
    if not clusters and nper == 0:
        print("  (no YOLO detections — this run had no perception server; path + LiDAR shown.)")
    else:
        for lab in sorted(seen_labels):
            print("  detected:", lab)


if __name__ == "__main__":
    main()
