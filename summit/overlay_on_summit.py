#!/usr/bin/env python3
"""
overlay_on_summit.py  —  put a G1 run (from dataset/) onto the Summit ground-truth floor plan.

Uses g1_summit_transform.json (the G1<->Summit rigid transform from the A/B common points) to map the
G1 trajectory into the Summit map frame, then draws it over the Summit 2D occupancy map. Lets us compare
the G1's run (firmware or ours) against REAL geometry (true walls + door).

Usage:
  python3 overlay_on_summit.py ../dataset/<run>.json [out.png]

Needs (in this folder): g1_summit_transform.json, rbk_2026_06_26_16_22_47.yaml + .png
Deps: numpy, pyyaml, pillow, matplotlib
"""
import sys, os, json, math
import numpy as np, yaml
from PIL import Image
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
MAP_YAML = os.path.join(HERE, "rbk_2026_06_26_16_22_47.yaml")


def load_tf():
    T = json.load(open(os.path.join(HERE, "g1_summit_transform.json")))["g1_to_summit"]
    s, th, t = T["scale"], math.radians(T["rot_deg"]), np.array(T["t"])
    R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
    return lambda p: s * (R @ np.asarray(p, float)) + t


def main():
    if len(sys.argv) < 2:
        print(__doc__); return
    run = json.load(open(sys.argv[1]))
    out = sys.argv[2] if len(sys.argv) > 2 else "g1_run_on_summit.png"
    g2s = load_tf()
    y = yaml.safe_load(open(MAP_YAML)); res = y["resolution"]; ox, oy = y["origin"][:2]
    img = np.array(Image.open(os.path.join(HERE, y["image"]))); H, W = img.shape
    ext = [ox, ox + W * res, oy, oy + H * res]

    traj = np.array([g2s((s["x"], s["y"])) for s in run.get("samples", []) if "x" in s])
    ev = [g2s((e["x"], e["y"])) for e in run.get("events", []) if e.get("kind") == "collision"]
    goal = run.get("goal"); gp = g2s((goal["x"], goal["y"])) if goal else None

    fig, ax = plt.subplots(figsize=(13, 8))
    ax.imshow(img, cmap="gray", origin="upper", extent=ext, interpolation="nearest")
    if len(traj):
        ax.plot(traj[:, 0], traj[:, 1], "-", c="#1565c0", lw=2.2, label=f"G1 {run.get('mode','run')} trajectory")
        ax.plot(traj[0, 0], traj[0, 1], "o", c="#1a9850", ms=10, label="start")
    for e in ev:
        ax.plot(e[0], e[1], "X", c="red", ms=15, zorder=6)
    if gp is not None:
        ax.plot(gp[0], gp[1], "*", c="#f39c12", ms=22, mec="k", label=f"goal {run.get('label','')}")
    sm = run.get("summary", {})
    ax.set_title(f"G1 {run.get('mode','')} '{run.get('label','')}' on Summit ground-truth  |  "
                 f"t={sm.get('time_s','?')}s path={sm.get('path_m','?')}m col={sm.get('collisions','?')}")
    ax.grid(True, alpha=0.3, color="#2a8", lw=0.4); ax.set_xlabel("x summit (m)"); ax.set_ylabel("y summit (m)")
    ax.legend(loc="lower left", fontsize=9)
    fig.savefig(out, dpi=110, bbox_inches="tight")
    print("saved", out, "| trajectory pts:", len(traj), "| collisions:", len(ev))


if __name__ == "__main__":
    main()
