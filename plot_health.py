#!/usr/bin/env python3
"""
plot_health.py — visualize the robot's hardware self-report over a run.

Adrian: "we also have access to battery, robot temperature and motor health for
each joint" — Renxi: "that is also very helpful". This plots the telemetry stream
that g1_goto.py logs (~1 Hz) in each dataset run:

  - battery %  and  battery / CPU temperature  over time
  - max motor temperature + hottest joint
  - per-joint motor TEMPERATURE heatmap (joint x time)
  - motor error count over time

Usage:
  python plot_health.py dataset/<run>.json [out.png]
  python plot_health.py            # newest dataset/*.json
"""
import sys, os, glob, json
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def newest():
    fs = [f for f in glob.glob("dataset/*.json")
          if not f.endswith("_end.json") and "_col" not in f and "_noise" not in f]
    return max(fs, key=os.path.getmtime) if fs else None


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else newest()
    if not path or not os.path.exists(path):
        print("No run JSON. Pass one: python plot_health.py dataset/<run>.json"); return
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.splitext(path)[0] + "_health.png"
    d = json.load(open(path))
    tel = d.get("telemetry", [])
    if not tel:
        print("This run has no telemetry stream (older run). Re-run after the health-logging update."); return

    t = [r.get("t", i) for i, r in enumerate(tel)]
    bat = [r.get("bat") for r in tel]
    batT = [r.get("batT") for r in tel]
    cpuT = [r.get("cpuT") for r in tel]
    motTmax = [r.get("motTmax") for r in tel]
    motThot = [r.get("motThot") for r in tel]
    merr = [r.get("merr") for r in tel]
    mtemps = [r.get("motorTemp") for r in tel if isinstance(r.get("motorTemp"), list)]

    fig = plt.figure(figsize=(13, 9))
    gs = fig.add_gridspec(3, 2, height_ratios=[1, 1, 1.2])

    a = fig.add_subplot(gs[0, 0])
    a.plot(t, bat, c="#2e7d32"); a.set_title("battery (%)"); a.set_xlabel("s"); a.set_ylim(0, 100); a.grid(alpha=0.3)

    a = fig.add_subplot(gs[0, 1])
    if any(v is not None for v in batT): a.plot(t, batT, c="#c0392b", label="battery T")
    if any(v is not None for v in cpuT): a.plot(t, cpuT, c="#e67e22", label="CPU T")
    a.set_title("temperatures (°C)"); a.set_xlabel("s"); a.legend(fontsize=8); a.grid(alpha=0.3)

    a = fig.add_subplot(gs[1, 0])
    a.plot(t, motTmax, c="#8e44ad"); a.set_title("max motor temperature (°C)"); a.set_xlabel("s"); a.grid(alpha=0.3)

    a = fig.add_subplot(gs[1, 1])
    a.plot(t, merr, c="#c0392b"); a.set_title("motor error count"); a.set_xlabel("s"); a.grid(alpha=0.3)

    a = fig.add_subplot(gs[2, :])
    if mtemps:
        M = np.array([m for m in mtemps], dtype=float).T   # joints x time
        im = a.imshow(M, aspect="auto", cmap="inferno", origin="lower",
                      extent=[t[0], t[-1], 0, M.shape[0]])
        fig.colorbar(im, ax=a, label="motor temp (°C)", shrink=0.8)
        a.set_title("per-joint motor temperature (joint × time)")
        a.set_xlabel("s"); a.set_ylabel("joint index")
    else:
        a.text(0.5, 0.5, "no per-joint motorTemp array in telemetry\n(re-run after the health-logging update)",
               ha="center", va="center"); a.axis("off")

    sm = d.get("summary", {})
    fig.suptitle(f"{d.get('mode','')} → {d.get('label','')}   hardware health   "
                 f"(battery {bat[0] if bat else '?'}→{bat[-1] if bat else '?'}%, "
                 f"result {d.get('result','?')})", fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(out, dpi=100, bbox_inches="tight")
    print("saved", out)
    if bat and bat[0] is not None:
        print(f"  battery {bat[0]}→{bat[-1]}%   max motor T {max(v for v in motTmax if v is not None) if any(motTmax) else '?'}°C"
              f"   joints logged: {len(mtemps[0]) if mtemps else 0}")


if __name__ == "__main__":
    main()
