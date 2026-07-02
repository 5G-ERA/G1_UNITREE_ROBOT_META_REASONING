#!/usr/bin/env python3
"""
autopsy.py — one-command HTML report for a single run (the full story of the traverse).

Bundles into one self-contained file: summary, phase-colored trajectory with events,
timelines (progress, clearances, vision, noise, planner), events table, and every photo of the
run (tNNNs filmstrip, collision and pre-impact frames) embedded as base64.

Usage:
  python autopsy.py dataset/20260702_130231_ours_B.json          # -> autopsy_<run>.html
  python autopsy.py --latest                                     # most recent run in dataset/
"""
import base64
import glob
import io
import json
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def _b64png(fig):
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=110, bbox_inches="tight")
    plt.close(fig)
    return base64.b64encode(buf.getvalue()).decode()


def _b64jpg(path):
    try:
        return base64.b64encode(open(path, "rb").read()).decode()
    except Exception:
        return None


PHASE_COLORS = {"DWA": "#2b8cbe", "DOOR": "#e6550d", "BRK": "#756bb1", "SEEK": "#d62728",
                "R-": "#8c564b", "STOP": "#7f7f7f"}


def _phase_color(ph):
    p = ph.replace("AGR-", "")
    for k, c in PHASE_COLORS.items():
        if p.startswith(k):
            return c
    return "#333333"


def build(run_json, out=None):
    d = json.load(open(run_json))
    s = d.get("samples", [])
    sm = d.get("summary", {})
    ev = d.get("events", [])
    base = os.path.splitext(run_json)[0]
    name = os.path.basename(base)
    out = out or f"autopsy_{name}.html"

    # ---------------- fig 1: trayectoria ----------------
    fig, ax = plt.subplots(figsize=(7, 6))
    for i in range(1, len(s)):
        a, b = s[i - 1], s[i]
        ax.plot([a["x"], b["x"]], [a["y"], b["y"]], color=_phase_color(a["phase"]), lw=1.6)
    if s:
        ax.plot(s[0]["x"], s[0]["y"], "g^", ms=11, label="start")
    g = d.get("goal", {})
    if g:
        ax.plot(g.get("x"), g.get("y"), "b*", ms=16, label="goal")
    for e in ev:
        if e["kind"] == "collision":
            ax.plot(e["x"], e["y"], "rx", ms=13, mew=3)
        elif e["kind"] == "astar_fail":
            ax.plot(e["x"], e["y"], "o", color="#d62728", ms=5, mfc="none")
        elif e["kind"].startswith("reloc"):
            ax.plot(e["x"], e["y"], "md", ms=7)
    # leyenda de fases
    for k, c in PHASE_COLORS.items():
        ax.plot([], [], color=c, label=k)
    ax.plot([], [], "rx", label="collision")
    ax.plot([], [], "o", color="#d62728", mfc="none", label="A*-fail")
    ax.set_aspect("equal"); ax.grid(alpha=0.3); ax.legend(fontsize=7, loc="best")
    ax.set_title(f"{name} — trajectory by phase")
    img_traj = _b64png(fig)

    # ---------------- fig 2: lineas de tiempo ----------------
    t = [x["t"] for x in s]
    fig, axs = plt.subplots(5, 1, figsize=(9, 10), sharex=True)
    axs[0].plot(t, [x["d"] for x in s], "b-"); axs[0].set_ylabel("dist to goal (m)")
    axs[0].grid(alpha=0.3)
    axs[1].plot(t, [x["c0"] for x in s], label="c0 (all obstacles)", lw=1)
    if any("c0_hard" in x for x in s):
        axs[1].plot(t, [x.get("c0_hard") for x in s], label="c0_hard (walls/persistent)", lw=1)
    axs[1].set_ylabel("clearance (m)"); axs[1].legend(fontsize=7); axs[1].grid(alpha=0.3)
    axs[2].plot(t, [x.get("perc_n", 0) for x in s], label="perc_n (vision cells)", lw=1)
    if any(x.get("color_pts") is not None for x in s):
        axs[2].plot(t, [x.get("color_pts") or 0 for x in s], label="color_pts", lw=1)
    axs[2].set_ylabel("vision"); axs[2].legend(fontsize=7); axs[2].grid(alpha=0.3)
    axs[3].plot(t, [x.get("laser_noise", 0) for x in s], label="laser_noise", lw=1)
    axs[3].plot(t, [x.get("filt_rej", 0) for x in s], label="filt_rej", lw=1)
    axs[3].set_ylabel("noise"); axs[3].legend(fontsize=7); axs[3].grid(alpha=0.3)
    pn = [x.get("plan_n", None) for x in s]
    axs[4].plot(t, [v if v is not None else float("nan") for v in pn], lw=1, label="plan_n")
    fails = [x["t"] for x, v in zip(s, pn) if v == 0]
    for tf in fails:
        axs[4].axvline(tf, color="r", alpha=0.25, lw=0.8)
    axs[4].set_ylabel("plan"); axs[4].set_xlabel("t (s)"); axs[4].legend(fontsize=7); axs[4].grid(alpha=0.3)
    for e in ev:
        if e["kind"] == "collision":
            for a in axs:
                a.axvline(e["t"], color="r", ls="--", alpha=0.6)
    img_tl = _b64png(fig)

    # ---------------- fotos (película + colisiones + pre-frames) ----------------
    photos = sorted(glob.glob(base + "_t*.jpg")) + sorted(glob.glob(base + "_col*.jpg"))
    tiles = []
    for p in photos:
        b = _b64jpg(p)
        if b:
            tag = os.path.basename(p)[len(name) + 1:-4]
            tiles.append(f"<div class='ph'><img src='data:image/jpeg;base64,{b}'><span>{tag}</span></div>")

    # ---------------- html ----------------
    rows = "".join(f"<tr><td>{k}</td><td>{v}</td></tr>" for k, v in sm.items())
    evrows = "".join(f"<tr><td>{e['t']}</td><td>{e['kind']}</td><td>({e['x']:.2f},{e['y']:.2f})</td>"
                     f"<td>{ {k: v for k, v in e.items() if k not in ('t', 'kind', 'x', 'y', 'omap_near')} }</td></tr>"
                     for e in ev)
    from collections import Counter
    ph = Counter(x["phase"] for x in s)
    phrows = "".join(f"<tr><td>{k}</td><td>{v}</td></tr>" for k, v in ph.most_common())
    html = f"""<!doctype html><meta charset='utf-8'><title>autopsy {name}</title>
<style>body{{font-family:system-ui;margin:20px;background:#fafafa}} h2{{margin:24px 0 8px}}
table{{border-collapse:collapse;font-size:13px}} td{{border:1px solid #ccc;padding:3px 8px}}
.ph{{display:inline-block;margin:4px;text-align:center;font-size:11px}}
.ph img{{width:236px;display:block;border:1px solid #999}}
.flex{{display:flex;gap:20px;flex-wrap:wrap}}</style>
<h1>Run autopsy — {name}</h1>
<p><b>result: {d.get('result', '?')}</b> · started {d.get('started', '?')} · {len(s)} ticks · {len(ev)} events</p>
<div class='flex'>
<div><h2>Summary</h2><table>{rows}</table></div>
<div><h2>Phases</h2><table>{phrows}</table></div>
</div>
<h2>Trajectory</h2><img src='data:image/png;base64,{img_traj}'>
<h2>Timelines</h2><img src='data:image/png;base64,{img_tl}'>
<h2>Events</h2><table><tr><td>t</td><td>kind</td><td>pos</td><td>detail</td></tr>{evrows}</table>
<h2>Photos ({len(tiles)})</h2>{''.join(tiles)}
"""
    open(out, "w").write(html)
    print(f"-> {out}")
    return out


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    if "--latest" in sys.argv:
        cands = [f for f in sorted(glob.glob("dataset/*_ours_*.json"))
                 if not any(t in f for t in ("_col", "_end", "_noise"))]
        if not cands:
            sys.exit("no runs in dataset/")
        build(cands[-1])
    elif args:
        build(args[0])
    else:
        print(__doc__)
