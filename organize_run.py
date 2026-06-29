#!/usr/bin/env python3
"""
organize_run.py — bundle a run into a tidy, dated folder with all its images + a README.

Creates  runs/<YYYY-MM-DD_HH-MM-SS>_<label>/  containing:
  run.json                 (the dataset)
  01_trajectory.png        (path + LiDAR + YOLO objects)
  02_metrics.png           (clearance / progression / reliability)
  03_health.png            (battery / temperatures / per-joint motors)
  collision_*.jpg          (camera photo at each collision, if any)
  README.md                (a clean summary of the run)

Usage:
  python organize_run.py dataset/<run>.json
  python organize_run.py                       # newest run
"""
import sys, os, glob, json, shutil, subprocess
from collections import Counter

HERE = os.path.dirname(os.path.abspath(__file__))


def newest():
    fs = [f for f in glob.glob("dataset/*.json")
          if not f.endswith("_end.json") and "_col" not in f and "_noise" not in f]
    return max(fs, key=os.path.getmtime) if fs else None


def pretty_dt(base):
    # base like 20260629_144821 -> 2026-06-29_14-48-21
    d = os.path.basename(base)
    p = d.split("_")
    if len(p) >= 2 and len(p[0]) == 8 and len(p[1]) >= 6:
        ymd, hms = p[0], p[1]
        return f"{ymd[:4]}-{ymd[4:6]}-{ymd[6:8]}_{hms[:2]}-{hms[2:4]}-{hms[4:6]}"
    return d


def fnum(v, nd=2):
    try:
        return f"{float(v):.{nd}f}"
    except (TypeError, ValueError):
        return "—"


def main():
    run = sys.argv[1] if len(sys.argv) > 1 else newest()
    if not run or not os.path.exists(run):
        print("No run JSON. Pass one: python organize_run.py dataset/<run>.json"); return
    d = json.load(open(run)); s = d["samples"]; sm = d.get("summary", {})
    base = run[:-5]
    label = d.get("label", "B")
    folder = os.path.join(HERE, "runs", f"{pretty_dt(base)}_{d.get('mode','ours')}_{label}")
    os.makedirs(folder, exist_ok=True)

    # (re)generate the three figures
    for tool in ("plot_trajectory.py", "plot_metrics.py", "plot_health.py"):
        try:
            subprocess.run([sys.executable, os.path.join(HERE, tool), run],
                           cwd=HERE, capture_output=True, timeout=120)
        except Exception as e:
            print("warn:", tool, e)

    # copy in, with tidy names
    shutil.copy(run, os.path.join(folder, "run.json"))
    pairs = [("_trajectory.png", "01_trajectory.png"), ("_metrics.png", "02_metrics.png"),
             ("_health.png", "03_health.png")]
    for suf, dst in pairs:
        src = base + suf
        if os.path.exists(src):
            shutil.copy(src, os.path.join(folder, dst))
    ncol = 0
    for jpg in sorted(glob.glob(base + "_col*.jpg")):
        ncol += 1
        shutil.copy(jpg, os.path.join(folder, f"collision_{ncol}.jpg"))

    # stats for the README
    def mean(key):
        v = [r.get(key) for r in s if r.get(key) is not None]
        return sum(v) / len(v) if v else None
    labs = Counter()
    for r in s:
        for dd in (r.get("dets") or []):
            labs[dd[0]] += 1
    tel = d.get("telemetry", [])
    bat = [t.get("bat") for t in tel if t.get("bat") is not None]
    mott = [t.get("motTmax") for t in tel if t.get("motTmax") is not None]
    objs = ", ".join(f"{k} x{v}" for k, v in labs.most_common()) or "none (vision off or no objects)"

    md = f"""# G1 run — {pretty_dt(base).replace('_', '  ')}

**Goal {label}** · result: **{d.get('result','?').upper()}** · duration {fnum(sm.get('time_s'),1)} s

## Outcome
| | |
|---|---|
| Reached | {'YES' if d.get('result')=='reached' else 'NO'} |
| Time | {fnum(sm.get('time_s'),1)} s |
| Path length | {fnum(sm.get('path_m'))} m (straight {fnum(sm.get('straight_m'))} m, efficiency {fnum(sm.get('efficiency'))}) |
| Collisions | {sm.get('collisions','?')} |
| Min clearance | {fnum(sm.get('c0min'))} m |

## Perception (vision)
| | |
|---|---|
| Vision queries (OK) | {sm.get('perc_queries','?')} |
| YOLO objects seen | {objs} |
| Depth obstacle cells (perc_n max) | {max((r.get('perc_n',0) for r in s), default=0)} |

## Robot "experience" (means over the run)
| metric | mean |
|---|---|
| clearance (free space ahead) | {fnum(mean('clearance'))} |
| progression (advancing to goal) | {fnum(mean('progression'))} |
| sensing reliability (self-trust) | {fnum(mean('reliability'))} |
| localisation match | {fnum(mean('loc_match'))} |

## Hardware
| | |
|---|---|
| Battery | {bat[0] if bat else '—'} → {bat[-1] if bat else '—'} % |
| Max motor temp | {max(mott) if mott else '—'} °C |
| Samples / telemetry rows | {len(s)} / {len(tel)} |

## Files
- `01_trajectory.png` — path + everything the LiDAR saw + YOLO objects on the map
- `02_metrics.png` — clearance / progression / sensing reliability over time
- `03_health.png` — battery, temperatures, per-joint motor heatmap
- `run.json` — the full dataset (schema g1_goto_run/v1)
"""
    open(os.path.join(folder, "README.md"), "w").write(md)
    print("bundled ->", folder)
    for f in sorted(os.listdir(folder)):
        print("   ", f)


if __name__ == "__main__":
    main()
