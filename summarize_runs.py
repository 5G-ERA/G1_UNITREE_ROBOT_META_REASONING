#!/usr/bin/env python3
"""
summarize_runs.py — turn every run in dataset/ into one comparison table (DCE vs FSM vs default).

Writes runs_summary.csv with one row per run. Most columns are filled AUTOMATICALLY from each run's
dataset; you only fill 'condition' and 'notes' by hand. This is the raw table for the paper's results.

Usage:
  python summarize_runs.py            # scans dataset/, writes runs_summary.csv (+ prints a table)
"""
import glob, json, os, csv


def mean(s, k):
    v = [r.get(k) for r in s if r.get(k) is not None]
    return round(sum(v) / len(v), 3) if v else ""


def main():
    rows = []
    for f in sorted(glob.glob("dataset/*.json")):
        if any(t in f for t in ("_col", "_end", "_noise")):
            continue
        try:
            d = json.load(open(f))
        except Exception:
            continue
        s = d.get("samples", [])
        if not s:
            continue
        sm = d.get("summary", {}); ev = d.get("events", [])
        rows.append({
            "file": os.path.basename(f),
            "governance": d.get("governance", "?"),        # DCE / DCE-<ablation> / FSM / default
            "mode": d.get("mode", ""),                     # ours / native
            "condition": "",                               # <-- FILL: door / payload_no_lid / human / low_batt
            "result": d.get("result", ""),
            "time_s": sm.get("time_s", ""),
            "path_m": sm.get("path_m", ""),
            "efficiency": sm.get("efficiency", ""),
            "collisions": sm.get("collisions", sum(1 for e in ev if e.get("kind") == "collision")),
            "c0min": sm.get("c0min", ""),
            "spills_human": sm.get("spills_human", ""),
            "perc_queries": sm.get("perc_queries", ""),
            "meta_switches": sum(1 for e in ev if e.get("kind") == "meta_switch"),
            "fsm_interventions": sum(1 for e in ev if e.get("kind") == "fsm_intervention"),
            "mean_clearance": mean(s, "clearance"),
            "mean_progression": mean(s, "progression"),
            "mean_reliability": mean(s, "reliability"),
            "notes": "",                                   # <-- FILL: appropriate switch? anything odd?
        })
    cols = ["file", "governance", "mode", "condition", "result", "time_s", "path_m", "efficiency",
            "collisions", "c0min", "spills_human", "perc_queries", "meta_switches", "fsm_interventions",
            "mean_clearance", "mean_progression", "mean_reliability", "notes"]
    with open("runs_summary.csv", "w", newline="") as fo:
        w = csv.DictWriter(fo, fieldnames=cols); w.writeheader()
        for r in rows:
            w.writerow(r)
    print(f"wrote runs_summary.csv  ({len(rows)} runs)\n")
    print(f"{'file':32s} {'gov':14s} {'result':8s} {'t':>6s} {'col':>3s} {'spill':>5s} {'sw':>3s} {'iv':>3s}")
    for r in rows:
        print(f"{r['file'][:32]:32s} {str(r['governance'])[:14]:14s} {str(r['result']):8s} "
              f"{str(r['time_s']):>6s} {str(r['collisions']):>3s} {str(r['spills_human']):>5s} "
              f"{r['meta_switches']:>3d} {r['fsm_interventions']:>3d}")
    print("\nOpen runs_summary.csv in Excel/Sheets and fill the 'condition' and 'notes' columns.")


if __name__ == "__main__":
    main()
