#!/usr/bin/env python3
"""Make English explainer diagrams of the traversal logs:
   docs/dataset_schema.png  — structure of dataset/<run>.json (schema g1_goto_run/v1)
   docs/log_anatomy.png     — anatomy of goto.log (header / per-tick line / end lines)
Run: python docs/make_log_diagrams.py
Deps: matplotlib.
"""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

HERE = os.path.dirname(os.path.abspath(__file__))
NAVY = "#1F3B73"

def box(ax, x, y, w, h, title, lines, header):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.012,rounding_size=0.02",
                                ec="#9aa6b2", fc="white", lw=1.2, zorder=2))
    ax.add_patch(FancyBboxPatch((x, y + h - 0.052), w, 0.052, boxstyle="round,pad=0.012,rounding_size=0.02",
                                ec=header, fc=header, lw=0, zorder=3))
    ax.text(x + 0.012, y + h - 0.026, title, color="white", fontsize=10.5, fontweight="bold",
            va="center", ha="left", zorder=4)
    ax.text(x + 0.014, y + h - 0.072, lines, color="#222", fontsize=8.1, va="top", ha="left",
            family="DejaVu Sans", zorder=4, linespacing=1.45)


# ============================ Diagram 1: JSON schema ============================
def dataset_schema():
    fig, ax = plt.subplots(figsize=(13, 8.6)); ax.set_xlim(0, 1); ax.set_ylim(0, 1); ax.axis("off")
    ax.text(0.5, 0.975, "Traversal dataset:  dataset/<date>_ours_B.json", ha="center",
            fontsize=15, fontweight="bold", color=NAVY)
    ax.text(0.5, 0.948, "schema \"g1_goto_run/v1\"  —  one JSON file per A→B run (also _native_ for the firmware benchmark)",
            ha="center", fontsize=9.5, color="#555")

    box(ax, 0.02, 0.78, 0.62, 0.145, "meta  (run header)",
        "mode, label, goal{x,y}, pcd (map name), hband\n"
        "started (date), frame_check (start-vs-waypoint offset)\n"
        "result: \"reached\" / \"aborted\"   ·   duration_s", NAVY)
    box(ax, 0.66, 0.78, 0.32, 0.145, "summary  (outcome)",
        "time_s, path_m, straight_m\n"
        "efficiency, collisions, c0min\n"
        "perc_queries (vision calls OK), start{x,y}", "#1a7a33")

    box(ax, 0.02, 0.355, 0.47, 0.40, "samples[]   ~10 Hz, one per control tick",
        "MOTION / ODOMETRY:\n"
        "  t, x, y, yaw, d (dist to goal), spd, phase, cmd[ly,rx]\n\n"
        "PERCEPTION (the 'experience'):\n"
        "  c0 (forward clearance m), nobs, clearance, clearance_m\n"
        "  perc_n (cells vision added), dets[[label,conf,brng,rng]]\n\n"
        "PERFORMANCE:\n"
        "  progression, progress_rate\n\n"
        "SELF-CAPACITY (sensing trust):\n"
        "  reliability, laser_noise, loc_conf, c0_std,\n"
        "  scan_churn, reloc_rate10s, loc_match\n\n"
        "QUICK HEALTH:  bat, cpuT, merr, err", "#1565c0")

    box(ax, 0.51, 0.355, 0.47, 0.40, "telemetry[]   ~1 Hz, hardware self-report",
        "BATTERY:  bat (%), vol, amp, batT\n\n"
        "COMPUTE:  cpuT, cpuU, cpuMem, cpuFreq\n\n"
        "MOTORS (per joint):\n"
        "  motorTemp[29], motorError[29]\n"
        "  motTmax, motThot (hottest joint), merr\n\n"
        "IMU / BODY:\n"
        "  accel[3], gyro[3], rpy[3], quat[4], legtau\n"
        "  pose_cov (firmware reports zeros)\n\n"
        "gait, sport (mode)", "#8e44ad")

    box(ax, 0.02, 0.13, 0.30, 0.205, "events[]",
        "one entry per:\n"
        "  collision {t,x,y,src}\n"
        "  reloc_jump {t,x,y,dist}\n\n"
        "(this run: 0 collisions)", "#c0392b")
    box(ax, 0.34, 0.13, 0.30, 0.205, "laser_snapshots[]   ~0.5 Hz",
        "t, pts[[x,y],...]\n\n"
        "the LiDAR points the robot\n"
        "saw, in MAP coordinates ->\n"
        "rebuilds 'what it saw'.", "#16a085")
    box(ax, 0.66, 0.13, 0.32, 0.205, "clouds[] / cams[]",
        "filenames of side files:\n"
        "  _col1.json = 3D cloud at a collision\n"
        "  _col1.jpg  = camera photo there\n"
        "  _end.json  = final 3D cloud", "#e67e22")

    ax.text(0.5, 0.075, "Plot it:  python plot_metrics.py <run>.json   ·   python plot_health.py <run>.json   ·   "
            "python plot_trajectory.py <run>.json",
            ha="center", fontsize=9.2, color=NAVY, fontweight="bold")
    ax.text(0.5, 0.04, "Tip: samples are the 10 Hz time series (clearance/progression/reliability live here); "
            "telemetry is the 1 Hz hardware track; laser_snapshots + clouds reconstruct what the robot perceived.",
            ha="center", fontsize=8.3, color="#555")
    fig.savefig(os.path.join(HERE, "dataset_schema.png"), dpi=130, bbox_inches="tight")
    print("saved docs/dataset_schema.png")


# ============================ Diagram 2: goto.log anatomy ============================
def log_anatomy():
    fig, ax = plt.subplots(figsize=(13, 8.0)); ax.set_xlim(0, 1); ax.set_ylim(0, 1); ax.axis("off")
    ax.text(0.5, 0.975, "goto.log  —  human-readable text log (one block per run, appended)",
            ha="center", fontsize=15, fontweight="bold", color=NAVY)

    # the real example block
    ax.add_patch(FancyBboxPatch((0.03, 0.70), 0.94, 0.20, boxstyle="round,pad=0.01",
                                ec="#9aa6b2", fc="#0f1720", lw=1.0))
    mono = [
        ("#7fd18b", "=== RUN ours 'B' -> (-4.73,+3.04)  2026-06-29 13:20:32 ===     <- header: date + goal"),
        ("#d8e0e8", "t=  0.0 DWA-F pos=(-1.60,-0.34) yaw=+113.9 d=2.73 c0=0.94 clear=1.00 prog=0.10 rel=0.92 obs=159 plan=2 cmd=(ly=+0.40,rx=+0.45)"),
        ("#9aa6b2", "                              ... one line per control tick (~10 Hz) ..."),
        ("#d8e0e8", "t= 47.6 DWA-F pos=(-5.12,+3.00) yaw=+70.9 d=0.39 c0=0.96 clear=0.64 prog=0.17 rel=0.69 obs=916 plan=3 cmd=(ly=+0.28,rx=+0.45)"),
        ("#ffd479", "REACHED B err=0.30 ncol=0  2026-06-29 13:21:20              <- outcome + wall-clock time"),
        ("#ffd479", "FASES {'DWA-F': 58, 'DOOR-AL': 7, 'DOOR-GO': 3}             <- time spent in each phase"),
        ("#7fd18b", "FIN"),
    ]
    yy = 0.876
    for col, txt in mono:
        ax.text(0.045, yy, txt, color=col, fontsize=7.0, family="DejaVu Sans Mono", va="top")
        yy -= 0.0246

    # legend of the per-tick fields
    ax.text(0.03, 0.63, "Per-tick line — what each field means", fontsize=12, fontweight="bold", color=NAVY)
    rows = [
        ("t=47.6", "time since run start (seconds)"),
        ("DWA-F", "phase / behaviour now (DWA-F=forward, DOOR-AL=align to door, DOOR-GO=cross, BRK/RECOV=recover)"),
        ("pos=(x,y)", "robot position in the map (metres)"),
        ("yaw=+70.9", "heading (degrees)"),
        ("d=0.39", "distance left to the goal B (metres)"),
        ("c0=0.96", "forward clearance: nearest obstacle straight ahead (metres)"),
        ("clear=0.64", "CLEARANCE metric 0..1 (free space ahead; high = open)"),
        ("prog=0.17", "PROGRESSION metric 0..1 (advancing to B; 0 = stopped)"),
        ("rel=0.69", "SENSING RELIABILITY 0..1 (trust in its own perception)"),
        ("obs=916", "number of obstacle cells the planner currently holds (LiDAR + vision)"),
        ("plan=3", "number of waypoints in the current A* path (0 = no route found)"),
        ("cmd=(ly,rx)", "command sent: ly = forward speed, rx = turn rate"),
    ]
    y = 0.575; x1, x2 = 0.05, 0.30
    for k, v in rows:
        ax.text(x1, y, k, fontsize=9, family="DejaVu Sans Mono", color="#1565c0", va="top", fontweight="bold")
        ax.text(x2, y, v, fontsize=9, color="#222", va="top")
        y -= 0.0445

    ax.text(0.03, 0.045, "End lines:  REACHED/ABORT (+ error, #collisions, time)  ·  FASES (phase histogram)  ·  FIN."
            "   The same per-tick fields are also stored, numerically, in the JSON samples[].",
            fontsize=8.6, color="#555")
    fig.savefig(os.path.join(HERE, "log_anatomy.png"), dpi=130, bbox_inches="tight")
    print("saved docs/log_anatomy.png")


if __name__ == "__main__":
    dataset_schema()
    log_anatomy()
