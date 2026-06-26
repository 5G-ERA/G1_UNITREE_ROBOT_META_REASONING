# Benchmark — Unitree firmware native navigation A→B

Reference run of the **G1's own (native) navigation** to waypoint **B**, used as the baseline to
compare against our navigation stack.

- Command sent: `rt/api/slam_operate/request` `api_id 1102` (anyPointNavigation), `targetPose B`, `mode 1`
  — i.e. the **exact command the Unitree app itself sends** (confirmed by app-sniff).
- The firmware drives on its own internal map/planner (a black box; its planned path is **never**
  transmitted, so only the executed odometry trajectory is observable).

## Result (run 2026-06-26 19:22:04)
- Relocalization: **perfect** (FRAME-CHECK offset 0.06 m).
- Outcome: **DID NOT REACH B**. Aborted after 28.9 s.
- Path: **2.23 m of ~6.2 m** (~36%). **2 collisions**, min clearance 0.76 m.
- Failure mode: got **stuck at the doorway** between the two rooms (collision at ≈(-0.31, 1.20)),
  pushing against an obstacle its head-LiDAR cannot see.

This is the key baseline finding: native nav handles short/clear goals (C) but **cannot cross the
doorway to B** — the limitation our camera + IMU-contact stack is meant to overcome.

## Files
- `run.json` — full dataset run (trajectory, telemetry, events, summary; schema g1_goto_run/v1).
- `*_col1/col2.json` + `.jpg` — 3D cloud + camera photo at each collision.
- `*_end.json` — 3D cloud at the stall point.
- `session.log` — text log of the session.
- `firmware_B_result.png` — trajectory on the map (Summit-aligned), collisions, stall at the door.
