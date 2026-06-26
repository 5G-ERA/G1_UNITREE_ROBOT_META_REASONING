# 🏆 OURS — A→B, v3 — FIRST SUCCESSFUL TRAVERSAL

**The milestone run: our navigation stack REACHED B**, crossing the doorway where the Unitree
firmware gets stuck. Run 2026-06-26 20:05:01.

## Result
- **REACHED B** — final error **0.31 m** (goal -4.73, 3.04).
- Time 157.8 s, path 23.0 m (straight 6.3 m, efficiency 0.28 — slow/winding but it made it).
- 10 collisions, min clearance 0.05 m, localization confidence (loc_match) avg **0.87**.
- Crossed using the **vision-assisted door entry** (`DOOR-GOv`): where the LiDAR is noisy at the
  table/doorway, the camera's floor-segmentation confirmed clear floor and the robot pushed through;
  IMU-contact stayed as the safety net.

## Why this matters (vs the firmware baseline `../firmware_native_B/`)
| | Firmware (native) | Ours v3 |
|---|---|---|
| **Reached B** | ❌ stuck at the door | ✅ **0.31 m** |
| Approach | drove into the table, stalled | fast DWA on live laser |
| Door crossing | impossible (LiDAR-only, no recovery) | **vision over laser + IMU contact** |
| Localization confidence | not reported by robot | measured (loc_match 0.87) |

The firmware (LiDAR-only, black-box planner, no contact recovery) cannot cross the table-blocked
doorway. Our stack does it by combining: clean-map global planning (door open), live-laser local
avoidance, **vision-priority at the noisy door**, IMU-contact recovery, and an aggressive mode that
drops inflation/clearance to a safety floor. This is the core thesis result.

## Next (to make it clean/fast)
Efficiency 0.28 + 10 collisions = lots of fighting at the door. Tune: trust vision earlier, reduce
collisions by approaching the door more head-on, smoother aggressive transition.

## Files
- run.json (trajectory, telemetry incl. loc_match/IMU/battery, events, summary)
- collision clouds/photos, end cloud
- ours_B_v3_REACHED.png (trajectory coloured by time; star = B reached)
