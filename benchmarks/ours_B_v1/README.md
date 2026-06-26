# OURS — navigation stack, A→B, v1

First real A→B run of **our** navigation (A* + DWA + camera + IMU-contact), to compare against the
Unitree firmware baseline (`../firmware_native_B/`).

Run: 2026-06-26 19:23:32. Background map = Summit ground-truth (G1-aligned).

## What WORKED (vs all previous attempts)
- **Live laser now arrives** (nobs avg ~796, max 1035) — fixed by `get_live_cdp` (connect to the live
  WebView page, not a dead one).
- **Self-estimated localization confidence `loc_match` works**: avg **0.73** (min 0.33, max 0.94).
  The robot can now tell when it is well/poorly localized — the meta-cognition signal.
- Relocalization good (FRAME-CHECK offset 0.16 m).
- It **crossed toward B**: min distance reached **3.10 m** (from 6.2 m) — got into the doorway region,
  further than nothing.

## What did NOT work yet
- **Did not reach B** (aborted by user at 112 s).
- **Very inefficient**: path **13.97 m** for ~3 m of net progress; **3 collisions**; min clearance 0.05 m.
- Phase histogram shows heavy unsticking/turning: SEEK-T 184, SEEK-S 130, **BRK-TR 197** — the robot
  repeatedly gets stuck and spins/maneuvers. Strong sign the **turn-sign / spin issue** (pending
  `turntest`) is degrading execution: it plans fine but executes turns poorly and loops.

## Next steps
1. Run `turntest` to settle the steering-sign bug → should drastically cut the SEEK/BRK churn.
2. Re-run; expect a much cleaner, shorter path that actually reaches B through the door.

## Files
- `run.json` — full dataset run (trajectory, telemetry incl. loc_match + battery + IMU, events, summary).
- `*_col1/2/3.json` + `.jpg` — 3D cloud + camera at each collision.
- `ours_B_v1_result.png` — left: trajectory on the map coloured by loc_match + collisions; right:
  loc_match and distance-to-B over time.

## Headline comparison (v1)
| | Firmware (native) | Ours v1 |
|---|---|---|
| Reached B | No (stuck at door) | No (aborted) |
| Path | 2.23 m (gave up early) | 13.97 m (kept trying) |
| Min dist to B | 4.5 m | **3.10 m** |
| Collisions | 2 | 3 |
| Localization confidence | not reported by robot | **measured (loc_match avg 0.73)** |
