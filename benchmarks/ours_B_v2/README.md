# OURS — A→B, v2

Second run of our nav. Cleaner than v1 (path 7.9 m vs 14 m; 1 collision vs 3; loc_match avg **0.90**),
but **still did not reach B**: stalled at ~(-3.2, 0), **min distance 3.09 m** from B — "scared of the
doorway": the inflated costmap closes the narrow passage, so A* can't route through and the robot
loops/unsticks (SEEK 104, BRK-TR 55).

This run motivated the **AGGRESSIVE mode** (added after): when stuck without approaching B for
`AGGR_AFTER`=12 s, the planner drops costmap inflation (A* threads narrow doors) and the DWA clearance
drops to a safety floor `AGGR_ROBOT_R`=0.13 m — so it pushes through the door while keeping a minimum
safety margin. Enable always with `G1_AGGRESSIVE=1`, else it auto-triggers when stuck.

Files: run.json, collision clouds/photos, ours_B_v2_result.png (trajectory coloured by loc_match).
