# G1 navigation — GPU perception upgrade (offboard, 2×RTX + 120 GB)

Design notes for the next big jump in `g1_goto.py`. Motivated directly by the
log of the first successful A→B run (`dataset/20260626_200501_ours_B.json`,
saved as `benchmarks/ours_B_v3_REACHED/`).

## 1. What the successful run actually showed (the problem to kill)

From the v3 "reached" log (157.8 s, error 0.31 m, 10 collisions):

| Symptom | Measured | Meaning |
|---|---|---|
| Spinning in place | **42 % of ticks** | recovery thrash, not progress |
| Longest stall | **356 ticks (~50 s)**, ending t=78.5 at d=2.13 m | stuck fighting the doorway |
| Collisions | **10, all at x≈−3.3…−4.4** | the table at the door |
| Vision-assist used | **DOOR-GOv only 1.3 % of ticks** | classical `floor_free_bands` too weak/late |
| Battery | **22 % → 19 %** | degraded gait/LiDAR/balance |

Root cause (one line): **the table in the doorway is invisible to the head
LiDAR**, so A* keeps planning through it, the robot bumps it, recovers, spins,
and repeats for ~50 s. The current camera heuristic barely fires.

**Cheap immediate win, no code:** run trials at **>80 % battery**. Low battery
measurably worsened gait and LiDAR stability and fed the spinning.

## 2. The fix: an offboard GPU perception server

Keep `g1_goto.py` as a *thin control client*. Move heavy perception to a
separate process on the Ubuntu box and stream results back.

```
G1 (WebRTC/CDP on Mac)  ──camera frames + 'location' laser──▶  Perception server (Ubuntu, 2×RTX)
        ▲                                                              │
        └──────────── fused costmap + free-space + objects ◀──────────┘
                          (local HTTP / ZeroMQ / websocket, ~15 Hz)
```

- Robot link stays exactly as today (Mac + `get_live_cdp` + `grab_cam`).
- New: publish each camera frame (already a `data:` image) + pose to the server.
- Server returns: a **forward obstacle strip / costmap delta**, a **free-space
  mask**, and a **detected-object list** (table, door, person…).
- `g1_goto.py` fuses that into the existing A*/DWA costmap. Drop-in: replace
  `cam_floor_clear()` internals with the server's mask; keep the same return
  signature so the door logic doesn't change shape.

GPU split: depth on GPU0, segmentation+detection on GPU1, run in parallel →
~15–30 Hz. 120 GB RAM → buffer and log every frame for the paper datasets.
Optional TensorRT/FP16 for latency.

## 3. Models to add (in priority order)

1. **Metric monocular depth** — Depth Anything V2 (metric) / Metric3D v2 /
   UniDepth. Per-pixel metric depth → project the floor-height band to a 2D
   "virtual scan" in front of the robot. **This is the #1 fix: it sees the
   table the LiDAR misses.** Fuse this virtual scan into `build_costmap`.
2. **Semantic segmentation** — SegFormer/Mask2Former (ADE20K indoor). Real
   floor / wall / door / furniture masks → replaces the `floor_free_bands`
   heuristic with a true free-space mask, and gives a **door** class.
3. **Object detection** — YOLO11 / RT-DETR. Detect *table, chair, door frame,
   person*. Person → drives the **human-proximity analogy** the DCA/DCE paper
   needs; furniture → semantic costmap.
4. **Door pose** (from seg/detection keypoints) — estimate door centre + axis →
   align-to-centre then straight push, with depth clearance on both jambs.
   Removes the 50 s table fight.
5. *(optional)* **Small VLM scene check** (e.g. Qwen2-VL) — slow meta-signal:
   "is the doorway clear?" used only as a low-rate sanity vote.

## 4. Depth→costmap fusion (the core change)

- Build a forward occupancy strip from metric depth (floor-height band, same
  z-window logic as the LiDAR cloud, `HBAND_LO/HI`).
- Fuse: `obstacle = laser ∪ depth_virtual_scan`, confidence-weighted.
- **In the door zone, trust depth > laser** — this is the earlier insight, now
  backed by a real depth model instead of a heuristic.
- A* then plans around the real table → far fewer collisions and far less
  spinning (the 42 % should collapse).

## 5. Smoother recovery

The 42 % spin also came from `plan=0` recovery thrash. With real depth obstacles
A* almost always finds a route, so recovery rarely triggers. Additionally:
replace in-place spin recovery with a **short back-arc** (reverse + gentle turn)
to reduce loc drift and re-localise faster.

## 6. Why this doubles as the paper instrumentation

Depth + segmentation + detection produce a much richer **Shared Experience
Interface** for the DCA/DCE work:

| Paper meta-parameter | New source |
|---|---|
| clearance_pressure | metric depth virtual scan (sees the table) |
| alignment_stability | door-pose axis vs heading |
| human_proximity_pressure | person detection |
| sensing_reliability | depth/laser agreement as a plausibility signal |
| payload_stability | torso IMU (taped cup) — unchanged |

So the same upgrade that makes door crossing reliable also generates the
evidence streams the DCE experiments need. Build once, use for both.

## 7. Suggested build order

1. Stand up the perception server skeleton (frame in → JSON out) + a `--perc`
   flag in `g1_goto.py` to use it (fallback to current heuristic if offline).
2. Add metric depth → virtual scan → costmap fusion. Re-run `gotoviz B`,
   compare collisions/spin/time vs v3.
3. Add segmentation free-space mask (replace `cam_floor_clear` internals).
4. Add detection (table/person/door) + door-pose alignment.
5. Log everything to the dataset schema for the paper.

Target after step 2–4: door crossing with **0–1 collisions, <60 s, <10 % spin**
(vs 10 / 158 s / 42 % today).
