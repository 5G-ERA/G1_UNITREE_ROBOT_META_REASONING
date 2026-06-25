# Autonomous Navigation — Algorithm, Lessons & Improvement Roadmap

This document explains how `g1_nav.py` perceives, decides and acts, the hard-won lessons behind each
design choice, and a concrete roadmap — ending with the **meta-reasoning** direction that gives the
project its name.

---

## 1. The control loop (≈10 Hz)

Every cycle:
1. Read **odometry** (pose `x,y,yaw`) from the captured `slam_mapping/odom`.
2. Query the **LiDAR clean grid** for forward clearance `c0` (and, when escaping, a 360° scan).
3. Read the latest **camera vision** verdict (obstacle ahead? near/far? which side is freer?).
4. Run the **collision detector** (commanded-forward but odom not moving ⇒ bumped something).
5. Pick a command `(lx, ly, rx, ry)` from a small **behaviour stack** (below).
6. Set `window.__cmd`; an in-page 20 Hz driver publishes it on `rt/wirelesscontroller` with a
   **600 ms dead-man** (if Python or the bridge dies, the robot stops).

Behaviour priority (highest first): **collision recovery → vision avoidance (VAV) → coverage
redirection → reactive obstacle escape (ESC) → go forward**.

---

## 2. Perception — three complementary layers

No single sensor is enough on a consumer biped; each has a blind spot, so we fuse three.

### 2.1 LiDAR "clean grid" (the workhorse)
The app's raw decoded cloud is **noisy for navigation**: the robot's own body and the floor (picked
up as the head pitches while walking) get baked in as a **phantom ~0.2–0.4 m ring** that follows the
robot, which a naïve corridor check reads as "blocked in every direction" → the robot spins like it's
drunk. The fix that finally worked is a **purpose-built occupancy grid** built in the page:
- **Near-field exclusion:** never map points within ~0.45 m of the robot at capture time (that's
  where body/pitch-floor noise lives).
- **Persistence:** a cell only counts as an obstacle after ≥2 hits (filters one-off bursts).
- **Decay:** every cloud message, all cells lose 1; re-seen cells gain 2 (cap 8). A wall you keep
  seeing stays; the smear you leave behind **fades in ~3 s**. This single change removed the
  within-run "drunk" spinning.
- Height band chosen to dodge the pitch-floor (≈[-0.5, +0.8] in the cloud's up axis).

The grid is reset at the start of each `explore` run (the smear otherwise accumulated **across**
runs and re-created the ring).

### 2.2 Camera vision (catches what the LiDAR can't)
LiDAR misses **tables** (horizontal top above the band, thin legs), **glass/whiteboards**, and
narrow furniture gaps. The camera (grabbed from the app's WebView `<video>`) adds, in one worker
thread:
- **Floor segmentation (adaptive):** learns the carpet colour from the strip at the robot's feet
  each frame, classifies "floor" by similarity in saturation+brightness, and measures the free-floor
  fraction per band (left/centre/right). Class-agnostic, lighting-adaptive.
- **Edge / thin-obstacle:** a run of consecutive centre columns where the floor is interrupted close
  ⇒ a chair/table **leg** the floor-fraction average would miss.
- **YOLOv8s:** furniture classes (table, chair, sofa, fridge…) — fires on smaller/off-centre boxes
  because *any* furniture is an obstacle; box position/size gives a near/mid/far estimate.
- **MiDaS monocular depth:** the robust, class-agnostic layer — compares depth at mid-image-height
  vs the floor at the feet; if the "floor" at mid-height is as close as the near floor, there's a
  **vertical surface ahead** (wall/whiteboard/glass) regardless of colour. Runs on Apple-GPU (MPS).

### 2.3 Collision detector (the safety net / teacher)
When nothing geometric saw it, odometry does: **commanded forward for ≥1.5 s but moved <5 cm** ⇒
collision. The robot then backs up (rear-checked), turns to the freer side, nudges forward to leave
the spot, **and injects the obstacle into the grid** so it won't repeat — plus saves a labelled crash
snapshot.

---

## 3. Control — moving a biped you don't "own"

You can't command joints on the Air (low-level is EDU-only and not actuated). You command **gait
velocity**: `rt/wirelesscontroller {lx,ly,rx,ry}` at 20 Hz, injected on the app's datachannel. Key
facts learned the hard way:
- **Dead-zone:** the robot ignores |value| below ~0.3; the app's joystick never sends below ~0.5.
  Use ~0.4.
- **Format:** `{"type":"msg","topic":"rt/wirelesscontroller","data":{"lx":..,"ly":..,"rx":..,"ry":..}}`
  on the datachannel labelled `data`.
- **Axes:** `+ly` forward, `+rx` turns right (yaw decreases), `+lx` strafes. Robot is **slow**
  (~0.12 m/s) — which is good for reaction margin.
- The robot's **own balance policy** keeps it upright; we only set desired velocity.

---

## 4. Exploration

Reactive wander with two coverage mechanisms:
- **Novelty bias:** track visited 0.4 m cells; when blocked, among *open* directions pick the one
  whose ray (sampled to 3 m) is **least-visited**.
- **Periodic redirection:** even with a clear front, every ~6 s, if a clearly more-open and
  less-visited heading exists, commit a turn toward it.
- Visited coverage is **persisted in the page** (`window.__visited`) so it survives Python restarts
  within the same SLAM session.

---

## 5. Lessons (what actually moved the needle)

1. **Don't trust the raw cloud for control.** Build your own grid with near-field exclusion +
   persistence + **decay**. Decay was the difference between "drunk" and smooth.
2. **Sensor fusion beats any single sensor**, but only if each is *cheap and timely*. A great
   detector that arrives stale is worse than none (see §6).
3. **Collisions are data, not just failures.** Inject-on-bump + a labelled crash dataset turns every
   mistake into map memory and training signal.
4. **Reactive ≠ complete.** Wander + novelty covers more but still gets wedged; systematic coverage
   needs a map and a planner (§6).
5. **Calibrate from real logs.** Each detector got tuned from `vsee` + crash `.txt` (e.g. the carpet
   is near-grey S≈5; white walls are the same colour → only depth separates them).

---

## 6. Improvement roadmap

### Near-term (close the current gaps)
- **Make vision act in time.** Right now MiDaS+YOLO can run but the verdict arrives too late to
  avoid (it only confirms the bump). Fixes: run vision on MPS at low res; **react proportionally at
  medium distance** (slow + steer) instead of a binary block-when-close; widen the freshness window
  for a slow robot; and **lower forward speed when the vision pipeline is degraded** (self-monitoring,
  see meta-reasoning). The `VHEALTH` log line already measures per-frame time and staleness.
- **Unified costmap.** Fuse LiDAR-grid + camera-projected obstacles + depth into **one** world-frame
  costmap with confidence and decay; the controller/planner reads only that. Class-agnostic by
  design ("there is something at distance d in direction θ").
- **Metric depth.** MiDaS is relative; use the known floor plane (it must lie below the horizon and
  recede) to fit scale → metric obstacle distance, so thresholds are physical, not tuned per scene.

### Mid-term (from reactive to deliberative)
- **Frontier exploration + planning.** Maintain free/occupied/unknown; detect frontiers (free↔unknown
  boundary); plan a path (A*/D*-lite on the costmap) to the nearest frontier. Guarantees coverage and
  **avoids dead-ends/narrow gaps** the reactive wander walks into.
- **Recovery behaviours library** with explicit pre/post-conditions instead of fixed timers.

### Long-term — meta-reasoning (the project's namesake)
The robot reasoning about *its own cognition*, which is exactly where a consumer robot with
unreliable, partial perception needs to live:

1. **Sensor reliability model (context-conditioned).** The robot *knows* the LiDAR misses
   tables/glass and the camera confuses same-colour walls. Learn, per context (open room vs furniture
   cluster vs kitchen), **which sensor to trust** and weight the fused costmap accordingly.
2. **Confidence-aware action (metacognitive control).** Act on *perceptual confidence*: when sensors
   disagree or the scene matches a known-hard context, **slow down / widen margins**; when confidently
   clear, move fast. Uncertainty → caution is the metacognitive loop.
3. **Self-monitoring of the perception process.** Track latency/staleness/sensor-liveness (the
   `VHEALTH`/odom-frozen guards are seeds of this) and adapt behaviour when degraded — the robot
   noticing *"my eyes are lagging, I should slow down."*
4. **Learning from failure + analogy (the PhD link).** We literally saw the robot avoid one piece of
   furniture and then hit an **identical** one elsewhere. A meta-reasoner recognises the *analogy*
   ("this is like the chair I avoided") and **transfers** the avoidance. Concretely: embed crash
   snapshots, match new views by similarity, and pre-emptively avoid analogous obstacles — and/or
   fine-tune the detector on the actual furniture in this environment from the crash dataset.
5. **Explanation.** Because every decision is fused from named signals (LiDAR grid, depth_ratio, YOLO
   label, floor fraction), the robot can **explain** *why* it turned or *why* it failed — a substrate
   for introspective reasoning and for debugging perception.

The throughline: on hardware that gives you one noisy channel and three blind-spotted sensors, the
winning move isn't a better single sensor — it's a system that **knows what it doesn't know** and
acts accordingly.
