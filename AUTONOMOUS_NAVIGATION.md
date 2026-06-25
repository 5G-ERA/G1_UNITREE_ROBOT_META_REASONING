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

## 5b. Perception & control hardening (the debugging that actually mattered)

Frontier mode worked on paper but the robot was *ultra-conservative* — it spun in place in a ~1.5 m
patch and never reached a frontier. Reading the exported map + JSON against the logs uncovered a chain
of root causes, each fixed in turn:

- **Phantom near-field ring (the big one).** In an empty room the LiDAR clean-grid reported a *stable*
  obstacle at 0.26–0.40 m that the persistent map didn't believe — the body/pitch-floor return, baked
  in at high confidence because the build-time near-field exclusion (~0.45 m, measured from the odom
  origin which is **offset** from the sensor) didn't cover it. This made the robot think it was boxed
  in everywhere. Fix: **trust the LiDAR only beyond `NEAR_BLIND`=0.6 m**; below that the **camera**
  (calibrated metric depth) and the collision detector own perception. Applied in both the corridor
  query (`clear_ahead`) and the obstacle map accumulation, so the phantom no longer poisons ESC or A*.
- **Metric depth from the floor plane (inverse-perspective).** The camera now estimates *meters* to an
  obstacle's base via `d = K/(contact_frac − horizon)` (the row where the floor segmentation is
  interrupted), **calibrated against the LiDAR** by `floorcal auto` (linear regression of contact-row
  vs 1/d — no tape measure). Per-frame noise is high, so `contact_frac` is **median-smoothed** over 8
  frames. This drives the `close` decision in real meters and stopped the robot fearing furniture
  detected far away (YOLO box size is no longer trusted for distance).
- **Forgetting (dynamic obstacles).** The obstacle map is now a **decaying** map (cells expire after
  `OMAP_TTL`=20 s unless re-seen), so a person who walks past — or a false collision — is **removed**
  once it's gone, while walls you keep seeing stay refreshed.
- **Camera as a *transient* costmap layer.** Camera detections feed a short-lived (`COBS_TTL`=4 s)
  layer A* plans around (so it *routes around* a chair instead of fighting its own path), but never
  the permanent map — earlier over-injection walled the robot in.
- **Target commitment (no flip-flop).** Re-picking the nearest frontier every few seconds made the
  robot oscillate between two equidistant frontiers ("Buridan's ass"). Now it **commits** to a
  frontier until it's reached / A* fails / 12 s without progress.
- **Arc steering.** Bang-bang "turn-in-place then go" flickered at the alignment threshold with the
  gait's yaw wobble (looked drunk). Now it **arcs** — forward *and* turn simultaneously — so it keeps
  moving while correcting heading; plus a **forward bias** in frontier selection to stop side-to-side
  ping-pong. Result: coverage area went from ~1.5×1.5 m to ~3.9×3.4 m and the path *stretches* instead
  of orbiting (trail/area ratio 7.5 → 1.7).
- **Observability.** Every run exports `map_latest.png` + `.json` (explored / obstacles / odometry
  trail) for offline inspection, and `frontier … viz` shows a live **map + robot-camera** window.
- **"The LiDAR has the final say" (sensor authority).** On a **reflective blue floor** (very different
  from the gray carpet the vision was tuned for) the camera *again* cried "obstacle close" in every
  direction on clearly-open floor (floor-fraction flickering + MiDaS confused by the uniform reflective
  surface). The fix: **the camera cannot veto a path the LiDAR confirms clear** (`c0 ≥ CAM_TRUST_C0`).
  The camera still catches what the LiDAR misses, but only via a **strong, reliable** signal (a big
  YOLO box = object right there, or a very-high MiDaS ratio = real wall) — the *weak* floor-segmentation
  signal is LiDAR-gated. A self-monitoring **"camera degraded"** detector (blocks while spinning ⇒
  unreliable ⇒ ignore 6 s) is the backstop, and it **dumps an annotated frame** to `vision_debug/`
  (original | floor-mask overlay + numbers) so failures are diagnosable — that dump is what pinpointed
  the reflective-floor cause.
- **No blind reversing.** The LiDAR can't confirm the *rear* near-field (same 0.6 m blind zone, no rear
  camera), so backing up risks tipping over backwards. All recovery/escape now **pivots in place**
  instead of reversing — it rotates without translating, which can't fall backwards.
- **Generic obstacles.** YOLO now flags **any** object class in the path (not just a furniture
  whitelist) as an obstacle.

The throughline: most of the "fear" was **false positives in the near field**, not real obstacles —
the fix was giving each sensor authority only in the range/context where it's reliable. This is the
project's meta-reasoning thesis in miniature: *a robot that knows which of its senses to trust, when.*

---

## 6. Improvement roadmap

> Ordered by impact-per-effort, June 2026, after the perception/control hardening above. The robot now
> *moves* (covers ~3.4×5.5 m, stretches not orbits); the next gains are in **where** it chooses to go,
> **not hitting** the things the LiDAR can't see, and **fusing** the senses with confidence.

### A. Explore *more and better* — smarter frontier choice (biggest coverage win)
Today it picks the **nearest** reachable frontier (+ a forward bias). That nibbles the edge of the
explored blob and produces the "cross" shape we saw. Better:
- **Information-gain frontiers.** Cluster adjacent frontier cells and estimate the **unknown area
  behind** each cluster (ray-cast a few metres into unknown space). Choose `gain / travel_cost`, not
  nearest. A doorway is a *big* frontier opening onto a whole room; a nook is a tiny one — this sends
  the robot **through doors into new rooms** instead of grooming a corner.
- **Commit to a far goal.** Pick an exploration goal several metres out and drive it decisively
  (already half-done via target commitment); combine with gain so the commitment is to *high-value*
  goals.
- **Boustrophedon finish.** When frontiers run low, switch to a lawnmower sweep of the largest known-
  free region for *complete* coverage, then return to start.

### B. Don't hit what the LiDAR can't see — faster, learned collision avoidance
The 0.6 m LiDAR blind zone + table-tops mean the camera and contact are the only defence up close.
- **IMU contact detection.** The G1 streams IMU; a sudden deceleration/jerk is a contact in ~0.1–0.2 s
  vs the current ~1.5 s odom-stall. Stop *as* it touches, not after pushing into furniture.
- **Learning from collisions + analogy (the PhD core).** We already save a labelled crash dataset.
  Embed each crash frame (a small CNN / color-texture descriptor); on every frame, match the current
  view against past crashes — if it's *similar* to something we hit before (the same chair elsewhere),
  pre-emptively avoid it **even when the LiDAR is clear**. This is direct **analogical transfer**: "this
  looks like the thing I bumped last time."
- **Proactive caution zone.** Slow down when entering *unknown* cells or when sensors disagree — less
  speed = less collision energy and more reaction time.

### C. Fuse the senses into one confidence costmap (robustness)
Replace the ad-hoc override stack (LiDAR-gate, cobs, ESC, VAV) with **one** world-frame costmap where
each cell has an **occupancy probability + source + decay**. LiDAR writes >0.6 m, camera-projected
obstacles write with lower confidence, depth and collisions write high confidence; the planner reads
only the fused map. Then:
- **Confidence-aware speed** (metacognition): fast where confidently clear, slow where uncertain.
- **Metric MiDaS.** Anchor MiDaS's relative depth to the calibrated floor plane → *metric* distance to
  **any** surface (not just floor-contact), class-agnostic, robust to reflective floors.
- This subsumes the floor-segmentation fragility that keeps biting us on new floor types.

### D. Use the robot's own SLAM map
The app already builds a real SLAM occupancy map (with loop closure) that's far better than our 0.4 m
voxel grid. If we can read/export it (`slam_g1_mapping`), use it as the **global** costmap and keep our
grid only for fast local reaction — fixes odometry drift and gives a true room map.

### Mid-term (from reactive to deliberative)
- **Frontier exploration + planning.** ✅ *Implemented* as `frontier` mode. Free space = visited 0.4 m
  cells; occupied = the LiDAR clean grid (projected cloud→odom); unknown = the rest. A **frontier** is
  an unknown cell adjacent to explored space; the controller picks the **nearest reachable** one
  (line-of-sight against the obstacle grid, with a relaxed fallback) and heads to it — turn-to-bearing
  then forward — while collision recovery, camera VAV and LiDAR ESC stay as priority overrides, and it
  **replans** on reach / every 6 s / after any avoidance. This gives systematic coverage and gets the
  robot **out of cluttered corners** instead of orbiting them. *Next:* swap the straight-line heading
  for an A*/D*-lite path on a fused costmap so it routes *around* obstacles rather than relying on the
  reactive layer to peel off them.
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
