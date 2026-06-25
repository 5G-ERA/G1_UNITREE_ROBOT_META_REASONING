# G1 Unitree — Autonomous Navigation & Meta-Reasoning (consumer "Air", no EDU/DDS)

Autonomous exploration and obstacle avoidance on a **stock Unitree G1 "Air"** humanoid — a
consumer unit that exposes **only a single WebRTC session through the official app** (no ROS 2, no
DDS, no SSH, no low-level SDK). Everything here is built **on top of that one channel**, by tapping
the app's own WebView over USB.

> The robot maps and navigates while we read its LiDAR/odometry and drive it — all without EDU
> hardware — and learns from its own collisions. The longer-term goal is **meta-reasoning**: a robot
> that reasons about the reliability of its own perception and adapts.

---

## TL;DR — what this does

- **Reads the robot's SLAM live** (point cloud + odometry) from Python, over USB, by hooking the
  app's WebView with `ios-webkit-debug-proxy` (no second WebRTC session needed).
- **Drives the robot** (walk / turn / strafe) by **injecting `rt/wirelesscontroller` velocity
  commands into the app's existing WebRTC datachannel** — the only way to move it programmatically
  without a 2nd session.
- **Autonomous explore mode**: wanders a room, builds coverage, avoids obstacles fusing **LiDAR +
  camera (floor segmentation, edge, YOLOv8s, MiDaS depth) + odometry collision detection**, and
  **remembers obstacles it bumps into**.
- **Live visualizer** of the LiDAR cloud (2D/3D) + robot pose + path + camera with YOLO boxes.

This was reverse-engineered against the owner's **own** robot/app for interoperability. The APK,
keys and proprietary assets are **not** redistributed here.

---

## Why it's hard (the constraints)

The G1 Air is **not** the EDU/developer unit:

| Want | Air reality |
|------|-------------|
| ROS 2 / DDS / CycloneDDS | ❌ internal bus not exposed on the WiFi AP (EDU-only) |
| SSH / low-level SDK (`rt/arm_sdk`, `lowcmd`) | ❌ forwarded but **not actuated** |
| Raw LiDAR cloud over WebRTC | ❌ only `odom` + `slam_info` reach a 3rd-party client; the dense cloud is decoded **only inside the app's WebView** |
| Two WebRTC sessions (one app, one ours) | ❌ robot allows **one** peer — the app holds it |

So the trick is: **don't fight the app — live inside it.** We attach to the WebView's JS over USB
(immune to the robot AP's client isolation), read the decoded cloud + odom, and publish velocity on
the app's own datachannel.

---

## Architecture

```
 iPhone (Unitree app, WebView)  ──WebRTC──  G1 robot
        │   ▲
   USB  │   │  ios-webkit-debug-proxy (CDP over USB)
        ▼   │
   Mac (Python)
     ├─ Perception
     │    ├─ LiDAR clean grid  (cloud → world-frame voxel grid: near-field exclusion + persistence + decay)
     │    ├─ Camera vision     (floor-color seg · edge · YOLOv8s · MiDaS depth — class-agnostic "obstacle ahead")
     │    └─ Collision sensor  (odometry stall = bumped something no sensor saw)
     ├─ Control     (inject rt/wirelesscontroller {lx,ly,rx,ry} @20Hz, in-page driver + dead-man)
     └─ Exploration (reactive wander + coverage novelty + redirection to unexplored)
```

Coordinate note: the cloud is in **Three.js Y-up** frame (the app's renderer), the odometry is in
**ROS Z-up** frame — they are reconciled in code (`cloud_x≈odom_x`, height=idx1, `cloud_z=-odom_y`).

---

## The scripts

**Navigation / autonomy**
- `g1_nav.py` — **the main program.** Unified capture + control + exploration. Modes:
  - `watch` — live odom + cloud point count (read-only)
  - `clr` — read-only obstacle clearances around the robot (LiDAR grid)
  - `vsee` — read-only camera vision readout (floor fractions, YOLO label, MiDaS depth_ratio)
  - `forward N` / `turn DEG` / `gorel F L` / `goto X Y` — closed-loop primitives (odom feedback)
  - `nav X Y` / `navrel F L` — go to a point **with reactive obstacle avoidance**
  - `explore [secs]` — **reactive** autonomous mapping/exploration (wander + coverage novelty)
  - `frontier [secs]` — **deliberative** exploration: goes to the nearest reachable **frontier**
    (edge of the explored map) for systematic coverage, with the full avoidance stack as override
- `g1_inject_teleop.py` — option-C teleop injection (sniff/capture/drive) — proof that we can move the robot via the app's datachannel
- `g1_teleop.py` — direct walking via `rt/wirelesscontroller` (app closed)

**Perception / viz**
- `g1_inspector_bridge.py` — live LiDAR cloud (2D/3D) + pose + path + **camera window with YOLO boxes**
- `g1_cam_probe.py` — probe the app's `<video>` element (camera)
- `slam_g1_mapping.py` — start/stop/save SLAM from Python (app closed)
- `g1_slam_viz.py`, `g1_map_viz.py` — odometry / saved-map visualizers

**Reverse-engineering / diagnostics**
- `dump_services.py`, `discover_slam_api.py`, `query_api.py`, `g1_slaminfo_dump.py`, … — how the API was mapped

**Docs**
- `AUTONOMOUS_NAVIGATION.md` — perception/control/exploration **algorithm** + a roadmap of
  improvements (incl. the meta-reasoning direction)
- `G1_Air_SLAM_SOLVED.md` / `.pdf` — the SLAM/WebRTC reverse-engineering writeup

---

## Quick start

Prereqs (on the Mac, in a venv):
```bash
brew install ios-webkit-debug-proxy
pip install websocket-client requests numpy matplotlib pillow ultralytics timm
# (MiDaS depth pulls its weights on first run; runs on Apple-GPU/MPS automatically)
```

Bring-up:
1. iPhone: Unitree app connected to the robot, **standing**, on the **SLAM/map screen**, **camera on**.
   iPhone Web Inspector ON; **don't** open Safari's inspector on that page (one debugger per page).
2. USB-connect the iPhone to the Mac (trust).
3. Terminal 1: `ios_webkit_debug_proxy`
4. Terminal 2:
   ```bash
   python g1_nav.py watch        # confirm odometry is live (x/y/yaw change when you move the robot)
   python g1_nav.py explore 90   # autonomous exploration
   ```

**Safety:** keep the physical remote in hand as a kill switch (L2+B = damping/stop), clear 2–3 m of
space, and start with the robot freshly charged (>80%) and standing in walk mode.

---

## Learning from failure (toward meta-reasoning)

Each real collision is treated as a perception signal:
- **Memory:** the bumped obstacle (which the LiDAR couldn't see) is *injected into the obstacle grid*
  so the robot doesn't hit it again.
- **Dataset:** every collision saves a camera snapshot + a `.txt` of *what each sensor reported at
  that instant* (`crashes/`) — i.e. *why* it failed (LiDAR blind, camera saw floor, …). This is the
  raw material for improving the visual model and for the robot to reason about its own blind spots.

The repo's name — *meta reasoning* — points at where this goes: a robot that knows the LiDAR misses
tables/glass and the camera confuses same-colour walls, **weights its sensors by context**, slows
down when its perception is uncertain or degraded, and **transfers** an avoidance learned on one
chair to an identical chair elsewhere (analogy). See `AUTONOMOUS_NAVIGATION.md`.

---

## Status

Working: SLAM read, teleop injection, closed-loop motion, reactive A→B, autonomous exploration with
coverage bias, collision memory, live cloud+camera viz. Known limits and the improvement roadmap are
in `AUTONOMOUS_NAVIGATION.md` (current focus: making the camera/depth layer act *in time*, and moving
from reactive wander to frontier exploration with planning).

## Disclaimer

Research/interoperability work on the author's own hardware. Not affiliated with Unitree. Moving a
bipedal robot autonomously is inherently risky — supervise it, keep a kill switch, use a clear space.
