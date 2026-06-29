# Running everything on the Ubuntu box (2× RTX) — setup + calibration

Goal: run **both** `python g1_goto.py gotoviz B` **and** `perception_server.py` on
the same Ubuntu machine, so perception is local (low latency) and there is no Mac
in the loop.

The hard part is **not** the GPU perception — it is moving the **robot link**
(iPhone Unitree app → WKWebView → CDP over USB) from macOS to Linux. That is the
risky step; everything else is routine. If iOS-on-Linux debugging fights you, use
the **fallback** at the end (robot on the Mac, perception on Ubuntu over LAN — the
code already supports it via `G1_PERC`).

---

## 0. What has to be true

`g1_goto.py` talks to the robot exactly like on the Mac:
`requests.get("http://localhost:9221/json")` → finds the iPhone → opens a CDP
WebSocket to the app's WebView. So on Ubuntu you must reproduce the
`ios_webkit_debug_proxy` on `localhost:9221`. The proxy port is hard-coded in
`g1_nav_v2.py` (`PROXY = "http://localhost:9221"`), so keep 9221.

Prerequisite that is independent of host OS: the app's WKWebView must be
**inspectable**. It already is on your Mac today, and inspectability is a property
of the app/iOS (not the host), so Linux will see the same pages — *provided* the
proxy supports your iOS version (see step 2).

---

## 1. GPU stack (drivers + CUDA + PyTorch)

```bash
# NVIDIA driver (Ubuntu 22.04/24.04)
sudo ubuntu-drivers autoinstall      # or install a specific nvidia-driver-5xx
sudo reboot
nvidia-smi                            # must list BOTH RTX cards

# Python env
sudo apt install -y python3-venv python3-pip
python3 -m venv ~/g1env && source ~/g1env/bin/activate

# PyTorch matching your CUDA (check https://pytorch.org for the right index-url)
pip install torch torchvision --index-url https://download.pytorch.org/cu124
python -c "import torch; print('CUDA', torch.cuda.is_available(), 'GPUs', torch.cuda.device_count())"
# expect: CUDA True  GPUs 2

# Perception models + robot-link + viz deps
pip install transformers ultralytics opencv-python pillow numpy matplotlib \
            websocket-client requests
```

Two-GPU split is already wired: depth on `cuda:0`, segmentation+detection on
`cuda:1` (flags `--depth-device/--seg-device/--det-device`).

---

## 2. iPhone ↔ Linux: the robot link over USB

```bash
sudo apt install -y usbmuxd libimobiledevice6 libimobiledevice-utils \
                    ios-webkit-debug-proxy
sudo systemctl enable --now usbmuxd

# plug the iPhone in by USB, unlock it, tap "Trust this computer"
idevice_id -l            # prints the device UDID -> USB + pairing OK
idevicepair pair         # if needed: "SUCCESS: Paired with device"
```

On the **iPhone**: Settings → Safari → Advanced → **Web Inspector = ON**.

Start the proxy (leave it running in its own terminal):

```bash
ios_webkit_debug_proxy -c null:9221,:9222-9322 -d
# then verify it sees the device and the app's WebView pages:
curl -s http://localhost:9221/json | python3 -m json.tool | head
```

You should see a device entry and, drilling into it, the app's WebView page(s).
That is exactly what `g1_goto.py` reads.

> ⚠️ **iOS version support is the #1 failure point on Linux.** The apt package can
> be old. If `curl .../json` shows the device but no WebView pages (or none for a
> recent iOS), build the latest proxy from source:
> ```bash
> sudo apt install -y autoconf automake libtool pkg-config libssl-dev libplist-dev
> git clone https://github.com/google/ios-webkit-debug-proxy
> cd ios-webkit-debug-proxy && ./autogen.sh && make && sudo make install
> ```
> If it still won't enumerate pages on your iOS, use the **fallback** (section 8).

---

## 3. (Recommended) bump the camera frame to 640 px

`g1_nav_v2.CAM_JS` sends frames at width **320** by default. Depth is better at
640, and your PC can afford it. In `g1_nav_v2.py`, in `CAM_JS`, change:

```js
var W=320, ...   ->   var W=640,
```

Whatever width you choose, your **intrinsics must be calibrated at that width**
(next step). Don't mix.

---

## 4. Camera calibration (intrinsics + extrinsics)

`perception_server.py` projects depth pixels to ground obstacles. That needs:

- **Intrinsics** `fx, fy, cx, cy` — at the delivered frame resolution.
- **Extrinsics** `--cam-h` (camera height above floor, m) and `--cam-pitch`
  (degrees, negative = looking down).

### 4a. Intrinsics — checkerboard (best)

Print a chessboard (e.g. 10×7 squares = **9×6 inner corners**, 25 mm squares).
With the robot link up:

```bash
python calibrate_cam.py grab 20          # move the board around the view
python calibrate_cam.py intrinsics 9 6 25
# -> prints:  --fx ... --fy ... --cx ... --cy ...   and saves calib/intrinsics.json
```

Aim for reproj error < 0.5 px. The printed resolution MUST match CAM_JS.

Quick alternative (no board): estimate from horizontal FOV φ:
`fx ≈ (W/2) / tan(φ/2)`, `fy ≈ fx`, `cx ≈ W/2`, `cy ≈ H/2`. Use only for a first try.

### 4b. Extrinsics — measure

- `--cam-h`: with the robot **standing**, measure the front camera height to the
  floor with a tape (≈ 1.1–1.3 m on the G1; measure yours).
- `--cam-pitch`: the head/camera downward tilt. Measure the angle, or start at
  `-10` and refine in 4c.

### 4c. Validate depth scale (the important sanity check)

Start the server (section 5), put a box at a **measured** distance straight ahead
(e.g. 1.50 m), then:

```bash
python calibrate_cam.py rangecheck 127.0.0.1:8008
# RANGO CENTRAL: X m   <- compare to your tape measure
```

- Range matches tape → done.
- Off by a roughly constant factor → the metric-depth model needs a scale tweak
  (apply a multiplier in `run_depth`).
- Noisy / inconsistent with height → fix `--cam-pitch` / `--cam-h` first.

---

## 5. Start the perception server

```bash
# first, prove the pipeline with NO GPU:
python perception_server.py --stub --port 8008 &
curl -s http://localhost:8008/health        # {"ok": true, "mode": "stub", ...}

# then the real thing (models download on first run):
python perception_server.py --host 127.0.0.1 --port 8008 \
    --depth depth_anything_v2 --seg segformer --det yolo \
    --fx <FX> --fy <FY> --cx <CX> --cy <CY> --cam-h <H> --cam-pitch <PITCH>
curl -s http://localhost:8008/health        # mode: "gpu", gpus: [two cards]
```

---

## 6. Run navigation with local perception

```bash
# viz needs a desktop session (X11). Run from the Ubuntu GUI, not headless SSH.
export G1_PERC=127.0.0.1:8008
python g1_goto.py gotoviz B
```

On start it prints `[perc] http://127.0.0.1:8008 -> OK ...`. During the run, the
doorway table should now appear as obstacles (A* routes around it) and the door
maneuver uses depth/seg free-space. Check the dataset summary: `perc_queries > 0`.

Headless (no GUI, no viz window): use `python g1_goto.py goto B` instead of
`gotoviz`.

---

## 7. Safety (unchanged)

Kill switch **L2+B** in hand. 2–3 m clear space. Battery **> 80%** (low battery
degrades gait + LiDAR — it hurt the last run). Head level so the LiDAR/camera see
forward. Keep the taped cup so it cannot fall onto the electronics.

---

## 8. Fallback if iOS-on-Linux won't cooperate

You do **not** have to move the robot link. Keep it on the Mac and run only the
heavy perception on the Ubuntu box across the LAN — this is what `G1_PERC` was
built for:

```
Mac:     ios_webkit_debug_proxy + python g1_goto.py gotoviz B   (G1_PERC=<ubuntu-ip>:8008)
Ubuntu:  python perception_server.py --host 0.0.0.0 --port 8008 ...
```

Same models, same result, zero iOS-on-Linux risk. The only cost is ~camera-frame
latency over the LAN (small; the server already reports `dt_ms`).

---

## Quick checklist

1. `nvidia-smi` shows 2 GPUs; `torch.cuda.device_count()==2`.
2. `curl localhost:9221/json` lists the app WebView page.
3. CAM_JS width = calibration width (320 or 640).
4. `calibrate_cam.py intrinsics ...` → reproj < 0.5 px.
5. `--cam-h`/`--cam-pitch` measured; `rangecheck` matches a tape measure.
6. `/health` = gpu; `g1_goto` prints `[perc] ... OK`; `perc_queries > 0`.
