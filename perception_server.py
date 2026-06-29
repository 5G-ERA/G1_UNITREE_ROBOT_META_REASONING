#!/usr/bin/env python3
"""
perception_server.py — offboard GPU perception for the G1 (run on the Ubuntu box, 2x RTX, 120 GB).

Turns the G1 front-camera frame into a VIRTUAL LiDAR SCAN built from METRIC DEPTH,
so the table in the doorway (invisible to the head LiDAR) becomes a real obstacle
for A*. Also returns a free-space estimate and object detections.

POST /perceive   {image: 'data:image/...;base64,...', pose:[x,y,yaw], hband:[lo,hi], max_range}
   -> {scan: [[bearing_deg, range_m], ...],   # robot frame, +bearing = left
       free_center: 0..1, near_run: int,
       detections: [{label, conf, bearing_deg, range_m}], dt_ms, mode}
GET  /health -> {ok, mode, models, gpus}

Models (lazy-loaded; install only what you use):
  - depth:   Depth Anything V2 (metric) | Metric3D | torch-hub. GPU 0.
  - seg:     SegFormer/Mask2Former (ADE20K) for floor/free-space.   GPU 1.
  - det:     YOLO (ultralytics) for person/table/door.              GPU 1.

Run:
  GPU:   python perception_server.py --host 0.0.0.0 --port 8008 \
             --depth depth_anything_v2 --seg segformer --det yolo \
             --fx 600 --fy 600 --cx 320 --cy 240 --cam-h 1.10 --cam-pitch -10
  Stub:  python perception_server.py --stub        # no GPU, brightness-based free-space, empty scan
  Debug: add --debug to ANY of the above -> opens a live window (needs a desktop/X11) showing the
         camera with detection boxes + distance, the free_center value, and a mini-radar of the
         virtual scan (red points = obstacle < 1 m, e.g. the doorway table). Press q to quit.

Camera calibration: --fx/--fy/--cx/--cy (pixels), --cam-h (camera height, m),
--cam-pitch (deg, negative=looking down). These default to rough G1 values and
MUST be calibrated for accurate depth->ground projection (see CALIBRATION note).
"""
import argparse, base64, io, json, math, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

ARGS = None
_MODELS = {}          # lazy cache
_LAST_VIZ = None      # latest annotated frame for the --debug window


# ----------------------------------------------------------------------------- image
def decode_image(datauri):
    from PIL import Image
    if "," in datauri:
        datauri = datauri.split(",", 1)[1]
    img = Image.open(io.BytesIO(base64.b64decode(datauri))).convert("RGB")
    return np.asarray(img)                      # HxWx3 uint8


# ----------------------------------------------------------------------------- depth
def get_depth_model():
    if "depth" in _MODELS:
        return _MODELS["depth"]
    import torch
    name = ARGS.depth
    dev = ARGS.depth_device
    if name == "depth_anything_v2":
        # pip install depth_anything_v2 (or transformers pipeline). Metric variant preferred.
        from transformers import pipeline
        m = pipeline(task="depth-estimation",
                     model="depth-anything/Depth-Anything-V2-Metric-Indoor-Large-hf",
                     device=0 if dev.startswith("cuda") else -1)
        _MODELS["depth"] = ("hf", m)
    elif name == "metric3d":
        m = torch.hub.load("yvanyin/metric3d", "metric3d_vit_small", pretrain=True).to(dev).eval()
        _MODELS["depth"] = ("metric3d", m)
    else:
        raise ValueError(f"unknown depth model {name}")
    return _MODELS["depth"]


def run_depth(rgb):
    """Returns HxW metric depth (meters, forward distance) or None."""
    kind, m = get_depth_model()
    if kind == "hf":
        from PIL import Image
        out = m(Image.fromarray(rgb))
        d = np.asarray(out["depth"], dtype=np.float32)
        return d
    if kind == "metric3d":
        import torch
        with torch.no_grad():
            t = torch.from_numpy(rgb).permute(2, 0, 1)[None].float().to(ARGS.depth_device) / 255.0
            pred, _, _ = m.inference({"input": t})
            return pred.squeeze().detach().cpu().numpy().astype(np.float32)
    return None


# ----------------------------------------------------------------------------- seg (free space)
def get_seg_model():
    if "seg" in _MODELS:
        return _MODELS["seg"]
    from transformers import pipeline
    _MODELS["seg"] = pipeline("image-segmentation",
                              model="nvidia/segformer-b2-finetuned-ade-512-512",
                              device=0 if ARGS.seg_device.startswith("cuda") else -1)
    return _MODELS["seg"]


FLOOR_LABELS = {"floor", "rug", "road", "earth", "ground", "carpet", "flooring"}


def run_seg_floor_mask(rgb):
    """HxW bool mask of floor/free-ground pixels, or None."""
    try:
        seg = get_seg_model()
    except Exception:
        return None
    from PIL import Image
    res = seg(Image.fromarray(rgb))
    H, W = rgb.shape[:2]
    mask = np.zeros((H, W), bool)
    for r in res:
        lab = str(r.get("label", "")).lower()
        if any(f in lab for f in FLOOR_LABELS):
            m = np.asarray(r["mask"])
            if m.shape == (H, W):
                mask |= m > 0
    return mask


# ----------------------------------------------------------------------------- detection
def get_det_model():
    if "det" in _MODELS:
        return _MODELS["det"]
    from ultralytics import YOLO
    m = YOLO(ARGS.det_weights)
    _MODELS["det"] = m
    return m


DET_KEEP = {"person", "diningtable", "table", "chair", "couch", "refrigerator", "door"}


def run_det(rgb, fx, cx):
    try:
        m = get_det_model()
    except Exception:
        return []
    res = m.predict(rgb, device=ARGS.det_device, verbose=False)[0]
    out = []
    for b in res.boxes:
        lab = res.names[int(b.cls)].lower()
        if DET_KEEP and lab not in DET_KEEP:
            continue
        x1, y1, x2, y2 = b.xyxy[0].tolist()
        u = 0.5 * (x1 + x2)
        bearing = -math.degrees(math.atan2(u - cx, fx))   # +left
        out.append({"label": lab, "conf": float(b.conf), "bearing_deg": round(bearing, 1),
                    "range_m": None, "box": [x1, y1, x2, y2]})
    return out


# ----------------------------------------------------------------------------- depth -> virtual scan
def depth_to_scan(depth, floor_mask, hband, max_range):
    """Project depth to a forward ground 'virtual LiDAR': per azimuth column, the nearest
    NON-floor obstacle whose height falls inside hband. Returns list of [bearing_deg, range_m]."""
    H, W = depth.shape
    fx, fy, cx, cy = ARGS.fx, ARGS.fy, ARGS.cx, ARGS.cy
    ph = math.radians(ARGS.cam_pitch)
    ch = ARGS.cam_h
    lo, hi = hband
    cosp, sinp = math.cos(ph), math.sin(ph)

    us = np.arange(W)
    bearings = -np.degrees(np.arctan2(us - cx, fx))       # per-column bearing (+left)
    nbins = int(ARGS.scan_bins)
    bin_min = np.full(nbins, np.inf)
    bmin, bmax = bearings.min(), bearings.max()
    span = max(1e-6, bmax - bmin)

    vs = np.arange(H)
    step = max(1, H // 240)                                # subsample rows for speed
    for v in range(0, H, step):
        Z = depth[v, :]
        valid = np.isfinite(Z) & (Z > 0.2) & (Z < max_range)
        if not valid.any():
            continue
        # camera-frame point: X right, Y down, Z forward
        Y = (v - cy) * Z / fy
        # rotate by pitch about X (camera looking down by -pitch), height above ground:
        height = ch - (Y * cosp + Z * sinp)               # world height of the pixel
        is_obst = valid & (height > lo) & (height < hi)
        if floor_mask is not None:
            is_obst &= ~floor_mask[v, :]
        if not is_obst.any():
            continue
        idx = np.where(is_obst)[0]
        bb = ((bearings[idx] - bmin) / span * (nbins - 1)).astype(int)
        for k, z in zip(bb, Z[idx]):
            if z < bin_min[k]:
                bin_min[k] = z
    scan = []
    for k in range(nbins):
        if np.isfinite(bin_min[k]):
            bearing = bmin + (k + 0.5) / nbins * span
            scan.append([round(float(bearing), 1), round(float(bin_min[k]), 2)])
    return scan, bmin, bmax


def free_center_from_scan(scan, near_m=1.2, center_deg=18.0):
    """Fraction of central bearings that are clear (range > near_m), and a near_run count."""
    if not scan:
        return None, None
    central = [r for (b, r) in scan if abs(b) <= center_deg]
    if not central:
        return None, None
    clear = sum(1 for r in central if r > near_m)
    near_run = sum(1 for r in central if r <= near_m)
    return clear / len(central), near_run


# ----------------------------------------------------------------------------- stub (no GPU)
def stub_perceive(rgb):
    """Cheap brightness-based free-space; empty scan. Lets you test the pipeline without GPU."""
    H, W = rgb.shape[:2]
    band = rgb[int(H * 0.6):, int(W * 0.35):int(W * 0.65)].mean()
    free = float(np.clip(band / 180.0, 0, 1))
    return {"scan": [], "free_center": round(free, 2), "near_run": 0,
            "detections": [], "mode": "stub"}


# ----------------------------------------------------------------------------- pipeline
def _annotate(rgb, scan, dets, free_center):
    """Debug overlay: detection boxes + label/distance, free_center, and a mini-radar of the virtual scan."""
    import cv2
    img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR).copy()
    H, W = img.shape[:2]
    for d in dets:
        b = d.get("box")
        if not b:
            continue
        x1, y1, x2, y2 = [int(v) for v in b]
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        rng = d.get("range_m")
        lab = f"{d['label']} {d.get('conf', 0):.2f}" + (f" {rng:.2f}m" if rng else "")
        cv2.putText(img, lab, (x1, max(15, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
    if free_center is not None:
        cv2.putText(img, f"free_center={free_center:.2f}", (8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2, cv2.LINE_AA)
    # mini-radar (virtual scan): 0 deg = forward = up, +bearing = left
    R = min(140, H // 3); cx, cy = R + 10, H - R - 10; maxr = ARGS.max_range
    cv2.circle(img, (cx, cy), R, (70, 70, 70), 1); cv2.circle(img, (cx, cy), R // 2, (70, 70, 70), 1)
    cv2.line(img, (cx, cy), (cx, cy - R), (70, 70, 70), 1)
    for (b, rng) in scan:
        rr = min(rng, maxr) / maxr * R
        ang = math.radians(b)
        px = int(cx - rr * math.sin(ang)); py = int(cy - rr * math.cos(ang))
        col = (0, 0, 255) if rng < 1.0 else (0, 255, 255) if rng < 2.0 else (0, 255, 0)
        cv2.circle(img, (px, py), 2, col, -1)
    cv2.putText(img, f"scan {len(scan)}pts (red<1m)", (cx - R, cy - R - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1, cv2.LINE_AA)
    return img


def perceive(payload):
    rgb = decode_image(payload["image"])
    hband = payload.get("hband", [0.10, 1.30])
    max_range = float(payload.get("max_range", ARGS.max_range))
    if ARGS.stub:
        out = stub_perceive(rgb)
    else:
        depth = run_depth(rgb)
        if depth is None:
            out = stub_perceive(rgb)
        else:
            floor = run_seg_floor_mask(rgb) if ARGS.seg != "off" else None
            scan, _, _ = depth_to_scan(depth, floor, hband, max_range)
            free_center, near_run = free_center_from_scan(scan)
            dets = run_det(rgb, ARGS.fx, ARGS.cx) if ARGS.det != "off" else []
            for dct in dets:                      # nearest scan bin -> distance per detection
                if scan:
                    b0 = dct["bearing_deg"]
                    dct["range_m"] = min(scan, key=lambda s: abs(s[0] - b0))[1]
            out = {"scan": scan, "free_center": free_center, "near_run": near_run,
                   "detections": dets, "mode": "gpu"}
    if ARGS.debug:
        try:
            global _LAST_VIZ
            _LAST_VIZ = _annotate(rgb, out.get("scan", []), out.get("detections", []), out.get("free_center"))
        except Exception:
            pass
    return out


# ----------------------------------------------------------------------------- http
class Handler(BaseHTTPRequestHandler):
    def _send(self, code, obj):
        b = json.dumps(obj).encode()
        try:
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(b)))
            self.end_headers()
            self.wfile.write(b)
        except (BrokenPipeError, ConnectionResetError):
            # the client (g1_goto) gave up waiting and closed the socket — harmless, ignore
            pass

    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path == "/health":
            self._send(200, {"ok": True, "mode": "stub" if ARGS.stub else "gpu",
                             "models": {"depth": ARGS.depth, "seg": ARGS.seg, "det": ARGS.det},
                             "gpus": _gpu_info()})
        elif self.path.startswith("/debug.mjpg"):   # live MJPEG stream (no refresh needed)
            self._stream_mjpg()
        elif self.path.startswith("/debug.jpg"):     # single latest annotated frame
            self._send_debug_jpg()
        elif self.path in ("/", "/view", "/debug"):  # auto-refreshing viewer page
            self._send_view()
        else:
            self._send(404, {"error": "not found"})

    def _send_view(self):
        html = (b"<!doctype html><meta charset=utf-8><title>G1 perception</title>"
                b"<body style='margin:0;background:#111;text-align:center'>"
                b"<img src='/debug.mjpg' style='max-width:100%;height:auto'>"
                b"<noscript>open /debug.jpg</noscript></body>")
        try:
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(html)))
            self.end_headers()
            self.wfile.write(html)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _frame_jpg(self):
        import cv2, numpy as np
        img = _LAST_VIZ
        if img is None:
            img = np.full((360, 640, 3), 40, np.uint8)
            cv2.putText(img, "no frame yet - run g1_goto with G1_PERC set", (20, 180),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)
        ok, buf = cv2.imencode(".jpg", img)
        return buf.tobytes() if ok else None

    def _stream_mjpg(self):
        try:
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            while True:
                b = self._frame_jpg()
                if b:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(b)}\r\n\r\n".encode())
                    self.wfile.write(b); self.wfile.write(b"\r\n")
                time.sleep(0.07)                      # ~14 fps
        except (BrokenPipeError, ConnectionResetError):
            pass                                      # browser tab closed — fine
        except Exception:
            pass

    def _send_debug_jpg(self):
        try:
            b = self._frame_jpg()
            if not b:
                raise RuntimeError("encode failed")
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(b)))
            self.end_headers()
            self.wfile.write(b)
        except (BrokenPipeError, ConnectionResetError):
            pass
        except Exception as e:
            self._send(503, {"error": f"debug image unavailable: {e}. Start the server with --debug."})

    def do_POST(self):
        if self.path != "/perceive":
            self._send(404, {"error": "not found"}); return
        try:
            n = int(self.headers.get("Content-Length", 0))
            payload = json.loads(self.rfile.read(n).decode())
            t0 = time.time()
            out = perceive(payload)
            out["dt_ms"] = round((time.time() - t0) * 1000.0, 1)
            self._send(200, out)
        except (BrokenPipeError, ConnectionResetError):
            pass                                  # client closed early; ignore quietly
        except Exception as e:
            import traceback; traceback.print_exc()
            self._send(500, {"error": str(e)})


def _gpu_info():
    try:
        import torch
        return [torch.cuda.get_device_name(i) for i in range(torch.cuda.device_count())]
    except Exception:
        return []


def main():
    global ARGS
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8008)
    ap.add_argument("--stub", action="store_true", help="no GPU; brightness free-space, empty scan")
    ap.add_argument("--depth", default="depth_anything_v2", choices=["depth_anything_v2", "metric3d"])
    ap.add_argument("--seg", default="segformer", help="'segformer' or 'off'")
    ap.add_argument("--det", default="yolo", help="'yolo' or 'off'")
    ap.add_argument("--det-weights", default="yolo11x.pt")
    ap.add_argument("--depth-device", default="cuda:0")
    ap.add_argument("--seg-device", default="cuda:1")
    ap.add_argument("--det-device", default="cuda:1")
    # camera intrinsics / extrinsics (CALIBRATE these for your G1 front camera!)
    ap.add_argument("--fx", type=float, default=600.0)
    ap.add_argument("--fy", type=float, default=600.0)
    ap.add_argument("--cx", type=float, default=320.0)
    ap.add_argument("--cy", type=float, default=240.0)
    ap.add_argument("--cam-h", type=float, default=1.10, help="camera height above ground (m)")
    ap.add_argument("--cam-pitch", type=float, default=-10.0, help="camera pitch deg (neg=down)")
    ap.add_argument("--scan-bins", type=int, default=72)
    ap.add_argument("--max-range", type=float, default=3.0)
    ap.add_argument("--debug", action="store_true",
                    help="open a live window: detection boxes + distance, free_center, and a scan radar")
    ARGS = ap.parse_args()
    print(f"[perception] {'STUB' if ARGS.stub else 'GPU'} on {ARGS.host}:{ARGS.port} "
          f"depth={ARGS.depth} seg={ARGS.seg} det={ARGS.det} gpus={_gpu_info()}"
          f"{' [DEBUG WINDOW]' if ARGS.debug else ''}")
    if not ARGS.stub:                              # warm up the models so the FIRST real request is fast
        try:
            import base64, io
            from PIL import Image
            buf = io.BytesIO(); Image.new("RGB", (640, 480), (120, 120, 120)).save(buf, format="JPEG")
            uri = "data:image/jpeg;base64," + base64.b64encode(buf.getvalue()).decode()
            print("[perception] warming up models (first inference loads weights)...", flush=True)
            t0 = time.time(); perceive({"image": uri, "pose": [0, 0, 0]})
            print(f"[perception] warm-up done in {time.time()-t0:.1f}s — ready.", flush=True)
        except Exception as e:
            print(f"[perception] warm-up skipped/failed: {e}", flush=True)
    if ARGS.debug:
        import threading
        srv = ThreadingHTTPServer((ARGS.host, ARGS.port), Handler)
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        url = f"http://{ARGS.host}:{ARGS.port}/"
        print(f"[perception] debug ON. LIVE VIDEO in a browser (no refresh): {url}")
        print(f"[perception] single frame: {url}debug.jpg   ·   raw stream: {url}debug.mjpg")
        print("[perception] a local OpenCV window will also open IF this machine has a display (X11).")
        try:
            import cv2, numpy as np
            blank = np.zeros((360, 640, 3), np.uint8)
            cv2.putText(blank, "waiting for frames from g1_goto...", (20, 180),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
            have_window = False
            while True:
                try:
                    cv2.imshow("G1 perception debug", _LAST_VIZ if _LAST_VIZ is not None else blank)
                    have_window = True
                    if (cv2.waitKey(30) & 0xFF) == ord("q"):
                        break
                except Exception as e:
                    if not have_window:           # no display (SSH/headless): keep serving the browser endpoint
                        print(f"[perception] no local window available ({e}).")
                        print(f"[perception] OPEN THIS IN A BROWSER instead: {url}   (Ctrl+C to stop)")
                        while True:
                            time.sleep(1.0)
                    break
        except KeyboardInterrupt:
            pass
        finally:
            srv.shutdown()
            try:
                import cv2; cv2.destroyAllWindows()
            except Exception:
                pass
    else:
        ThreadingHTTPServer((ARGS.host, ARGS.port), Handler).serve_forever()


if __name__ == "__main__":
    main()
