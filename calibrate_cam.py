#!/usr/bin/env python3
"""
calibrate_cam.py — get the camera parameters perception_server.py needs.

The depth->ground projection in perception_server.py needs the camera intrinsics
(fx, fy, cx, cy) AT THE RESOLUTION THE FRAME ARRIVES, plus the camera height and
pitch. This tool gets all of them from the live robot camera.

IMPORTANT: g1_nav_v2.CAM_JS downscales frames to width=320 by default. Intrinsics
MUST match that. Either calibrate at 320, or bump CAM_JS to width=640 (recommended
on a strong PC) and calibrate at 640 — but be consistent.

Subcommands
  grab N                      grab N camera frames to calib/ (move a printed
                              checkerboard around the view; ~20 varied shots)
  intrinsics COLS ROWS MM     run OpenCV checkerboard calibration on calib/*.jpg
                              (COLS x ROWS = INNER corners, MM = square size)
                              -> prints the exact --fx --fy --cx --cy flags
  rangecheck HOST:PORT        grab one frame, POST to the perception server, print
                              the central virtual-scan range. Put an object at a
                              measured distance and compare -> validates depth scale.

Needs: opencv-python (intrinsics), numpy, pillow; and a live robot link
(ios_webkit_debug_proxy running, robot relocalised in the app).
"""
import sys, os, time, glob, json, base64, io

CALIB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calib")


def _cdp_and_grab():
    import g1_goto as gg
    cdp = gg.get_live_cdp()
    if not cdp:
        print("No hay pagina viva del WebView. ¿proxy + app + relocalizado?"); sys.exit(1)
    return gg, cdp


def cmd_grab(n):
    gg, cdp = _cdp_and_grab()
    os.makedirs(CALIB_DIR, exist_ok=True)
    print(f"Grabando {n} frames a {CALIB_DIR}/ . Mueve el patron de ajedrez por el campo de vision.")
    got = 0
    while got < n:
        cam = gg.grab_cam(cdp)
        if cam and cam.startswith("data:image"):
            raw = base64.b64decode(cam.split(",", 1)[1])
            p = os.path.join(CALIB_DIR, f"calib_{got:03d}.jpg")
            open(p, "wb").write(raw)
            from PIL import Image
            w, h = Image.open(io.BytesIO(raw)).size
            got += 1
            print(f"  [{got}/{n}] {p}  ({w}x{h})")
        time.sleep(0.8)
    print("Listo. Ahora: python calibrate_cam.py intrinsics 9 6 25   (ajusta a tu patron)")


def cmd_intrinsics(cols, rows, mm):
    import numpy as np, cv2
    objp = np.zeros((cols * rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * (mm / 1000.0)
    objpoints, imgpoints = [], []
    files = sorted(glob.glob(os.path.join(CALIB_DIR, "*.jpg")))
    if not files:
        print("No hay frames en calib/. Corre primero: python calibrate_cam.py grab 20"); return
    shape = None
    found = 0
    for f in files:
        img = cv2.imread(f); gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY); shape = gray.shape[::-1]
        ok, corners = cv2.findChessboardCorners(gray, (cols, rows), None)
        if ok:
            found += 1
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                       (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
            objpoints.append(objp); imgpoints.append(corners)
    print(f"Tableros detectados: {found}/{len(files)} (a {shape[0]}x{shape[1]})")
    if found < 5:
        print("Pocos tableros. Graba mas frames variados (angulos/distancias)."); return
    err, K, dist, _, _ = cv2.calibrateCamera(objpoints, imgpoints, shape, None, None)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    print(f"\n  reproj error: {err:.3f} px  (idealmente < 0.5)")
    print(f"  RESOLUCION calibrada: {shape[0]}x{shape[1]}  (debe coincidir con CAM_JS)")
    print("\n  >>> Flags para perception_server.py:")
    print(f"      --fx {fx:.1f} --fy {fy:.1f} --cx {cx:.1f} --cy {cy:.1f}")
    json.dump({"fx": fx, "fy": fy, "cx": cx, "cy": cy, "w": shape[0], "h": shape[1],
               "reproj_err": err}, open(os.path.join(CALIB_DIR, "intrinsics.json"), "w"), indent=2)
    print(f"\n  guardado -> {CALIB_DIR}/intrinsics.json")


def cmd_rangecheck(endpoint):
    import urllib.request
    if not endpoint.startswith("http"):
        endpoint = "http://" + endpoint
    gg, cdp = _cdp_and_grab()
    cam = gg.grab_cam(cdp)
    if not cam:
        print("Sin frame de camara."); return
    body = json.dumps({"image": cam, "pose": [0, 0, 0], "hband": [0.1, 1.3], "max_range": 4.0}).encode()
    req = urllib.request.Request(endpoint + "/perceive", data=body,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=5) as r:
        j = json.loads(r.read().decode())
    scan = j.get("scan", [])
    central = [s for s in scan if abs(s[0]) <= 6.0]   # +-6 deg around center
    rng = min((s[1] for s in central), default=None)
    print(f"  mode={j.get('mode')} dt={j.get('dt_ms')}ms scan_pts={len(scan)} free_center={j.get('free_center')}")
    print(f"  RANGO CENTRAL: {rng} m  <- pon un objeto a una distancia MEDIDA y compara.")
    print("  si es consistente pero escalado, corrige la escala del modelo de depth; si es ruidoso, revisa pitch/altura.")


def main():
    a = sys.argv[1:]
    if not a:
        print(__doc__); return
    if a[0] == "grab":
        cmd_grab(int(a[1]) if len(a) > 1 else 20)
    elif a[0] == "intrinsics":
        cols = int(a[1]) if len(a) > 1 else 9
        rows = int(a[2]) if len(a) > 2 else 6
        mm = float(a[3]) if len(a) > 3 else 25.0
        cmd_intrinsics(cols, rows, mm)
    elif a[0] == "rangecheck":
        cmd_rangecheck(a[1] if len(a) > 1 else "127.0.0.1:8008")
    else:
        print(__doc__)


if __name__ == "__main__":
    main()
