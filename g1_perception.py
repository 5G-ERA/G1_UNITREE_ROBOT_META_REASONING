"""
g1_perception.py — thin client to the offboard GPU perception server.

The heavy models (metric depth, segmentation, detection) run on the powerful
Ubuntu box in `perception_server.py`. This client just POSTs the current camera
frame + pose and gets back:
  - a VIRTUAL SCAN (bearing, range) built from metric depth, so obstacles the
    head LiDAR misses (the table in the doorway) become visible to A*;
  - a free-space estimate (center fraction + near_run) to replace the classical
    `floor_free_bands` heuristic in the door maneuver;
  - detections (person/table/door) for the human-proximity analogy / costmap.

Design rules:
  - ZERO hard dependency: uses only stdlib (urllib). If the server is down or
    slow, `query()` returns None and the caller keeps its current behaviour.
  - The server is stateless w.r.t. the map: it returns robot-frame results; this
    client transforms the virtual scan into MAP cells using the live pose.
"""
from __future__ import annotations
import json, math, time, urllib.request


class PerceptionResult:
    __slots__ = ("cells", "free_center", "near_run", "detections", "latency_ms", "raw")

    def __init__(self, cells, free_center, near_run, detections, latency_ms, raw):
        self.cells = cells                 # set of (cx,cy) MAP cells (obstacles from depth)
        self.free_center = free_center     # 0..1 fraction of clear floor straight ahead (None if n/a)
        self.near_run = near_run           # int: count of near obstacle columns ahead (None if n/a)
        self.detections = detections       # list of {label, conf, bearing_deg, range_m}
        self.latency_ms = latency_ms
        self.raw = raw


class PerceptionClient:
    """Talks to perception_server.py. endpoint e.g. 'http://192.168.1.50:8008'."""

    def __init__(self, endpoint, ocell=0.2, hband=(0.10, 1.30), max_range=3.0, timeout=0.25):
        self.endpoint = endpoint.rstrip("/")
        self.ocell = ocell
        self.hband = hband                 # obstacle height band (m) the server should keep
        self.max_range = max_range
        self.timeout = timeout
        self.ok = False
        self.last_err = None
        self.n_ok = 0
        self.n_fail = 0

    def health(self):
        try:
            with urllib.request.urlopen(self.endpoint + "/health", timeout=1.0) as r:
                j = json.loads(r.read().decode())
            self.ok = bool(j.get("ok"))
            return j
        except Exception as e:
            self.ok = False; self.last_err = str(e); return None

    def query(self, cam_datauri, x, y, yaw_deg):
        """Returns PerceptionResult or None. cam_datauri = 'data:image/...;base64,...'."""
        if not cam_datauri or not isinstance(cam_datauri, str):
            return None
        body = json.dumps({
            "image": cam_datauri,
            "pose": [x, y, yaw_deg],
            "hband": list(self.hband),
            "max_range": self.max_range,
        }).encode()
        t0 = time.time()
        try:
            req = urllib.request.Request(self.endpoint + "/perceive", data=body,
                                         headers={"Content-Type": "application/json"})
            with urllib.request.urlopen(req, timeout=self.timeout) as r:
                j = json.loads(r.read().decode())
            self.ok = True; self.n_ok += 1
        except Exception as e:
            self.ok = False; self.last_err = str(e); self.n_fail += 1
            return None
        lat = (time.time() - t0) * 1000.0
        cells = self._scan_to_cells(j.get("scan", []), x, y, yaw_deg)
        return PerceptionResult(cells, j.get("free_center"), j.get("near_run"),
                                j.get("detections", []), lat, j)

    def _scan_to_cells(self, scan, x, y, yaw_deg):
        """Virtual scan [(bearing_deg, range_m), ...] (robot frame) -> set of MAP cells."""
        out = set()
        yr = math.radians(yaw_deg)
        for item in scan:
            try:
                b, rng = item[0], item[1]
            except Exception:
                continue
            if rng is None or rng <= 0.05 or rng > self.max_range:
                continue
            a = yr + math.radians(b)
            mx = x + rng * math.cos(a)
            my = y + rng * math.sin(a)
            out.add((round(mx / self.ocell), round(my / self.ocell)))
        return out

    def nearest_person(self):
        return None  # convenience hook; detections are on the result object


def make_client_from_env(ocell=0.2):
    """If G1_PERC is set (host:port or full URL), return a probed client, else None."""
    import os
    ep = os.environ.get("G1_PERC")
    if not ep:
        return None
    if not ep.startswith("http"):
        ep = "http://" + ep
    c = PerceptionClient(ep, ocell=ocell)
    h = c.health()
    print(f"  [perc] {ep} -> {'OK ' + str(h) if h else 'NO RESPONDE (uso heuristica clasica)'}")
    return c if h else None
