#!/usr/bin/env python3
"""
g1_slam_capture_points.py  -  Captura UNA muestra de la nube del SLAM para ver su formato

Arranca el mapeo (slam_operate 1801), se suscribe a rt/unitree/slam_mapping/points y /odom,
y al primer mensaje IMPRIME su estructura (claves, tipos, tamaños de arrays + muestra) y la
guarda en ~/g1/points_sample.json . Luego cancela el mapeo (1803). READ-mostly (arranca/cancela
mapeo, no mueve el robot).

USO (app cerrada, robot de pie con algo de entorno alrededor; muévelo un poco a mano/teleop
para que el LiDAR genere nube):
  cd ~/unitree_webrtc_connect && source .venv/bin/activate
  python "<ruta>/g1_slam_capture_points.py"
"""
import asyncio, json, os, sys, logging, time

from unitree_webrtc_connect.webrtc_driver import (
    UnitreeWebRTCConnection, WebRTCConnectionMethod)

logging.basicConfig(level=logging.FATAL)
SLAM_OP = "rt/api/slam_operate/request"
POINTS = "rt/unitree/slam_mapping/points"
ODOM = "rt/unitree/slam_mapping/odom"
OUT = os.path.expanduser("~/g1/points_sample.json")


def describe(obj, depth=0, maxd=4):
    """Describe estructura: claves, tipos, longitudes de listas."""
    ind = "  " * depth
    if isinstance(obj, dict):
        out = []
        for k, v in obj.items():
            out.append(f"{ind}{k}: {describe(v, depth+1, maxd).lstrip()}")
        return "{\n" + "\n".join(out) + f"\n{ind}}}"
    if isinstance(obj, list):
        n = len(obj)
        head = describe(obj[0], depth+1, maxd) if n and depth < maxd else "..."
        return f"list[{n}] de -> {head}"
    if isinstance(obj, (bytes, bytearray)):
        return f"bytes(len={len(obj)}) {bytes(obj[:16]).hex()}..."
    if isinstance(obj, str):
        return f"str(len={len(obj)}) {obj[:40]!r}"
    return f"{type(obj).__name__}={obj}"


async def main():
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalAP, ip="192.168.12.1")
    await conn.connect()
    print("Conectado.\n")
    ps = conn.datachannel.pub_sub

    # la nube suele estar suprimida por ahorro de trafico -> desactivarlo
    try:
        ok = await conn.datachannel.disableTrafficSaving(True)
        print("disableTrafficSaving ->", ok)
    except Exception as e:
        print("disableTrafficSaving err:", e)

    # candidatos de nube (pescamos en todos)
    CLOUD_CANDS = [
        "rt/unitree/slam_mapping/points",
        "rt/unitree/slam_relocation/points",
        "rt/utlidar/voxel_map",
        "rt/utlidar/voxel_map_compressed",
        "rt/mapping/grid_map",
        "rt/uslam/frontend/cloud_world_ds",
    ]
    got = {"points": None, "odom": None, "which": None}
    counts = {}
    def mk(name):
        def cb(m):
            counts[name] = counts.get(name, 0) + 1
            if got["points"] is None:
                got["points"] = m; got["which"] = name
                print(f"\n*** NUBE recibida por: {name}")
        return cb
    for t in CLOUD_CANDS:
        try: ps.subscribe(t, mk(t))
        except Exception as e: print("sub err", t, e)
    def cb_o(m):
        if got["odom"] is None:
            got["odom"] = m
    ps.subscribe(ODOM, cb_o)

    # arrancar mapeo
    try:
        r = await asyncio.wait_for(ps.publish_request_new(
            SLAM_OP, {"api_id": 1801, "parameter": {"data": {"slam_type": "indoor"}}}), timeout=10)
        print("start mapping code:", r["data"]["header"]["status"]["code"])
    except Exception as e:
        print("start mapping err:", e)

    print("Esperando nube ~40s... CAMINA el robot con el MANDO FÍSICO (trasládalo, no solo girar)\n")
    for _ in range(80):
        if got["points"] is not None:
            break
        await asyncio.sleep(0.5)
    print("Conteo por topic:", counts or "ninguno emitió")
    if got["which"]:
        print("Topic ganador:", got["which"])

    if got["odom"] is not None:
        print("=== ODOM estructura ===")
        print(describe(got["odom"]))
        print()
    if got["points"] is not None:
        print("=== POINTS estructura ===")
        print(describe(got["points"]))
        with open(OUT, "w") as f:
            json.dump(got["points"], f, default=str)
        print(f"\nGuardado crudo en {OUT}")
        # tamaño aproximado del campo de datos si existe
        try:
            d = got["points"]["data"]
            print("tipo de points.data:", type(d).__name__,
                  "len:", len(d) if hasattr(d, "__len__") else "n/a")
        except Exception:
            pass
    else:
        print("No llegó nube. (¿mapeo activo? ¿LiDAR con entorno? prueba moviéndolo)")

    # cancelar mapeo (no guardamos)
    try:
        await asyncio.wait_for(ps.publish_request_new(SLAM_OP, {"api_id": 1803}), timeout=5)
        print("\nmapeo cancelado.")
    except Exception:
        pass


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        sys.exit(0)
