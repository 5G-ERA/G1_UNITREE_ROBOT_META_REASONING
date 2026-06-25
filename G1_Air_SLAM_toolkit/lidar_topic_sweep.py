#!/usr/bin/env python3
"""
lidar_topic_sweep.py  -  Barrido POR TANDAS de nombres de topic de LiDAR del G1 Air

Genera ~150+ candidatos (namespaces x hojas + conocidos) y los prueba en TANDAS
(por defecto 20 a la vez) para no saturar el data channel: suscribe tanda -> escucha
-> apunta lo que emita -> desuscribe -> siguiente tanda.

Secuencia correcta de LiDAR: disableTrafficSaving(True) -> switch "on".
READ-ONLY. App cerrada (sesion unica).

Uso:
    cd ~/unitree_webrtc_connect && source .venv/bin/activate
    python lidar_topic_sweep.py
    python lidar_topic_sweep.py --batch 15 --wait 12
"""
import asyncio, logging, sys, time, argparse, itertools
from collections import Counter

from unitree_webrtc_connect.webrtc_driver import (
    UnitreeWebRTCConnection, WebRTCConnectionMethod)

logging.basicConfig(level=logging.FATAL)

NAMESPACES = [
    "rt/utlidar", "rt/uslam", "rt/uslam/frontend", "rt/uslam/localization",
    "rt/lidar", "rt/livox", "rt/livox/lidar", "rt/mid360", "rt/mid_360",
    "rt/lio_sam_ros2/mapping", "rt/mapping", "rt/perception", "rt/sensor",
    "rt/g1/utlidar", "rt/g1/lidar", "rt/cloud", "rt/pointcloud_node",
]
LEAVES = [
    "voxel_map", "voxel_map_compressed", "cloud", "cloud_world", "cloud_world_ds",
    "cloud_registered", "cloud_deskewed", "cloud_base", "point_cloud", "pointcloud",
    "pointcloud2", "points", "scan", "height_map", "range_map", "lidar_state",
    "robot_pose", "odom", "imu", "grid_map", "map",
]
KNOWN = [
    "rt/utlidar/voxel_map", "rt/utlidar/voxel_map_compressed",
    "rt/utlidar/height_map", "rt/utlidar/range_map", "rt/utlidar/range_info",
    "rt/utlidar/lidar_state", "rt/utlidar/robot_pose",
    "rt/uslam/frontend/cloud_world_ds", "rt/uslam/cloud_map",
    "rt/lio_sam_ros2/mapping/cloud_registered", "rt/mapping/grid_map",
    "utlidar/cloud", "rt/utlidar/cloud_deskewed",
]


def build_candidates():
    cands = set(KNOWN)
    for ns, leaf in itertools.product(NAMESPACES, LEAVES):
        cands.add(f"{ns}/{leaf}")
    return sorted(cands)


def chunks(lst, n):
    for i in range(0, len(lst), n):
        yield lst[i:i + n]


async def main(batch, wait):
    conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalAP, ip="192.168.12.1")
    await conn.connect()
    print("Conectado.\n")
    dc = conn.datachannel
    ps = dc.pub_sub

    seen = Counter()
    orig = ps.run_resolve
    def patched(parsed):
        try:
            t = parsed.get("topic", "")
            if t:
                if seen[t] == 0:
                    print(f"[{time.strftime('%H:%M:%S')}] *** NUEVO TOPIC: {t}")
                seen[t] += 1
        except Exception:
            pass
        return orig(parsed)
    ps.run_resolve = patched

    await dc.disableTrafficSaving(True)
    dc.set_decoder(decoder_type='libvoxel')
    for topic, payload in [("rt/utlidar/switch", "on"),
                           ("rt/lidar/switch", "on"),
                           ("rt/livox/switch", "on")]:
        try:
            ps.publish_without_callback(topic, payload)
        except Exception:
            pass

    cands = build_candidates()
    total = len(cands)
    nbatches = (total + batch - 1) // batch
    print(f"{total} candidatos en {nbatches} tandas de {batch} ({wait}s cada una)\n")

    hits = {}
    def unsub(topic):
        for name in ("unsubscribe", "unSubscribe", "remove_subscription"):
            fn = getattr(ps, name, None)
            if fn:
                try:
                    fn(topic); return
                except Exception:
                    pass

    for bi, group in enumerate(chunks(cands, batch), 1):
        before = sum(seen.values())
        for t in group:
            try:
                ps.subscribe(t, lambda m: None)
            except Exception:
                pass
        print(f"--- Tanda {bi}/{nbatches}: suscritos {len(group)}. Escuchando {wait}s...")
        await asyncio.sleep(wait)
        after = sum(seen.values())
        new = {t: seen[t] for t in group if seen[t] > 0}
        if new:
            print(f"    *** REACCION en tanda {bi}: {new}")
            hits.update(new)
        else:
            print(f"    (sin reaccion; total msgs +{after - before})")
        for t in group:           # desuscribir antes de la siguiente tanda
            unsub(t)
        await asyncio.sleep(0.5)

    print("\n=== RESULTADO FINAL ===")
    if not seen:
        print(f"  NINGUNO de {total} candidatos emitio. -> APK es el camino fiable.")
    else:
        for t, n in seen.most_common():
            print(f"  {n:6d}  {t}")
    if hits:
        print("\nGANADORES (topics que emitieron):")
        for t in hits:
            print("  ->", t)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--batch", type=int, default=20)
    ap.add_argument("--wait", type=float, default=10)
    args = ap.parse_args()
    try:
        asyncio.run(main(args.batch, args.wait))
    except KeyboardInterrupt:
        sys.exit(0)
