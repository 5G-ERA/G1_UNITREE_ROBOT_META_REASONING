#!/usr/bin/env python3
"""
g1_slam_sniff.py  -  Espía TODO el canal (texto + binario) con el mapeo SLAM activo

Engancha el parser binario (deal_array_buffer) y el dispatch (run_resolve) para registrar
CUALQUIER mensaje que llegue durante el mapeo: topic, si es binario, y tamaño. Asi sabemos
si la NUBE laser llega a nuestra conexion (y bajo que topic) o si solo nos dan odom/info.

USO (app cerrada). Camina el robot con el MANDO durante los ~40s:
  cd ~/unitree_webrtc_connect && source .venv/bin/activate
  python "<ruta>/g1_slam_sniff.py"
"""
import asyncio, sys, logging, time
from collections import Counter

from unitree_webrtc_connect.webrtc_driver import (
    UnitreeWebRTCConnection, WebRTCConnectionMethod)

logging.basicConfig(level=logging.FATAL)
SLAM_OP = "rt/api/slam_operate/request"


async def main():
    conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalAP, ip="192.168.12.1")
    await conn.connect()
    print("Conectado.\n")
    dc = conn.datachannel
    ps = dc.pub_sub

    topics = Counter()        # topics (texto) por run_resolve
    binmsgs = Counter()       # topics que llegaron como BINARIO
    first = {}

    # 1) hook del parser BINARIO (nube/lidar suelen venir aqui)
    orig_deal = dc.deal_array_buffer
    def patched_deal(buffer):
        res = orig_deal(buffer)
        try:
            t = res.get("topic", "?") if isinstance(res, dict) else "?"
            binmsgs[t] += 1
            if t not in first:
                first[t] = f"BINARIO ~{len(buffer)} bytes"
                print(f"[{time.strftime('%H:%M:%S')}] *** BINARIO topic={t} (~{len(buffer)} bytes)")
        except Exception as e:
            print("deal hook err", e)
        return res
    dc.deal_array_buffer = patched_deal

    # 2) hook del dispatch (todo mensaje parseado)
    orig_rr = ps.run_resolve
    def patched_rr(parsed):
        try:
            t = parsed.get("topic", "")
            if t:
                topics[t] += 1
                if t not in first:
                    first[t] = "texto"
                    print(f"[{time.strftime('%H:%M:%S')}] topic={t}")
        except Exception:
            pass
        return orig_rr(parsed)
    ps.run_resolve = patched_rr

    # destrabar ancho de banda + suscribir a candidatos de nube + arrancar mapeo
    try: await dc.disableTrafficSaving(True)
    except Exception as e: print("traffic err", e)
    for t in ["rt/unitree/slam_mapping/points", "rt/unitree/slam_mapping/odom",
              "rt/unitree/slam_relocation/points", "rt/slam_info", "rt/slam_key_info",
              "rt/utlidar/voxel_map", "rt/utlidar/voxel_map_compressed",
              "rt/mapping/grid_map", "rt/uslam/frontend/cloud_world_ds"]:
        try: ps.subscribe(t, lambda m: None)
        except Exception: pass
    try:
        r = await asyncio.wait_for(ps.publish_request_new(
            SLAM_OP, {"api_id": 1801, "parameter": {"data": {"slam_type": "indoor"}}}), timeout=10)
        print("start mapping code:", r["data"]["header"]["status"]["code"], "\n")
    except Exception as e:
        print("start err", e)

    print("Espiando 40s... CAMINA el robot con el mando (trasládalo)\n")
    await asyncio.sleep(40)

    print("\n=== TOPICS (texto) ===")
    for t, n in topics.most_common(): print(f"  {n:6d}  {t}")
    print("=== TOPICS (binario) ===")
    if binmsgs:
        for t, n in binmsgs.most_common(): print(f"  {n:6d}  {t}  (NUBE/LiDAR?)")
    else:
        print("  ningun mensaje binario llegó")
    try: await asyncio.wait_for(ps.publish_request_new(SLAM_OP, {"api_id": 1803}), timeout=5)
    except Exception: pass
    print("\nmapeo cancelado.")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        sys.exit(0)
