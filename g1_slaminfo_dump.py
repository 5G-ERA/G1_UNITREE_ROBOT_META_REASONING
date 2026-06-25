#!/usr/bin/env python3
"""
g1_slaminfo_dump.py  -  Vuelca el contenido COMPLETO de rt/slam_info y rt/slam_key_info

slam_info fluye ~10Hz durante el SLAM. Recoge ~12s, agrupa por el campo 'type' interno y
muestra UNA muestra completa de cada tipo distinto (por si hay varios: robot_data, map_data...).

USO (app cerrada):
  cd ~/unitree_webrtc_connect && source .venv/bin/activate
  python "<ruta>/g1_slaminfo_dump.py"
"""
import asyncio, json, sys, logging

from unitree_webrtc_connect.webrtc_driver import (
    UnitreeWebRTCConnection, WebRTCConnectionMethod)

logging.basicConfig(level=logging.FATAL)
SLAM_OP = "rt/api/slam_operate/request"
INFO = "rt/slam_info"
KEY = "rt/slam_key_info"


def parse(d):
    if isinstance(d, str):
        try: return json.loads(d)
        except Exception: return d
    return d


async def main():
    conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalAP, ip="192.168.12.1")
    await conn.connect()
    print("Conectado.\n")
    ps = conn.datachannel.pub_sub

    samples = {}     # type -> muestra completa
    keysmp = {"v": None}
    def cb_info(m):
        d = parse(m.get("data"))
        t = d.get("type") if isinstance(d, dict) else "?"
        if t not in samples:
            samples[t] = d
    def cb_key(m):
        if keysmp["v"] is None:
            keysmp["v"] = parse(m.get("data"))
    ps.subscribe(INFO, cb_info)
    ps.subscribe(KEY, cb_key)

    # arrancar mapeo (slam_info suele fluir con SLAM activo)
    try:
        await asyncio.wait_for(ps.publish_request_new(
            SLAM_OP, {"api_id": 1801, "parameter": {"data": {"slam_type": "indoor"}}}), timeout=10)
    except Exception as e:
        print("start mapping err (sigo):", e)

    print("Recogiendo slam_info ~12s...\n")
    await asyncio.sleep(12)

    print("=== TIPOS de slam_info encontrados:", list(samples.keys()), "===\n")
    for t, d in samples.items():
        print(f"----- slam_info type={t!r} (completo) -----")
        print(json.dumps(d, indent=1, default=str)[:3000])
        print()
    print("----- slam_key_info (completo) -----")
    print(json.dumps(keysmp["v"], indent=1, default=str)[:2000] if keysmp["v"] else "  (no llegó)")

    try:
        await asyncio.wait_for(ps.publish_request_new(SLAM_OP, {"api_id": 1803}), timeout=5)
    except Exception:
        pass
    print("\nmapeo cancelado.")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        sys.exit(0)
