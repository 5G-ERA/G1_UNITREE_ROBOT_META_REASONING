#!/usr/bin/env python3
"""
discover_slam_api.py  -  Busca un api-topic de SLAM/navegacion oculto y afina formato

Dos pruebas:
  A) FORMATO motion_switcher 1002: probar parameter como DICT vs STRING, por si el
     7002 era solo el envoltorio del parametro.
  B) DESCUBRIMIENTO de api-topics: enviar un request a topics candidatos
     (rt/api/uslam/request, rt/api/slam/request, ...). Si el topic EXISTE, el robot
     contesta en .../response (un code, aunque sea error). Si NO existe -> timeout.
     Una respuesta = ese servicio es alcanzable por WebRTC -> hilo para el SLAM.

READ-MOSTLY: solo consultas/GETs. No envia locomocion.

Uso (app cerrada):
    cd ~/unitree_webrtc_connect && source .venv/bin/activate
    python discover_slam_api.py
"""
import asyncio, json, logging, sys

from unitree_webrtc_connect.webrtc_driver import (
    UnitreeWebRTCConnection, WebRTCConnectionMethod)
from unitree_webrtc_connect.constants import RTC_TOPIC

logging.basicConfig(level=logging.FATAL)


async def req(ps, topic, options, label, timeout=6):
    try:
        resp = await asyncio.wait_for(
            ps.publish_request_new(topic, options), timeout=timeout)
        code = None
        try:
            code = resp["data"]["header"]["status"]["code"]
        except Exception:
            pass
        print(f"  [RESP code={code}] {label}")
        print("      ", json.dumps(resp, default=str)[:400])
        return resp
    except asyncio.TimeoutError:
        print(f"  [timeout]       {label}")
    except Exception as e:
        print(f"  [error {e!r}]   {label}")
    return None


async def main():
    conn = UnitreeWebRTCConnection(
        WebRTCConnectionMethod.LocalAP, ip="192.168.12.1")
    await conn.connect()
    print("Conectado.\n")
    ps = conn.datachannel.pub_sub

    # ---- A) formato de motion_switcher 1002 (SELECT) ----
    print("== A) motion_switcher 1002 SELECT: dict vs string ==")
    MS = RTC_TOPIC["MOTION_SWITCHER"]
    for name in ("ai", "normal", "advanced"):
        await req(ps, MS, {"api_id": 1002, "parameter": {"name": name}},
                  f"1002 parameter=DICT name={name}")
    # tambien probar seleccionar por 'form'
    for form in ("1", "2"):
        await req(ps, MS, {"api_id": 1002, "parameter": {"name": "ai", "form": form}},
                  f"1002 DICT name=ai form={form}")

    # ---- B) descubrir api-topics de SLAM / navegacion ----
    print("\n== B) descubrimiento de api-topics (RESP=existe, timeout=no existe) ==")
    candidate_topics = [
        "rt/api/uslam/request",
        "rt/api/slam/request",
        "rt/api/navigation/request",
        "rt/api/nav/request",
        "rt/api/mapping/request",
        "rt/api/gridmap/request",
        "rt/api/lidar/request",
        "rt/api/lidar_state/request",
        "rt/api/localization/request",
        "rt/api/map/request",
        "rt/api/qt/request",
        "rt/api/robot_state/request",
        "rt/api/config/request",
    ]
    # api_id 1001 suele ser un GET inocuo; si no existe el topic -> timeout
    for t in candidate_topics:
        await req(ps, t, {"api_id": 1001}, t, timeout=4)

    # ---- C) robot_state: a veces lista servicios/estados utiles ----
    print("\n== C) robot_state GETs (si el topic existe) ==")
    for aid in (1001, 1002, 1003):
        await req(ps, "rt/api/robot_state/request", {"api_id": aid},
                  f"robot_state api_id {aid}", timeout=4)

    print("\n--- FIN ---")
    print("Cualquier topic de (B) con [RESP ...] EXISTE y es alcanzable -> candidato real para SLAM.")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        sys.exit(0)
