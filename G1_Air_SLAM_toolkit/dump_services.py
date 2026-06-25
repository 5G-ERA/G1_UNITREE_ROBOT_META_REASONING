#!/usr/bin/env python3
"""
dump_services.py  -  Vuelca la lista COMPLETA de servicios del G1 Air (rt/servicestate)

READ-ONLY. Conecta, pide el estado de servicios y lo imprime entero y ordenado.
Objetivo: ver si existe un servicio de SLAM/navegacion/lidar/mapping y su status
(0 = parado, 1 = activo). Eso nos dice que hay que ARRANCAR para que la app/SLAM
funcione, y el nombre exacto del servicio.

Uso (app cerrada, sesion unica):
    cd ~/unitree_webrtc_connect && source .venv/bin/activate
    python dump_services.py
"""
import asyncio, json, logging, sys

from unitree_webrtc_connect.webrtc_driver import (
    UnitreeWebRTCConnection, WebRTCConnectionMethod)
from unitree_webrtc_connect.constants import RTC_TOPIC

logging.basicConfig(level=logging.FATAL)


async def main():
    conn = UnitreeWebRTCConnection(
        WebRTCConnectionMethod.LocalAP, ip="192.168.12.1")
    await conn.connect()
    print("Conectado. Pidiendo lista de servicios...\n")
    ps = conn.datachannel.pub_sub

    got = asyncio.Event()
    services = {}

    def cb(msg):
        raw = msg.get("data", msg)
        if not got.is_set():
            print("RAW servicestate recibido:\n", str(raw)[:2000], "\n")
        try:
            data = json.loads(raw) if isinstance(raw, str) else raw
        except Exception:
            data = raw
        if isinstance(data, list):
            for s in data:
                services[s.get("name", "?")] = s
            got.set()
    ps.subscribe(RTC_TOPIC["SERVICE_STATE"], cb)

    # rt/servicestate es intermitente: reconsultamos cada 3s hasta 45s.
    for attempt in range(15):
        if got.is_set():
            break
        try:
            await ps.publish_request_new(RTC_TOPIC["MOTION_SWITCHER"], {"api_id": 1001})
        except Exception as e:
            print("query motion_switcher:", e)
        try:
            await asyncio.wait_for(got.wait(), timeout=3)
        except asyncio.TimeoutError:
            print(f"  ...esperando servicestate (intento {attempt+1}/15)")

    if not got.is_set():
        print("No llego rt/servicestate en 45s. Vuelve a intentarlo (es intermitente).")
        return

    print(f"=== SERVICIOS ({len(services)}) ===")
    for name in sorted(services):
        s = services[name]
        flag = "ACTIVO" if s.get("status") == 1 else "parado"
        print(f"  [{flag:6s}] {name:32s} status={s.get('status')} "
              f"protect={s.get('protect')} v{s.get('version','')}")

    # Resalta los relacionados con SLAM/nav/lidar/map
    print("\n=== Relacionados con SLAM / navegacion / lidar / mapa ===")
    keys = ("slam", "uslam", "lidar", "map", "nav", "loc", "qt")
    hits = [n for n in sorted(services) if any(k in n.lower() for k in keys)]
    if hits:
        for n in hits:
            print(f"  -> {n}: {json.dumps(services[n], default=str)}")
    else:
        print("  (ninguno con esos nombres)")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        sys.exit(0)
