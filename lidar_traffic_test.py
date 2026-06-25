#!/usr/bin/env python3
"""
lidar_traffic_test.py  -  Prueba LiDAR con traffic-saving DESACTIVADO + sniff de topics

Hallazgo: la libreria tiene conn.datachannel.disableTrafficSaving(True), con el
comentario "Should turn it on when subscribed to ulidar topic". El robot SUPRIME
los topics de alto ancho de banda (LiDAR) por defecto. Por eso veiamos 0 mensajes.

Este script:
  1. Conecta.
  2. Instala un CATCH-ALL: registra el 'topic' de TODO mensaje entrante (sin
     filtrar por nombre) -> asi vemos los nombres REALES que usa este firmware.
  3. Se suscribe a los topics de LiDAR/SLAM conocidos.
  4. Llama a disableTrafficSaving(True)  <-- la clave.
  5. Escucha y reporta que topics emiten y cuantos mensajes.

READ-ONLY: no envia locomocion.

Uso (app cerrada):
    cd ~/unitree_webrtc_connect && source .venv/bin/activate
    python lidar_traffic_test.py --secs 30
"""
import asyncio, json, logging, sys, argparse, time
from collections import Counter

from unitree_webrtc_connect.webrtc_driver import (
    UnitreeWebRTCConnection, WebRTCConnectionMethod)
from unitree_webrtc_connect.constants import RTC_TOPIC

logging.basicConfig(level=logging.FATAL)

SUBS = [
    "ULIDAR", "ULIDAR_ARRAY", "ULIDAR_STATE", "ROBOTODOM",
    "GRID_MAP", "SLAM_ODOMETRY",
    "LIDAR_MAPPING_ODOM", "LIDAR_MAPPING_CLOUD_POINT", "LIDAR_MAPPING_PCD_FILE",
    "LIDAR_LOCALIZATION_ODOM", "LIDAR_MAPPING_SERVER_LOG", "SLAM_QT_NOTICE",
]


async def main(secs):
    conn = UnitreeWebRTCConnection(
        WebRTCConnectionMethod.LocalAP, ip="192.168.12.1")
    await conn.connect()
    print("Conectado.\n")
    dc = conn.datachannel
    ps = dc.pub_sub

    # --- CATCH-ALL: ver TODOS los topics reales que entran ---
    all_topics = Counter()
    first_seen = {}
    orig_run_resolve = ps.run_resolve
    def patched_run_resolve(parsed):
        try:
            t = parsed.get("topic", "")
            if t:
                all_topics[t] += 1
                if t not in first_seen:
                    first_seen[t] = json.dumps(parsed, default=str)[:200]
                    print(f"[{time.strftime('%H:%M:%S')}] NUEVO TOPIC: {t}")
        except Exception:
            pass
        return orig_run_resolve(parsed)
    ps.run_resolve = patched_run_resolve

    # Suscribir a los conocidos (por si el catch-all no basta)
    for k in SUBS:
        try:
            ps.subscribe(RTC_TOPIC[k], lambda m: None)
        except Exception as e:
            print("sub skip", k, e)
    print("Suscrito a topics LiDAR/SLAM conocidos.\n")

    # --- LA CLAVE: desactivar traffic saving ---
    try:
        ok = await dc.disableTrafficSaving(True)
        print(f">>> disableTrafficSaving(True) -> {ok}\n")
    except Exception as e:
        print(">>> disableTrafficSaving ERROR:", repr(e))

    print(f"Escuchando {secs}s (todo lo que entre)...\n")
    await asyncio.sleep(secs)

    print("\n=== TODOS los topics vistos (catch-all) ===")
    if not all_topics:
        print("  NINGUNO. Ni con traffic-saving off llega nada de LiDAR/SLAM.")
    for t, n in all_topics.most_common():
        print(f"  {n:5d}  {t}")
    print("\n(Compara estos nombres reales con los del constants.py del Go2.)")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--secs", type=int, default=30)
    args = ap.parse_args()
    try:
        asyncio.run(main(args.secs))
    except KeyboardInterrupt:
        sys.exit(0)
