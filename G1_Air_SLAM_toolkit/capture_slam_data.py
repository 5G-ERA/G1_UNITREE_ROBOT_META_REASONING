#!/usr/bin/env python3
"""
capture_slam_data.py  -  Captura datos de LiDAR/SLAM del G1 Air por WebRTC
mientras la APP ejecuta el SLAM.

READ-ONLY: no envia ningun comando de movimiento. Solo escucha y guarda.

Idea: lanzas el SLAM desde la app de Unitree (la app dispara el servicio a bordo)
y este script, conectado en paralelo por el AP local, intenta ver y guardar lo que
el robot publique en los topics de mapping (nube, odometria, grid map, mapa final).

Pre-requisito de coexistencia: el robot suele permitir UNA sola sesion WebRTC.
Para que app + este script convivan, ten el movil en DATOS MOVILES (la app va por
nube) y el Mac en el AP local del robot. Si el script no conecta mientras la app
esta activa -> es sesion unica; en ese caso ejecutalo DESPUES de mapear.

Uso:
    cd ~/unitree_webrtc_connect && source .venv/bin/activate
    python capture_slam_data.py                 # corre hasta Ctrl+C
    python capture_slam_data.py --secs 180       # o un tiempo fijo
    python capture_slam_data.py --outdir ~/g1/slam_dump
"""
import asyncio, json, logging, sys, argparse, os, time

from unitree_webrtc_connect.webrtc_driver import (
    UnitreeWebRTCConnection, WebRTCConnectionMethod)
from unitree_webrtc_connect.constants import RTC_TOPIC

logging.basicConfig(level=logging.FATAL)

# Topics a vigilar. Los "*_dump" se vuelcan a disco al primer mensaje.
LISTEN = [
    "ULIDAR_STATE",              # rt/utlidar/lidar_state
    "ROBOTODOM",                 # rt/utlidar/robot_pose
    "GRID_MAP",                  # rt/mapping/grid_map           <- mapa de ocupacion
    "SLAM_ODOMETRY",            # rt/lio_sam_ros2/mapping/odometry
    "LIDAR_MAPPING_ODOM",        # rt/uslam/frontend/odom
    "LIDAR_MAPPING_CLOUD_POINT", # rt/uslam/frontend/cloud_world_ds  <- nube viva
    "LIDAR_MAPPING_PCD_FILE",    # rt/uslam/cloud_map            <- MAPA GUARDADO
    "LIDAR_MAPPING_SERVER_LOG",  # rt/uslam/server_log
    "LIDAR_LOCALIZATION_ODOM",   # rt/uslam/localization/odom
]
# Cuales guardar a fichero (payloads valiosos: mapa y nube)
DUMP = {"GRID_MAP", "LIDAR_MAPPING_PCD_FILE", "LIDAR_MAPPING_CLOUD_POINT"}


async def main(secs, outdir):
    os.makedirs(outdir, exist_ok=True)
    conn = UnitreeWebRTCConnection(
        WebRTCConnectionMethod.LocalAP, ip="192.168.12.1")
    await conn.connect()
    print(f"Conectado - READ ONLY. Volcando a: {outdir}\n")

    seen, dumped = {}, set()

    def make_cb(name):
        def cb(msg):
            seen[name] = seen.get(name, 0) + 1
            if name not in seen or seen[name] == 1:
                print(f"[{time.strftime('%H:%M:%S')}] PRIMER {name}")
            if name in DUMP and name not in dumped:
                path = os.path.join(outdir, f"{name}.json")
                try:
                    with open(path, "w") as f:
                        json.dump(msg, f, default=str)
                    print(f"   -> guardado {path}")
                    dumped.add(name)
                except Exception as e:
                    print(f"   -> error guardando {name}: {e}")
        return cb

    ps = conn.datachannel.pub_sub
    for key in LISTEN:
        try:
            ps.subscribe(RTC_TOPIC[key], make_cb(key))
            print("suscrito:", key, "->", RTC_TOPIC[key])
        except Exception as e:
            print("skip", key, "->", e)

    print("\nLanza ahora el SLAM desde la app y muevela por la zona.")
    print("Ctrl+C para terminar.\n")

    t0 = time.time()
    try:
        while secs == 0 or (time.time() - t0) < secs:
            await asyncio.sleep(5)
            live = {k: v for k, v in seen.items() if v}
            print(f"[{time.strftime('%H:%M:%S')}] activos: {live or 'ninguno todavia'}")
    except (KeyboardInterrupt, asyncio.CancelledError):
        pass

    print("\n--- RESUMEN ---")
    for k in LISTEN:
        print(f"{k:28s}: {seen.get(k,0)} msgs")
    print("Ficheros volcados:", sorted(dumped) or "ninguno")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--secs", type=int, default=0, help="0 = hasta Ctrl+C")
    ap.add_argument("--outdir", default=os.path.expanduser("~/g1/slam_dump"))
    args = ap.parse_args()
    try:
        asyncio.run(main(args.secs, args.outdir))
    except KeyboardInterrupt:
        sys.exit(0)
