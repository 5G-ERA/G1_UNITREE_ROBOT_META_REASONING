#!/usr/bin/env python3
"""
probe_slam_lidar.py  -  Diagnóstico READ-ONLY para Unitree G1 Air (WebRTC)

NO envía ningún comando de movimiento. Solo:
  1. Conecta por WebRTC (LocalAP, 192.168.12.1)
  2. (Opcional) enciende el LiDAR -> rt/utlidar/switch  (sensor, no motor)
  3. Se suscribe a todos los topics de LiDAR + SLAM durante N segundos
  4. Cuenta cuántos mensajes emite cada uno

Objetivo: saber EMPÍRICAMENTE si este Air tiene LiDAR activo y servicio uslam,
antes de intentar mapear. Si un topic da 0 msgs, el robot no lo emite (o requiere
arrancar el mapping primero).

Uso:
    cd ~/unitree_webrtc_connect
    source .venv/bin/activate
    python probe_slam_lidar.py            # solo escucha
    python probe_slam_lidar.py --lidar-on # además intenta encender el LiDAR
    python probe_slam_lidar.py --secs 30
"""
import asyncio, json, logging, sys, argparse

from unitree_webrtc_connect.webrtc_driver import (
    UnitreeWebRTCConnection, WebRTCConnectionMethod)
from unitree_webrtc_connect.constants import RTC_TOPIC

logging.basicConfig(level=logging.FATAL)

# Topics relevantes para LiDAR / SLAM / odometría (claves de RTC_TOPIC)
PROBE_KEYS = [
    "ULIDAR_STATE",              # rt/utlidar/lidar_state   -> estado del LiDAR
    "ULIDAR",                    # rt/utlidar/voxel_map
    "ULIDAR_ARRAY",             # rt/utlidar/voxel_map_compressed
    "ROBOTODOM",                 # rt/utlidar/robot_pose    -> odometría/pose
    "GRID_MAP",                  # rt/mapping/grid_map      -> occupancy grid
    "SLAM_ODOMETRY",            # rt/lio_sam_ros2/mapping/odometry
    "LIDAR_MAPPING_ODOM",        # rt/uslam/frontend/odom
    "LIDAR_MAPPING_CLOUD_POINT", # rt/uslam/frontend/cloud_world_ds
    "LIDAR_MAPPING_PCD_FILE",    # rt/uslam/cloud_map       -> mapa guardado
    "LIDAR_MAPPING_SERVER_LOG",  # rt/uslam/server_log      -> log del servicio
    "LIDAR_LOCALIZATION_ODOM",   # rt/uslam/localization/odom
    "SERVICE_STATE",            # rt/servicestate
]


async def main(secs: int, lidar_on: bool):
    conn = UnitreeWebRTCConnection(
        WebRTCConnectionMethod.LocalAP, ip="192.168.12.1")
    await conn.connect()
    print("Conectado - READ ONLY. No se enviara ningun movimiento.\n")

    ps = conn.datachannel.pub_sub
    seen = {}

    def make_cb(name):
        def cb(msg):
            if name not in seen:
                print(f"\n=== PRIMER MENSAJE: {name} ===")
                print(json.dumps(msg, default=str)[:600])
            seen[name] = seen.get(name, 0) + 1
        return cb

    for key in PROBE_KEYS:
        try:
            ps.subscribe(RTC_TOPIC[key], make_cb(key))
            print("suscrito:", key, "->", RTC_TOPIC[key])
        except Exception as e:
            print("skip", key, "->", e)

    # Encender el LiDAR es accionar el sensor, no un motor de locomocion.
    if lidar_on:
        try:
            ps.publish_without_callback(RTC_TOPIC["ULIDAR_SWITCH"], {"data": True})
            print("\n[+] Enviado encendido de LiDAR (rt/utlidar/switch).")
        except Exception as e:
            print("\n[!] No se pudo encender LiDAR:", e)

    print(f"\nEscuchando {secs} s ...")
    await asyncio.sleep(secs)

    print("\n--- RESULTADOS (msgs en {}s) ---".format(secs))
    if not seen:
        print("NINGUN topic emitio. -> El Air no expone LiDAR/SLAM por WebRTC,")
        print("o el servicio no esta activo. (Probablemente sin LiDAR.)")
    for k in PROBE_KEYS:
        print(f"{k:28s}: {seen.get(k, 0)} msgs")
    print("\nLectura: si ULIDAR_STATE / ROBOTODOM emiten -> hay LiDAR vivo.")
    print("Si los rt/uslam/* emiten -> el servicio de mapping esta disponible.")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--secs", type=int, default=20)
    ap.add_argument("--lidar-on", action="store_true")
    args = ap.parse_args()
    try:
        asyncio.run(main(args.secs, args.lidar_on))
    except KeyboardInterrupt:
        sys.exit(0)
