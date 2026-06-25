#!/usr/bin/env python3
"""
slam_command_probe.py  -  Sonda empirica para arrancar el SLAM del G1 Air por WebRTC

Objetivo: descubrir el comando que activa el servicio uslam, probando payloads
candidatos en rt/uslam/client_command y rt/qt_command, y ESCUCHANDO las respuestas
y feedback del robot (rt/qt_notice, rt/uslam/server_log, rt/uslam/frontend/*).

NO envia ningun comando de locomocion. Arrancar el mapping no mueve el robot
(tu lo conduces despues). Aun asi: robot en zona despejada o en grua, mando a mano.

Como funciona:
  - Conecta (sesion unica -> cierra la app antes).
  - Se suscribe a TODOS los topics de feedback y los imprime con timestamp.
  - Enciende el LiDAR (varios formatos).
  - Envia, uno a uno y con pausa, una bateria de comandos candidatos de "start
    mapping". Tras cada uno espera y observa si algo reacciona.

Si tras un candidato empieza a emitir rt/uslam/frontend/odom o cloud, o llega un
rt/qt_notice / server_log -> ESE es el formato bueno. Apunta cual fue.

Uso:
    cd ~/unitree_webrtc_connect && source .venv/bin/activate
    python slam_command_probe.py
    python slam_command_probe.py --wait 8     # mas espera entre candidatos
"""
import asyncio, json, logging, sys, argparse, time

from unitree_webrtc_connect.webrtc_driver import (
    UnitreeWebRTCConnection, WebRTCConnectionMethod)
from unitree_webrtc_connect.constants import RTC_TOPIC

logging.basicConfig(level=logging.FATAL)

# Topics de feedback/salida que delatan si el SLAM se activo
LISTEN = [
    "SLAM_QT_NOTICE",            # rt/qt_notice            <- respuesta a qt_command
    "LIDAR_MAPPING_SERVER_LOG",  # rt/uslam/server_log     <- log del servidor SLAM
    "LIDAR_MAPPING_ODOM",        # rt/uslam/frontend/odom
    "LIDAR_MAPPING_CLOUD_POINT", # rt/uslam/frontend/cloud_world_ds
    "LIDAR_MAPPING_PCD_FILE",    # rt/uslam/cloud_map
    "LIDAR_LOCALIZATION_ODOM",   # rt/uslam/localization/odom
    "ULIDAR_STATE",              # rt/utlidar/lidar_state
    "ROBOTODOM",                 # rt/utlidar/robot_pose
    "GRID_MAP",                  # rt/mapping/grid_map
    "SERVICE_STATE",            # rt/servicestate
]

# Candidatos de "encender LiDAR"
LIDAR_ON = [
    (RTC_TOPIC["ULIDAR_SWITCH"], {"data": True}),
    (RTC_TOPIC["ULIDAR_SWITCH"], {"data": "ON"}),
]

# Candidatos de "start mapping". Son CONJETURAS (el esquema no es publico).
# Formato: (topic, payload, "metodo")  metodo: "ff"=fire&forget, "req"=request
# Las keys del keyDemo oficial: Q=start map, W=stop, A=reloc, S=add node, D=nav.
CANDIDATES = [
    # --- rt/uslam/client_command (posible std_msgs/String) ---
    (RTC_TOPIC["LIDAR_MAPPING_CMD"], {"data": "start_mapping"}, "ff"),
    (RTC_TOPIC["LIDAR_MAPPING_CMD"], {"data": "mapping_start"}, "ff"),
    (RTC_TOPIC["LIDAR_MAPPING_CMD"], {"data": "StartMapping"}, "ff"),
    (RTC_TOPIC["LIDAR_MAPPING_CMD"], {"data": json.dumps({"command": "start_mapping"})}, "ff"),
    (RTC_TOPIC["LIDAR_MAPPING_CMD"], {"data": json.dumps({"cmd": 1})}, "ff"),
    (RTC_TOPIC["LIDAR_MAPPING_CMD"], {"data": "Q"}, "ff"),
    # --- rt/qt_command (QtCommand_, esquema desconocido) ---
    (RTC_TOPIC["SLAM_QT_COMMAND"], {"cmd": 1, "seq": 1}, "ff"),
    (RTC_TOPIC["SLAM_QT_COMMAND"], {"command": 1, "seq": 1}, "ff"),
    (RTC_TOPIC["SLAM_QT_COMMAND"], {"command": "start_mapping", "seq": 1}, "ff"),
    # --- via motion_switcher: cambiar a una "form" de navegacion/slam ---
    (RTC_TOPIC["MOTION_SWITCHER"], {"api_id": 1002, "parameter": {"name": "navigation"}}, "req"),
    (RTC_TOPIC["MOTION_SWITCHER"], {"api_id": 1002, "parameter": {"name": "slam"}}, "req"),
]


async def main(wait):
    conn = UnitreeWebRTCConnection(
        WebRTCConnectionMethod.LocalAP, ip="192.168.12.1")
    await conn.connect()
    print("Conectado. Sonda de comandos SLAM. (No se envia locomocion.)\n")
    ps = conn.datachannel.pub_sub

    counts = {}
    def make_cb(name):
        def cb(msg):
            counts[name] = counts.get(name, 0) + 1
            print(f"[{time.strftime('%H:%M:%S')}] <<< {name}: "
                  f"{json.dumps(msg, default=str)[:300]}")
        return cb
    for key in LISTEN:
        try:
            ps.subscribe(RTC_TOPIC[key], make_cb(key))
        except Exception as e:
            print("skip sub", key, e)
    print("Suscrito a feedback:", ", ".join(LISTEN), "\n")

    # Query estado de servicios (no mueve nada)
    try:
        await ps.publish_request_new(RTC_TOPIC["MOTION_SWITCHER"], {"api_id": 1001})
        print(">>> motion_switcher GET (api_id 1001)")
    except Exception as e:
        print("motion_switcher query error:", e)
    await asyncio.sleep(wait)

    # Encender LiDAR
    for topic, payload in LIDAR_ON:
        try:
            ps.publish_without_callback(topic, payload)
            print(f">>> LIDAR ON {topic} {payload}")
        except Exception as e:
            print("lidar on error:", e)
    await asyncio.sleep(wait)

    # Probar candidatos de start mapping
    base = dict(counts)
    for i, (topic, payload, method) in enumerate(CANDIDATES, 1):
        print(f"\n=== CANDIDATO {i}/{len(CANDIDATES)} [{method}] {topic} "
              f"{json.dumps(payload)[:120]}")
        try:
            if method == "req":
                await ps.publish_request_new(topic, payload)
            else:
                ps.publish_without_callback(topic, payload)
        except Exception as e:
            print("   envio error:", e)
            continue
        await asyncio.sleep(wait)
        # Hubo reaccion nueva?
        new = {k: counts[k] - base.get(k, 0) for k in counts
               if counts[k] - base.get(k, 0) > 0}
        if new:
            print(f"   *** REACCION tras candidato {i}: {new}  <-- ANOTA ESTE")
        else:
            print("   (sin reaccion)")
        base = dict(counts)

    print("\n--- RESUMEN feedback total ---")
    for k in LISTEN:
        print(f"{k:28s}: {counts.get(k,0)}")
    print("\nSi algun candidato provoco emision de rt/uslam/frontend/* o un "
          "qt_notice/server_log, ese es el comando. Si todo 0, el servicio uslam "
          "no esta activo/accesible por este canal.")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--wait", type=float, default=5.0)
    args = ap.parse_args()
    try:
        asyncio.run(main(args.wait))
    except KeyboardInterrupt:
        sys.exit(0)
