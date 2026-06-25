#!/usr/bin/env python3
"""
g1_lidar_stream.py  -  Stream de LiDAR del G1 Air por WebRTC (secuencia CORRECTA)

Calcado del ejemplo que SI funciona (examples/go2/data_channel/lidar/lidar_stream.py),
adaptado a LocalAP (G1 Air, 192.168.12.1). Corrige los dos fallos previos:
  1. Encender LiDAR = publicar el STRING "on" a rt/utlidar/switch (NO {"data":True}).
  2. El topic que streamea es rt/utlidar/voxel_map_compressed (NO voxel_map).

Secuencia: connect -> disableTrafficSaving(True) -> set_decoder('libvoxel')
           -> switch "on" -> subscribe voxel_map_compressed.

READ-ONLY: no envia locomocion.

Uso (app cerrada):
    cd ~/unitree_webrtc_connect && source .venv/bin/activate
    python g1_lidar_stream.py
"""
import asyncio, logging, sys, time

from unitree_webrtc_connect.webrtc_driver import (
    UnitreeWebRTCConnection, WebRTCConnectionMethod)

logging.basicConfig(level=logging.FATAL)


async def main():
    conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalAP, ip="192.168.12.1")
    await conn.connect()
    print("Conectado.\n")

    # 1. Desactivar ahorro de trafico (imprescindible para LiDAR)
    await conn.datachannel.disableTrafficSaving(True)

    # 2. Decoder de nube
    conn.datachannel.set_decoder(decoder_type='libvoxel')

    # 3. Encender LiDAR  -> payload STRING "on" (este era el fallo)
    conn.datachannel.pub_sub.publish_without_callback("rt/utlidar/switch", "on")
    print(">>> LiDAR switch 'on' enviado a rt/utlidar/switch\n")

    count = {"n": 0}
    def lidar_callback(message):
        count["n"] += 1
        if count["n"] <= 3:
            d = message.get("data", {})
            # message['data']['data'] trae los puntos decodificados
            info = {k: v for k, v in d.items() if k != "data"} if isinstance(d, dict) else d
            print(f"[{time.strftime('%H:%M:%S')}] LIDAR msg #{count['n']} meta={info}")
        elif count["n"] % 10 == 0:
            print(f"[{time.strftime('%H:%M:%S')}] LIDAR msgs recibidos: {count['n']}")

    # 4. Suscribir al voxel map COMPRIMIDO (el que streamea)
    conn.datachannel.pub_sub.subscribe("rt/utlidar/voxel_map_compressed", lidar_callback)
    print(">>> Suscrito a rt/utlidar/voxel_map_compressed\n")
    print("Escuchando 40s... (mueve la mano delante del LiDAR de la cabeza)\n")

    await asyncio.sleep(40)

    print(f"\n=== TOTAL mensajes LiDAR: {count['n']} ===")
    if count["n"] == 0:
        print("Aun 0. Si el ejemplo Go2 funciona y aqui no, el G1 Air no reenvia el")
        print("voxel_map por WebRTC -> queda el APK para ver el topic real del G1.")
    else:
        print("¡LiDAR streameando! Siguiente: convertir la nube a PointCloud2 / PCD")
        print("y correr slam_toolbox en el PC.")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nInterrumpido")
        sys.exit(0)
