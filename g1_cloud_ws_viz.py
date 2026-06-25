#!/usr/bin/env python3
"""
g1_cloud_ws_viz.py  -  Visor EN VIVO de la nube laser del G1 (recibida desde la WebView del iPhone)

La WebView de la app decodifica la nube del SLAM y, con un gancho JS, nos la envia por WebSocket.
Este servidor la recibe ({count, xyz:[x,y,z,...]}) y la pinta en tiempo real (vista en planta,
coloreada por altura) acumulando el mapa.

Requisitos: pip install websockets matplotlib numpy
USO:
  python "<ruta>/g1_cloud_ws_viz.py"        # escucha en 0.0.0.0:8765
Luego, en la consola del inspector Safari de la WebView, pega el gancho JS (te lo doy aparte)
con la IP de tu Mac.
"""
import asyncio, json, threading, sys
import numpy as np

import matplotlib
for _bk in ("MacOSX", "TkAgg", "QtAgg"):
    try: matplotlib.use(_bk); break
    except Exception: continue
import matplotlib.pyplot as plt

try:
    import websockets
except ImportError:
    print("Falta 'websockets'. Instala:  pip install websockets matplotlib numpy"); sys.exit(1)

PORT = 8765
lock = threading.Lock()
# mapa acumulado (voxelizado grueso para no crecer infinito)
acc = {}            # (ix,iy,iz) -> (x,y,z)
latest = {"n": 0, "pts": 0, "msgs": 0}
VOX = 0.05          # 5 cm de rejilla para deduplicar


def add_points(flat):
    with lock:
        for i in range(0, len(flat) - 2, 3):
            x, y, z = flat[i], flat[i+1], flat[i+2]
            key = (round(x/VOX), round(y/VOX), round(z/VOX))
            acc[key] = (x, y, z)
        latest["msgs"] += 1
        latest["pts"] = len(acc)


async def handler(ws):
    print(">>> WebView conectada")
    async for msg in ws:
        try:
            d = json.loads(msg)
            xyz = d.get("xyz") or d.get("directOutput") or []
            if xyz:
                add_points(xyz)
        except Exception as e:
            print("msg err:", e)
    print(">>> WebView desconectada")


def start_server():
    async def run():
        async with websockets.serve(handler, "0.0.0.0", PORT, max_size=None):
            print(f"Servidor WebSocket en ws://0.0.0.0:{PORT} (escuchando)")
            await asyncio.Future()
    asyncio.run(run())


def main():
    threading.Thread(target=start_server, daemon=True).start()

    plt.ion()
    fig, ax = plt.subplots(figsize=(9, 9))
    ax.set_aspect("equal"); ax.grid(True, alpha=0.3)
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    sc = ax.scatter([], [], s=2)
    print("Ventana abierta. Esperando nube de la WebView... mueve el robot para mapear.")
    try:
        while plt.fignum_exists(fig.number):
            with lock:
                pts = list(acc.values()); m = latest["msgs"]
            if pts:
                arr = np.array(pts)
                sc.set_offsets(arr[:, :2])
                sc.set_array(arr[:, 2])           # color por altura z
                sc.set_clim(arr[:, 2].min(), arr[:, 2].max())
                ax.set_xlim(arr[:, 0].min()-0.5, arr[:, 0].max()+0.5)
                ax.set_ylim(arr[:, 1].min()-0.5, arr[:, 1].max()+0.5)
                ax.set_title(f"G1 LiDAR/SLAM en vivo — {len(pts)} puntos | mensajes: {m}")
            plt.pause(0.1)
    except KeyboardInterrupt:
        pass
    print("\nFin.")


if __name__ == "__main__":
    main()
