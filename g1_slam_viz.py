#!/usr/bin/env python3
"""
g1_slam_viz.py  -  Visualizacion EN VIVO de la odometria/pose del G1 (se mueve en tiempo real)

Lee rt/unitree/slam_mapping/odom (pose en frame "map", ~10Hz) y rt/slam_info (telemetria) por
WebRTC, y dibuja en una ventana: el robot (triangulo que gira), su estela, y bateria/CPU.

Requisitos en el venv:
    pip install matplotlib numpy
    # si la ventana no abre en mac:  brew install python-tk

USO (app cerrada). Mueve el robot con el mando para verlo desplazarse:
    cd ~/unitree_webrtc_connect && source .venv/bin/activate
    python "<ruta>/g1_slam_viz.py"
"""
import asyncio, threading, math, sys, json, logging
from collections import deque

import matplotlib
# intenta un backend con ventana
for _bk in ("MacOSX", "TkAgg", "QtAgg"):
    try:
        matplotlib.use(_bk); break
    except Exception:
        continue
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

from unitree_webrtc_connect.webrtc_driver import (
    UnitreeWebRTCConnection, WebRTCConnectionMethod)

logging.basicConfig(level=logging.FATAL)
SLAM_OP = "rt/api/slam_operate/request"
ODOM = "rt/unitree/slam_mapping/odom"
INFO = "rt/slam_info"

st = {"x": 0.0, "y": 0.0, "yaw": 0.0, "n": 0,
      "batt": None, "cpu": None, "cpuT": None}
traj = deque(maxlen=8000)
lock = threading.Lock()
stop = {"v": False}


def quat_to_yaw(q):
    x, y, z, w = q["x"], q["y"], q["z"], q["w"]
    return math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))


async def webrtc_loop():
    conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalAP, ip="192.168.12.1")
    await conn.connect()
    print("Conectado (mapeo + odom en vivo).")
    ps = conn.datachannel.pub_sub

    def on_odom(m):
        try:
            p = m["data"]["pose"]["pose"]
            x, y = p["position"]["x"], p["position"]["y"]
            yaw = quat_to_yaw(p["orientation"])
            with lock:
                st["x"], st["y"], st["yaw"] = x, y, yaw
                st["n"] += 1
                traj.append((x, y))
        except Exception:
            pass

    def on_info(m):
        try:
            d = m.get("data")
            d = json.loads(d) if isinstance(d, str) else d
            if d.get("type") == "robot_data":
                dd = d["data"]
                with lock:
                    st["batt"] = dd.get("batteryPower")
                    st["cpu"] = dd.get("cpuUsage")
                    st["cpuT"] = dd.get("cpuTemp")
        except Exception:
            pass

    ps.subscribe(ODOM, on_odom)
    ps.subscribe(INFO, on_info)
    try:
        await asyncio.wait_for(ps.publish_request_new(
            SLAM_OP, {"api_id": 1801, "parameter": {"data": {"slam_type": "indoor"}}}), timeout=10)
    except Exception as e:
        print("start mapping err (sigo con odom):", e)

    while not stop["v"]:
        await asyncio.sleep(0.2)
    # cancelar mapeo al salir
    try:
        await asyncio.wait_for(ps.publish_request_new(SLAM_OP, {"api_id": 1803}), timeout=4)
    except Exception:
        pass


def robot_triangle(x, y, yaw, size=0.18):
    pts = [(size, 0), (-size*0.6, size*0.5), (-size*0.6, -size*0.5)]
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + px*c - py*s, y + px*s + py*c) for px, py in pts]


def main():
    th = threading.Thread(target=lambda: asyncio.run(webrtc_loop()), daemon=True)
    th.start()

    plt.ion()
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.grid(True, alpha=0.3); ax.set_aspect("equal")
    (trail,) = ax.plot([], [], "-", lw=1.5, color="tab:blue", alpha=0.8)
    robot = Polygon(robot_triangle(0, 0, 0), closed=True, fc="red", ec="black", zorder=5)
    ax.add_patch(robot)
    info_txt = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top",
                       fontsize=9, family="monospace",
                       bbox=dict(boxstyle="round", fc="white", alpha=0.7))

    print("Ventana abierta. Mueve el robot con el mando. Cierra la ventana o Ctrl+C para salir.")
    try:
        while plt.fignum_exists(fig.number):
            with lock:
                xs = [p[0] for p in traj]; ys = [p[1] for p in traj]
                x, y, yaw, n = st["x"], st["y"], st["yaw"], st["n"]
                batt, cpu, cpuT = st["batt"], st["cpu"], st["cpuT"]
            if xs:
                trail.set_data(xs, ys)
                robot.set_xy(robot_triangle(x, y, yaw))
                pad = 1.2
                ax.set_xlim(min(xs)-pad, max(xs)+pad)
                ax.set_ylim(min(ys)-pad, max(ys)+pad)
            info_txt.set_text(
                f"odom msgs: {n}\n"
                f"pose: x={x:+.2f} y={y:+.2f}\n"
                f"yaw:  {math.degrees(yaw):+.0f}\xb0\n"
                f"bateria: {batt if batt is not None else '?'} %\n"
                f"cpu: {cpu:.0f}% {cpuT:.0f}\xb0C" if cpu is not None else "cpu: ?")
            ax.set_title("G1 — odometria/pose en tiempo real")
            plt.pause(0.08)
    except KeyboardInterrupt:
        pass
    finally:
        stop["v"] = True
        print("\nSaliendo (cancelando mapeo)...")
        import time; time.sleep(1.0)


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print("Error:", repr(e))
        print("Si es por el backend grafico en mac, instala tkinter:  brew install python-tk")
        sys.exit(1)
