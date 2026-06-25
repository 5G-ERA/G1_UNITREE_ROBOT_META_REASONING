#!/usr/bin/env python3
"""
g1_arm_during_walk.py  -  Inyectar rt/arm_sdk MIENTRAS el robot camina

Hipotesis (de tu observacion): el robot mueve los brazos al caminar -> el controlador de
locomocion actua sobre los brazos SOLO durante la marcha. El arm SDK (weight) es parte de ese
controlador, asi que quiza solo honra rt/arm_sdk MIENTRAS camina (con el robot quieto no actuaba).

Test: arranca a caminar (rt/wirelesscontroller 20Hz) y, 1s despues, empieza a publicar rt/arm_sdk
@50Hz fijando el brazo DERECHO en una pose objetivo (weight->1). Observa si el brazo derecho deja
de balancearse y se queda en la pose mientras las piernas siguen andando.

SEGURIDAD: robot en GRUA o zona MUY despejada. Mando a mano. Velocidad baja, tiempo corto.
Sesion unica (app del movil cerrada).

USO:
  cd ~/unitree_webrtc_connect && source .venv/bin/activate
  python "<ruta>/g1_arm_during_walk.py"                 # pose cafe por defecto, 3s, speed 0.15
  python "<ruta>/g1_arm_during_walk.py" "0.3,-0.2,0,1.4,0,0,0" 3 0.15 70
"""
import asyncio, sys, logging, argparse

from unitree_webrtc_connect.webrtc_driver import (
    UnitreeWebRTCConnection, WebRTCConnectionMethod)

logging.basicConfig(level=logging.FATAL)
LOWSTATE = "rt/lf/lowstate"
ARM_SDK = "rt/arm_sdk"
WC = "rt/wirelesscontroller"
RIGHT = list(range(22, 29))
WIDX = 29
NUM = 35
DEFAULT = [0.30, -0.20, 0.0, 1.40, 0.0, 0.0, 0.0]


def arm_msg(qmap, weight, kp):
    mc = [{"mode": 0, "q": 0.0, "dq": 0.0, "tau": 0.0, "kp": 0.0, "kd": 0.0, "reserve": 0}
          for _ in range(NUM)]
    for i in RIGHT:
        mc[i] = {"mode": 1, "q": qmap[i], "dq": 0.0, "tau": 0.0, "kp": kp, "kd": 1.5, "reserve": 0}
    mc[WIDX]["q"] = weight
    return {"mode_pr": 0, "mode_machine": 0, "motor_cmd": mc, "reserve": [0, 0, 0, 0], "crc": 0}


async def read_pose(ps, timeout=5):
    fut = asyncio.get_event_loop().create_future()
    def cb(m):
        if not fut.done():
            try: fut.set_result({i: m["data"]["motor_state"][i]["q"] for i in RIGHT})
            except Exception as e: fut.set_exception(e)
    ps.subscribe(LOWSTATE, cb)
    return await asyncio.wait_for(fut, timeout)


async def main(target, secs, speed, kp):
    conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalAP, ip="192.168.12.1")
    await conn.connect()
    print("Conectado.\n")
    ps = conn.datachannel.pub_sub
    cur = await read_pose(ps)
    tgt = {RIGHT[k]: target[k] for k in range(7)}
    print("Brazo dcho actual:", [round(cur[i], 3) for i in RIGHT])
    print("Objetivo         :", [round(target[k], 3) for k in range(7)])

    secs = min(secs, 4.0)
    dt = 0.02                      # 50 Hz
    total = int(secs / dt)
    inject_at = int(1.0 / dt)      # empezar a inyectar el brazo a 1s
    ramp = int(1.0 / dt)           # rampa de weight en 1s
    print(f">>> UN solo bucle 50Hz: camina {secs}s (ly={speed}); brazo desde 1s (kp={kp})")
    try:
        for k in range(total):
            # marcha: joystick CONTINUO cada tick (50Hz)
            ps.publish_without_callback(WC, {"lx": 0.0, "ly": speed, "rx": 0.0, "ry": 0.0})
            # brazo: inyectar arm_sdk a partir de 1s con rampa de weight
            if k >= inject_at:
                w = min(1.0, (k - inject_at) / ramp)
                ps.publish_without_callback(ARM_SDK, arm_msg(tgt, weight=w, kp=kp))
            await asyncio.sleep(dt)
    finally:
        # parar marcha y soltar brazo
        for j in range(ramp):
            ps.publish_without_callback(WC, {"lx": 0.0, "ly": 0.0, "rx": 0.0, "ry": 0.0})
            ps.publish_without_callback(ARM_SDK, arm_msg(tgt, weight=max(0.0, 1 - j / ramp), kp=kp))
            await asyncio.sleep(dt)
        print(">>> STOP marcha + brazo soltado.")
    print("\n>>> ¿El brazo derecho se quedó en la pose mientras las piernas seguían andando?")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("qcsv", nargs="?", default="d")
    ap.add_argument("secs", nargs="?", type=float, default=3.0)
    ap.add_argument("speed", nargs="?", type=float, default=0.15)
    ap.add_argument("kp", nargs="?", type=float, default=70.0)
    a = ap.parse_args()
    if a.qcsv and a.qcsv != "d":
        try:
            target = [float(x) for x in a.qcsv.split(",")]; assert len(target) == 7
        except Exception:
            print("q invalido -> pose por defecto"); target = DEFAULT
    else:
        target = DEFAULT
    a.speed = min(a.speed, 0.3); a.kp = min(a.kp, 100.0)
    try:
        asyncio.run(main(target, a.secs, a.speed, a.kp))
    except KeyboardInterrupt:
        sys.exit(0)
