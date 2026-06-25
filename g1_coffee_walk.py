#!/usr/bin/env python3
"""
g1_coffee_walk.py  -  Brazo derecho en pose de "cafe" MANTENIDA mientras el G1 camina

Usa el arm SDK (rt/arm_sdk, confirmado que el bridge del Air lo reenvia):
  - lleva el brazo derecho (juntas 22..28) a una pose objetivo y la mantiene (weight=1, 50Hz)
  - EN PARALELO conduce la marcha por rt/wirelesscontroller (20Hz)
El 'weight' del arm SDK mezcla el brazo con la locomocion -> pose fija + caminar.

SEGURIDAD (IMPORTANTE):
  - Primer ensayo: robot en GRUA / suspendido. Luego, de pie en zona MUY despejada.
  - Mando fisico a mano como parada de emergencia.
  - kp/weight moderados; el brazo se MUEVE a la pose: aleja manos.
  - Sesion unica: cierra la app del movil.

CAPTURAR TU POSE DE CAFE (recomendado):
  1) coloca el brazo derecho como quieras (a mano, o reproduce tu gesto 'Cafe' una vez)
  2) python g1_arm_sdk.py read   -> copia los 7 valores del "Brazo DERECHO"
  3) pásalos aqui:  python g1_coffee_walk.py "0.3,-0.2,0.0,1.4,0.0,0.0,0.0" 2 0.15

USO:
  python "<ruta>/g1_coffee_walk.py" pose-only "q22,..,q28"      # solo pose (sin caminar)
  python "<ruta>/g1_coffee_walk.py" "q22,..,q28" <secs> <speed>  # pose + caminar
  (sin q: usa una pose de cafe por defecto)
"""
import asyncio, sys, logging, argparse

from unitree_webrtc_connect.webrtc_driver import (
    UnitreeWebRTCConnection, WebRTCConnectionMethod)

logging.basicConfig(level=logging.FATAL)
LOWSTATE = "rt/lf/lowstate"
ARM_SDK = "rt/arm_sdk"
WC = "rt/wirelesscontroller"
RIGHT_ARM = list(range(22, 29))
WEIGHT_IDX = 29
NUM = 35
# pose de cafe por defecto (radianes): hombro algo adelante, codo ~80°, muñeca neutra
DEFAULT_COFFEE = [0.30, -0.20, 0.0, 1.40, 0.0, 0.0, 0.0]


def empty():
    return {"mode_pr": 0, "mode_machine": 0,
            "motor_cmd": [{"mode": 0, "q": 0.0, "dq": 0.0, "tau": 0.0,
                           "kp": 0.0, "kd": 0.0, "reserve": 0} for _ in range(NUM)],
            "reserve": [0, 0, 0, 0], "crc": 0}


def arm_msg(qmap, weight, kp=60.0, kd=1.5):
    m = empty()
    for i in RIGHT_ARM:
        m["motor_cmd"][i] = {"mode": 1, "q": qmap[i], "dq": 0.0, "tau": 0.0,
                             "kp": kp, "kd": kd, "reserve": 0}
    m["motor_cmd"][WEIGHT_IDX]["q"] = weight
    return m


async def read_pose(ps, timeout=5):
    fut = asyncio.get_event_loop().create_future()
    def cb(msg):
        if not fut.done():
            try: fut.set_result({i: msg["data"]["motor_state"][i]["q"] for i in RIGHT_ARM})
            except Exception as e: fut.set_exception(e)
    ps.subscribe(LOWSTATE, cb)
    return await asyncio.wait_for(fut, timeout)


async def main(target, secs, speed, walk, kp=60.0):
    conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalAP, ip="192.168.12.1")
    await conn.connect()
    print("Conectado.\n")
    ps = conn.datachannel.pub_sub

    cur = await read_pose(ps)
    print("Pose actual brazo dcho:", [round(cur[i], 3) for i in RIGHT_ARM])
    tgt = {RIGHT_ARM[k]: target[k] for k in range(7)}
    print("Pose objetivo (cafe)  :", [round(target[k], 3) for k in range(7)])

    held = {"q": dict(cur)}      # ultima pose comandada (para mantener)
    running = {"on": True}

    async def arm_loop():
        # 1) rampa de weight 0->1 + interpolar a la pose objetivo en ~2.5s
        ramp = int(2.5 / 0.02)
        for k in range(ramp):
            ph = (k + 1) / ramp
            q = {i: cur[i] * (1 - ph) + tgt[i] * ph for i in RIGHT_ARM}
            held["q"] = q
            ps.publish_without_callback(ARM_SDK, arm_msg(q, weight=min(1.0, ph), kp=kp))
            await asyncio.sleep(0.02)
        print(f">>> Brazo en pose (kp={kp}). Manteniendo...")
        # 2) mantener la pose a 50Hz mientras running
        while running["on"]:
            ps.publish_without_callback(ARM_SDK, arm_msg(tgt, weight=1.0, kp=kp))
            await asyncio.sleep(0.02)
        # 3) soltar: weight->0
        for k in range(int(1.0 / 0.02)):
            ps.publish_without_callback(ARM_SDK, arm_msg(tgt, weight=max(0.0, 1 - (k + 1) / 50), kp=kp))
            await asyncio.sleep(0.02)
        print(">>> Brazo soltado (weight->0).")

    async def walk_loop():
        await asyncio.sleep(3.0)   # esperar a que el brazo llegue a la pose
        if walk:
            print(f">>> Caminando {secs}s (ly={speed}) con el brazo en pose...")
            n = int(min(secs, 4.0) / 0.05)
            for _ in range(n):
                ps.publish_without_callback(WC, {"lx": 0.0, "ly": speed, "rx": 0.0, "ry": 0.0})
                await asyncio.sleep(0.05)
            for _ in range(12):
                ps.publish_without_callback(WC, {"lx": 0.0, "ly": 0.0, "rx": 0.0, "ry": 0.0})
                await asyncio.sleep(0.05)
            print(">>> STOP marcha.")
        else:
            await asyncio.sleep(3.0)   # solo mantener la pose un rato
        running["on"] = False

    arm_task = asyncio.ensure_future(arm_loop())
    try:
        await walk_loop()
    finally:
        running["on"] = False
        await arm_task


if __name__ == "__main__":
    # args: [q22,..,q28 | "d"]  [secs (0 = solo pose, sin caminar)]  [speed]
    ap = argparse.ArgumentParser()
    ap.add_argument("qcsv", nargs="?", default="d", help='7 q separados por coma, o "d" = pose cafe por defecto')
    ap.add_argument("secs", nargs="?", type=float, default=0.0, help="segundos de marcha (0 = solo pose)")
    ap.add_argument("speed", nargs="?", type=float, default=0.15)
    ap.add_argument("kp", nargs="?", type=float, default=60.0)
    a = ap.parse_args()

    if a.qcsv and a.qcsv != "d":
        try:
            target = [float(x) for x in a.qcsv.split(",")]
            assert len(target) == 7
        except Exception:
            print("q invalido -> uso pose de cafe por defecto"); target = DEFAULT_COFFEE
    else:
        target = DEFAULT_COFFEE

    walk = a.secs > 0
    a.speed = min(a.speed, 0.3)
    a.kp = min(a.kp, 100.0)
    try:
        asyncio.run(main(target, a.secs, a.speed, walk, a.kp))
    except KeyboardInterrupt:
        sys.exit(0)
