#!/usr/bin/env python3
"""
g1_arm_sdk.py  -  Control directo de articulaciones del brazo del G1 por WebRTC (arm SDK)

Formato del arm SDK (unitree_sdk2, topic rt/arm_sdk, msg unitree_hg LowCmd):
  - motor_cmd[i]: {mode, q, dq, tau, kp, kd} por junta
  - Brazo DERECHO = indices 22..28 (shoulder p/r/y, elbow, wrist r/p/y)
  - weight en motor_cmd[29].q (0=loco controla el brazo, 1=tu lo controlas). Mezcla con la marcha.
  - publicar ~50 Hz

INCERTIDUMBRE: la app NO usa rt/arm_sdk; no sabemos si el bridge del Air lo reenvia.
Por eso 'read' (solo lee) y 'hold' (apunta a la pose ACTUAL = sin movimiento) lo testean seguro.

SEGURIDAD: robot en GRUA o de pie en zona despejada, mando a mano. Empezamos con weight y kp
bajos y target=pose actual (no debe moverse). Si el brazo se tensa -> el bridge reenvia.

USO (app cerrada):
  cd ~/unitree_webrtc_connect && source .venv/bin/activate
  python "<ruta>/g1_arm_sdk.py" read           # lee y muestra los angulos del brazo derecho
  python "<ruta>/g1_arm_sdk.py" hold           # mantiene la pose actual ~6s (test de alcance)
"""
import asyncio, sys, logging, argparse, copy

from unitree_webrtc_connect.webrtc_driver import (
    UnitreeWebRTCConnection, WebRTCConnectionMethod)

logging.basicConfig(level=logging.FATAL)

LOWSTATE = "rt/lf/lowstate"
ARM_SDK = "rt/arm_sdk"
ARM_FEEDBACK = "rt/arm_Feedback"

RIGHT_ARM = list(range(22, 29))   # 22..28
LEFT_ARM = list(range(15, 22))
WEIGHT_IDX = 29
NUM_MOTORS = 35


def empty_lowcmd():
    mc = [{"mode": 0, "q": 0.0, "dq": 0.0, "tau": 0.0, "kp": 0.0, "kd": 0.0, "reserve": 0}
          for _ in range(NUM_MOTORS)]
    return {"mode_pr": 0, "mode_machine": 0, "motor_cmd": mc,
            "reserve": [0, 0, 0, 0], "crc": 0}


async def get_arm_pose(ps, timeout=5):
    """Lee motor_state[i].q de rt/lf/lowstate para las juntas del brazo."""
    fut = asyncio.get_event_loop().create_future()
    def cb(msg):
        if fut.done():
            return
        try:
            ms = msg["data"]["motor_state"]
            q = {i: ms[i]["q"] for i in (LEFT_ARM + RIGHT_ARM)}
            fut.set_result(q)
        except Exception as e:
            if not fut.done():
                fut.set_exception(e)
    ps.subscribe(LOWSTATE, cb)
    return await asyncio.wait_for(fut, timeout)


async def main(action, secs, weight_max, kp):
    conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalAP, ip="192.168.12.1")
    await conn.connect()
    print("Conectado.\n")
    ps = conn.datachannel.pub_sub

    # feedback (por si el bridge contesta)
    fb = {"n": 0}
    try:
        ps.subscribe(ARM_FEEDBACK, lambda m: fb.__setitem__("n", fb["n"] + 1))
    except Exception:
        pass

    print("Leyendo pose actual del brazo (rt/lf/lowstate)...")
    try:
        pose = await get_arm_pose(ps)
    except Exception as e:
        print("No pude leer lowstate:", repr(e)); return
    print("Brazo DERECHO (22..28) q =", [round(pose[i], 3) for i in RIGHT_ARM])
    print("Brazo IZQ    (15..21) q =", [round(pose[i], 3) for i in LEFT_ARM])

    if action == "read":
        print("\n(solo lectura) Estos son los angulos actuales del brazo.")
        return

    # action == "hold": mantener la pose ACTUAL (sin movimiento) con rampa de weight
    print(f"\nHOLD pose actual {secs}s  (weight 0->{weight_max}, kp={kp}, target=pose actual)")
    print("OBSERVA: ¿se tensa el brazo derecho / contesta arm_Feedback? -> el bridge reenvia.\n")
    steps = int(secs / 0.02)
    ramp = int(0.5 / 0.02)  # rampa de weight en 0.5s
    try:
        for k in range(steps):
            w = weight_max * min(1.0, k / ramp)
            msg = empty_lowcmd()
            for i in RIGHT_ARM:                 # solo brazo derecho, target=actual
                msg["motor_cmd"][i] = {"mode": 1, "q": pose[i], "dq": 0.0,
                                       "tau": 0.0, "kp": kp, "kd": 1.0, "reserve": 0}
            msg["motor_cmd"][WEIGHT_IDX]["q"] = w
            ps.publish_without_callback(ARM_SDK, msg)
            await asyncio.sleep(0.02)
    finally:
        # soltar: rampa de weight a 0
        for k in range(ramp):
            msg = empty_lowcmd()
            msg["motor_cmd"][WEIGHT_IDX]["q"] = weight_max * (1 - k / ramp)
            try: ps.publish_without_callback(ARM_SDK, msg)
            except Exception: pass
            await asyncio.sleep(0.02)
        print(f"\n>>> Soltado (weight->0). arm_Feedback recibidos: {fb['n']}")
        print(">>> ¿Notaste el brazo derecho tensarse/resistir? (si=bridge reenvia rt/arm_sdk)")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("action", choices=["read", "hold"])
    ap.add_argument("secs", nargs="?", type=float, default=6.0)
    ap.add_argument("weight_max", nargs="?", type=float, default=0.3)  # bajo para el test
    ap.add_argument("kp", nargs="?", type=float, default=30.0)         # bajo para el test
    a = ap.parse_args()
    a.secs = min(a.secs, 10.0); a.weight_max = min(a.weight_max, 0.6); a.kp = min(a.kp, 60.0)
    try:
        asyncio.run(main(a.action, a.secs, a.weight_max, a.kp))
    except KeyboardInterrupt:
        sys.exit(0)
