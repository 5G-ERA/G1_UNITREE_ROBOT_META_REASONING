#!/usr/bin/env python3
"""
g1_arm_teaching.py  -  Crear/disparar gestos aprendidos (Demo Teaching) del G1 por WebRTC

API extraída de la app (TeachPlay/TeachCreateViewModel + DogApiId). Topic: rt/api/arm/request
api_id:
  7107 listar acciones aprendidas (G1_TEACHING_LIST)
  7108 reproducir            (G1_TEACHING_PLAY)   param {"action_name": "<nombre>"}
  7110 empezar a grabar      (G1_TEACHING_START)  param {"action_name": "<nombre>"}
  7113 parar grabar/reproducir (G1_SPORT_ARM_TEACH)
  7111 pausar · 7112 borrar · 7109 renombrar · 7101 damp (G1_SPORT_DAMP, topic sport)

Envoltorio (igual que la app): publish_request_new("rt/api/arm/request",
  {"api_id": <id>, "parameter": <dict>})  -> parameter va como JSON (sin envoltura {data:})

SEGURIDAD: el brazo se mueve. Robot de pie por mando o en grúa, zona despejada, mando a mano.
Sesión única: cierra la app del móvil.

USO (app cerrada):
  cd ~/unitree_webrtc_connect && source .venv/bin/activate
  python "<ruta>/g1_arm_teaching.py" list
  python "<ruta>/g1_arm_teaching.py" play "Cafe"
  python "<ruta>/g1_arm_teaching.py" stop
  python "<ruta>/g1_arm_teaching.py" walktest "Cafe"   # reproduce y prueba a caminar (test bloqueo)
"""
import asyncio, json, sys, logging, argparse

from unitree_webrtc_connect.webrtc_driver import (
    UnitreeWebRTCConnection, WebRTCConnectionMethod)

logging.basicConfig(level=logging.FATAL)

ARM = "rt/api/arm/request"
SPORT = "rt/api/sport/request"


async def req(ps, topic, api_id, parameter=None, label=""):
    opt = {"api_id": api_id}
    if parameter is not None:
        opt["parameter"] = parameter
    print(f">>> {label or api_id}  topic={topic} {json.dumps(opt)[:160]}")
    try:
        r = await asyncio.wait_for(ps.publish_request_new(topic, opt), timeout=10)
        code = None
        try: code = r["data"]["header"]["status"]["code"]
        except Exception: pass
        data = None
        try: data = r["data"]["data"]
        except Exception: pass
        print(f"    code={code}  data={str(data)[:500]}")
        if code == 7404:
            print("    (7404 = FSM no disponible: pon el robot DE PIE con el mando primero)")
        return r
    except asyncio.TimeoutError:
        print("    (timeout)")
    except Exception as e:
        print("    ERROR:", repr(e))
    return None


async def main(action, name):
    conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalAP, ip="192.168.12.1")
    await conn.connect()
    print("Conectado.\n")
    ps = conn.datachannel.pub_sub

    if action == "list":
        await req(ps, ARM, 7107, {}, "LISTAR (7107)")
    elif action == "play":
        await req(ps, ARM, 7108, {"action_name": name}, f"PLAY '{name}' (7108)")
    elif action == "stop":
        await req(ps, ARM, 7113, None, "STOP (7113)")
    elif action == "damp":
        await req(ps, SPORT, 7101, None, "DAMP (7101)")
    elif action == "playmp":
        # OPCION 1: play con motion_paused:false + test de caminar (misma sesion)
        await req(ps, ARM, 7108, {"action_name": name, "motion_paused": False},
                  f"PLAY '{name}' + motion_paused:false (7108)")
        print("\nEspera a que el brazo llegue a la pose... 4s")
        await asyncio.sleep(4)
        await _walk(ps, 2.0)
        print(">>> ¿Mantuvo el brazo Y se movieron las piernas?")
    elif action == "release":
        # OPCION 2: entrar en fsm 550 (brazo control, piernas NO pausadas)
        await req(ps, ARM, 7100, {"fsm_id": 550, "api_id": 2, "motion_paused": False},
                  "ReleaseArm fsm550 motion_paused:false (7100)")
    elif action == "releasewalk":
        # OPCION 2b: fsm550 y luego caminar
        await req(ps, ARM, 7100, {"fsm_id": 550, "api_id": 2, "motion_paused": False},
                  "ReleaseArm fsm550 (7100)")
        await asyncio.sleep(2)
        await _walk(ps, 2.0)
        print(">>> ¿Se movieron las piernas en fsm550?")
    elif action == "releaseplaywalk":
        # OPCION 2c: entrar fsm550 -> reproducir gesto -> caminar (todo en una sesion)
        await req(ps, ARM, 7100, {"fsm_id": 550, "api_id": 2, "motion_paused": False},
                  "ReleaseArm fsm550 (7100)")
        await asyncio.sleep(1.5)
        await req(ps, ARM, 7108, {"action_name": name}, f"PLAY '{name}' (7108)")
        await asyncio.sleep(4)
        await _walk(ps, 2.0)
        print(">>> ¿Mantuvo el brazo Y caminó?")
    elif action == "walktest":
        # OPCION 3: play -> esperar -> intentar caminar
        await req(ps, ARM, 7108, {"action_name": name}, f"PLAY '{name}' (7108)")
        print("\nEspera a que el brazo llegue a la pose... 4s")
        await asyncio.sleep(4)
        await _walk(ps, 2.0)
        print(">>> ¿Se movieron las piernas o solo el brazo?")
    elif action == "playthenstand":
        # OPCION 3b: play -> BalanceStand -> caminar
        await req(ps, ARM, 7108, {"action_name": name}, f"PLAY '{name}' (7108)")
        await asyncio.sleep(4)
        await req(ps, SPORT, 1002, None, "BalanceStand (1002)")
        await asyncio.sleep(1)
        await _walk(ps, 2.0)
        print(">>> ¿Caminó manteniendo el brazo?")


async def _walk(ps, secs):
    print(f">>> MOVE adelante (ly=0.15) {secs}s ...")
    n = int(secs / 0.2)
    for _ in range(n):
        try:
            ps.publish_without_callback("rt/wirelesscontroller",
                {"lx": 0.0, "ly": 0.15, "rx": 0.0, "ry": 0.0, "keys": 0})
        except Exception as e:
            print("move err", e); break
        await asyncio.sleep(0.2)
    ps.publish_without_callback("rt/wirelesscontroller",
        {"lx": 0.0, "ly": 0.0, "rx": 0.0, "ry": 0.0, "keys": 0})
    print(">>> STOP movimiento.")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("action", choices=["list", "play", "stop", "damp", "walktest",
                                       "playmp", "release", "releasewalk", "playthenstand",
                                       "releaseplaywalk"])
    ap.add_argument("name", nargs="?", default="")
    a = ap.parse_args()
    try:
        asyncio.run(main(a.action, a.name))
    except KeyboardInterrupt:
        sys.exit(0)
