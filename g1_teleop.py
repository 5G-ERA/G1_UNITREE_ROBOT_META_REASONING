#!/usr/bin/env python3
"""
g1_teleop.py  -  Teleop (caminar) del G1 por WebRTC, formato EXACTO de la app

La app conduce el G1 publicando en rt/wirelesscontroller el objeto {lx,ly,rx,ry}
a 20 Hz CONTINUO (setInterval 50ms). Mapeo:
  ly = adelante/atras   lx = lateral   rx = giro (yaw)   ry = euler
(NO lleva campo 'keys'.) Al parar manda todo a 0.

REQUISITO: el robot debe estar en modo de MARCHA (de pie y "listo para caminar",
como cuando el mando lo mueve). Si solo está bloqueado/damping, el joystick no hace nada.

SEGURIDAD: zona MUY despejada o grúa. Mando físico a mano como parada de emergencia.
Empieza con velocidad y tiempo bajos. Sesión única (cierra la app del móvil).

USO (app cerrada):
  cd ~/unitree_webrtc_connect && source .venv/bin/activate
  python "<ruta>/g1_teleop.py" forward 2 0.3      # adelante 2s a 0.3
  python "<ruta>/g1_teleop.py" turn 2 0.3         # girar
  python "<ruta>/g1_teleop.py" armwalk Cafe 3 0.3 # reproduce gesto y luego camina (test)
"""
import asyncio, sys, logging, argparse

from unitree_webrtc_connect.webrtc_driver import (
    UnitreeWebRTCConnection, WebRTCConnectionMethod)

logging.basicConfig(level=logging.FATAL)
WC = "rt/wirelesscontroller"
ARM = "rt/api/arm/request"


def _stop(ps):
    for _ in range(12):  # parar: ceros sostenidos
        try:
            ps.publish_without_callback(WC, {"lx": 0.0, "ly": 0.0, "rx": 0.0, "ry": 0.0})
        except Exception:
            pass


async def drive(ps, secs, lx=0.0, ly=0.0, rx=0.0, ry=0.0):
    """Publica el joystick a 20 Hz durante 'secs', luego SIEMPRE para (ceros)."""
    # límites de seguridad
    secs = max(0.0, min(secs, 3.0))      # nunca más de 3 s por orden
    cl = lambda v: max(-0.4, min(0.4, v))
    lx, ly, rx, ry = cl(lx), cl(ly), cl(rx), cl(ry)
    n = int(secs / 0.05)
    print(f">>> drive {secs}s  lx={lx} ly={ly} rx={rx} ry={ry}  (20Hz)")
    try:
        for _ in range(n):
            ps.publish_without_callback(WC, {"lx": lx, "ly": ly, "rx": rx, "ry": ry})
            await asyncio.sleep(0.05)
    finally:
        _stop(ps)               # se ejecuta SIEMPRE (fin normal, error o Ctrl+C)
        print(">>> STOP (ceros enviados)")


async def main(action, name, secs, speed):
    conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalAP, ip="192.168.12.1")
    await conn.connect()
    print("Conectado.\n")
    ps = conn.datachannel.pub_sub

    if action == "forward":
        await drive(ps, secs, ly=speed)
    elif action == "back":
        await drive(ps, secs, ly=-speed)
    elif action == "turn":
        await drive(ps, secs, rx=speed)
    elif action == "strafe":
        await drive(ps, secs, lx=speed)
    elif action == "walkthenplay":
        # ULTIMA permutacion: caminar PRIMERO y meter el play a media zancada
        print(">>> Empieza a caminar; a 1s se inyecta el gesto sin dejar de andar")
        async def fire_play():
            await asyncio.sleep(1.0)
            try:
                r = await asyncio.wait_for(
                    ps.publish_request_new(ARM, {"api_id": 7108, "parameter": {"action_name": name}}),
                    timeout=10)
                print("    PLAY code=", r["data"]["header"]["status"]["code"])
            except Exception as e:
                print("    play err", e)
        task = asyncio.ensure_future(fire_play())
        await drive(ps, max(secs, 2.5), ly=speed)   # camina >=2.5s; el play entra a 1s
        await task
        print(">>> ¿Siguió caminando cuando llegó el gesto, o se paró?")
    elif action == "armwalk":
        opt = {"api_id": 7108, "parameter": {"action_name": name}}
        print(f">>> PLAY '{name}' (7108)")
        try:
            r = await asyncio.wait_for(ps.publish_request_new(ARM, opt), timeout=10)
            code = r["data"]["header"]["status"]["code"]
            print(f"    code={code}")
        except Exception as e:
            print("    play err", e)
        print("Espera 4s a que el brazo llegue a la pose...")
        await asyncio.sleep(4)
        await drive(ps, secs, ly=speed)
        print(">>> ¿Caminó manteniendo el brazo?")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("action", choices=["forward", "back", "turn", "strafe", "armwalk", "walkthenplay"])
    ap.add_argument("name", nargs="?", default="")
    ap.add_argument("secs", nargs="?", type=float, default=2.0)
    ap.add_argument("speed", nargs="?", type=float, default=0.3)
    a = ap.parse_args()
    # permitir: forward <secs> <speed>  (sin name)
    if a.action not in ("armwalk", "walkthenplay") and a.name:
        try:
            a.secs = float(a.name); a.name = ""
        except ValueError:
            pass
    try:
        asyncio.run(main(a.action, a.name, a.secs, a.speed))
    except KeyboardInterrupt:
        sys.exit(0)
