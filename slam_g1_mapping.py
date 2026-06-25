#!/usr/bin/env python3
"""
slam_g1_mapping.py  -  SLAM del Unitree G1 (Air) por WebRTC desde Python

API REAL extraída de la app Unitree Explore (no la del Go2):
  - Comando:  rt/api/slam_operate/request   (request/response)
  - Nube:     rt/unitree/slam_mapping/points
  - Odom:     rt/unitree/slam_mapping/odom
  - Info:     rt/slam_info, rt/slam_key_info

api_id (slam_operate):
  1801 startBuildMap  data {slam_type:"indoor"}
  1802 endBuildMap    data {address:"/unitree/data/unitree_slam/<nombre>.pcd"}
  1803 cancelBuildMap
  1804 initRelocation / 1805 closeRelocation / 1102 anyPointNavigation / 1901 closeAll

Envoltorio (igual que la app): la lib publica
  {header:{identity:{id, api_id}}, parameter: json({data: <data>})}
que se consigue con: publish_request_new(topic, {"api_id":id, "parameter":{"data":data}})

SEGURIDAD: arrancar el mapeo NO mueve el robot, pero para construir el mapa lo conduces tú.
Robot de pie (mando), zona despejada o grúa, mando a mano como parada.
Sesión única: cierra la app del móvil y otros scripts.

USO (app cerrada):
  cd ~/unitree_webrtc_connect && source .venv/bin/activate
  python slam_g1_mapping.py map           # arranca mapeo, escucha; Ctrl+C -> pregunta guardar
  python slam_g1_mapping.py start         # solo enviar startBuildMap
  python slam_g1_mapping.py save MiCasa   # solo enviar endBuildMap+guardar como MiCasa.pcd
  python slam_g1_mapping.py cancel        # cancelar mapeo
  python slam_g1_mapping.py listen        # solo suscribir y ver qué llega (sin comandos)
"""
import asyncio, json, logging, sys, os, time, argparse

from unitree_webrtc_connect.webrtc_driver import (
    UnitreeWebRTCConnection, WebRTCConnectionMethod)

logging.basicConfig(level=logging.FATAL)

SLAM_OPERATE = "rt/api/slam_operate/request"
SUBS = [
    "rt/unitree/slam_mapping/points",
    "rt/unitree/slam_mapping/odom",
    "rt/unitree/slam_relocation/points",
    "rt/unitree/slam_relocation/odom",
    "rt/slam_info",
    "rt/slam_key_info",
]
OUTDIR = os.path.expanduser("~/g1/slam_map")


async def slam_cmd(ps, api_id, data=None):
    opt = {"api_id": api_id}
    if data is not None:
        opt["parameter"] = {"data": data}
    print(f">>> slam_operate api_id={api_id} data={data}")
    try:
        resp = await asyncio.wait_for(ps.publish_request_new(SLAM_OPERATE, opt), timeout=10)
        code = None
        try: code = resp["data"]["header"]["status"]["code"]
        except Exception: pass
        print(f"    RESP code={code}: {json.dumps(resp, default=str)[:400]}")
        return resp
    except asyncio.TimeoutError:
        print("    (timeout - sin respuesta)")
    except Exception as e:
        print("    ERROR:", repr(e))
    return None


def attach_listeners(ps):
    os.makedirs(OUTDIR, exist_ok=True)
    counts = {}
    saved = set()
    def mk(name):
        def cb(msg):
            counts[name] = counts.get(name, 0) + 1
            if name not in saved:
                saved.add(name)
                p = os.path.join(OUTDIR, name.replace("/", "_") + ".first.json")
                try:
                    open(p, "w").write(json.dumps(msg, default=str))
                    print(f"[{time.strftime('%H:%M:%S')}] PRIMER {name} -> guardado {p}")
                    print("     muestra:", json.dumps(msg, default=str)[:300])
                except Exception as e:
                    print("     error guardando", name, e)
            elif counts[name] % 20 == 0:
                print(f"[{time.strftime('%H:%M:%S')}] {name}: {counts[name]} msgs")
        return cb
    for t in SUBS:
        try: ps.subscribe(t, mk(t))
        except Exception as e: print("sub err", t, e)
    return counts


async def main(action, name):
    conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalAP, ip="192.168.12.1")
    await conn.connect()
    print("Conectado.\n")
    ps = conn.datachannel.pub_sub

    if action in ("map", "listen"):
        counts = attach_listeners(ps)

    if action == "cancel":
        await slam_cmd(ps, 1803)
        return
    if action == "start":
        await slam_cmd(ps, 1801, {"slam_type": "indoor"})
        return
    if action == "save":
        await slam_cmd(ps, 1802, {"address": f"/unitree/data/unitree_slam/{name}.pcd"})
        return

    if action == "listen":
        print("Escuchando 60s (sin enviar comandos)...")
        await asyncio.sleep(60)
        print("\n--- conteos ---", counts); return

    # action == "map": flujo completo
    await slam_cmd(ps, 1801, {"slam_type": "indoor"})
    print("\nMapeo arrancado. CONDUCE el robot despacio por la zona.")
    print("Cuando termines: Ctrl+C -> te pregunto si guardar.\n")
    try:
        while True:
            await asyncio.sleep(5)
            live = {k: v for k, v in counts.items() if v}
            print(f"[{time.strftime('%H:%M:%S')}] recibido: {live or 'nada aun'}")
    except (KeyboardInterrupt, asyncio.CancelledError):
        pass
    print("\n--- conteos finales ---", dict(counts))
    try:
        ans = input("Guardar mapa? nombre (vacío = cancelar): ").strip()
    except EOFError:
        ans = ""
    if ans:
        await slam_cmd(ps, 1802, {"address": f"/unitree/data/unitree_slam/{ans}.pcd"})
        print(f"Mapa guardado en el robot: /unitree/data/unitree_slam/{ans}.pcd")
        print("Para bajarlo al PC luego: getBigFile (api_id 1934).")
    else:
        await slam_cmd(ps, 1803)
        print("Mapeo cancelado.")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("action", choices=["map", "start", "save", "cancel", "listen"], default="map", nargs="?")
    ap.add_argument("name", nargs="?", default="map1")
    args = ap.parse_args()
    try:
        asyncio.run(main(args.action, args.name))
    except KeyboardInterrupt:
        sys.exit(0)
