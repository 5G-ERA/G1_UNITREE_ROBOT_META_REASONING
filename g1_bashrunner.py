#!/usr/bin/env python3
"""
g1_bashrunner.py  -  Sonda del ejecutor de scripts del G1 (rt/api/bashrunner/request)

La app llama: publishReqNew("rt/api/bashrunner/request", {api_id:1001, data:{script:"x.sh"}})
Ejecuta SCRIPTS por nombre en el robot. Objetivo del sondeo (SOLO LECTURA / diagnostico):
  1) confirmar que funciona con un script conocido
  2) ver como responde a un nombre INEXISTENTE (¿whitelist? ¿"file not found"? ¿ruta?)
  3) ver si acepta un comando/lectura arbitraria (id, ls) -> revela si hay shell real

AVISO: esto sondea una interfaz privilegiada del robot. Usa SOLO lecturas (version, ls, id).
NO ejecutes nada destructivo: riesgo de dejar el robot inservible. Es tu robot, tu decision.

USO:
  cd ~/unitree_webrtc_connect && source .venv/bin/activate
  python "<ruta>/g1_bashrunner.py" "get_software_version.sh"   # baseline conocido
  python "<ruta>/g1_bashrunner.py" "zzz_no_existe.sh"          # ver error de nombre
  python "<ruta>/g1_bashrunner.py" "id"                        # ¿shell arbitrario?
  python "<ruta>/g1_bashrunner.py" "ls /unitree"              # ¿lista ficheros?
"""
import asyncio, json, sys, logging

from unitree_webrtc_connect.webrtc_driver import (
    UnitreeWebRTCConnection, WebRTCConnectionMethod)

logging.basicConfig(level=logging.FATAL)
BASH = "rt/api/bashrunner/request"


async def main(script):
    conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalAP, ip="192.168.12.1")
    await conn.connect()
    print("Conectado.\n")
    ps = conn.datachannel.pub_sub
    print(f">>> bashrunner script={script!r}")
    try:
        r = await asyncio.wait_for(
            ps.publish_request_new(BASH, {"api_id": 1001, "data": {"script": script}}),
            timeout=12)
        code = None
        try: code = r["data"]["header"]["status"]["code"]
        except Exception: pass
        out = None
        try: out = r["data"]["data"]
        except Exception: pass
        print(f"    code={code}")
        print(f"    salida: {str(out)[:1500]}")
    except asyncio.TimeoutError:
        print("    (timeout)")
    except Exception as e:
        print("    ERROR:", repr(e))


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print('uso: python g1_bashrunner.py "<script_o_comando>"'); sys.exit(1)
    try:
        asyncio.run(main(sys.argv[1]))
    except KeyboardInterrupt:
        sys.exit(0)
