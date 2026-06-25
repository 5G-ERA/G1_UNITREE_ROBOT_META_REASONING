#!/usr/bin/env python3
"""
query_api.py  -  Interroga la API request/response del G1 Air e IMPRIME las respuestas

Clave: en el data channel de Unitree, las llamadas a topics rt/api/* son
request/response -> la respuesta viene como VALOR DE RETORNO de
publish_request_new(...), NO por suscripcion. La sonda anterior lo ignoraba.

Este script captura y muestra esos retornos. Sirve para:
  1. Confirmar que el canal request/response funciona (motion_switcher 1001).
  2. Ver que "forms"/modos admite el robot (pista para activar navegacion/slam).
  3. Barrer api_ids y leer codigos de estado (un error YA es informacion: dice
     que el topic/ api_id existe y como se equivoca el payload).

READ-MOSTLY: solo consulta. No envia locomocion. motion_switcher 1001 = GET.
Los SET (1002...) se prueban con valores inocuos para LEER el codigo de error.

Uso (app cerrada):
    cd ~/unitree_webrtc_connect && source .venv/bin/activate
    python query_api.py
"""
import asyncio, json, logging, sys

from unitree_webrtc_connect.webrtc_driver import (
    UnitreeWebRTCConnection, WebRTCConnectionMethod)
from unitree_webrtc_connect.constants import RTC_TOPIC

logging.basicConfig(level=logging.FATAL)


async def req(ps, topic, options, label):
    print(f"\n>>> {label}\n    topic={topic} options={json.dumps(options)}")
    try:
        resp = await asyncio.wait_for(
            ps.publish_request_new(topic, options), timeout=8)
        print("    RESP:", json.dumps(resp, default=str)[:800])
        return resp
    except asyncio.TimeoutError:
        print("    (timeout - sin respuesta)")
    except Exception as e:
        print("    ERROR:", repr(e))
    return None


async def main():
    conn = UnitreeWebRTCConnection(
        WebRTCConnectionMethod.LocalAP, ip="192.168.12.1")
    await conn.connect()
    print("Conectado. Interrogando API (request/response).")
    ps = conn.datachannel.pub_sub

    MS = RTC_TOPIC["MOTION_SWITCHER"]

    # 1. GET estado/modo actual (sabemos que responde: form 'ai')
    await req(ps, MS, {"api_id": 1001}, "motion_switcher GET current mode (1001)")

    # 2. Barrido de api_ids de motion_switcher para descubrir interfaz
    for aid in (1000, 1002, 1003, 1004, 1005, 1006, 1007):
        await req(ps, MS, {"api_id": aid}, f"motion_switcher api_id {aid} (sin params)")

    # 3. Intentos de SELECT de modo (leemos el codigo de error que devuelva)
    for name in ("normal", "ai", "navigation", "nav", "slam", "mapping"):
        await req(ps, MS, {"api_id": 1002, "parameter": json.dumps({"name": name})},
                  f"motion_switcher SELECT '{name}' (1002, parameter como string JSON)")

    # 4. Sport GetState (1034) - request/response, no mueve
    await req(ps, RTC_TOPIC["SPORT_MOD"], {"api_id": 1034}, "sport GetState (1034)")

    # 5. obstacles_avoid SWITCH_GET (1002) - request/response
    await req(ps, RTC_TOPIC["OBSTACLES_AVOID"], {"api_id": 1002},
              "obstacles_avoid SWITCH_GET (1002)")

    print("\n--- FIN. Mira los RESP y los codigos 'status'/'code'. ---")
    print("Un code 0 = OK; otros codigos = el api_id existe pero el payload/estado falla.")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        sys.exit(0)
