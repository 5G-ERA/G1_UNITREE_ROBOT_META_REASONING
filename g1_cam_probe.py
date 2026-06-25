#!/usr/bin/env python3
"""
g1_cam_probe.py  -  Sondea los elementos <video> de la WebView (para sacar la camara despues).

Conecta por ios_webkit_debug_proxy (USB) e inyecta JS que lista los <video>/<canvas> de la pagina:
resolucion real (videoWidth/Height), readyState (>=2 = con imagen), si esta en pausa, y si usa
srcObject (stream WebRTC). Con eso decido como capturar los frames.

PRE: ios_webkit_debug_proxy corriendo; app conectada; ACTIVA la camara con su boton en la app.
USO: python g1_cam_probe.py
"""
import json, sys, time
import requests
import websocket  # websocket-client

PROXY = "http://localhost:9221"

PROBE_JS = r"""
(function(){
  var vids = Array.prototype.slice.call(document.querySelectorAll('video'));
  var info = vids.map(function(v,i){
    return {i:i, w:v.videoWidth, h:v.videoHeight, rs:v.readyState, paused:v.paused,
            cw:v.clientWidth, ch:v.clientHeight, srcObj:!!v.srcObject,
            src:((v.currentSrc||v.src||'')+'').slice(0,50)};
  });
  var cans = Array.prototype.slice.call(document.querySelectorAll('canvas'));
  var cinfo = cans.map(function(c,i){return {i:i, w:c.width, h:c.height, cw:c.clientWidth, ch:c.clientHeight};});
  return JSON.stringify({nv:vids.length, vids:info, ncanvas:cans.length, canvas:cinfo});
})()
"""


def discover_ws():
    devs = requests.get(PROXY + "/json", timeout=5).json()
    if not devs:
        raise RuntimeError("No hay dispositivos. ¿iPhone + ios_webkit_debug_proxy?")
    durl = devs[0]["url"]
    pages = requests.get(f"http://{durl}/json", timeout=5).json()
    if not pages:
        raise RuntimeError("No hay paginas. ¿App abierta? ¿Inspector de Safari CERRADO?")
    def score(p):
        u = (p.get("url", "") + " " + p.get("title", "")).lower()
        return ("b2app" in u) * 4 + ("8084" in u) * 3 + ("slam" in u) * 2 + 1
    page = sorted(pages, key=score, reverse=True)[0]
    print("Pagina:", page.get("title"), page.get("url"))
    return page["webSocketDebuggerUrl"]


class CDP:
    def __init__(self, url):
        self.ws = websocket.create_connection(url, max_size=None)
        self.id = 0; self.target = None
        self._discover_target()
    def _discover_target(self, timeout=6):
        end = time.time() + timeout
        while time.time() < end and not self.target:
            self.ws.settimeout(max(0.1, end - time.time()))
            try:
                msg = json.loads(self.ws.recv())
            except Exception:
                continue
            if msg.get("method") == "Target.targetCreated":
                ti = msg["params"]["targetInfo"]
                if ti.get("type") == "page":
                    self.target = ti["targetId"]; print("Target page:", self.target)
        if not self.target:
            raise RuntimeError("No llegó Target.targetCreated")
    def call(self, method, params=None, timeout=12):
        self.id += 1; iid = self.id
        inner = json.dumps({"id": iid, "method": method, "params": params or {}})
        self.ws.send(json.dumps({"id": iid, "method": "Target.sendMessageToTarget",
                                 "params": {"targetId": self.target, "message": inner}}))
        end = time.time() + timeout
        while time.time() < end:
            self.ws.settimeout(max(0.1, end - time.time()))
            try:
                msg = json.loads(self.ws.recv())
            except Exception:
                continue
            if msg.get("method") == "Target.dispatchMessageFromTarget":
                im = msg["params"]["message"]; im = json.loads(im) if isinstance(im, str) else im
                if im.get("id") == iid:
                    if "error" in im:
                        raise RuntimeError(im["error"])
                    return im
        raise TimeoutError(method)
    def eval(self, expr):
        r = self.call("Runtime.evaluate", {"expression": expr, "returnByValue": True})
        return (r.get("result") or {}).get("result", {}).get("value")


def main():
    url = discover_ws(); print("Inspector WS:", url)
    cdp = CDP(url)
    try:
        cdp.call("Runtime.enable")
    except Exception:
        pass
    print("Sondeando 12s. ACTIVA la camara en la app y observa si aparece un <video> con w/h>0 y rs>=2.\n")
    try:
        for _ in range(24):
            r = json.loads(cdp.eval(PROBE_JS))
            print(f"  videos={r['nv']}  canvas={r['ncanvas']}")
            for v in r["vids"]:
                print(f"    video#{v['i']}: {v['w']}x{v['h']} rs={v['rs']} paused={v['paused']} "
                      f"srcObj={v['srcObj']} disp={v['cw']}x{v['ch']} src={v['src']}")
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    print("\nFin probe.")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print("Error:", repr(e)); sys.exit(1)
