#!/usr/bin/env python3
"""
g1_inspector_bridge.py  -  LiDAR/SLAM del G1 EN VIVO por USB (via ios-webkit-debug-proxy)

Lee la nube de puntos que la WebView de la app decodifica, a traves del canal de inspeccion
(USB) que expone ios_webkit_debug_proxy. No usa red WiFi -> a prueba de aislamiento del AP.

Como funciona:
  - ios_webkit_debug_proxy debe estar corriendo (expone localhost:9221 / 9222).
  - Este script descubre la pagina del SLAM, se conecta a su inspector (CDP), instala un gancho
    que mete los puntos decodificados en window.__buf, y cada ~0.4s lee+vacia ese buffer.
  - Acumula los puntos (rejilla 5cm) y los pinta en planta, coloreados por altura, en vivo.

Pre:
  brew install ios-webkit-debug-proxy ; ios_webkit_debug_proxy   (en otra terminal, corriendo)
  pip install websocket-client requests matplotlib numpy
  IMPORTANTE: cierra la ventana del Inspector Web de Safari para esa pagina (solo un depurador
  a la vez). Manten el iPhone en la pantalla del mapa.

USO:
  python "<ruta>/g1_inspector_bridge.py"
"""
import json, sys, time, threading, math
from collections import deque
import requests
import numpy as np
import websocket  # websocket-client

import matplotlib
for _bk in ("MacOSX", "TkAgg", "QtAgg"):
    try: matplotlib.use(_bk); break
    except Exception: continue
import matplotlib.pyplot as plt

PROXY = "http://localhost:9221"
VOX = 0.05
acc = {}
robot = {}                 # pose actual {x,y,z,yaw}
traj = deque(maxlen=6000)  # estela
lock = threading.Lock()
stat = {"polls": 0, "pts": 0}
cam = {"jpg": None, "n": 0, "annot": None, "an": 0}   # frame crudo (jpg) y anotado por YOLO (annot)

CAM_JS = (
    "(function(){var v=document.querySelector('video');"
    "if(!v||!v.videoWidth)return '';"
    "var W=320,H=Math.round(W*v.videoHeight/v.videoWidth);"
    "var c=window.__camc||(window.__camc=document.createElement('canvas'));"
    "c.width=W;c.height=H;c.getContext('2d').drawImage(v,0,0,W,H);"
    "try{return c.toDataURL('image/jpeg',0.5);}catch(e){return 'ERR';}})()"
)

HOOK_JS = r"""
(function(){
  window.__buf = window.__buf || [];
  // gancho de ODOMETRIA: captura slam_mapping/odom via JSON.parse (sirva o no el worker)
  if(!window.__odomHook){ window.__odomHook = 1;
    var jp = JSON.parse;
    JSON.parse = function(s){ var v = jp.apply(this, arguments);
      try{ if(v && v.topic && (''+v.topic).indexOf('slam_mapping/odom') >= 0){
        var p = v.data.pose.pose;
        window.__odom = [p.position.x, p.position.y, p.position.z,
                         p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w];
      }}catch(e){}
      return v;
    };
  }
  if(window.__hookInstalled) return 'already';
  window.__hookInstalled = 1;
  var seen = new WeakSet();
  var o = Worker.prototype.postMessage;
  Worker.prototype.postMessage = function(m){
    var args = arguments;
    if(!seen.has(this)){ seen.add(this);
      this.addEventListener('message', function(ev){ try{
        var d = ev.data;
        if(d && d.type==='newMap' && d.data && d.data.directOutput!=null){
          var a = d.data.directOutput;
          a = (ArrayBuffer.isView(a)) ? Array.from(a) : Object.values(a);
          for(var i=0;i<a.length;i++) window.__buf.push(a[i]);
          if(window.__buf.length > 800000) window.__buf.splice(0, window.__buf.length-800000);
        }
      }catch(e){} });
    }
    return o.apply(this, args);
  };
  return 'installed';
})();
"""
POLL_JS = "(function(){var b=window.__buf||[];window.__buf=[];return JSON.stringify({buf:b,odom:(window.__odom||null)});})()"


def discover_ws():
    devs = requests.get(PROXY + "/json", timeout=5).json()
    if not devs:
        raise RuntimeError("No hay dispositivos. ¿iPhone conectado y ios_webkit_debug_proxy corriendo?")
    durl = devs[0]["url"]   # ej localhost:9222
    pages = requests.get(f"http://{durl}/json", timeout=5).json()
    if not pages:
        raise RuntimeError("No hay paginas inspeccionables. ¿App en la pantalla del SLAM? ¿Inspector de Safari CERRADO?")
    # elige la del SLAM si la hay
    page = next((p for p in pages if "slam" in (p.get("url", "").lower())), pages[0])
    print("Pagina:", page.get("title"), page.get("url"))
    return page["webSocketDebuggerUrl"]


class CDP:
    """Cliente para ios_webkit_debug_proxy: envuelve comandos en Target.sendMessageToTarget."""
    def __init__(self, url):
        self.ws = websocket.create_connection(url, max_size=None)
        self.id = 0
        self.target = None
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
                    self.target = ti["targetId"]
                    print("Target page:", self.target)
        if not self.target:
            raise RuntimeError("No llegó Target.targetCreated (¿pagina del SLAM abierta?)")
    def call(self, method, params=None, timeout=12, debug=False):
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
                im = msg["params"]["message"]
                im = json.loads(im) if isinstance(im, str) else im
                if debug:
                    print("  <<", str(im)[:200])
                if im.get("id") == iid:
                    if "error" in im:
                        raise RuntimeError(im["error"])
                    return im
        raise TimeoutError(method)
    def eval(self, expr):
        r = self.call("Runtime.evaluate", {"expression": expr, "returnByValue": True})
        res = (r.get("result") or {}).get("result") or {}
        return res.get("value")


def poller(url):
    cdp = CDP(url)
    try:
        cdp.call("Runtime.enable")
    except Exception as e:
        print("(Runtime.enable:", e, ")")
    print("Instalando gancho...", cdp.eval(HOOK_JS))
    while True:
        try:
            s = cdp.eval(POLL_JS)
            if s:
                obj = json.loads(s)
                flat = obj.get("buf") or []
                od = obj.get("odom")
                with lock:
                    for i in range(0, len(flat) - 2, 3):
                        x, y, z = flat[i], flat[i+1], flat[i+2]
                        acc[(round(x/VOX), round(y/VOX), round(z/VOX))] = (x, y, z)
                    if od:
                        x, y, z, qx, qy, qz, qw = od
                        yaw = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
                        robot.update({"x": x, "y": y, "z": z, "yaw": yaw})
                        if not traj or (abs(traj[-1][0]-x) + abs(traj[-1][1]-y)) > 0.01:
                            traj.append((x, y, z))
                    stat["polls"] += 1; stat["pts"] = len(acc)
            # frame de camara (mismo hilo/CDP: seguro)
            try:
                j = cdp.eval(CAM_JS)
                if j and j.startswith("data:image"):
                    with lock:
                        cam["jpg"] = j; cam["n"] += 1
            except Exception:
                pass
        except Exception as e:
            print("poll err:", e); time.sleep(1.0)
        time.sleep(0.4)


def yolo_worker():
    """Hilo: coge el ultimo frame de camara, corre YOLOv8n, dibuja cajas+labels -> cam['annot']."""
    import base64, io
    from ultralytics import YOLO
    from PIL import Image, ImageDraw
    print("Cargando YOLOv8n... (la 1a vez descarga ~6MB)")
    model = YOLO("yolov8n.pt")
    print("YOLO listo.")
    last = -1
    while True:
        with lock:
            jpg = cam["jpg"]; n = cam["n"]
        if not jpg or n == last:
            time.sleep(0.05); continue
        last = n
        try:
            raw = base64.b64decode(jpg.split(",", 1)[1])
            img = Image.open(io.BytesIO(raw)).convert("RGB")
            res = model.predict(img, imgsz=320, conf=0.40, verbose=False)[0]
            draw = ImageDraw.Draw(img)
            for b in res.boxes:
                x1, y1, x2, y2 = b.xyxy[0].tolist()
                name = model.names[int(b.cls[0])]; conf = float(b.conf[0])
                draw.rectangle([x1, y1, x2, y2], outline=(0, 255, 0), width=2)
                draw.text((x1 + 1, max(0, y1 - 11)), f"{name} {conf:.2f}", fill=(0, 255, 0))
            with lock:
                cam["annot"] = np.asarray(img); cam["an"] += 1
        except Exception as e:
            print("yolo err:", e); time.sleep(0.2)


def main():
    mode3d = "3d" in sys.argv              # 2D (planta) por defecto; pasa "3d" para vista 3D
    url = discover_ws()
    print("Inspector WS:", url)
    threading.Thread(target=poller, args=(url,), daemon=True).start()

    # ventana de camara (opcional, requiere Pillow)
    cam_show = "nocam" not in sys.argv
    figc = imart = None
    try:
        import base64, io
        from PIL import Image
    except ImportError:
        if cam_show:
            print("(camara desactivada: falta Pillow -> pip install Pillow. O usa 'nocam'.)")
        cam_show = False

    plt.ion()
    if cam_show:
        figc, axc = plt.subplots(figsize=(6, 3.6))
        axc.axis("off"); axc.set_title("G1 camara + YOLO (USB)")
        imart = axc.imshow(np.zeros((180, 320, 3), dtype=np.uint8))
        print("Ventana de camara abierta (pasa 'nocam' para desactivarla).")
        # YOLO opcional
        if "noyolo" not in sys.argv:
            try:
                import ultralytics  # noqa
                threading.Thread(target=yolo_worker, daemon=True).start()
            except ImportError:
                print("(YOLO desactivado: pip install ultralytics. O usa 'noyolo'.)")

    if mode3d:
        from mpl_toolkits.mplot3d import Axes3D  # noqa
        fig = plt.figure(figsize=(10, 9))
        ax = fig.add_subplot(111, projection="3d")
        ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.set_zlabel("z (m)")
        sc = ax.scatter([], [], [], s=2)
        print("Ventana 3D abierta. ARRASTRA para rotar, rueda para zoom. Pasea el robot. Ctrl+C salir.")
    else:
        fig, ax = plt.subplots(figsize=(10, 10))
        ax.set_aspect("equal"); ax.grid(True, alpha=0.3)
        ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
        sc = ax.scatter([], [], s=2)
        print("Ventana (planta) abierta. Pasea el robot. Ctrl+C salir.")

    # artistas del robot: BOLA roja = posicion actual; LINEA roja = path recorrido
    if mode3d:
        robot_sc = ax.scatter([], [], [], c="red", s=180, marker="o",
                              edgecolors="black", linewidths=0.5, depthshade=False, zorder=10)
        traj_line, = ax.plot([], [], [], "-", color="red", lw=2.5, zorder=9)
    else:
        robot_sc = ax.scatter([], [], c="red", s=180, marker="o",
                              edgecolors="black", linewidths=0.5, zorder=10)
        traj_line, = ax.plot([], [], "-", color="red", lw=2.5, zorder=9)

    MAXP = 40000          # tope de puntos a dibujar (rendimiento del 3D)
    last = -1; cam_last = -1
    try:
        while plt.fignum_exists(fig.number):
            if cam_show and figc is not None and plt.fignum_exists(figc.number):
                with lock:
                    annot = cam["annot"]; an = cam["an"]; jpg = cam["jpg"]; cn = cam["n"]
                img = None
                if annot is not None and an != cam_last:           # frame anotado por YOLO
                    img = annot; cam_last = an
                elif annot is None and jpg and cn != cam_last:     # sin YOLO aun -> crudo
                    try:
                        raw = base64.b64decode(jpg.split(",", 1)[1])
                        img = np.asarray(Image.open(io.BytesIO(raw)).convert("RGB")); cam_last = cn
                    except Exception:
                        img = None
                if img is not None:
                    imart.set_data(img); figc.canvas.draw_idle()
            with lock:
                pts = list(acc.values()); pl = stat["polls"]
            if pts and len(pts) != last:                 # redibuja solo si hay puntos nuevos
                last = len(pts)
                a = np.array(pts)
                if len(a) > MAXP:                        # submuestreo para fluidez
                    a = a[np.random.choice(len(a), MAXP, replace=False)]
                z = a[:, 2]
                if mode3d:
                    sc._offsets3d = (a[:, 0], a[:, 1], a[:, 2])
                    sc.set_array(z); sc.set_clim(z.min(), z.max())
                    ax.set_xlim(a[:, 0].min(), a[:, 0].max())
                    ax.set_ylim(a[:, 1].min(), a[:, 1].max())
                    ax.set_zlim(z.min(), z.max())
                    rx = max(np.ptp(a[:, 0]), 0.1); ry = max(np.ptp(a[:, 1]), 0.1); rz = max(np.ptp(z), 0.1)
                    ax.set_box_aspect((rx, ry, rz))      # proporciones reales (no toca el angulo de vista)
                else:
                    sc.set_offsets(a[:, :2]); sc.set_array(z); sc.set_clim(z.min(), z.max())
                    ax.set_xlim(a[:, 0].min()-0.5, a[:, 0].max()+0.5)
                    ax.set_ylim(a[:, 1].min()-0.5, a[:, 1].max()+0.5)
                ax.set_title(f"G1 LiDAR EN VIVO (USB) — {len(pts)} puntos | polls: {pl}")
            # robot (pose) + estela: actualiza cada frame aunque no haya puntos nuevos
            with lock:
                rb = dict(robot); tj = list(traj)
            if rb:
                if mode3d:
                    robot_sc._offsets3d = ([rb["x"]], [rb["y"]], [rb["z"]])
                    if tj:
                        traj_line.set_data([p[0] for p in tj], [p[1] for p in tj])
                        traj_line.set_3d_properties([p[2] for p in tj])
                else:
                    robot_sc.set_offsets([[rb["x"], rb["y"]]])
                    if tj:
                        traj_line.set_data([p[0] for p in tj], [p[1] for p in tj])
            plt.pause(0.2)
    except KeyboardInterrupt:
        pass
    print("\nFin.")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print("Error:", repr(e))
        sys.exit(1)
