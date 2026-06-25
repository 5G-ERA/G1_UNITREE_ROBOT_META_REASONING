#!/usr/bin/env python3
"""
g1_nav.py  -  FASE 1: captura (nube+odom) + control (teleop) UNIFICADOS en un proceso,
              por el datachannel de la app (inspector USB). Primer lazo cerrado.

Junta lo de g1_inspector_bridge (captura odom+nube hookeando la WebView) con lo de
g1_inject_teleop (driver en-pagina a 20Hz que inyecta rt/wirelesscontroller). Con eso ya
podemos leer odometria y mandar velocidad EN EL MISMO bucle -> base de la navegacion.

PRE (igual que siempre):
  - ios_webkit_debug_proxy corriendo (otra terminal). iPhone con la app conectada al robot,
    de pie, en la pantalla de SLAM/mapa (odom fluye ahi). Inspector de Safari CERRADO.
  - pip install websocket-client requests
  - SEGURIDAD: espacio libre, MANDO en la mano (L2+B = stop), valores moderados, distancias cortas.
    Hay deadzone: por debajo de ~0.3 el robot no se mueve; la app usa ~0.5-0.73.

USO:
  python g1_nav.py watch                 # solo lee: muestra odom (x,y,yaw) y nº de puntos en vivo
  python g1_nav.py forward 0.5           # avanza 0.5 m (medido por odom) y para
  python g1_nav.py forward 0.5 0.4       # 0.5 m a velocidad ly=0.4
  python g1_nav.py turn 90               # gira 90° (positivo=izquierda/CCW)
  python g1_nav.py goto 1.0 0.5          # va al punto (x=1.0, y=0.5) en frame map: gira+avanza+para
  python g1_nav.py gorel 1.2 0           # objetivo RELATIVO sin esquiva (para con obstaculo)
  python g1_nav.py nav 1.5 0.0           # A->B CON ESQUIVA reactiva (frame map)
  python g1_nav.py navrel 2.0 0          # A->B con esquiva, objetivo relativo (2 m adelante)
  python g1_nav.py explore 60            # WANDER 60s para autonomous mapping (Ctrl+C = stop)
  python g1_nav.py probe                 # micro-pulsos por eje + delta de odom -> descubre signos
"""
import json, sys, time, math, threading, os, random
import requests
import websocket  # websocket-client

LOGPATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "g1_nav.log")

# ---- VISION (cámara + YOLO) como capa de obstáculos complementaria al láser ----
vlock = threading.Lock()
vision = {"jpg": None, "n": 0, "block": False, "label": "", "side": 1, "cf": 1.0, "ts": 0}
CAM_JS = (
    "(function(){var v=document.querySelector('video');"
    "if(!v||!v.videoWidth)return '';"
    "var W=320,H=Math.round(W*v.videoHeight/v.videoWidth);"
    "var c=window.__camc||(window.__camc=document.createElement('canvas'));"
    "c.width=W;c.height=H;c.getContext('2d').drawImage(v,0,0,W,H);"
    "try{return c.toDataURL('image/jpeg',0.5);}catch(e){return '';}})()"
)


FLOOR_MIN = 0.10      # fraccion minima de suelo en el centro; muy bajo -> camara solo bloquea si esta casi todo tapado
EDGE_RUN = 5          # columnas seguidas con el suelo interrumpido cerca = pata/objeto fino delante
DEPTH_TH = 0.63       # umbral de profundidad MiDaS (suelo limpio ~0.5; obstaculos reales 0.66+; 0.55 bloqueaba de mas)
# muebles/obstaculos que YOLO conoce -> tratarlos como obstaculo aunque la caja sea menor
YOLO_FURNITURE = {"diningtable", "table", "chair", "couch", "bed", "bench", "refrigerator", "oven",
                  "sink", "suitcase", "tvmonitor", "tv", "microwave", "toilet", "pottedplant",
                  "vase", "backpack", "handbag", "boat", "bicycle", "motorcycle"}
SAT_FLOOR_MAX = 45    # suelo = poco saturado (moqueta gris S~5); obstaculos saturados (S>100)


def floor_free_bands(img):
    """Suelo = pixeles PARECIDOS a la moqueta de los pies (modelo ADAPTATIVO: aprende S y V de la franja
    inferior cada frame). Asi se adapta a la luz y excluye paredes/muebles BLANCOS (mas brillantes) o
    saturados. Devuelve fraccion de suelo izq/centro/dcha, refS (diag) y near_run (patas finas delante)."""
    import numpy as np
    hsv = np.asarray(img.convert("HSV")).astype(int)   # H,S,V en 0..255
    H, W = hsv.shape[:2]
    S = hsv[:, :, 1]; V = hsv[:, :, 2]
    strip = hsv[int(H * 0.88):, :]                      # moqueta justo a los pies (franja inferior ancha)
    refS = float(np.median(strip[:, :, 1])); refV = float(np.median(strip[:, :, 2]))
    floor = (np.abs(S - refS) < 42) & (np.abs(V - refV) < 55)   # parecido a la moqueta en saturacion y brillo
    lo = int(H * 0.45)                                  # mitad inferior (lo cercano)

    def frac(c1, c2):
        return float(floor[lo:, int(W * c1):int(W * c2)].mean())

    # EDGE: por columna del centro, altura de suelo CONTIGUO desde abajo; si se interrumpe en el 30%
    # inferior (obstaculo cerca), marca la columna. near_run = mayor racha de columnas marcadas.
    cen = floor[:, int(W * 0.40):int(W * 0.60)]
    cont = np.cumprod(cen[::-1, :], axis=0)             # 1 hasta el primer no-suelo desde abajo
    colh = cont.sum(axis=0) / H                         # fraccion de suelo contiguo por columna
    low = colh < 0.30
    run = mx = 0
    for v in low:
        run = run + 1 if v else 0
        if run > mx:
            mx = run
    return frac(0.0, 0.35), frac(0.35, 0.65), frac(0.65, 1.0), refS, int(mx)


def save_crash_image(cdp, n, x, y, yaw, c0):
    """En cada colision: captura un frame FRESCO de la camara y lo guarda en G1 ROBOT/crashes/
    junto a un .txt con lo que cada sensor reporto (para mejorar la vision a futuro: ver con QUE
    chocó y POR QUE no lo detecto -> ajustar YOLO/segmentacion de suelo)."""
    import base64
    # frame fresco; si falla, el ultimo cacheado
    cj = None
    try:
        j = cdp.eval(CAM_JS)
        if j and j.startswith("data:image"):
            cj = j
    except Exception:
        pass
    if not cj:
        with vlock:
            cj = vision.get("jpg")
    if not cj or not cj.startswith("data:image"):
        print("    (sin frame de camara para la colision; ¿camara activa en la app?)"); return
    try:
        cdir = os.path.join(os.path.dirname(LOGPATH), "crashes")
        os.makedirs(cdir, exist_ok=True)
        base = os.path.join(cdir, f"crash_{n:02d}_{time.strftime('%H%M%S')}_x{x:+.2f}_y{y:+.2f}")
        with open(base + ".jpg", "wb") as f:
            f.write(base64.b64decode(cj.split(",", 1)[1]))
        with vlock:
            lf = vision.get("lf", 0); cf = vision.get("cf", 0); rf = vision.get("rf", 0)
            vlb = vision.get("label", ""); vblk = vision.get("block", False); vdr = vision.get("dratio", 0)
        with open(base + ".txt", "w") as f:
            f.write(f"colision #{n}  {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"pose: x={x:+.2f} y={y:+.2f} yaw={yaw:+.0f}\n")
            f.write(f"LASER frente (rejilla): c0={'libre' if c0>900 else f'{c0:.2f} m'}  <- {'NO lo vio' if c0>900 else 'lo veia'}\n")
            f.write(f"CAMARA suelo: izq={lf:.2f} centro={cf:.2f} dcha={rf:.2f}  block={vblk} label='{vlb}'  depth_ratio={vdr}\n")
            f.write("nota: si laser=NO lo vio y camara centro alto -> obstaculo que ambos sensores fallaron "
                    "(cristal/metal/frente sin mapear). Usar esta imagen para ajustar la deteccion.\n")
        print(f"    colision guardada -> {base}.jpg (+ .txt con contexto)")
    except Exception as e:
        print("    (no pude guardar la colision:", e, ")")


def inject_obstacle(cdp, x, y, yaw_deg):
    """Tras una colision: marca en la rejilla el obstaculo que el laser NO vio (mesa, etc.), justo
    delante del robot, para que lo RECUERDE y no vuelva a comerselo (la colision enseña el mapa)."""
    yr = math.radians(yaw_deg)
    fx = math.cos(yr); fz = -math.sin(yr)       # forward en plano nube (CLOUD_SIGN=-1)
    perpx = -fz; perpz = fx                      # perpendicular
    rcx = x; rcz = -y                            # robot en plano nube
    cells = set()
    for d in (0.30, 0.40, 0.50, 0.60):
        for L in (-0.25, -0.12, 0.0, 0.12, 0.25):
            ox = rcx + d * fx + L * perpx
            oz = rcz + d * fz + L * perpz
            cells.add((round(ox * 10), round(oz * 10)))
    js = ("(function(){var g=window.__grid||(window.__grid={});"
          + "".join(f"g['{a},{b}']=30;" for a, b in cells) + "return 1;})()")
    try:
        cdp.eval(js)
    except Exception:
        pass


def yolo_worker():
    """Hilo de VISION: segmentacion de suelo (azul) SIEMPRE + YOLO si esta disponible.
    Marca vision['block'] si la camara ve un obstaculo delante, y vision['side'] hacia donde
    hay mas suelo libre (para girar bien)."""
    import base64, io
    import numpy as np
    from PIL import Image
    dev = "cpu"                                            # usa la GPU del Mac (MPS) si esta -> mucho mas rapido
    try:
        import torch
        if torch.backends.mps.is_available():
            dev = "mps"
    except Exception:
        pass
    print("Dispositivo de vision:", dev)
    model = None
    try:
        from ultralytics import YOLO
        print("Cargando YOLOv8s para vision... (1a vez descarga ~22MB; detecta muebles mucho mejor)")
        model = YOLO("yolov8s.pt"); print("YOLO listo (v8s).")
    except Exception:
        print("(YOLO no disponible; uso solo segmentacion de suelo. pip install ultralytics para añadirlo.)")
    # MiDaS: profundidad monocular -> distancia de CUALQUIER cosa (pizarra/cristal/mismo color) antes de llegar
    midas = None; mid_tf = None
    if "nodepth" not in sys.argv:
        try:
            import torch
            print("Cargando MiDaS (profundidad)... (1a vez descarga ~80MB, tarda)")
            _hub_load = torch.hub.load                          # parche: auto-confiar TODO (tb. lo anidado)
            def _trusted_load(*a, **k):
                k.setdefault("trust_repo", True)
                return _hub_load(*a, **k)
            torch.hub.load = _trusted_load
            midas = torch.hub.load("intel-isl/MiDaS", "MiDaS_small"); midas.to(dev).eval()
            mid_tf = torch.hub.load("intel-isl/MiDaS", "transforms").small_transform
            torch.hub.load = _hub_load                          # restaura
            print("MiDaS listo (profundidad activa, dev=" + dev + ").")
        except Exception as e:
            print("(MiDaS no disponible:", repr(e), "-> sigo sin profundidad. pip install timm)"); midas = None
    print("VISION activa (suelo" + (" + YOLO" if model else "") + (" + MiDaS" if midas else "") + ").")
    last = -1; smooth = []; dsmooth = []; tlast = 0.0
    while True:
        with vlock:
            jpg = vision["jpg"]; n = vision["n"]
        if not jpg or n == last:
            time.sleep(0.05); continue
        last = n
        _tf0 = time.time()
        try:
            raw = base64.b64decode(jpg.split(",", 1)[1])
            img = Image.open(io.BytesIO(raw)).convert("RGB")
            W, H = img.size
            blk = False; lbl = ""; close = False; dist = ""
            # 1) suelo por color + EDGE (patas finas) (con suavizado temporal de 4 frames)
            lf0, cf0, rf0, refS, nrun = floor_free_bands(img)
            smooth.append((lf0, cf0, rf0)); smooth = smooth[-4:]
            lf = sum(s[0] for s in smooth) / len(smooth)
            cf = sum(s[1] for s in smooth) / len(smooth)
            rf = sum(s[2] for s in smooth) / len(smooth)
            if cf < FLOOR_MIN:
                blk = True; lbl = "floor"
            elif nrun >= EDGE_RUN:                     # pata/objeto fino delante (el suelo-frac no lo pilla)
                blk = True; lbl = "edge"; dist = "medio"
            if cf < 0.10 or nrun >= EDGE_RUN + 5:       # casi nada de suelo / pata ancha -> PEGADO
                close = True
            side = 1 if lf >= rf else -1                # +1 = mas suelo a la izquierda
            # 2) YOLO. Muebles = obstaculo -> dispara con caja MENOR y mas a los lados (mesa, silla...)
            if model is not None:
                res = model.predict(img, imgsz=384, conf=0.25, verbose=False, device=dev)[0]
                for bx in res.boxes:
                    x1, y1, x2, y2 = bx.xyxy[0].tolist()
                    cx = (x1 + x2) / 2 / W; bh = (y2 - y1) / H
                    area = ((x2 - x1) / W) * bh; bottom = y2 / H
                    name = res.names[int(bx.cls[0])]
                    furn = name.replace(" ", "") in YOLO_FURNITURE
                    if (furn and 0.18 < cx < 0.82 and area > 0.05) or \
                       (0.25 < cx < 0.75 and (area > 0.12 or (bh > 0.45 and bottom > 0.6))):
                        blk = True; lbl = name
                        # distancia por la caja: base baja en el frame / area grande -> mas cerca
                        dist = "cerca" if (bottom > 0.82 or area > 0.22) else \
                               ("medio" if (bottom > 0.62 or area > 0.10) else "lejos")
                        if dist == "cerca":
                            close = True
                        break
            # 3) MiDaS profundidad: ¿hay superficie VERTICAL cerca delante? (pilla pizarra/cristal/mismo color)
            if midas is not None:
                try:
                    import torch
                    arr = np.asarray(img)                      # RGB HWC
                    inp = mid_tf(arr).to(dev)
                    with torch.no_grad():
                        pr = midas(inp)
                        dd = pr.squeeze().cpu().numpy()        # resolucion nativa de MiDaS (sin upsample -> mas rapido)
                    dn = (dd - dd.min()) / (dd.max() - dd.min() + 1e-6)   # 0..1 (1 = mas cerca)
                    Hd, Wd = dn.shape
                    near_ref = float(np.median(dn[int(Hd*0.85):, int(Wd*0.30):int(Wd*0.70)]))  # suelo a los pies (cerca)
                    midv = float(np.median(dn[int(Hd*0.33):int(Hd*0.50), int(Wd*0.38):int(Wd*0.62)]))  # frente, mas arriba (mas lejos si despejado)
                    ratio = midv / (near_ref + 1e-6)           # ~1 = algo tan cerca como el suelo de los pies -> pared
                    dsmooth.append(ratio); dsmooth = dsmooth[-3:]
                    ratio = sum(dsmooth) / len(dsmooth)        # suavizado 3 frames
                    vision["dratio"] = round(ratio, 2)
                    if ratio > DEPTH_TH:                       # superficie vertical cerca delante
                        blk = True
                        if not lbl:
                            lbl = "depth"
                        dist = "cerca" if ratio > DEPTH_TH + 0.12 else "medio"
                        if ratio > DEPTH_TH + 0.12:
                            close = True
                except Exception:
                    pass

            with vlock:
                vision["block"] = blk; vision["label"] = lbl; vision["side"] = side; vision["close"] = close
                vision["dist"] = dist
                vision["lf"] = lf; vision["cf"] = cf; vision["rf"] = rf
                vision["refS"] = refS; vision["ts"] = time.time()   # (bug arreglado: 'refH' ya no existe y abortaba el ts)
            dtf = time.time() - _tf0                          # tiempo por frame de vision
            with vlock:
                vision["dtf"] = round(dtf, 2)
            if time.time() - tlast > 5:
                print(f"  [vision {dtf:.2f}s/frame dev={dev}]"); tlast = time.time()
        except Exception:
            time.sleep(0.1)


def cmd_vsee():
    """Calibracion de VISION (read-only): muestra suelo libre izq/centro/dcha, bloqueo y etiqueta."""
    cdp = get_cdp()
    threading.Thread(target=yolo_worker, daemon=True).start()
    print("VSEE: apunta a suelo libre (cf alto) y a una mesa/obstaculo (cf baja, block). Ctrl+C salir.\n")
    try:
        while True:
            try:
                j = cdp.eval(CAM_JS)
                if j and j.startswith("data:image"):
                    with vlock:
                        vision["jpg"] = j; vision["n"] += 1
            except Exception:
                pass
            with vlock:
                lf = vision.get("lf", 0); cf = vision.get("cf", 0); rf = vision.get("rf", 0)
                blk = vision["block"]; lbl = vision["label"]; dr = vision.get("dratio", 0)
            bar = "BLOCK(" + lbl + ")" if blk else "libre"
            print(f"  suelo izq={lf:.2f} centro={cf:.2f} dcha={rf:.2f}  depth_ratio={dr}  -> {bar}")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nFin vsee.")

PROXY = "http://localhost:9221"

# ---- JS combinado: captura (nube+odom) + driver de teleop (20Hz, hombre-muerto) ----
INSTALL_JS = r"""
(function(){
  // ---------- CAPTURA ----------
  window.__buf = window.__buf || [];
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
  if(!window.__cloudHook){ window.__cloudHook = 1;
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
  }
  // ---------- REJILLA LIMPIA (hook independiente; se instala aunque el de la nube ya existiera) ----------
  if(!window.__gridHook){ window.__gridHook = 1;
    var seenG = new WeakSet();
    var og = Worker.prototype.postMessage;
    Worker.prototype.postMessage = function(m){
      var ag = arguments;
      if(!seenG.has(this)){ seenG.add(this);
        this.addEventListener('message', function(ev){ try{
          var d = ev.data;
          if(d && d.type==='newMap' && d.data && d.data.directOutput!=null){
            var a = d.data.directOutput;
            a = (ArrayBuffer.isView(a)) ? Array.from(a) : Object.values(a);
            var od = window.__odom;
            if(od){
              var g = window.__grid || (window.__grid = {});
              for(var dk in g){ if(--g[dk] <= 0) delete g[dk]; }   // DECAY: lo no re-visto se desvanece (mata el reguero)
              var rcx = od[0], rcz = -od[1];            // robot en plano nube (CLOUD_SIGN=-1)
              for(var j=0;j+2<a.length;j+=3){
                var px=a[j], ph=a[j+1], pz=a[j+2];
                if(ph < -0.5 || ph > 0.8) continue;      // banda de torso
                var ex=px-rcx, ez=pz-rcz;
                if(ex*ex+ez*ez < 0.20) continue;         // ignora <0.45m (cuerpo/suelo del cabeceo)
                var key = Math.round(px*10)+','+Math.round(pz*10);
                var c = (g[key]||0)+2; g[key] = c<8?c:8;  // +2 al re-ver; tope 8 (paredes persisten, reguero cae)
              }
            }
          }
        }catch(e){} });
      }
      return og.apply(this, ag);
    };
  }
  // ---------- CONTROL ----------
  if(!window.__dcHook){ window.__dcHook = 1;
    var S = RTCDataChannel.prototype.send;
    RTCDataChannel.prototype.send = function(d){
      try{ if((this.label||'')==='data') window.__dc = this; }catch(e){}
      return S.apply(this, arguments);
    };
  }
  if(!window.__drv){
    window.__cmd = {lx:0,ly:0,rx:0,ry:0}; window.__cmdTs = 0; window.__sent = 0;
    window.__send = function(c){
      if(!window.__dc) return;
      var msg = {type:'msg', topic:'rt/wirelesscontroller', data:{lx:c.lx, ly:c.ly, rx:c.rx, ry:c.ry}};
      try{ window.__dc.send(JSON.stringify(msg)); window.__sent++; }catch(e){}
    };
    window.__drv = setInterval(function(){
      var c = window.__cmd || {lx:0,ly:0,rx:0,ry:0};
      if(Date.now() - (window.__cmdTs||0) > 600){ c = {lx:0,ly:0,rx:0,ry:0}; }  // hombre-muerto
      window.__send(c);
    }, 50);
  }
  return JSON.stringify({dc: !!window.__dc, odom: !!window.__odom, buf: (window.__buf||[]).length});
})();
"""

POLL_JS = ("(function(){var b=window.__buf||[];var n=b.length;"
           "return JSON.stringify({odom:(window.__odom||null), npts:n, sent:(window.__sent||0), dc:!!window.__dc});})()")


def set_cmd_js(lx, ly, rx, ry):
    return ("(function(){window.__cmd={lx:%g,ly:%g,rx:%g,ry:%g};window.__cmdTs=Date.now();return 'ok';})()"
            % (lx, ly, rx, ry))


STOP_JS = "(function(){window.__cmd={lx:0,ly:0,rx:0,ry:0};window.__cmdTs=Date.now();return 'stop';})()"


def yaw_of(odom):
    x, y, z, qx, qy, qz, qw = odom
    return math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))


# ---- descubrimiento + CDP (igual que los otros scripts, protocolo Target) ----
def discover_ws():
    devs = requests.get(PROXY + "/json", timeout=5).json()
    if not devs:
        raise RuntimeError("No hay dispositivos. ¿iPhone conectado y ios_webkit_debug_proxy corriendo?")
    durl = devs[0]["url"]
    pages = requests.get(f"http://{durl}/json", timeout=5).json()
    if not pages:
        raise RuntimeError("No hay paginas. ¿App en SLAM? ¿Inspector de Safari CERRADO?")
    def score(p):
        u = (p.get("url", "") + " " + p.get("title", "")).lower()
        return ("b2app" in u) * 4 + ("8084" in u) * 3 + ("slam" in u) * 2 + 1
    page = sorted(pages, key=score, reverse=True)[0]
    print("Pagina:", page.get("title"), page.get("url"))
    return page["webSocketDebuggerUrl"]


class CDP:
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
            raise RuntimeError("No llegó Target.targetCreated (¿pagina de la app abierta?)")

    def call(self, method, params=None, timeout=12):
        self.id += 1
        iid = self.id
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
                if im.get("id") == iid:
                    if "error" in im:
                        raise RuntimeError(im["error"])
                    return im
        raise TimeoutError(method)

    def eval(self, expr):
        r = self.call("Runtime.evaluate", {"expression": expr, "returnByValue": True})
        res = (r.get("result") or {}).get("result") or {}
        return res.get("value")


def get_cdp():
    url = discover_ws()
    print("Inspector WS:", url)
    cdp = CDP(url)
    try:
        cdp.call("Runtime.enable")
    except Exception:
        pass
    print("Instalando captura+driver:", cdp.eval(INSTALL_JS))
    return cdp


def read_poll(cdp):
    s = cdp.eval(POLL_JS)
    return json.loads(s) if s else {}


def wait_for_odom(cdp, timeout=6):
    end = time.time() + timeout
    while time.time() < end:
        p = read_poll(cdp)
        if p.get("odom"):
            return p["odom"]
        time.sleep(0.3)
    return None


# ---------------- modos ----------------
def cmd_watch():
    cdp = get_cdp()
    print("Leyendo (Ctrl+C para salir). Pasea el robot con la app para ver cambiar la odom.\n")
    try:
        while True:
            p = read_poll(cdp)
            od = p.get("odom")
            if od:
                print(f"  x={od[0]:+.2f} y={od[1]:+.2f} yaw={math.degrees(yaw_of(od)):+6.1f}°  "
                      f"pts={p.get('npts',0)}  dc={p.get('dc')}")
            else:
                print(f"  (sin odom todavia)  pts={p.get('npts',0)}  dc={p.get('dc')}")
            time.sleep(0.4)
    except KeyboardInterrupt:
        print("\nFin.")


def cmd_forward(meters, speed):
    meters = max(0.05, min(2.0, meters))     # tope de seguridad
    speed = max(0.3, min(0.6, speed))        # deadzone ~0.3
    cdp = get_cdp()
    od0 = wait_for_odom(cdp)
    if not od0:
        print("!! No llega odometria. ¿Estas en la pantalla de SLAM/mapa con mapeo activo?")
        return
    x0, y0 = od0[0], od0[1]
    print(f">>> AVANZAR {meters:.2f} m a ly={speed:.2f} (Ctrl+C = stop).  inicio x={x0:+.2f} y={y0:+.2f}")
    tmax = meters / 0.15 + 6.0              # tope de tiempo generoso (vel real desconocida)
    t0 = time.time()
    try:
        while True:
            p = read_poll(cdp)
            od = p.get("odom") or od0
            d = math.hypot(od[0]-x0, od[1]-y0)
            if d >= meters:
                print(f"  llegado: {d:.2f} m"); break
            if time.time() - t0 > tmax:
                print(f"  (timeout de seguridad a {d:.2f} m)"); break
            cdp.eval(set_cmd_js(0, speed, 0, 0))   # ly adelante
            time.sleep(0.1)
    except KeyboardInterrupt:
        print(" [interrumpido]")
    finally:
        cdp.eval(STOP_JS); time.sleep(0.3); cdp.eval(STOP_JS)
        p = read_poll(cdp); od = p.get("odom") or od0
        print(f"STOP. recorrido ~{math.hypot(od[0]-x0, od[1]-y0):.2f} m")


def scan_js(zlo=-9.9, zhi=9.9, fmax=2.0, half=0.40):
    """Expresion JS: escanea window.__buf en el corredor frontal (frame robot) y devuelve resumen.
    Calcula EN LA PAGINA (no saca los 800k puntos)."""
    return ("(function(){var od=window.__odom;var b=window.__buf||[];"
            "if(!od||b.length<3)return JSON.stringify({ok:false});"
            "var rx=od[0],ry=od[1],qx=od[3],qy=od[4],qz=od[5],qw=od[6];"
            "var yaw=Math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz));"
            "var c=Math.cos(yaw),s=Math.sin(yaw);"
            "var HALF=%g,FMAX=%g,ZLO=%g,ZHI=%g;"
            "var near=999,cnt=0,zmin=999,zmax=-999;"
            "for(var i=0;i+2<b.length;i+=3){var dx=b[i]-rx,dy=b[i+1]-ry,z=b[i+2];"
            "var f=dx*c+dy*s,l=-dx*s+dy*c;"
            "if(f>0.15&&f<FMAX&&Math.abs(l)<HALF){if(z<zmin)zmin=z;if(z>zmax)zmax=z;"
            "if(z>ZLO&&z<ZHI){cnt++;if(f<near)near=f;}}}"
            "return JSON.stringify({ok:true,near:near,cnt:cnt,zmin:zmin,zmax:zmax,robz:od[2],n:b.length});})()"
            % (half, fmax, zlo, zhi))


def cmd_scan():
    cdp = get_cdp()
    if not wait_for_odom(cdp):
        print("!! Sin odometria."); return
    print("SCAN read-only. Corredor: 0.15-2.0 m delante, |lateral|<0.40 m. SIN filtro z.")
    print("Apunta el robot a una pared (debe salir 'near' pequeño) y a hueco abierto (near alto/none).\n")
    try:
        while True:
            r = json.loads(cdp.eval(scan_js()))
            if not r.get("ok"):
                print("  (sin datos)"); time.sleep(0.5); continue
            near = r["near"]
            ns = f"{near:.2f} m" if near < 900 else "—"
            print(f"  cerca={ns:>7}  pts_corredor={r['cnt']:5d}  z=[{r['zmin']:+.2f},{r['zmax']:+.2f}]  "
                  f"robz={r['robz']:+.2f}")
            time.sleep(0.4)
    except KeyboardInterrupt:
        print("\nFin scan.")


ZHIST_JS = ("(function(){var od=window.__odom;var b=window.__buf||[];"
            "if(!od||b.length<3)return JSON.stringify({ok:false});"
            "var rx=od[0],ry=od[1],qx=od[3],qy=od[4],qz=od[5],qw=od[6];"
            "var yaw=Math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz));"
            "var c=Math.cos(yaw),s=Math.sin(yaw);"
            "var LO=-3.5,STEP=0.5,NB=17;var h=[];for(var k=0;k<NB;k++)h.push(0);"
            "for(var i=0;i+2<b.length;i+=3){var dx=b[i]-rx,dy=b[i+1]-ry,z=b[i+2];"
            "var f=dx*c+dy*s,l=-dx*s+dy*c;"
            "if(f>0.15&&f<2.0&&Math.abs(l)<0.40){var bi=Math.floor((z-LO)/STEP);if(bi>=0&&bi<NB)h[bi]++;}}"
            "return JSON.stringify({ok:true,lo:LO,step:STEP,h:h,robz:od[2]});})()")


def cmd_zhist():
    cdp = get_cdp()
    if not wait_for_odom(cdp):
        print("!! Sin odometria."); return
    print("Histograma de alturas (z) en el corredor frontal. Apunta a una PARED cercana.\n")
    try:
        for _ in range(6):
            r = json.loads(cdp.eval(ZHIST_JS))
            if not r.get("ok"):
                time.sleep(0.5); continue
            h = r["h"]; lo = r["lo"]; step = r["step"]; mx = max(h) or 1
            print(f"--- robz={r['robz']:+.2f} ---")
            for k, c in enumerate(h):
                z0 = lo + k * step
                bar = "#" * int(40 * c / mx)
                print(f"  z[{z0:+.1f},{z0+step:+.1f}) {c:6d} {bar}")
            print()
            time.sleep(0.8)
    except KeyboardInterrupt:
        pass
    print("Fin zhist.")


BBOX_JS = ("(function(){var od=window.__odom;var b=window.__buf||[];"
           "if(!od||b.length<3)return JSON.stringify({ok:false,n:b.length});"
           "var xmn=1e9,xmx=-1e9,ymn=1e9,ymx=-1e9,zmn=1e9,zmx=-1e9;"
           "for(var i=0;i+2<b.length;i+=3){var x=b[i],y=b[i+1],z=b[i+2];"
           "if(x<xmn)xmn=x;if(x>xmx)xmx=x;if(y<ymn)ymn=y;if(y>ymx)ymx=y;if(z<zmn)zmn=z;if(z>zmx)zmx=z;}"
           "return JSON.stringify({ok:true,n:Math.round(b.length/3),x:[xmn,xmx],y:[ymn,ymx],z:[zmn,zmx],"
           "rob:[od[0],od[1],od[2]]});})()")


def cmd_bbox():
    cdp = get_cdp()
    if not wait_for_odom(cdp):
        print("!! Sin odometria."); return
    print("BBOX de toda la nube vs pose del robot (¿estable? ¿robot dentro?)\n")
    try:
        for _ in range(10):
            r = json.loads(cdp.eval(BBOX_JS))
            if not r.get("ok"):
                print("  (sin nube)  n=", r.get("n")); time.sleep(0.5); continue
            x, y, z, rb = r["x"], r["y"], r["z"], r["rob"]
            print(f"  n={r['n']:6d}  x[{x[0]:+.2f},{x[1]:+.2f}] y[{y[0]:+.2f},{y[1]:+.2f}] "
                  f"z[{z[0]:+.2f},{z[1]:+.2f}]  rob=({rb[0]:+.2f},{rb[1]:+.2f},{rb[2]:+.2f})")
            time.sleep(0.7)
    except KeyboardInterrupt:
        pass
    print("Fin bbox.")


CLOUD_SIGN = -1       # idx2 = CLOUD_SIGN*odom_y (verificado: A = -1)
# Banda de altura (idx1) para "obstaculo": sensor en 0, suelo ~ -1.3, techo ~ +1.35.
# Rebanada de torso: excluye suelo (anillo falso) y techo. Calibrable con el modo 'clr'.
HBAND_LO = -0.50    # subido (era -0.80) para no colar el suelo por el cabeceo; bajos -> YOLO+colision
HBAND_HI = 0.80


FMIN = 0.20      # ignora celdas a menos de esto (ya filtramos <0.45m al construir la rejilla)
MINHITS = 2      # 2 impactos: filtra el anillo fantasma del cabeceo (con 1 reaparece)


def scan2_js(sign=CLOUD_SIGN, hlo=None, hhi=None, fmax=2.5, half=0.35, yoff=0.0, minbin=None):
    """Consulta la REJILLA LIMPIA window.__grid (celdas de 10cm con conteo de impactos; ya excluye
    el campo cercano <0.6m al construirse). near = celda persistente (>=MINHITS) mas cercana en el
    corredor frontal. yoff = offset de rumbo (rad) para mirar a los lados sin girar."""
    return ("(function(){var od=window.__odom;var g=window.__grid||{};"
            "if(!od)return JSON.stringify({ok:false});"
            "var qx=od[3],qy=od[4],qz=od[5],qw=od[6];"
            "var yaw=Math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))+(%g);"
            "var SG=%g;var rcx=od[0],rcz=SG*od[1];"
            "var fx=Math.cos(yaw),fz=SG*Math.sin(yaw);"
            "var HALF=%g,FMAX=%g,FMIN=%g,MH=%d;"
            "var near=999,cnt=0;"
            "for(var k in g){if(g[k]<MH)continue;"
            "var p=k.split(',');var cx=p[0]/10,cz=p[1]/10;"
            "var dx=cx-rcx,dz=cz-rcz;var f=dx*fx+dz*fz;"
            "if(f>FMIN&&f<FMAX){var lat=Math.sqrt(Math.max(0,dx*dx+dz*dz-f*f));"
            "if(lat<HALF){cnt++;if(f<near)near=f;}}}"
            "return JSON.stringify({ok:true,near:near,cnt:cnt});})()"
            % (yoff, sign, half, fmax, FMIN, MINHITS))


def clear_ahead(cdp, off_deg=0.0):
    """Distancia al obstaculo mas cercano en el corredor (con offset de rumbo en grados). 999 si libre."""
    try:
        r = json.loads(cdp.eval(scan2_js(yoff=math.radians(off_deg))))
        return r.get("near", 999) if r.get("ok") else 999
    except Exception:
        return 999


def cmd_clr():
    """Calibracion read-only: holgura en frente/izq/dcha con la banda de altura actual.
    En espacio libre debe salir todo '—' (libre); frente a pared, la distancia real."""
    cdp = get_cdp()
    if not wait_for_odom(cdp):
        print("!! Sin odometria."); return
    print(f"CLR (banda altura idx1 [{HBAND_LO},{HBAND_HI}]). Libre = '—'.")
    print("Prueba: (1) en medio del cuarto SIN nada cerca -> todo '—'. (2) frente a pared -> dist real.\n")
    def s(v):
        return f"{v:.2f}" if v < 900 else "  — "
    try:
        while True:
            c0 = clear_ahead(cdp, 0)
            cl = clear_ahead(cdp, +AV_OFF)
            cr = clear_ahead(cdp, -AV_OFF)
            cll = clear_ahead(cdp, +70)
            crr = clear_ahead(cdp, -70)
            try:
                gn = int(cdp.eval("Object.keys(window.__grid||{}).length") or 0)
            except Exception:
                gn = -1
            print(f"  izq70={s(cll)}  izq30={s(cl)}  FRENTE={s(c0)}  dcha30={s(cr)}  dcha70={s(crr)}   celdas={gn}")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nFin clr.")


def cmd_scan2():
    sign = -1
    if len(sys.argv) > 2 and sys.argv[2].upper() == "B":
        sign = 1
    cdp = get_cdp()
    if not wait_for_odom(cdp):
        print("!! Sin odometria."); return
    print(f"SCAN2 (hipotesis {'A' if sign<0 else 'B'}, altura=idx1 banda[-1.1,1.1]). "
          "Apunta a una PARED a distancia conocida.\n")
    try:
        while True:
            r = json.loads(cdp.eval(scan2_js(sign=sign)))
            if not r.get("ok"):
                print("  (sin datos)"); time.sleep(0.5); continue
            near = r["near"]; ns = f"{near:.2f} m" if near < 900 else "—"
            print(f"  cerca_delante={ns:>7}   pts={r['cnt']:5d}")
            time.sleep(0.4)
    except KeyboardInterrupt:
        print("\nFin scan2.")


def cmd_probe():
    """Micro-pulso por eje + delta de odom -> descubre signos/mapeo (forward/turn/strafe)."""
    cdp = get_cdp()
    if not wait_for_odom(cdp):
        print("!! Sin odometria; no puedo medir. Pantalla SLAM con mapeo."); return
    tests = [("ly+ (adelante?)", 0, 0.4, 0, 0), ("rx+ (giro?)", 0, 0, 0.5, 0), ("lx+ (lateral?)", 0.4, 0, 0, 0)]
    for name, lx, ly, rx, ry in tests:
        od0 = read_poll(cdp).get("odom")
        print(f"\n-- {name}: pulso 0.6s --")
        t0 = time.time()
        while time.time() - t0 < 0.6:
            cdp.eval(set_cmd_js(lx, ly, rx, ry)); time.sleep(0.1)
        cdp.eval(STOP_JS); time.sleep(1.0)
        od1 = read_poll(cdp).get("odom")
        if od0 and od1:
            dx, dy = od1[0]-od0[0], od1[1]-od0[1]
            dyaw = math.degrees(yaw_of(od1) - yaw_of(od0))
            print(f"   Δpos=({dx:+.2f},{dy:+.2f}) m  |Δ|={math.hypot(dx,dy):.2f}  Δyaw={dyaw:+.1f}°")
        time.sleep(0.5)
    cdp.eval(STOP_JS)
    print("\nProbe hecho.")


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


# Signo de giro (de probe): rx+ -> yaw BAJA. Para aumentar yaw (girar izq) -> rx negativo.
TURN_SPEED = 0.45     # > deadzone
FWD_SPEED = 0.40      # no adelantar a la percepcion (la rejilla del frente va con algo de retardo)
ALIGN_TOL = math.radians(15)   # alineado para empezar a andar
GO_BREAK = math.radians(35)    # si el rumbo se desvia mas que esto, vuelve a girar
GOAL_TOL = 0.20                # m (incluye margen de sobrepaso ~0.1)
STOP_DIST = 0.50               # m desde el SENSOR de cabeza (pie ~0.25 m por delante) -> holgura real ~0.25 m


def cmd_turn(deg):
    """Gira 'deg' grados (positivo = izquierda/CCW = aumentar yaw)."""
    cdp = get_cdp()
    od = wait_for_odom(cdp)
    if not od:
        print("!! Sin odometria."); return
    target = wrap(yaw_of(od) + math.radians(deg))
    print(f">>> GIRAR {deg:+.0f}° (Ctrl+C = stop)")
    t0 = time.time()
    try:
        while True:
            od = read_poll(cdp).get("odom") or od
            e = wrap(target - yaw_of(od))
            if abs(e) < math.radians(8):
                break
            if time.time() - t0 > 12:
                print("  (timeout giro)"); break
            cdp.eval(set_cmd_js(0, 0, -math.copysign(TURN_SPEED, e), 0))   # rx = -sign(e)*spd
            time.sleep(0.1)
    except KeyboardInterrupt:
        print(" [interrumpido]")
    finally:
        cdp.eval(STOP_JS); time.sleep(0.3); cdp.eval(STOP_JS)
        od = read_poll(cdp).get("odom") or od
        print(f"STOP. yaw final = {math.degrees(yaw_of(od)):+.1f}°")


def _run_goto(cdp, tx, ty, od):
    x0, y0 = od[0], od[1]
    d0 = math.hypot(tx - x0, ty - y0)
    if d0 > 3.0:
        print(f"!! Objetivo a {d0:.1f} m (>3 m). Por seguridad acércalo o ve por tramos."); return
    print(f">>> GOTO ({tx:+.2f},{ty:+.2f})  desde ({x0:+.2f},{y0:+.2f})  dist={d0:.2f} m  (Ctrl+C = stop)")
    t0 = time.time()
    try:
        while True:
            od = read_poll(cdp).get("odom") or od
            x, y, yaw = od[0], od[1], yaw_of(od)
            dx, dy = tx - x, ty - y
            dist = math.hypot(dx, dy)
            if dist < GOAL_TOL:
                print(f"  LLEGADO. dist={dist:.2f} m"); break
            if time.time() - t0 > 40:
                print(f"  (timeout a {dist:.2f} m)"); break
            e = wrap(math.atan2(dy, dx) - yaw)        # error de rumbo
            if abs(e) > ALIGN_TOL:
                cmd = (0, 0, -math.copysign(TURN_SPEED, e), 0)   # girar hacia el objetivo (seguro)
                ph = "TURN"
            else:
                near = clear_ahead(cdp)                          # ¿obstaculo delante?
                if near < STOP_DIST:
                    cdp.eval(STOP_JS)
                    print(f"\n  BLOQUEADO: obstaculo a {near:.2f} m. Paro (aun sin esquiva).")
                    break
                cmd = (0, FWD_SPEED, 0, 0)                        # avanzar recto
                ph = f"GO(libre {near:.1f}m)" if near < 900 else "GO"
            print(f"  {ph}: dist={dist:.2f} e={math.degrees(e):+5.0f}°    ", end="\r")
            cdp.eval(set_cmd_js(*cmd))
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n [interrumpido]")
    finally:
        cdp.eval(STOP_JS); time.sleep(0.3); cdp.eval(STOP_JS)
        od = read_poll(cdp).get("odom") or od
        print(f"\nSTOP. pos final=({od[0]:+.2f},{od[1]:+.2f})  resto={math.hypot(tx-od[0], ty-od[1]):.2f} m")


def cmd_goto(tx, ty):
    cdp = get_cdp()
    od = wait_for_odom(cdp)
    if not od:
        print("!! Sin odometria. ¿Pantalla SLAM con mapeo?"); return
    _run_goto(cdp, tx, ty, od)


def cmd_gorel(fwd, left):
    """Objetivo relativo a la pose ACTUAL: fwd adelante, left a la izquierda (frame robot)."""
    cdp = get_cdp()
    od = wait_for_odom(cdp)
    if not od:
        print("!! Sin odometria."); return
    x, y, yaw = od[0], od[1], yaw_of(od)
    tx = x + fwd * math.cos(yaw) - left * math.sin(yaw)
    ty = y + fwd * math.sin(yaw) + left * math.cos(yaw)
    _run_goto(cdp, tx, ty, od)


AVOID_TRIG = 0.55  # empieza a esquivar si el frente < esto
GO_RESUME = 0.85   # deja de esquivar solo cuando el frente > esto (histeresis)
CLOSE_DIST = 0.40  # si el obstaculo < esto, girar no saca el punto del corredor
REAR_SAFE = 0.60   # solo retrocede si DETRAS hay al menos esto de hueco
BACK_SPEED = 0.40
AV_OFF = 30.0      # grados que mira a izq/dcha para decidir el lado
AV_TURN = 0.45
COMMIT = 0.8       # s: mantiene el lado de esquiva fijo (evita titubeo izq/dcha)


def nav_reactive(cdp, tx, ty, od):
    x0, y0 = od[0], od[1]
    d0 = math.hypot(tx - x0, ty - y0)
    if d0 > 3.0:
        print(f"!! Objetivo a {d0:.1f} m (>3 m). Acércalo."); return
    print(f">>> NAV a ({tx:+.2f},{ty:+.2f})  dist={d0:.2f} m  ESQUIVA ON  (Ctrl+C = stop)")
    print(f"    log -> {LOGPATH}")
    lg = open(LOGPATH, "a")
    lg.write(f"\n=== NAV {time.strftime('%H:%M:%S')} target=({tx:+.2f},{ty:+.2f}) d0={d0:.2f} ===\n")
    t0 = time.time(); best = d0; tbest = t0; tprint = 0
    avoiding = False; side = 1; side_t = 0    # estado de esquiva (con compromiso de lado)
    try:
        while True:
            od = read_poll(cdp).get("odom") or od
            x, y, yaw = od[0], od[1], yaw_of(od)
            dist = math.hypot(tx - x, ty - y)
            if dist < GOAL_TOL:
                print(f"\n  LLEGADO. dist={dist:.2f} m"); lg.write("LLEGADO\n"); break
            if dist < best - 0.05:
                best = dist; tbest = time.time()
            if time.time() - tbest > 15:
                print(f"\n  ATASCADO a {dist:.2f} m (sin progreso 15 s). Abortando."); lg.write("ATASCADO\n"); break
            if time.time() - t0 > 60:
                print(f"\n  (timeout 60 s a {dist:.2f} m)"); break
            egoal = wrap(math.atan2(ty - y, tx - x) - yaw)
            c0 = clear_ahead(cdp, 0)
            cl = cr = None
            # --- maquina de estados con histeresis ---
            if not avoiding and c0 < AVOID_TRIG:
                avoiding = True; side_t = 0          # entra en esquiva
            elif avoiding and c0 > GO_RESUME:
                avoiding = False                     # frente despejado: vuelve a navegar
            rear = None
            if avoiding:
                if time.time() - side_t > COMMIT:    # (re)elige lado solo cada COMMIT s
                    cl = clear_ahead(cdp, +AV_OFF); cr = clear_ahead(cdp, -AV_OFF)
                    side = 1 if cl >= cr else -1     # +1 = izquierda
                    side_t = time.time()
                rx = -AV_TURN if side > 0 else AV_TURN
                if c0 < CLOSE_DIST:                  # demasiado cerca: girar no basta
                    rear = clear_ahead(cdp, 180)     # ¡mira atras antes de retroceder!
                    if rear > REAR_SAFE:
                        cmd = (0, -BACK_SPEED, 0, 0); ph = "BACK "   # retroceso RECTO (no arquea)
                    else:
                        cmd = (0, 0, rx, 0); ph = "PIVOT"           # detras bloqueado: gira en sitio
                else:
                    cmd = (0, 0, rx, 0); ph = "AVOID"
            elif abs(egoal) > ALIGN_TOL:
                cmd = (0, 0, -math.copysign(TURN_SPEED, egoal), 0); ph = "TURN "
            else:
                cmd = (0, FWD_SPEED, 0, 0); ph = "GO   "
            sc = lambda v: f"{v:.2f}" if (v is not None and v < 900) else ("—" if v is not None else "·")
            line = (f"t={time.time()-t0:5.1f} {ph} dist={dist:.2f} e={math.degrees(egoal):+5.0f} "
                    f"c0={sc(c0)} L={sc(cl)} R={sc(cr)} rear={sc(rear)} side={'L' if side>0 else 'R'} "
                    f"cmd=(ly={cmd[1]:+.2f},rx={cmd[2]:+.2f})")
            lg.write(line + "\n"); lg.flush()
            if time.time() - tprint > 0.35:
                print("  " + line); tprint = time.time()
            cdp.eval(set_cmd_js(*cmd))
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n [interrumpido]")
    finally:
        cdp.eval(STOP_JS); time.sleep(0.3); cdp.eval(STOP_JS)
        od = read_poll(cdp).get("odom") or od
        print(f"\nSTOP. pos=({od[0]:+.2f},{od[1]:+.2f})  resto={math.hypot(tx-od[0], ty-od[1]):.2f} m")
        lg.write(f"STOP pos=({od[0]:+.2f},{od[1]:+.2f})\n"); lg.close()


EXP_FWD_MIN = 0.55     # punto medio: gira con algo de margen sin ser timido
EXP_FWD_GOOD = 0.65    # deja de girar y avanza en cuanto hay hueco razonable (menos spinning)
EXP_ESCAPE_MAX = 25.0  # s intentando escapar de un atasco antes de rendirse
REDIR_SEC = 6.0        # cada cuanto reorientar hacia una zona nueva y abierta (cobertura)
WIDE_OFFS = [-150, -120, -90, -60, -30, 0, 30, 60, 90, 120, 150, 180]  # escaneo 360 (+=izquierda)


def cmd_explore(secs):
    """Wander reactivo para autonomous mapping. Clave: al bloquearse elige UN sentido y barre
    hacia ese lado SIN cambiarlo hasta encontrar un hueco real, y entonces avanza (no titubea).
    Ctrl+C para inmediatamente (hombre-muerto a 600ms)."""
    secs = max(5, min(180, secs))
    cdp = get_cdp()
    if not wait_for_odom(cdp):
        print("!! Sin odometria. ¿SLAM activo de pie?"); return
    print(f">>> EXPLORE {secs:.0f}s para mapear.  Ctrl+C = STOP. Mando en la mano (L2+B).")
    print(f"    ESPACIO LIBRE alrededor (puede retroceder). log -> {LOGPATH}")
    lg = open(LOGPATH, "a")
    lg.write(f"\n=== EXPLORE {time.strftime('%H:%M:%S')} {secs:.0f}s ===\n")
    vision_on = "novision" not in sys.argv
    if vision_on:
        threading.Thread(target=yolo_worker, daemon=True).start()
    try:
        cdp.eval("window.__grid={}")            # REJILLA FRESCA: borra el reguero de runs anteriores
        print("Rejilla reiniciada. Esperando ~2s a que se llene del entorno actual...")
        time.sleep(2.0)
    except Exception:
        pass
    t0 = time.time(); tprint = 0
    state = "GO"; esc_t0 = 0; scan_t = 0; best_off = 0; best_clr = 0; esc_dir = 0
    redir_until = 0; redir_dir = 0; last_redir = time.time()          # redireccion hacia zonas nuevas
    visited = {}                                                      # cobertura: celdas 0.4m ya pisadas
    try:                                                              # carga cobertura previa (persiste entre reinicios)
        s = cdp.eval("JSON.stringify(window.__visited||{})")
        if s:
            for k, v in json.loads(s).items():
                a, b = k.split(","); visited[(int(a), int(b))] = v
        print(f"Cobertura previa cargada: {len(visited)} celdas (persisten entre reinicios de Python).")
    except Exception:
        pass

    def save_visited():
        try:
            cdp.eval("window.__visited=" + json.dumps({f"{a},{b}": v for (a, b), v in visited.items()}))
        except Exception:
            pass
    fhist = []; prev_fwd = False; recov = None; ncol = 0; rside = 1   # deteccion de colision por odom
    vcam_t = 0; v_active = False; vside = 1                            # vision (camara+YOLO)
    last_od = None; od_change_t = time.time()                          # vigilancia de odom congelada
    vhealth_t = 0; vstale_t = 0
    def sc(v):
        return f"{v:.2f}" if (v is not None and v < 900) else ("—" if v is not None else "·")
    try:
        while time.time() - t0 < secs:
            now = time.time()
            od = read_poll(cdp).get("odom")
            x = y = yaw = 0.0
            if od:
                x, y, yaw = od[0], od[1], math.degrees(yaw_of(od))
                vk = (round(x / 0.4), round(y / 0.4))             # cobertura: marca celda pisada
                visited[vk] = visited.get(vk, 0) + 1

            def novelty(o):                                       # suma de celdas pisadas A LO LARGO del rayo (yaw+o) hasta 3m
                h = math.radians(yaw + o); c = math.cos(h); s = math.sin(h); tot = 0
                for d in (0.6, 1.1, 1.6, 2.1, 2.6, 3.1):
                    kx = round((x + d * c) / 0.4); ky = round((y + d * s) / 0.4)
                    tot += visited.get((kx, ky), 0)
                return tot

            if int(now - t0) % 5 == 0:                            # guarda cobertura cada ~5s
                save_visited()
            c0 = clear_ahead(cdp, 0); rear = None; cmd = None; ph = ""
            cf_cam = 1.0; vfresh = False                       # defaults (por si la camara esta off)

            # --- VIGILANCIA DE ODOM CONGELADA (feed muerto != colision) ---
            if od is not None:
                if last_od is None or od[0] != last_od[0] or od[1] != last_od[1] or od[6] != last_od[6]:
                    od_change_t = now                          # la odom cambio -> feed vivo
                last_od = od
            odom_live = (now - od_change_t) < 1.5
            if now - od_change_t > 3.0:
                print("\n  ODOMETRIA CONGELADA (no cambia hace 3s). STOP.")
                print("  -> El SLAM perdio tracking o se paro. Reactiva/recoloca el mapeo en la app y reintenta.")
                lg.write("ODOM-FROZEN\n"); break

            # --- DETECCION DE COLISION: avanzaba pero la odom no se mueve (choque que el laser no ve) ---
            if prev_fwd and od and odom_live:                  # solo con odom VIVA (si no, es feed muerto)
                fhist.append((now, x, y))
            fhist = [h for h in fhist if now - h[0] <= 1.8]
            if recov is None and len(fhist) >= 2 and now - fhist[0][0] >= 1.5:
                disp = math.hypot(x - fhist[0][1], y - fhist[0][2])
                if disp < 0.05:                                  # avanzando 1.5s y PRACTICAMENTE parado (choque real)
                    ncol += 1
                    inject_obstacle(cdp, x, y, yaw)              # MARCA la mesa en la rejilla (memoria)
                    cl = clear_ahead(cdp, +55); cr = clear_ahead(cdp, -55)
                    rside = 1 if cl >= cr else -1                # recupera girando al lado MAS despejado
                    recov = {"ph": "BACK", "t0": now}; fhist = []
                    print(f"\n  COLISION #{ncol} (disp={disp:.2f}). Marco obstaculo en el mapa y giro al lado libre.")
                    lg.write(f"COLISION #{ncol} disp={disp:.2f} pos=({x:+.2f},{y:+.2f}) yaw={yaw:+.0f} c0={sc(c0)} L={sc(cl)} R={sc(cr)}\n")
                    save_crash_image(cdp, ncol, x, y, yaw, c0)   # foto + contexto para mejorar la vision

            # --- RECUPERACION (prioridad sobre todo) ---
            if recov is not None:
                el = now - recov["t0"]
                if recov["ph"] == "BACK":
                    rear = clear_ahead(cdp, 180)
                    if rear > REAR_SAFE and el < 0.9:
                        cmd = (0, -BACK_SPEED, 0, 0); ph = "R-BACK"
                    else:
                        recov = {"ph": "TURN", "t0": now}; el = 0
                if recov is not None and recov["ph"] == "TURN":
                    if el < 1.3:
                        cmd = (0, 0, -AV_TURN if rside > 0 else AV_TURN, 0); ph = "R-TURN"
                    else:
                        recov = {"ph": "GO", "t0": now}; el = 0
                if recov is not None and recov["ph"] == "GO":
                    if el < 1.0 and c0 > EXP_FWD_MIN:     # avanza para salir, PERO solo si el frente esta libre
                        cmd = (0, FWD_SPEED, 0, 0); ph = "R-GO "
                    else:
                        recov = None; state = "GO"       # si hay algo (mesa marcada) delante -> deja que ESC gire

            # --- VISION: la camara ve algo que el laser no (mesa, etc.) -> girar para esquivar ---
            vb = False; vlbl = ""; dr = 0
            if vision_on:
                if now - vcam_t > 0.5:                       # captura un frame (mismo CDP, hilo principal)
                    try:
                        j = cdp.eval(CAM_JS)
                        if j and j.startswith("data:image"):
                            with vlock:
                                vision["jpg"] = j; vision["n"] += 1
                    except Exception:
                        pass
                    vcam_t = now
                vclose = False; vdist = ""
                with vlock:
                    dr = vision.get("dratio", 0); cf_cam = vision.get("cf", 1.0); vts = vision.get("ts", 0)
                    vfresh = (now - vts < 3.0)                # ventana amplia (robot lento)
                    if vision["block"] and vfresh:
                        vb = True; vlbl = vision["label"]; vside = vision["side"]
                        vclose = vision.get("close", False); vdist = vision.get("dist", "")
                if not vfresh and now - t0 > 6 and now - vstale_t > 5:   # aviso: la camara NO actualiza (feed caido/worker atascado)
                    print("  [!] vision caducada >3s: ¿camara activa en la app? ¿worker lento? (navego solo con laser)")
                    lg.write("VISION-STALE\n"); vstale_t = now
            if cmd is None and vb:                           # la camara ve un obstaculo
                if vclose or c0 < 0.35:                       # SOLO si esta de verdad pegado -> retrocede
                    rear = clear_ahead(cdp, 180)
                    if rear > REAR_SAFE:
                        cmd = (0, -BACK_SPEED, 0, 0); ph = "VAV-BK"
                    else:
                        cmd = (0, 0, -AV_TURN if vside > 0 else AV_TURN, 0); ph = "VAV-" + vlbl[:6]
                else:                                        # hay hueco -> GIRA hacia el lado mas despejado (no retrocede)
                    cmd = (0, 0, -AV_TURN if vside > 0 else AV_TURN, 0); ph = "VAV-" + vlbl[:6]
            elif not vb:
                v_active = False

            if cmd is not None:
                pass
            elif state == "GO":
                if c0 <= EXP_FWD_MIN:
                    state = "ESC"; esc_t0 = now; scan_t = 0; esc_dir = 0   # obstaculo -> esquiva
                else:
                    # cada REDIR_SEC, si hay direccion ABIERTA y POCO VISITADA, vira hacia ella (cobertura)
                    if now >= redir_until and now - last_redir > REDIR_SEC:
                        last_redir = now
                        ws = {o: clear_ahead(cdp, o) for o in WIDE_OFFS}
                        openo = [o for o in WIDE_OFFS if ws[o] > EXP_FWD_GOOD]
                        if openo:
                            bo = min(openo, key=novelty)          # direccion abierta que lleva a lo MENOS pisado
                            if abs(bo) >= 30 and novelty(bo) + 1 < novelty(0):   # bastante mas nueva que seguir recto
                                redir_dir = -AV_TURN if bo >= 0 else AV_TURN
                                redir_until = now + min(2.2, abs(bo) / 45.0)
                                lg.write(f"REDIR-> {bo:+d} nov(recto)={novelty(0)} nov(elegido)={novelty(bo)}\n")
                    if now < redir_until:
                        cmd = (0, 0, redir_dir, 0); ph = "REDIR"
                    else:
                        # PROPORCIONAL: si la camara (fresca) ve el suelo algo tapado delante, FRENA (mas margen)
                        spd = 0.30 if (vision_on and vfresh and cf_cam < 0.45) else FWD_SPEED
                        cmd = (0, spd, 0, 0); ph = "GO-sl" if spd < FWD_SPEED else "GO   "; esc_t0 = 0
            if cmd is None and state == "ESC":
                resume = EXP_FWD_GOOD if (now - esc_t0 < 4.0) else EXP_FWD_MIN   # anti-spin: tras 4s, sale al primer hueco
                if c0 > resume:                              # hay hueco -> avanza (no sigue girando)
                    state = "GO"; cmd = (0, FWD_SPEED, 0, 0); ph = "GO   "; esc_t0 = 0
                elif time.time() - esc_t0 > EXP_ESCAPE_MAX:
                    print(f"\n  ATASCADO {EXP_ESCAPE_MAX:.0f}s. STOP. Muévelo un poco a mano y reanuda (revisa cabeza nivelada).")
                    lg.write("ATASCADO\n"); break
                else:
                    if esc_dir == 0 or time.time() - scan_t > 2.5:   # elige lado UNA vez (re-evalua solo si atascado)
                        ws = {o: clear_ahead(cdp, o) for o in WIDE_OFFS}
                        # entre las direcciones TRANSITABLES, prefiere la MENOS visitada (cobertura)
                        def novelty(o):
                            h = math.radians(yaw + o)
                            nx = x + 1.3 * math.cos(h); ny = y + 1.3 * math.sin(h)
                            kx = round(nx / 0.4); ky = round(ny / 0.4)
                            return sum(visited.get((kx + dx, ky + dy), 0)
                                       for dx in (-1, 0, 1) for dy in (-1, 0, 1))
                        passable = [o for o in WIDE_OFFS if ws[o] > EXP_FWD_MIN]
                        if passable:
                            best_off = min(passable, key=novelty)    # menos pisada
                        else:
                            best_off = max(WIDE_OFFS, key=lambda o: ws[o])   # nada abierto -> la mas abierta
                        best_clr = ws[best_off]
                        esc_dir = -AV_TURN if best_off >= 0 else AV_TURN
                        scan_t = time.time()
                        lg.write("  SCAN " + " ".join(f"{o:+d}:{sc(ws[o])}" for o in WIDE_OFFS) +
                                 f"  best={best_off:+d}({sc(best_clr)}) nov={novelty(best_off)}\n")
                    if c0 < 0.25:                            # PEGADISIMO: girar no lo saca -> retrocede si hay sitio
                        rear = clear_ahead(cdp, 180)
                        if rear > REAR_SAFE:
                            cmd = (0, -BACK_SPEED, 0, 0); ph = "BACK "
                        else:
                            cmd = (0, 0, esc_dir, 0); ph = "PIVOT"
                    else:                                    # GIRO COMPROMETIDO hacia el hueco (hasta abrir frente)
                        cmd = (0, 0, esc_dir, 0); ph = f"TURN{'L' if esc_dir < 0 else 'R'}"
            line = (f"t={time.time()-t0:5.1f}/{secs:.0f} {ph} pos=({x:+.2f},{y:+.2f}) yaw={yaw:+6.1f} "
                    f"c0={sc(c0)} rear={sc(rear)} vis={(vlbl+':'+vdist) if vb else '-'} dr={dr} best={best_off:+d}({sc(best_clr)}) "
                    f"cmd=(lx={cmd[0]:+.2f},ly={cmd[1]:+.2f},rx={cmd[2]:+.2f})")
            lg.write(line + "\n"); lg.flush()
            if vision_on and now - vhealth_t > 3:            # salud de la vision (lentitud/caducidad)
                with vlock:
                    vage = now - vision.get("ts", 0); vdtf = vision.get("dtf", -1)
                    vblk = vision.get("block"); vd = vision.get("dratio")
                lg.write(f"  VHEALTH dtf={vdtf}s age={vage:.1f}s block={vblk} dr={vd}\n")
                vhealth_t = now
            if time.time() - tprint > 0.4:
                print("  " + line); tprint = time.time()
            prev_fwd = (cmd[1] > 0.1)        # ¿este ciclo manda avanzar? (para detectar stall)
            cdp.eval(set_cmd_js(*cmd))
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n [STOP por Ctrl+C]")
    finally:
        cdp.eval(STOP_JS); time.sleep(0.3); cdp.eval(STOP_JS)
        save_visited()                                   # persiste cobertura para el proximo run
        print(f"STOP. Fin. Cobertura: {len(visited)} celdas (guardadas, persisten entre reinicios).")
        lg.write("FIN\n"); lg.close()


def cmd_nav(tx, ty):
    cdp = get_cdp()
    od = wait_for_odom(cdp)
    if not od:
        print("!! Sin odometria."); return
    nav_reactive(cdp, tx, ty, od)


def cmd_navrel(fwd, left):
    cdp = get_cdp()
    od = wait_for_odom(cdp)
    if not od:
        print("!! Sin odometria."); return
    x, y, yaw = od[0], od[1], yaw_of(od)
    tx = x + fwd * math.cos(yaw) - left * math.sin(yaw)
    ty = y + fwd * math.sin(yaw) + left * math.cos(yaw)
    nav_reactive(cdp, tx, ty, od)


def main():
    if len(sys.argv) < 2:
        print(__doc__); return
    cmd = sys.argv[1]
    if cmd == "watch":
        cmd_watch()
    elif cmd == "forward":
        m = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
        sp = float(sys.argv[3]) if len(sys.argv) > 3 else 0.4
        cmd_forward(m, sp)
    elif cmd == "turn":
        cmd_turn(float(sys.argv[2]) if len(sys.argv) > 2 else 45)
    elif cmd == "goto":
        if len(sys.argv) < 4:
            print("uso: goto <x> <y>"); return
        cmd_goto(float(sys.argv[2]), float(sys.argv[3]))
    elif cmd == "gorel":
        if len(sys.argv) < 3:
            print("uso: gorel <adelante_m> [izquierda_m]"); return
        fwd = float(sys.argv[2])
        left = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
        cmd_gorel(fwd, left)
    elif cmd == "nav":
        if len(sys.argv) < 4:
            print("uso: nav <x> <y>"); return
        cmd_nav(float(sys.argv[2]), float(sys.argv[3]))
    elif cmd == "navrel":
        if len(sys.argv) < 3:
            print("uso: navrel <adelante_m> [izquierda_m]"); return
        cmd_navrel(float(sys.argv[2]), float(sys.argv[3]) if len(sys.argv) > 3 else 0.0)
    elif cmd == "explore":
        cmd_explore(float(sys.argv[2]) if len(sys.argv) > 2 else 60)
    elif cmd == "scan":
        cmd_scan()
    elif cmd == "vsee":
        cmd_vsee()
    elif cmd == "zhist":
        cmd_zhist()
    elif cmd == "bbox":
        cmd_bbox()
    elif cmd == "scan2":
        cmd_scan2()
    elif cmd == "clr":
        cmd_clr()
    elif cmd == "probe":
        cmd_probe()
    else:
        print("comando desconocido:", cmd); print(__doc__)


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print("Error:", repr(e))
        sys.exit(1)
