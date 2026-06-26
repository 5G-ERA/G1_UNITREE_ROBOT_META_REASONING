#!/usr/bin/env python3
"""
g1_nav_v2.py  -  VERSION EXPERIMENTAL (no toca g1_nav.py estable). Novedades v2:
  * MODELOS (MacBook M4): YOLOv8m por defecto; profundidad MiDaS_small (arranque instantaneo).
      Para mas calidad: G1_DEPTH=DPT_Hybrid (descarga ~470MB la 1a vez). Tambien G1_YOLO=yolov8l.pt|yolo11m.pt
      Fallback automatico a v8s / MiDaS_small si el modelo grande no carga.
  * FRONTERAS POR GANANCIA: agrupa fronteras y va a la apertura GRANDE (puertas), no a la mas cercana.
  * ANALOGIA DESDE COLISIONES (tesis): huella visual de cada crash; si la vista actual se PARECE a algo
      que ya choco -> senal fuerte (lo evita aunque el laser este libre).
  * VELOCIDAD SEGUN CONFIANZA: frena en zonas dudosas (laser cerca / camara degradada / suelo tapado).
  * IMU (scaffold): hook que captura 'imu/lowstate/sportmode' + comando 'imu' para descubrir su estructura
      (futuro: deteccion de contacto por sacudida, mas rapida que la odom).
  Todo lo demas hereda del estable: laser manda sobre camara, sin marcha atras (pivota), TTL del mapa,
  arco, compromiso de frontera, export PNG/JSON + ventana mapa+camara, volcado vision_debug.

g1_nav  -  FASE 1: captura (nube+odom) + control (teleop) UNIFICADOS en un proceso,
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
import json, sys, time, math, threading, os, random, heapq
import requests
import websocket  # websocket-client

LOGPATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "g1_nav.log")

# ---- VISION (cámara + YOLO) como capa de obstáculos complementaria al láser ----
vlock = threading.Lock()
vision = {"jpg": None, "n": 0, "block": False, "label": "", "side": 1, "cf": 1.0, "ts": 0, "dump": 0}
CAM_JS = (
    "(function(){var v=document.querySelector('video');"
    "if(!v||!v.videoWidth)return '';"
    "var W=320,H=Math.round(W*v.videoHeight/v.videoWidth);"
    "var c=window.__camc||(window.__camc=document.createElement('canvas'));"
    "c.width=W;c.height=H;c.getContext('2d').drawImage(v,0,0,W,H);"
    "try{return c.toDataURL('image/jpeg',0.5);}catch(e){return '';}})()"
)


FLOOR_MIN = 0.10      # fraccion minima de suelo en el centro; muy bajo -> camara solo bloquea si esta casi todo tapado
EDGE_RUN = 7          # columnas seguidas con el suelo interrumpido cerca = pata/objeto fino (subido: menos nervioso)
DEPTH_TH = 0.63       # umbral de profundidad MiDaS (suelo limpio ~0.5; obstaculos reales 0.66+; 0.55 bloqueaba de mas)
# muebles/obstaculos que YOLO conoce -> tratarlos como obstaculo aunque la caja sea menor
YOLO_FURNITURE = {"diningtable", "table", "chair", "couch", "bed", "bench", "refrigerator", "oven",
                  "sink", "suitcase", "tvmonitor", "tv", "microwave", "toilet", "pottedplant",
                  "vase", "backpack", "handbag", "boat", "bicycle", "motorcycle"}
SAT_FLOOR_MAX = 45    # suelo = poco saturado (moqueta gris S~5); obstaculos saturados (S>100)
# --- DISTANCIA METRICA por plano de suelo (inverse perspective): d = K/(contact_frac - horizon) ---
# contact_frac = fila (0=arriba,1=abajo) donde el suelo se interrumpe = base del obstaculo.
# Calibrar con 'python g1_nav.py floorcal D' a dos distancias (ver instrucciones del comando).
FLOOR_HORIZON = 0.267 # CALIBRADO (floorcal auto robusto, 25-jun, err frac=0.036 vs LiDAR)
FLOOR_K = 0.299       # CALIBRADO. Recalibrar con 'floorcal auto' si cambia camara/altura
CLOSE_M = 0.45        # < esto = "cerca" (esquiva). BAJADO: mantenia demasiada distancia de seguridad
MED_M = 1.10          # < esto = "medio" (no frena, solo informativo). >= esto = "lejos"
CAM_TRUST_C0 = 1.30   # si el LASER ve el corredor despejado mas alla de esto, la CAMARA no puede vetar
                      # (sus 'cerca' son falsos en suelos reflectantes/raros). El laser manda.
# v3-firmware: estilo del nav nativo (puro LiDAR, NO gira reactivo: para y REPLANIFICA rodeando).
CAM_REACTIVE = (os.environ.get("G1_CAM_REACTIVE", "0") == "1")   # def OFF: camara SOLO informa el mapa, no gira
IMU_CONTACT = (os.environ.get("G1_IMU_CONTACT", "1") == "1")     # def ON: contacto rapido por par/accel de lowstate
STOP_CLEAR = 0.65     # obstaculo del LASER directamente delante a < esto -> NO embestir: marca + A* rodea


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
    # base del obstaculo (mediana entre columnas del centro): cuanto suelo contiguo hay desde abajo.
    # mcont alto = suelo despejado lejos; bajo = algo cerca interrumpe el suelo pronto.
    mcont = float(np.median(colh))
    return frac(0.0, 0.35), frac(0.35, 0.65), frac(0.65, 1.0), refS, int(mx), mcont


def save_vision_dump(img, lf, cf, rf, mcont, dmet, nrun, dratio, tag=""):
    """Guarda en vision_debug/ la imagen de la camara + un OVERLAY con lo que el segmentador cree que es
    suelo (verde) vs no-suelo (rojo) + numeros. Para analizar POR QUE la camara dice 'cerca en todo rumbo'.
    Pista de lectura: si casi no hay verde (poco suelo) aunque el suelo este despejado -> el modelo de suelo
    (referencia de la franja de los pies) esta mal: cabeza inclinada (pies/cuerpo en la franja) o luz/suelo raro."""
    try:
        import numpy as np
        from PIL import Image
        cdir = os.path.join(os.path.dirname(LOGPATH), "vision_debug")
        os.makedirs(cdir, exist_ok=True)
        ts = time.strftime('%H%M%S')
        base = os.path.join(cdir, f"vdump_{tag}_{ts}")
        hsv = np.asarray(img.convert("HSV")).astype(int)
        H, W = hsv.shape[:2]
        S = hsv[:, :, 1]; V = hsv[:, :, 2]
        strip = hsv[int(H * 0.88):, :]
        refS = float(np.median(strip[:, :, 1])); refV = float(np.median(strip[:, :, 2]))
        floor = (np.abs(S - refS) < 42) & (np.abs(V - refV) < 55)       # MISMO criterio que floor_free_bands
        rgb = np.asarray(img.convert("RGB")).copy()
        over = rgb.copy()
        over[floor] = (over[floor] * 0.4 + np.array([0, 255, 0]) * 0.6).astype(np.uint8)   # suelo -> verde
        nf = (~floor) & (np.arange(H)[:, None] > int(H * 0.45))         # no-suelo en mitad inferior -> rojo
        over[nf] = (over[nf] * 0.5 + np.array([255, 0, 0]) * 0.5).astype(np.uint8)
        # franja de los pies (referencia) en azul
        over[int(H * 0.88):, :] = (over[int(H * 0.88):, :] * 0.5 + np.array([0, 0, 255]) * 0.5).astype(np.uint8)
        comb = np.concatenate([rgb, over], axis=1)                     # original | overlay
        Image.fromarray(comb).save(base + ".jpg", quality=80)
        with open(base + ".txt", "w") as f:
            f.write(f"VISION DUMP {tag} {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"suelo_frac: izq={lf:.2f} centro={cf:.2f} dcha={rf:.2f}   (1=todo suelo, 0=nada de suelo)\n")
            f.write(f"refS={refS:.0f} refV={refV:.0f}  (referencia de la franja de pies; si rara -> mal segmentado)\n")
            f.write(f"mcont(suelo contiguo desde abajo)={mcont:.2f}  contact_frac={1 - mcont:.2f}  dmet={dmet:.2f}m\n")
            f.write(f"edge nrun={nrun} (>=7 = pata/objeto fino)  depth_ratio={dratio} (>0.75 = pared cerca)\n")
            f.write("lectura: poco verde con suelo despejado => referencia de pies MALA (cabeza inclinada / luz).\n")
        print(f"    [vision dump -> {base}.jpg]")
    except Exception as e:
        print("    (no pude volcar vision:", repr(e), ")")


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


ANALOGY_TH = 0.95     # similitud min (coseno) para "esto se parece a algo que ya choque" (estricto = pocos falsos)


def frame_embedding(img):
    """Huella visual LIGERA (sin modelo extra) de la franja central-inferior (donde esta el obstaculo
    en el camino): histograma de color HSV + estadisticos de gradiente (textura/forma). Para comparar
    la vista actual con colisiones pasadas (transferencia ANALOGICA: 'esto es como lo que me comi')."""
    import numpy as np
    from PIL import Image
    a = np.asarray(img.convert("RGB")); H, W = a.shape[:2]
    c = a[int(H * 0.35):, int(W * 0.25):int(W * 0.75)]    # franja central-inferior (el obstaculo del camino)
    ci = Image.fromarray(c)
    hsv = np.asarray(ci.convert("HSV"))
    hh = np.histogram(hsv[:, :, 0], bins=12, range=(0, 256))[0]
    hs = np.histogram(hsv[:, :, 1], bins=8, range=(0, 256))[0]
    hv = np.histogram(hsv[:, :, 2], bins=8, range=(0, 256))[0]
    g = np.asarray(ci.convert("L")).astype(float)
    gx = np.abs(np.diff(g, axis=1)); gy = np.abs(np.diff(g, axis=0))
    grad = np.array([gx.mean(), gx.std(), gy.mean(), gy.std()]) * 4.0   # peso a la textura/bordes
    v = np.concatenate([hh, hs, hv, grad]).astype(float)
    return v / (np.linalg.norm(v) + 1e-6)


def load_crash_embeddings():
    """Carga las huellas de las fotos de colisiones previas (crashes/) para el matching analogico."""
    try:
        from PIL import Image
    except Exception:
        return []
    cdir = os.path.join(os.path.dirname(LOGPATH), "crashes")
    if not os.path.isdir(cdir):
        return []
    embs = []
    for fn in sorted(os.listdir(cdir)):
        if not fn.endswith(".jpg"):
            continue
        try:
            embs.append(frame_embedding(Image.open(os.path.join(cdir, fn))))
        except Exception:
            pass
    return embs


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
    # --- MODELOS (v2): por defecto MAS POTENTES (MacBook M4). Seleccionables por variable de entorno:
    #     G1_YOLO=yolov8m.pt|yolov8l.pt|yolo11m.pt   G1_DEPTH=DPT_Hybrid|DPT_Large|MiDaS_small ---
    model = None
    yolo_name = os.environ.get("G1_YOLO", "yolov8m.pt")    # v8m: bastante mejor que v8s, sobra en M4
    try:
        from ultralytics import YOLO
        print(f"Cargando {yolo_name} (M4: usa modelo grande sin problema; G1_YOLO para cambiar)...")
        try:
            model = YOLO(yolo_name); print(f"YOLO listo ({yolo_name}).")
        except Exception as e1:                            # fallback: si no descarga/carga, usa v8s
            print(f"({yolo_name} fallo: {e1!r} -> fallback yolov8s.pt)")
            model = YOLO("yolov8s.pt"); print("YOLO listo (yolov8s, fallback).")
    except Exception as e:
        print(f"(YOLO no disponible: {e!r}; solo segmentacion de suelo. pip install ultralytics)")
    # MiDaS/DPT: profundidad monocular -> distancia de CUALQUIER cosa (cristal/mismo color) antes de llegar
    midas = None; mid_tf = None
    # default MiDaS_small (ya cacheado, arranque instantaneo). Para calidad: G1_DEPTH=DPT_Hybrid (descarga ~470MB 1a vez)
    depth_name = os.environ.get("G1_DEPTH", "MiDaS_small")
    if "nodepth" not in sys.argv:
        try:
            import torch
            print(f"Cargando profundidad {depth_name} (1a vez descarga; M4 lo mueve de sobra)...")
            _hub_load = torch.hub.load                          # parche: auto-confiar TODO (tb. lo anidado)
            def _trusted_load(*a, **k):
                k.setdefault("trust_repo", True)
                return _hub_load(*a, **k)
            torch.hub.load = _trusted_load
            try:
                midas = torch.hub.load("intel-isl/MiDaS", depth_name); midas.to(dev).eval()
                _tfs = torch.hub.load("intel-isl/MiDaS", "transforms")
                mid_tf = _tfs.dpt_transform if depth_name.startswith("DPT") else _tfs.small_transform
                print(f"Profundidad lista ({depth_name}, dev={dev}).")
            except Exception as e1:                            # fallback: si DPT no descarga/carga, usa MiDaS_small
                print(f"({depth_name} fallo: {e1!r} -> fallback MiDaS_small)")
                midas = torch.hub.load("intel-isl/MiDaS", "MiDaS_small"); midas.to(dev).eval()
                mid_tf = torch.hub.load("intel-isl/MiDaS", "transforms").small_transform
                print("Profundidad lista (MiDaS_small, fallback).")
            torch.hub.load = _hub_load                          # restaura
        except Exception as e:
            print(f"(profundidad no disponible: {e!r} -> sin profundidad. pip install timm)"); midas = None
    print(f"VISION v2 activa (suelo{' + ' + yolo_name if model else ''}{' + ' + depth_name if midas else ''}).")
    crash_db = load_crash_embeddings()                    # ANALOGIA: huellas de colisiones pasadas
    if crash_db:
        print(f"Analogia: {len(crash_db)} colisiones previas cargadas (evitare obstaculos PARECIDOS).")
    last = -1; smooth = []; dsmooth = []; csm = []; tlast = 0.0
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
            blk = False; lbl = ""; close = False; dist = ""; strong = False   # strong=señal fiable (puede vetar aunque laser libre)
            # 1) suelo por color + EDGE (patas finas) (con suavizado temporal de 4 frames)
            lf0, cf0, rf0, refS, nrun, mcont0 = floor_free_bands(img)
            smooth.append((lf0, cf0, rf0)); smooth = smooth[-4:]
            lf = sum(s[0] for s in smooth) / len(smooth)
            cf = sum(s[1] for s in smooth) / len(smooth)
            rf = sum(s[2] for s in smooth) / len(smooth)
            # DISTANCIA METRICA por plano de suelo: base del obstaculo (contact_frac) -> metros
            # contact_frac por frame es MUY ruidoso (sd~0.13) -> MEDIANA de 8 frames (robusto a parpadeos)
            csm.append(mcont0); csm = csm[-8:]
            mcont = sorted(csm)[len(csm) // 2]          # mediana
            contact_frac = 1.0 - mcont                  # 0=arriba(lejos), 1=abajo(cerca)
            if contact_frac > FLOOR_HORIZON + 0.03:      # margen: evita la division casi por cero (57m falsos)
                dmet = min(FLOOR_K / (contact_frac - FLOOR_HORIZON), 9.9)
            else:
                dmet = 999.0                            # base en/por encima del horizonte = sin obstaculo cercano
            if cf < FLOOR_MIN:
                blk = True; lbl = "floor"
            elif nrun >= EDGE_RUN:                     # pata/objeto fino delante (el suelo-frac no lo pilla)
                blk = True; lbl = "edge"
            if blk and lbl in ("floor", "edge"):       # distancia REAL (m) a la base -> cerca/medio/lejos
                dist = "cerca" if dmet < CLOSE_M else ("medio" if dmet < MED_M else "lejos")
                close = (dmet < CLOSE_M)
                if dist == "lejos":                    # base lejana -> no es obstaculo real para esquivar
                    blk = False; lbl = ""
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
                    central = 0.20 < cx < 0.80
                    # GENERICO: CUALQUIER objeto (no solo muebles) en el camino central cuenta como obstaculo
                    if (central and area > 0.08) or (furn and 0.15 < cx < 0.85 and area > 0.05) \
                       or (central and bh > 0.45 and bottom > 0.6):
                        # distancia por la caja: base baja en el frame / area grande -> mas cerca
                        d = "cerca" if (bottom > 0.82 or area > 0.22) else \
                            ("medio" if (bottom > 0.62 or area > 0.10) else "lejos")
                        if d == "lejos":
                            continue                       # objeto LEJOS -> no bloquea, sigue (no asustarse)
                        blk = True; lbl = name; dist = d
                        # OBJETO BAJO RECONOCIDO (mesa/silla) que el LiDAR de cabeza NO ve: si esta CERCA
                        # (caja baja/grande), es señal FIABLE -> 'strong' para que el control PARE+RODEE.
                        # (Solo objeto YOLO real, no el segmentado de suelo -> no reabre falsos del suelo reflectante.)
                        if d == "cerca" or area > 0.30:
                            close = True; strong = True
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
                        if ratio > DEPTH_TH + 0.22:       # superficie vertical MUY clara (pared real, no reflejo) -> FUERTE
                            strong = True
                except Exception:
                    pass

            if dmet < CLOSE_M:                            # el suelo confirma que hay algo CERCA -> close (señal DEBIL, no fuerte)
                close = True
            # --- ANALOGIA (tesis): ¿esta vista se PARECE a algo que ya choque? ---
            # SOLO INFORMATIVA por ahora: el descriptor de COLOR coincide demasiado dentro de la misma
            # habitacion (crashes de la misma mesa/suelo) -> si vetara, reabriria el "camara bloquea todo".
            # Se registra (vis=analog) para la tesis; NO frena. Para que actue hace falta un embedding CNN.
            asim = 0.0
            if crash_db and blk:
                try:
                    e = frame_embedding(img)
                    asim = max(float(np.dot(e, ce)) for ce in crash_db)
                    if asim > ANALOGY_TH and (not lbl or lbl in ("depth", "edge", "floor")):
                        lbl = "analog"                        # etiqueta informativa, sin forzar esquiva
                except Exception:
                    pass
            with vlock:
                vision["block"] = blk; vision["label"] = lbl; vision["side"] = side; vision["close"] = close
                vision["strong"] = strong; vision["analogy"] = round(asim, 2)
                vision["dist"] = dist; vision["dmet"] = round(dmet, 2); vision["cfrac"] = round(contact_frac, 3)
                vision["lf"] = lf; vision["cf"] = cf; vision["rf"] = rf
                vision["refS"] = refS; vision["ts"] = time.time()   # (bug arreglado: 'refH' ya no existe y abortaba el ts)
            dtf = time.time() - _tf0                          # tiempo por frame de vision
            with vlock:
                vision["dtf"] = round(dtf, 2); dump_n = vision.get("dump", 0)
            if dump_n:                                        # el control pidio analizar (camara bloquea en todo rumbo)
                save_vision_dump(img, lf, cf, rf, mcont, dmet, nrun, vision.get("dratio", 0), tag=f"deg{int(dump_n)}")
                with vlock:
                    vision["dump"] = 0
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
                dm = vision.get("dmet", 999); cls = vision.get("close", False)
            bar = "BLOCK(" + lbl + ")" if blk else "libre"
            dms = "—" if dm > 900 else f"{dm:.2f}m"
            print(f"  suelo izq={lf:.2f} centro={cf:.2f} dcha={rf:.2f}  dist={dms}{' CERCA' if cls else ''}  depth_ratio={dr}  -> {bar}")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nFin vsee.")


def cmd_floorcal(arg=None):
    """Calibra la distancia metrica de la camara SIN cinta metrica, usando el LiDAR como regla.
    Modelo: contact_frac = horizon + K*(1/d)  (LINEAL en 1/d) -> regresion de (d_lidar, contact_frac).

    USO (modo auto, recomendado):
       1) Pon el robot mirando a una PARED o CAJA GRANDE (que el LiDAR vea bien).
       2) python g1_nav.py floorcal auto
       3) Acerca/aleja lentamente el robot (o mueve la caja) entre ~0.4 y ~2 m. Recoge pares solo.
       4) Ctrl+C -> imprime FLOOR_HORIZON y FLOOR_K ya calculados para pegar en el archivo.
    Sin 'auto' solo muestra lectura en vivo (contact_frac + dist del modelo actual)."""
    cdp = get_cdp()
    threading.Thread(target=yolo_worker, daemon=True).start()
    auto = (str(arg).lower() == "auto")
    samples = []                                          # (d_lidar, contact_frac)
    callog = os.path.join(os.path.dirname(LOGPATH), "floorcal.log")   # log para revisar despues
    lg = open(callog, "a")
    lg.write(f"\n=== FLOORCAL {'AUTO' if auto else 'LIVE'} {time.strftime('%Y-%m-%d %H:%M:%S')} ===\n")
    if auto:
        print(">>> FLOORCAL AUTO: mira a una PARED/CAJA grande. Acerca/aleja el robot entre ~0.4 y ~2 m.")
        print(f"    Recojo pares (LiDAR, camara). Ctrl+C para calcular.  log -> {callog}\n")
    else:
        print(f">>> FLOORCAL: lectura en vivo. Ctrl+C salir.  log -> {callog}\n")
    last_d = None
    try:
        while True:
            try:
                j = cdp.eval(CAM_JS)
                if j and j.startswith("data:image"):
                    with vlock:
                        vision["jpg"] = j; vision["n"] += 1
            except Exception:
                pass
            c0 = clear_ahead(cdp, 0)                       # distancia LiDAR al frente (m); 999 si no ve nada
            with vlock:
                cf = vision.get("cf", 0); dm = vision.get("dmet", 999); fr = vision.get("cfrac", 0.0)
            dms = "—" if dm > 900 else f"{dm:.2f}m"
            c0s = "—" if c0 > 900 else f"{c0:.2f}m"
            if auto:
                # recoge solo si el LiDAR ve algo solido en rango y la camara tiene base clara, y cambio de distancia
                ok = (0.35 < c0 < 2.2) and (fr > FLOOR_HORIZON - 0.05)
                if ok and (last_d is None or abs(c0 - last_d) > 0.04):
                    samples.append((c0, fr)); last_d = c0
                    lg.write(f"SAMPLE d_lidar={c0:.3f} contact_frac={fr:.3f}\n"); lg.flush()
                print(f"  LiDAR={c0s}  contact_frac={fr:.3f}  muestras={len(samples)}   "
                      + ("(recogiendo)" if ok else "(LiDAR sin objeto claro -> acerca a una pared)"))
            else:
                lg.write(f"LIVE LiDAR={c0s} contact_frac={fr:.3f} dist_modelo={dms} cf={cf:.2f}\n"); lg.flush()
                print(f"  LiDAR={c0s}  contact_frac={fr:.3f}  dist_modelo={dms}  (cf_centro={cf:.2f})")
            time.sleep(0.4)
    except KeyboardInterrupt:
        if not auto:
            lg.write("FIN (live)\n"); lg.close(); print("\nFin floorcal."); return
        # regresion lineal: frac = horizon + K*(1/d)
        if len(samples) < 8:
            msg = f"Pocas muestras ({len(samples)}). Repite acercando/alejando frente a una pared."
            print("\n  " + msg); lg.write("RESULT FAIL: " + msg + "\n"); lg.close(); return
        # AJUSTE ROBUSTO: agrupa por tramo de distancia (0.15 m) y usa la MEDIANA de frac (el frame es ruidoso)
        import statistics as _st
        bins = {}
        for d, f in samples:
            bins.setdefault(round(d / 0.15) * 0.15, []).append(f)
        med = [(d, _st.median(fs)) for d, fs in bins.items() if len(fs) >= 2]
        if len(med) < 3:
            print("\n  Pocos tramos. Cubre mas rango de distancias (0.4 a 2 m).")
            lg.write("RESULT FAIL: pocos tramos\n"); lg.close(); return
        pts = [(1.0 / d, f) for d, f in med]
        n = len(pts); sx = sum(p[0] for p in pts); sy = sum(p[1] for p in pts)
        sxx = sum(p[0] ** 2 for p in pts); sxy = sum(p[0] * p[1] for p in pts)
        den = (n * sxx - sx * sx)
        if abs(den) < 1e-9:
            print("\n  Muestras sin variacion de distancia. Mueve mas el robot.")
            lg.write("RESULT FAIL: sin variacion de distancia\n"); lg.close(); return
        K = (n * sxy - sx * sy) / den                     # pendiente
        horizon = (sy - K * sx) / n                       # intercepto
        err = sum(abs((horizon + K * x) - y) for x, y in pts) / n   # error medio sobre las medianas
        out = [f"=== CALIBRACION ({len(samples)} muestras, {n} tramos, error medio frac={err:.3f}) ===",
               f"FLOOR_HORIZON = {horizon:.3f}",
               f"FLOOR_K       = {K:.3f}",
               ("ajuste BUENO" if err < 0.05 else "ajuste flojo: repite con mas rango / camara nivelada"),
               "-> pega esos dos valores en g1_nav.py (lineas FLOOR_HORIZON / FLOOR_K).",
               "comprobacion (d_lidar -> d_estimada, por tramos):"]
        for d, f in sorted(med):
            de = K / (f - horizon) if (f - horizon) > 1e-3 else 999
            out.append(f"   {d:.2f}m -> {('—' if de > 900 else f'{de:.2f}m')}")
        print("\n  " + "\n  ".join(out))
        lg.write("RESULT " + " | ".join(out[:3]) + "\n")
        for ln in out:
            lg.write(ln + "\n")
        lg.write("FIN\n"); lg.close()

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
      // v2 SCAFFOLD IMU: captura cualquier mensaje con pinta de IMU/lowstate para DESCUBRIR su estructura
      // (comando 'imu' para inspeccionarlo). No rompe nada si no aparece. Luego se cablea jerk->colision.
      try{ var tp = v && v.topic ? (''+v.topic) : '';
        if(tp.indexOf('imu')>=0 || tp.indexOf('lowstate')>=0 || tp.indexOf('sportmode')>=0){
          window.__imu = {topic: tp, data: v.data, t: Date.now()};
        }
      }catch(e){}
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
# Hook INDEPENDIENTE (su propio guard, por si el hook principal ya esta instalado y no se reinstala):
# extrae de rt/lf/lowstate (~15Hz) el acelerometro/giro + el PAR MAXIMO de las patas (motores 0..11)
# -> deteccion de CONTACTO rapida (choque = pico de accel o de par, en ~0.1s vs 1.5s del odom-stall).
LOWSTATE_JS = r"""(function(){
  if(!window.__lowHook){ window.__lowHook=1;
    var jp = JSON.parse;
    JSON.parse = function(s){ var v = jp.apply(this, arguments);
      try{ if(v && v.topic && (''+v.topic).indexOf('lowstate')>=0 && v.data && v.data.imu_state){
        var ms = v.data.motor_state||[]; var mt=0;
        for(var i=0;i<12 && i<ms.length;i++){ var tq=Math.abs(ms[i].tau_est||0); if(tq>mt) mt=tq; }
        var a=v.data.imu_state.accelerometer||[0,0,0];
        window.__low = {ax:a[0], ay:a[1], az:a[2], legtau:mt, t:Date.now()};
      }}catch(e){}
      return v;
    };
  } return 1;
})()"""


def read_low(cdp):
    """Lee window.__low (accel horizontal + par max de patas) si esta. Devuelve dict o None."""
    try:
        s = cdp.eval("JSON.stringify(window.__low||null)")
        return json.loads(s) if s and s != "null" else None
    except Exception:
        return None


def read_imu(cdp):
    """Lee window.__imu (capturado por el hook) si existe. Devuelve dict o None. (v2 scaffold)"""
    try:
        s = cdp.eval("JSON.stringify(window.__imu||null)")
        return json.loads(s) if s and s != "null" else None
    except Exception:
        return None


def cmd_imu():
    """DESCUBRIMIENTO del IMU: muestra el mensaje 'imu/lowstate/sportmode' que captura el hook, para ver
    su estructura y luego cablear deteccion de contacto por sacudida (mas rapida que la odom). Mueve un
    poco el robot a mano y observa que campos cambian (aceleracion lineal)."""
    cdp = get_cdp()
    print(">>> IMU discovery. Si no aparece nada, el robot no publica un topic con 'imu/lowstate/sportmode'.")
    print("    Mueve/golpea suavemente el robot y mira que numeros cambian (= aceleracion). Ctrl+C salir.\n")
    try:
        while True:
            d = read_imu(cdp)
            if d is None:
                print("  (sin __imu todavia; ¿la app publica IMU? prueba a mover el robot)")
            else:
                txt = json.dumps(d.get("data", d))
                print(f"  topic={d.get('topic')}  data={txt[:300]}")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nFin imu.")


SNIFF_JS = r"""(function(){
  if(!window.__navSniff){ window.__navSniff=1; window.__navlog=[]; window.__navin=[]; window.__navtopics={};
    // SALIDA (app -> robot): comandos de nav que envia la app
    var S = RTCDataChannel.prototype.send;
    RTCDataChannel.prototype.send = function(d){
      try{ var s = (typeof d==='string')? d : '';
        if(s && (s.indexOf('slam_operate')>=0 || s.indexOf('targetPose')>=0 ||
                 s.indexOf('avigation')>=0 || s.indexOf('obstacles_avoid')>=0 ||
                 s.indexOf('api_id')>=0 || s.indexOf('qt_')>=0)){
          window.__navlog.push({t:Date.now(), msg:s.slice(0,3000)});
          if(window.__navlog.length>80) window.__navlog.shift();
        }
      }catch(e){}
      return S.apply(this, arguments);
    };
    // ENTRADA (robot -> app): cuenta TODOS los topics + captura los de nav/path/estado (lo que publica el firmware)
    var jp = JSON.parse;
    JSON.parse = function(s){ var v = jp.apply(this, arguments);
      try{ if(v && v.topic){ var tp=''+v.topic;
        window.__navtopics[tp]=(window.__navtopics[tp]||0)+1;
        if(/nav|path|plan|obst|traj|control|grid|cost|key_info|slam_info|state|reply|response|operate/i.test(tp)){
          window.__navin.push({t:Date.now(), topic:tp, data:JSON.stringify(v.data!==undefined?v.data:v).slice(0,1800)});
          if(window.__navin.length>200) window.__navin.shift();
        }
      }}catch(e){}
      return v;
    };
  }
  return 1;
})()"""


def cmd_sniffnav():
    """SNIFFER del nav nativo: captura TODO lo que pasa por el datachannel para ENTENDER como navega el
    robot. (1) los comandos que envia la app (goal/avoidance), (2) lo que el robot PUBLICA mientras navega
    (path/estado/trayectoria), (3) la tabla de TODOS los topics y su frecuencia. USO: lanzar, poner un
    destino de nav en la app, dejar que el robot vaya, y Ctrl+C. (No mueve el robot; solo escucha.)"""
    cdp = get_cdp()
    cdp.eval(SNIFF_JS)
    print(">>> SNIFF NAV instalado (salida+entrada+topics). EN LA APP pon un destino de navegacion y deja")
    print("    que el robot vaya. Capturo el goal y lo que el robot publica. Ctrl+C para el resumen.\n")
    callog = os.path.join(os.path.dirname(LOGPATH), "navsniff.log")
    lg = open(callog, "a"); lg.write(f"\n===== SNIFFNAV {time.strftime('%Y-%m-%d %H:%M:%S')} =====\n")
    seen_out = 0; seen_in = 0
    try:
        while True:
            try:
                out = json.loads(cdp.eval("JSON.stringify(window.__navlog||[])") or "[]")
                inc = json.loads(cdp.eval("JSON.stringify(window.__navin||[])") or "[]")
            except Exception:
                out = []; inc = []
            while seen_out < len(out):
                m = out[seen_out]; seen_out += 1
                ln = f"[APP->ROBOT] {m.get('msg')}"
                print("\n" + ln + "\n"); lg.write(ln + "\n"); lg.flush()
            while seen_in < len(inc):
                m = inc[seen_in]; seen_in += 1
                ln = f"[ROBOT->APP] {m.get('topic')}  {m.get('data')}"
                print("  " + ln); lg.write(ln + "\n"); lg.flush()
            time.sleep(0.4)
    except KeyboardInterrupt:
        try:
            tops = json.loads(cdp.eval("JSON.stringify(window.__navtopics||{})") or "{}")
        except Exception:
            tops = {}
        print("\n=== TOPICS vistos (frecuencia) ===")
        lg.write("\n=== TOPICS ===\n")
        for tp, n in sorted(tops.items(), key=lambda kv: -kv[1]):
            print(f"  {n:6d}  {tp}"); lg.write(f"{n:6d}  {tp}\n")
        lg.close()
        print(f"\nGuardado en {callog}. Pasamelo (o di 'mira el navsniff') y analizo como navega.")


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
NEAR_BLIND = 0.60   # el LASER no es fiable por debajo de esto (anillo fantasma del cabeceo cocido en la rejilla
                    # a 0.26-0.40m aunque no haya nada). Debajo manda la CAMARA (dmet) + colision.


def scan2_js(sign=CLOUD_SIGN, hlo=None, hhi=None, fmax=2.5, half=0.35, yoff=0.0, minbin=None, mh=None, fmin=None):
    """Consulta la REJILLA LIMPIA window.__grid (celdas de 10cm con conteo de impactos; ya excluye
    el campo cercano <0.6m al construirse). near = celda (>=mh impactos) mas cercana en el corredor
    frontal. yoff = offset de rumbo (rad). mh = umbral de impactos (def MINHITS; sube para ignorar
    el anillo fantasma/ruido transitorio que el mapa real no cree)."""
    mh = MINHITS if mh is None else mh
    fmin = FMIN if fmin is None else fmin
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
            % (yoff, sign, half, fmax, fmin, mh))


def clear_ahead(cdp, off_deg=0.0, mh=None):
    """Distancia al obstaculo mas cercano en el corredor (con offset de rumbo en grados). 999 si libre.
    Por defecto exige PERSIST_MIN impactos (mismo umbral que el mapa real) -> ignora el ruido fantasma
    de valor 2-3 que paralizaba al robot. mh override para otros usos."""
    if mh is None:
        mh = PERSIST_MIN
    try:
        r = json.loads(cdp.eval(scan2_js(yoff=math.radians(off_deg), mh=mh, fmin=NEAR_BLIND)))
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


EXP_FWD_MIN = 0.65     # ESC con obstaculo del LASER a <0.65m (el laser solo ve fiable >NEAR_BLIND=0.6)
EXP_FWD_GOOD = 0.75    # deja de girar y avanza en cuanto hay hueco razonable (menos spinning)
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
            if cmd is None and vb:                           # la camara ve un obstaculo -> PREFIERE GIRAR
                cl = clear_ahead(cdp, +55); cr = clear_ahead(cdp, -55)
                if max(cl, cr) > 0.5:                         # HAY hueco a un lado -> gira hacia el mas abierto
                    gside = 1 if cl >= cr else -1
                    cmd = (0, 0, -AV_TURN if gside > 0 else AV_TURN, 0); ph = "VAV-" + vlbl[:6]
                else:                                        # SIN espacio a los lados (encajonado) -> retrocede si hay sitio
                    rear = clear_ahead(cdp, 180)
                    if rear > REAR_SAFE:
                        cmd = (0, -BACK_SPEED, 0, 0); ph = "VAV-BK"
                    else:                                    # ni lados ni atras -> pivota (ultimo recurso)
                        cmd = (0, 0, -AV_TURN if cl >= cr else AV_TURN, 0); ph = "VAV-PV"
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


# ============================ EXPLORACION POR FRONTERAS ============================
F_CELL = 0.4          # celda de COBERTURA/frontera ('visited') — gruesa a proposito (menos fronteras)
OCELL = 0.2           # celda de OBSTACULOS/costmap/A* — FINA: preserva huecos que el robot si puede cruzar
F_ALIGN = 25.0        # tolerancia de rumbo (deg) antes de avanzar hacia la frontera
F_REACH = 0.45        # se considera alcanzada la frontera a esta distancia
F_REPLAN = 6.0        # recalcula la frontera objetivo cada X s aunque no la alcance
PERSIST_MIN = 4       # valor min en la rejilla (decay, cap 8) para FIJAR una celda en el mapa de obstaculos
OMAP_TTL = 120.0      # s que una pared sigue en el mapa sin reverla (SUBIDO 20->120: que las paredes NO se olviden;
                      # las paredes que sigues viendo se refrescan y se quedan)
                      # (>MINHITS=2: descarta el smear transitorio; las paredes reales suben a 8 y se quedan)
STUCK_SEC = 12.0      # si no avanza STUCK_DISP en este tiempo -> maniobra de DESATASCO (subido: no saltar al maniobrar)
STUCK_DISP = 0.30     # m: desplazamiento minimo para NO considerarse atascado
BRK_TURN_SEC = 2.4    # duracion del giro de desatasco (~150 deg al ritmo de giro del robot)
INJECT_DS = (0.45, 0.65, 0.85)   # distancias (m) por delante a las que se marca obstaculo visto por colision
COBS_TTL = 4.0        # s que vive una marca de obstaculo de CAMARA en la capa decayente (solo para A*, no permanente)


def pull_grid_raw(cdp):
    """Devuelve window.__grid crudo (dict 'ix,iz'->valor). Sin filtrar."""
    try:
        s = cdp.eval("JSON.stringify(window.__grid||{})")
        return json.loads(s) if s else {}
    except Exception:
        return {}


def grid_to_cells(g, minv):
    """Pasa la rejilla cruda (frame nube 0.1m) a celdas OBSTACULO (OCELL=0.2m) con valor >= minv.
    cloud->odom: odom_x=cloud_x, odom_y=-cloud_z."""
    out = set()
    for k, v in g.items():
        if v < minv:
            continue
        try:
            ix, iz = k.split(","); cx = int(ix) / 10.0; cz = int(iz) / 10.0
        except Exception:
            continue
        out.add((round(cx / OCELL), round((-cz) / OCELL)))
    return out


def ahead_cells(x, y, yaw_deg, dists=INJECT_DS):
    """Celdas OBSTACULO (OCELL) a 'dists' metros por delante del robot (camara/colision)."""
    h = math.radians(yaw_deg); c = math.cos(h); s = math.sin(h)
    return {(round((x + d * c) / OCELL), round((y + d * s) / OCELL)) for d in dists}


def omap_to_coarse(omap):
    """Convierte el mapa de obstaculos fino (OCELL) a celdas de cobertura (F_CELL) para la frontera."""
    return {(round(cx * OCELL / F_CELL), round(cy * OCELL / F_CELL)) for (cx, cy) in omap}


def pull_obstacles(cdp):
    """Lee window.__grid (rejilla de obstaculos del navegador, frame nube 0.1m) y la pasa a
    celdas ocupadas en frame ODOM 0.4m. cloud->odom: odom_x=cloud_x, odom_y=-cloud_z."""
    try:
        s = cdp.eval("JSON.stringify(window.__grid||{})")
        g = json.loads(s) if s else {}
    except Exception:
        return set()
    obs = set()
    for k, v in g.items():
        if v < MINHITS:
            continue
        try:
            ix, iz = k.split(","); cx = int(ix) / 10.0; cz = int(iz) / 10.0
        except Exception:
            continue
        obs.add((round(cx / F_CELL), round((-cz) / F_CELL)))   # cloud_z -> -odom_y
    return obs


def _line_blocked(a, b, obs):
    """¿Cruza la recta a->b (celdas enteras) alguna celda ocupada? (linea de vista aproximada)"""
    ax, ay = a; bx, by = b
    n = max(abs(bx - ax), abs(by - ay))
    if n == 0:
        return False
    for i in range(1, n + 1):
        c = (round(ax + (bx - ax) * i / n), round(ay + (by - ay) * i / n))
        if c in obs:
            return True
    return False


def pick_frontier(visited, obs, x, y, relax=False, bad=None, yaw=None):
    """Frontera = celda DESCONOCIDA (ni visitada ni obstaculo) adyacente a lo explorado.
    Devuelve el centro (tx,ty) en ODOM de la frontera mas cercana y alcanzable, o None.
    relax=True ignora linea de vista y amplia el radio (cuando la version estricta no halla nada).
    bad = set de celdas (F_CELL) descartadas. yaw (deg) = penaliza fronteras que obligan a girar
    mucho -> prefiere seguir hacia delante (evita el ping-pong entre fronteras a lados opuestos)."""
    if not visited:
        return None
    bad = bad or set()
    nb = ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1))
    cands = {}
    for (vx, vy) in visited:
        for dx, dy in nb:
            c = (vx + dx, vy + dy)
            if c in visited or c in obs or c in bad:
                continue
            cands[c] = cands.get(c, 0) + 1          # cuantas celdas exploradas tocan esta frontera (apertura)
    if not cands:
        return None
    rc = (round(x / F_CELL), round(y / F_CELL))
    dmax = 9.0 if relax else 4.0
    best = None; bestcost = 1e9
    for c, cnt in cands.items():
        cx_, cy_ = c[0] * F_CELL, c[1] * F_CELL
        dist = math.hypot(cx_ - x, cy_ - y)
        if dist < 0.6 or dist > dmax:
            continue
        if not relax and _line_blocked(rc, c, obs):
            continue
        cost = dist - 0.25 * cnt                     # cerca + frontera ancha (apertura) = mejor
        if yaw is not None:                          # sesgo hacia delante: penaliza girar mucho
            be = abs((math.degrees(math.atan2(cy_ - y, cx_ - x)) - yaw + 180) % 360 - 180)
            cost += 0.004 * be                       # 180deg -> +0.72 (≈ frontera 0.7m mas lejos)
        if cost < bestcost:
            bestcost = cost; best = (cx_, cy_)
    return best


def pick_frontier_gain(visited, obs, x, y, yaw=None, bad=None):
    """v2: frontera por GANANCIA DE INFORMACION. Agrupa las celdas frontera en clusters (componentes
    conexas) y prefiere el cluster GRANDE y cercano: una puerta/hueco ancho abre a una habitacion
    entera (cluster grande), un rincon es minusculo. Manda al robot a CRUZAR puertas, no a peinar
    esquinas. Devuelve (tx,ty) o None."""
    if not visited:
        return None
    bad = bad or set()
    nb = ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1))
    front = set()
    for (vx, vy) in visited:
        for dx, dy in nb:
            c = (vx + dx, vy + dy)
            if c in visited or c in obs or c in bad:
                continue
            front.add(c)
    if not front:
        return None
    clusters = []; seen = set()                          # componentes conexas de la frontera
    for f in front:
        if f in seen:
            continue
        stack = [f]; comp = []
        while stack:
            cc = stack.pop()
            if cc in seen:
                continue
            seen.add(cc); comp.append(cc)
            for dx, dy in nb:
                nn = (cc[0] + dx, cc[1] + dy)
                if nn in front and nn not in seen:
                    stack.append(nn)
        clusters.append(comp)
    rc = (round(x / F_CELL), round(y / F_CELL))
    best = None; bestscore = -1e9
    for comp in clusters:
        size = len(comp)                                 # apertura del hueco (proxy de ganancia)
        reach = [c for c in comp if not _line_blocked(rc, c, obs)]
        cand = reach or comp
        cnear = min(cand, key=lambda c: (c[0] - rc[0]) ** 2 + (c[1] - rc[1]) ** 2)
        cx_, cy_ = cnear[0] * F_CELL, cnear[1] * F_CELL
        dist = math.hypot(cx_ - x, cy_ - y)
        if dist < 0.5 or dist > 12.0:                    # permite objetivos MAS LEJOS
            continue
        score = 1.4 * size - 0.5 * dist                  # apertura GRANDE pesa mas; distancia penaliza poco -> goals lejos
        if yaw is not None:                              # sesgo hacia delante (menos giro)
            be = abs((math.degrees(math.atan2(cy_ - y, cx_ - x)) - yaw + 180) % 360 - 180)
            score -= 0.008 * be
        if score > bestscore:
            bestscore = score; best = (cx_, cy_)
    return best


# ---------------- A* sobre costmap (planificacion deliberativa) ----------------
INFL_HARD = 1          # celdas OCELL bloqueadas alrededor del obstaculo (1*0.2m=0.2m ~= radio del robot)
INFL_SOFT = 1          # celdas OCELL con coste extra (BAJADO a 0.2m: menos margen, pasa por huecos mas justos)
PLAN_SEC = 1.5         # recalcula el A* cada X s (el costmap cambia al descubrir obstaculos)
LOOKAHEAD = 0.8        # m: distancia del 'carrot' (punto del path al que apunta el robot, pure-pursuit)


def build_costmap(obs):
    """Infla los obstaculos: <=INFL_HARD celdas alrededor = bloqueado (inf); hasta INFL_SOFT = coste
    extra decreciente. Devuelve dict celda->coste (inf = intransitable)."""
    cost = {}
    for (ox, oy) in obs:
        for dx in range(-INFL_SOFT, INFL_SOFT + 1):
            for dy in range(-INFL_SOFT, INFL_SOFT + 1):
                c = (ox + dx, oy + dy); cheb = max(abs(dx), abs(dy))
                if cheb <= INFL_HARD:
                    cost[c] = math.inf
                elif cost.get(c) != math.inf:
                    cost[c] = max(cost.get(c, 0.0), float(INFL_SOFT - cheb + 1) * 3.0)
    return cost


def astar(start, goal, cost, margin=8, vcost=None):
    """A* 8-conexo en rejilla de celdas. Evita celdas 'inf' (salvo la meta). vcost = dict celda->coste
    extra por YA VISITADA (para que el path prefiera terreno NUEVO y no re-pise). Lista o None."""
    vcost = vcost or {}
    if start == goal:
        return [start]
    minx = min(start[0], goal[0]) - margin; maxx = max(start[0], goal[0]) + margin
    miny = min(start[1], goal[1]) - margin; maxy = max(start[1], goal[1]) + margin
    nbs = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
           (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414))

    def h(a):
        return math.hypot(a[0] - goal[0], a[1] - goal[1])
    openh = [(h(start), 0.0, start)]; came = {}; g = {start: 0.0}; closed = set(); it = 0
    while openh and it < 9000:
        it += 1
        _, gc, cur = heapq.heappop(openh)
        if cur == goal:
            path = [cur]
            while cur in came:
                cur = came[cur]; path.append(cur)
            return path[::-1]
        if cur in closed:
            continue
        closed.add(cur)
        for dx, dy, sw in nbs:
            nc = (cur[0] + dx, cur[1] + dy)
            if nc[0] < minx or nc[0] > maxx or nc[1] < miny or nc[1] > maxy:
                continue
            cv = cost.get(nc, 0.0)
            if cv == math.inf and nc != goal:
                continue
            ng = gc + sw * (1.0 + (0.0 if cv == math.inf else cv) + vcost.get(nc, 0.0))
            if ng < g.get(nc, 1e18):
                g[nc] = ng; came[nc] = cur; heapq.heappush(openh, (ng + h(nc), ng, nc))
    return None


def path_carrot(pts, x, y, look=LOOKAHEAD):
    """Punto del path ~'look' m por delante del robot (pure-pursuit simple). None si no hay path."""
    if not pts:
        return None
    di = min(range(len(pts)), key=lambda i: (pts[i][0] - x) ** 2 + (pts[i][1] - y) ** 2)
    acc = 0.0
    for i in range(di, len(pts) - 1):
        acc += math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
        if acc >= look:
            return pts[i + 1]
    return pts[-1]


def cmd_frontier(secs, vshare=None, lock=None, stop_event=None):
    """Exploracion DELIBERATIVA por fronteras: en vez de vagar reactivamente, construye el mapa
    libre/desconocido y va a la frontera (borde de lo explorado) mas cercana alcanzable, con toda
    la esquiva (colision + camara VAV + laser ESC) como prioridad. Cobertura sistematica: busca
    activamente la salida en vez de orbitar. Ctrl+C = STOP."""
    secs = max(5, min(600, secs))
    cdp = get_cdp()
    if not wait_for_odom(cdp):
        print("!! Sin odometria. ¿SLAM activo de pie?"); return
    print(f">>> FRONTIER {secs:.0f}s: cobertura sistematica (va a la frontera de lo explorado).")
    print(f"    ESPACIO LIBRE alrededor. Mando en la mano (L2+B). log -> {LOGPATH}")
    lg = open(LOGPATH, "a")
    lg.write(f"\n=== FRONTIER {time.strftime('%H:%M:%S')} {secs:.0f}s ===\n")
    vision_on = "novision" not in sys.argv
    if vision_on:
        threading.Thread(target=yolo_worker, daemon=True).start()
    try:
        cdp.eval("window.__grid={}")            # rejilla fresca (el reguero de runs viejos crea anillos)
        cdp.eval("window.__omap=[]")            # mapa de obstaculos FRESCO (se reconstruye del LiDAR; no arrastra falsos)
        cdp.eval(LOWSTATE_JS)                   # hook rt/lf/lowstate -> contacto rapido por IMU/par
        print("Rejilla y mapa reiniciados. Esperando ~2s a que se llene del entorno actual...")
        time.sleep(2.0)
    except Exception:
        pass
    t0 = time.time(); tprint = 0
    state = "GO"; esc_t0 = 0; scan_t = 0; best_off = 0; best_clr = 0; esc_dir = 0
    visited = {}                                 # cobertura: celdas 0.4m pisadas (= mapa LIBRE conocido)
    omap = {}                                    # MAPA de obstaculos con CADUCIDAD: celda -> ultimo instante visto
    oset = set()                                 # celdas ACTIVAS (no caducadas) -> costmap/A*/frontera/viz
    try:                                         # carga cobertura y mapa previos (continua donde lo dejo)
        s = cdp.eval("JSON.stringify(window.__visited||{})")
        if s:
            for k, v in json.loads(s).items():
                a, b = k.split(","); visited[(int(a), int(b))] = v
        s2 = cdp.eval("JSON.stringify(window.__omap||[])")
        if s2:
            t_ld = time.time()
            for k in json.loads(s2):
                a, b = k.split(","); omap[(int(a), int(b))] = t_ld
        print(f"Cobertura previa: {len(visited)} celdas, mapa: {len(omap)} obstaculos (continuo desde ahi).")
    except Exception:
        pass

    def save_state():
        try:
            cdp.eval("window.__visited=" + json.dumps({f"{a},{b}": v for (a, b), v in visited.items()}))
            cdp.eval("window.__omap=" + json.dumps([f"{a},{b}" for (a, b) in oset]))
        except Exception:
            pass
    fhist = []; prev_fwd = False; recov = None; ncol = 0; rside = 1; last_col_t = -99   # +enfriamiento colision
    low_t = 0; last_low = None; lt_base = []; ah_base = []; lowcal_t = 0   # contacto por lowstate (par/accel)
    vcam_t = 0; vside = 1; last_od = None; od_change_t = time.time()
    vhealth_t = 0; vstale_t = 0
    tgt = None; tgt_t = 0; ndone = 0              # frontera objetivo actual
    plan_pts = []; plan_t = 0; tgt_planned = None; infl_cells = []; carrot = None   # A* path
    badf = {}                                     # fronteras descartadas (celda F_CELL -> expira en t)
    tgt_best = 1e9; tgt_best_t = 0                 # progreso hacia el objetivo (compromiso, anti flip-flop)
    nstop = 0                                      # ciclos seguidos parado por obstaculo (firmware: para+rodea)
    cobs = {}                                      # capa de obstaculos de CAMARA decayente (OCELL -> expira en t); SOLO para A*
    vav_run = 0; vav_suppress_until = 0; ndeg = 0   # metacognicion: detecta camara DEGRADADA (bloquea en todo rumbo)
    last_obs = set(); viz_t = 0; viz_obs_t = 0; omap_t = 0    # estado ventana + acumulacion del mapa
    colmap = set()                                # colisiones PERMANENTES (no caducan: no re-chocar en el mismo sitio)
    dhist = []; brk = None; brk_cool = 0; nbrk = 0; nbrk_rescue = 0   # pos + desatasco + rescates al agotar fronteras
    trail = []; map_t = 0                                              # recorrido odom + guardado periodico del mapa
    def sc(v):
        return f"{v:.2f}" if (v is not None and v < 900) else ("—" if v is not None else "·")
    try:
        while time.time() - t0 < secs and not (stop_event is not None and stop_event.is_set()):
            now = time.time()
            od = read_poll(cdp).get("odom")
            x = y = yaw = 0.0
            if od:
                x, y, yaw = od[0], od[1], math.degrees(yaw_of(od))
                vk = (round(x / F_CELL), round(y / F_CELL))
                visited[vk] = visited.get(vk, 0) + 1
            if od and (not trail or math.hypot(x - trail[-1][0], y - trail[-1][1]) > 0.05):
                trail.append((x, y))                       # recorrido de odometria (para el mapa exportado)
            if int(now - t0) % 5 == 0:
                save_state()
            if now - map_t > 30:                           # guarda el mapa a PNG+JSON cada 30s (para revisarlo)
                save_map_snapshot(set(visited.keys()), oset, trail, tgt, tag=time.strftime('%H:%M:%S'))
                map_t = now
            # --- MAPA PERSISTENTE: acumula celdas firmes de la rejilla (no las olvida al girarse) ---
            if now - omap_t > 0.5:
                for c in grid_to_cells(pull_grid_raw(cdp), PERSIST_MIN):
                    if math.hypot(c[0] * OCELL - x, c[1] * OCELL - y) < NEAR_BLIND:
                        continue                           # IGNORA el campo cercano (anillo fantasma del cabeceo)
                    omap[c] = now                          # refresca lo que el LiDAR ve AHORA (>0.6m, fiable)
                omap = {c: t for c, t in omap.items() if now - t < OMAP_TTL}   # purga lo MUY viejo (persona que se fue)
                oset = set(omap.keys()) | colmap           # paredes (TTL largo) + colisiones PERMANENTES
                omap_t = now
            # --- HISTORIAL DE POSICION (para detectar atasco) ---
            if od:
                dhist.append((now, x, y))
            dhist = [h for h in dhist if now - h[0] <= STUCK_SEC]
            c0 = clear_ahead(cdp, 0); rear = None; cmd = None; ph = ""
            cf_cam = 1.0; vfresh = False

            # --- ODOM CONGELADA (feed muerto != colision) ---
            if od is not None:
                if last_od is None or od[0] != last_od[0] or od[1] != last_od[1] or od[6] != last_od[6]:
                    od_change_t = now
                last_od = od
            odom_live = (now - od_change_t) < 1.5
            if now - od_change_t > 3.0:
                print("\n  ODOMETRIA CONGELADA (3s). STOP. Reactiva el SLAM en la app y reintenta.")
                lg.write("ODOM-FROZEN\n"); break

            # --- COLISION por odom (choque que el laser no ve) ---
            # ENDURECIDO (anti falsos): avance SOSTENIDO (>=10 ciclos de avance real en la ventana) +
            # enfriamiento de 4s. Evita marcar colision por titubeos/giros en rincon apretado.
            if prev_fwd and od and odom_live:
                fhist.append((now, x, y))
            fhist = [h for h in fhist if now - h[0] <= 2.0]
            # lowstate ~5Hz: par max de patas + accel horizontal
            if now - low_t > 0.2:
                lw = read_low(cdp)
                if lw:
                    last_low = (math.hypot(lw.get("ax", 0.0), lw.get("ay", 0.0)), lw.get("legtau", 0.0))
                low_t = now
            cur_ah, cur_lt = last_low if last_low else (None, None)
            mvd = math.hypot(x - fhist[0][1], y - fhist[0][2]) if len(fhist) >= 2 else 0.0
            if prev_fwd and cur_lt is not None and mvd > 0.10:        # baseline cuando avanza NORMAL (se mueve)
                lt_base.append(cur_lt); lt_base = lt_base[-40:]
                ah_base.append(cur_ah); ah_base = ah_base[-40:]
            if now - lowcal_t > 3.0 and cur_lt is not None:           # log de calibracion (normal vs choque)
                bl = sorted(lt_base)[len(lt_base) // 2] if lt_base else 0
                ba = sorted(ah_base)[len(ah_base) // 2] if ah_base else 0
                lg.write(f"  CONTACT-CAL legtau={cur_lt:.1f}(base{bl:.1f}) accelH={cur_ah:.2f}(base{ba:.2f}) mvd={mvd:.2f}\n")
                lowcal_t = now
            # CONTACTO: odom-stall lento (fiable) O rapido por par/accel de lowstate (~0.6s, "empuja mucho")
            contact = False; ctype = ""
            if recov is None and now - last_col_t > 4.0:
                if len(fhist) >= 10 and now - fhist[0][0] >= 1.5 and mvd < 0.05:
                    contact = True; ctype = "odom"
                elif (IMU_CONTACT and prev_fwd and cur_lt is not None and len(fhist) >= 6
                      and now - fhist[0][0] >= 0.6 and mvd < 0.04):
                    bl = sorted(lt_base)[len(lt_base) // 2] if len(lt_base) >= 5 else 15.0
                    ba = sorted(ah_base)[len(ah_base) // 2] if len(ah_base) >= 5 else 1.5
                    if cur_lt > bl * 1.6 + 4.0 or cur_ah > ba + 2.5:    # par/accel muy por encima de andar normal
                        contact = True; ctype = "imu"
            if contact:
                    disp = mvd
                    ncol += 1; last_col_t = now
                    inject_obstacle(cdp, x, y, yaw)
                    # marca PERMANENTE y ANCHA donde choco (no vuelve a chocar ahi); blob lateral para que A* lo rodee
                    yr = math.radians(yaw); fxx, fyy = math.cos(yr), math.sin(yr); pxx, pyy = -fyy, fxx
                    for d in (0.35, 0.5, 0.65, 0.8):
                        for L in (-0.3, -0.15, 0.0, 0.15, 0.3):
                            colmap.add((round((x + d * fxx + L * pxx) / OCELL), round((y + d * fyy + L * pyy) / OCELL)))
                    oset |= colmap
                    cl = clear_ahead(cdp, +55); cr = clear_ahead(cdp, -55)
                    rside = 1 if cl >= cr else -1
                    recov = {"ph": "BACK", "t0": now}; fhist = []; tgt = None    # replanifica tras chocar
                    print(f"\n  COLISION #{ncol} [{ctype}] (disp={disp:.2f}). Marco obstaculo y recupero.")
                    lg.write(f"COLISION #{ncol} [{ctype}] disp={disp:.2f} pos=({x:+.2f},{y:+.2f}) yaw={yaw:+.0f} "
                             f"legtau={cur_lt} c0={sc(c0)}\n")
                    save_crash_image(cdp, ncol, x, y, yaw, c0)

            # --- RECUPERACION (prioridad). SIN MARCHA ATRAS: pivota en el sitio (no se puede caer de espaldas) ---
            if recov is not None:
                el = now - recov["t0"]
                if recov["ph"] == "BACK":                 # (ya no retrocede) -> pasa directo a pivotar
                    recov = {"ph": "TURN", "t0": now}; el = 0
                if recov is not None and recov["ph"] == "TURN":
                    if el < 1.6:                          # pivota un poco mas (compensa que no retrocede)
                        cmd = (0, 0, -AV_TURN if rside > 0 else AV_TURN, 0); ph = "R-TURN"
                    else:
                        recov = {"ph": "GO", "t0": now}; el = 0
                if recov is not None and recov["ph"] == "GO":
                    if el < 1.0 and c0 > EXP_FWD_MIN:
                        cmd = (0, FWD_SPEED, 0, 0); ph = "R-GO "
                    else:
                        recov = None; state = "GO"

            # --- DESATASCO: si no avanza en STUCK_SEC, retro + giro grande IGNORANDO la camara ---
            if (recov is None and brk is None and now > brk_cool and len(dhist) >= 2
                    and now - dhist[0][0] >= STUCK_SEC * 0.9
                    and math.hypot(x - dhist[0][1], y - dhist[0][2]) < STUCK_DISP):

                def nov90(o):
                    h = math.radians(yaw + o)
                    kx = round((x + 1.2 * math.cos(h)) / F_CELL); ky = round((y + 1.2 * math.sin(h)) / F_CELL)
                    return sum(visited.get((kx + dx, ky + dy), 0) for dx in (-1, 0, 1) for dy in (-1, 0, 1))
                bdir = -AV_TURN if nov90(+90) <= nov90(-90) else AV_TURN     # gira hacia el lado MENOS pisado
                nbrk += 1; brk = {"ph": "TURN", "t0": now, "dir": bdir}; tgt = None; plan_pts = []
                print(f"\n  DESATASCO #{nbrk}: {STUCK_SEC:.0f}s sin avanzar en ({x:+.2f},{y:+.2f}); giro grande (sin retroceso).")
                lg.write(f"DESATASCO #{nbrk} pos=({x:+.2f},{y:+.2f}) dir={'IZQ' if bdir < 0 else 'DCHA'} omap={len(oset)}\n")
            if brk is not None:                           # SIN MARCHA ATRAS: solo pivota (giro grande en el sitio)
                el = now - brk["t0"]
                if el < BRK_TURN_SEC:
                    cmd = (0, 0, brk["dir"], 0); ph = "BRK-TR"
                else:
                    brk = None; brk_cool = now + 6.0; dhist = []; tgt = None; plan_pts = []; state = "GO"

            # --- VISION (camara ve lo que el laser no) ---
            vb = False; vlbl = ""; dr = 0; vdist = ""; vclose = False; vstrong = False
            if vision_on:
                if now - vcam_t > 0.5:
                    try:
                        j = cdp.eval(CAM_JS)
                        if j and j.startswith("data:image"):
                            with vlock:
                                vision["jpg"] = j; vision["n"] += 1
                    except Exception:
                        pass
                    vcam_t = now
                with vlock:
                    dr = vision.get("dratio", 0); cf_cam = vision.get("cf", 1.0); vts = vision.get("ts", 0)
                    vfresh = (now - vts < 3.0)
                    if vision["block"] and vfresh:
                        vb = True; vlbl = vision["label"]; vside = vision["side"]
                        vdist = vision.get("dist", ""); vclose = vision.get("close", False)
                        vstrong = vision.get("strong", False)
                if not vfresh and now - t0 > 6 and now - vstale_t > 5:
                    print("  [!] vision caducada >3s (navego solo con laser)")
                    lg.write("VISION-STALE\n"); vstale_t = now
            # SOLO esquiva si el obstaculo esta CERCA (vclose). Lo 'medio'/'lejos' no le frena.
            # METACOGNICION: si la camara bloquea SIN PARAR mientras el robot gira (=ve "cerca" en TODAS
            # direcciones), esta DEGRADADA (suelo mal segmentado / cabeza inclinada) -> la ignoro un rato
            # y navego con laser, que ve libre. Asi no se queda girando para siempre.
            # EL LASER MANDA: si el corredor esta despejado mas alla de CAM_TRUST_C0, la camara NO veta
            # (sus 'cerca' son falsos en suelos reflectantes). Solo esquiva por camara si el laser corrobora.
            # EL LASER MANDA (puro, como el run bueno): la camara NO puede vetar un camino que el laser ve
            # despejado. (El bypass por señal fuerte se quito: combinado con la analogia reabria el atasco.)
            lidar_clear_ahead = (c0 >= CAM_TRUST_C0)
            # v2 MENOS COLISIONES: aunque el laser mande y NO se haga VAV, lo que la camara ve CERCA
            # (mesa que el laser no ve) se mete en cobs -> A* RODEA y no se choca. La camara INFORMA al
            # plan sin secuestrar el control (sin spin). Solo señales fiables (fuerte/yolo), no el suelo.
            if vb and vclose and (vstrong or vlbl not in ("floor", "edge", "depth")):
                for c in ahead_cells(x, y, yaw, (0.4, 0.6)):
                    cobs[c] = now + COBS_TTL
            # POR DEFECTO la camara NO gira (solo informo cobs arriba) = navegacion PURA LiDAR como el firmware.
            # G1_CAM_REACTIVE=1 reactiva el giro por camara (para comparar).
            vav_fired = (CAM_REACTIVE and cmd is None and vb and vclose and now > vav_suppress_until
                         and not lidar_clear_ahead)
            if vav_fired:
                vav_run += 1
                if vav_run > 30:                          # ~3-4s bloqueando en todo rumbo -> camara no fiable
                    vav_suppress_until = now + 6.0; vav_run = 0; vav_fired = False; ndeg += 1
                    with vlock:
                        vision["dump"] = ndeg             # pide al worker GUARDAR foto+diagnostico para analizar
                    lg.write(f"VAV-DEGRADED #{ndeg}: camara bloquea en todas direcciones -> foto+ignoro 6s (laser)\n")
                    print(f"  [!] camara bloquea en TODO rumbo -> guardo foto #{ndeg} y la ignoro 6s (navego laser)")
            else:
                vav_run = max(0, vav_run - 3)              # decae (no resetea de golpe: aguanta F-ARC puntuales)
            if vav_fired:                                # camara ve obstaculo CERCA -> PREFIERE GIRAR (reactivo)
                for c in ahead_cells(x, y, yaw, (0.4, 0.6)):
                    cobs[c] = now + COBS_TTL             # capa decayente para que A* RODEE lo que ve la camara
                cl = clear_ahead(cdp, +55); cr = clear_ahead(cdp, -55)
                gside = 1 if cl >= cr else -1            # SIN retroceso: SIEMPRE pivota al lado mas abierto
                cmd = (0, 0, -AV_TURN if gside > 0 else AV_TURN, 0)
                ph = ("VAV-" + vlbl[:6]) if max(cl, cr) > 0.5 else "VAV-PV"
                # NO anula tgt: A* (con la silla ya en cobs) replanifica una ruta que la RODEA

            # --- IR A LA FRONTERA (estilo firmware: A* + parar/rodear, NO spin reactivo) ---
            if cmd is not None:
                pass
            elif state == "GO":
                if True:
                    reached = tgt is not None and math.hypot(tgt[0] - x, tgt[1] - y) < F_REACH
                    # COMPROMISO: no re-elegir por temporizador (causaba flip-flop entre fronteras opuestas).
                    # Solo si se progresa hacia el objetivo; si 12s sin acercarse -> descarta y prueba otra.
                    if tgt is not None and not reached:
                        d_now = math.hypot(tgt[0] - x, tgt[1] - y)
                        if d_now < tgt_best - 0.12:
                            tgt_best = d_now; tgt_best_t = now
                        elif now - tgt_best_t > 12.0:
                            badf[(round(tgt[0] / F_CELL), round(tgt[1] / F_CELL))] = now + 6.0
                            lg.write(f"NO-PROGRESS tgt=({tgt[0]:+.1f},{tgt[1]:+.1f}) -> descarto\n")
                            tgt = None
                    if tgt is None or reached:
                        omap_coarse = omap_to_coarse(oset)       # mapa fino activo -> celdas de cobertura para la frontera
                        bad = {c for c, exp in badf.items() if exp > now}   # fronteras descartadas (A* fallo)
                        nt = pick_frontier_gain(visited, omap_coarse, x, y, yaw=yaw, bad=bad)   # v2: por GANANCIA (puertas)
                        if nt is None:                           # fallback: la mas cercana alcanzable
                            nt = pick_frontier(visited, omap_coarse, x, y, relax=False, bad=bad, yaw=yaw)
                        if nt is None:
                            nt = pick_frontier(visited, omap_coarse, x, y, relax=True, bad=bad, yaw=yaw)
                        tgt_t = now
                        if nt is None:
                            ndone += 1
                            if ndone >= 3:               # sin frontera alcanzable -> NO te rindas: rescate
                                if nbrk_rescue >= 4:     # ya lo intente varias veces -> de verdad agotado
                                    print("\n  EXPLORACION COMPLETA: sin fronteras tras varios rescates.")
                                    lg.write("FRONTIER-DONE\n"); break
                                badf.clear(); ndone = 0; nbrk_rescue += 1   # limpia descartes y SAL de la zona
                                cl = clear_ahead(cdp, +55); cr = clear_ahead(cdp, -55)
                                brk = {"ph": "BACK", "t0": now, "dir": -AV_TURN if cl >= cr else AV_TURN}
                                lg.write(f"FRONTIER-STUCK -> rescate #{nbrk_rescue}: limpio descartes + giro de salida\n")
                            tgt = None
                        else:
                            ndone = 0
                            if nt != tgt:
                                lg.write(f"FRONTIER-> ({nt[0]:+.2f},{nt[1]:+.2f}) d={math.hypot(nt[0]-x,nt[1]-y):.2f} "
                                         f"explored={len(visited)}\n")
                            tgt = nt; tgt_best = math.hypot(nt[0] - x, nt[1] - y); tgt_best_t = now   # reinicia progreso
                    # FIRMWARE + CAMARA-BAJA: hay obstaculo DELANTE si el LASER lo ve (c0) O la CAMARA reconoce
                    # un objeto bajo cerca (vstrong: mesa/silla que el LiDAR de cabeza no ve). Lo marco AHORA y
                    # fuerzo replan -> A* rutea rodeandolo este ciclo (el carrot ya apunta al rodeo).
                    lidar_block = (c0 <= STOP_CLEAR)
                    cam_block = (vb and vstrong)                     # objeto bajo fiable visto por la camara
                    blocked = lidar_block or cam_block
                    # NO re-marcar las paredes del LiDAR (ya estan en oset por la rejilla; re-marcarlas pintaba
                    # un ANILLO al girar y se auto-amurallaba). Solo la mesa de CAMARA (capa decayente) y replan.
                    if cam_block and not lidar_block and tgt is not None:
                        h0 = math.radians(yaw)
                        for dd in (0.5, 0.7):
                            oc = (round((x + dd * math.cos(h0)) / OCELL), round((y + dd * math.sin(h0)) / OCELL))
                            cobs[oc] = now + COBS_TTL                 # camara -> decayente (si fuera falso, se borra)
                        oset |= {c for c, ex in cobs.items() if ex > now}
                    if blocked and tgt is not None:
                        plan_pts = []                                # fuerza replan A* (rodea con el mapa actual)
                    if tgt is not None and ((not plan_pts) or (tgt != tgt_planned) or (now - plan_t > PLAN_SEC)):
                        # --- PLANIFICA A*: costmap = mapa LiDAR persistente + capa camara decayente ---
                        cobs_act = {c for c, exp in cobs.items() if exp > now}
                        cm = build_costmap(oset | cobs_act)
                        # v2 COSTE DE RE-PISAR: cada celda ya visitada (F_CELL) -> sus 4 sub-celdas OCELL con
                        # coste extra ~ veces pisada. Asi A* prefiere terreno NUEVO (menos loops/revisitas).
                        vcost = {}
                        for (vx, vy), cnt in visited.items():
                            pen = min(0.6 * cnt, 4.0)
                            for sx in (0, 1):
                                for sy in (0, 1):
                                    oc = (round(vx * F_CELL / OCELL) + sx, round(vy * F_CELL / OCELL) + sy)
                                    if vcost.get(oc, 0) < pen:
                                        vcost[oc] = pen
                        scell = (round(x / OCELL), round(y / OCELL))
                        gcell = (round(tgt[0] / OCELL), round(tgt[1] / OCELL))
                        cells = astar(scell, gcell, cm, vcost=vcost)
                        plan_pts = [(c[0] * OCELL, c[1] * OCELL) for c in cells] if cells else []
                        infl_cells = [(c[0] * OCELL, c[1] * OCELL) for c, v in cm.items()
                                      if v == math.inf and c not in oset]      # halo de inflado en METROS (viz)
                        plan_t = now; tgt_planned = tgt
                        if not plan_pts:                              # A* no llega -> descarta esa frontera 12s y prueba otra
                            badf[(round(tgt[0] / F_CELL), round(tgt[1] / F_CELL))] = now + 6.0
                            lg.write(f"A*-FAIL goal=({tgt[0]:+.1f},{tgt[1]:+.1f}) -> descarto, pruebo otra\n")
                            tgt = None; tgt_t = now
                    # --- DIRECCION estilo FIRMWARE: sigue el path A*; si hay obstaculo DELANTE, NO embiste:
                    #     lo marca, A* REPLANIFICA rodeando, y gira hacia la nueva ruta (parar+rodear, sin spin). ---
                    if tgt is not None and plan_pts:
                        carrot = path_carrot(plan_pts, x, y)          # del plan que YA rodea el obstaculo
                        bearing = math.degrees(math.atan2(carrot[1] - y, carrot[0] - x))
                        e = (bearing - yaw + 180) % 360 - 180         # wrap a [-180,180] EN GRADOS
                        turn = -AV_TURN if e > 0 else AV_TURN
                        if blocked:                                   # algo DELANTE -> gira hacia el rodeo, NO avances
                            cmd = (0, 0, turn, 0); ph = "REROUTE"
                            nstop += 1
                            if nstop > 25:                            # ~2.5s sin poder rodear -> encajonado -> media vuelta
                                cl = clear_ahead(cdp, +55); cr = clear_ahead(cdp, -55)
                                brk = {"ph": "TURN", "t0": now, "dir": -AV_TURN if cl >= cr else AV_TURN}
                                tgt = None; nstop = 0; lg.write("BOXED -> giro de salida\n")
                        else:
                            nstop = 0
                            cautious = ((vision_on and vfresh and cf_cam < 0.45) or c0 < 1.2 or (vb and vclose))
                            spd = 0.28 if cautious else FWD_SPEED
                            if abs(e) > 55:                           # muy desalineado -> pivota en el sitio
                                cmd = (0, 0, turn, 0); ph = "F-TRN"
                            elif abs(e) > 16:                         # desviado -> ARCO: avanza Y gira a la vez
                                cmd = (0, spd, turn, 0); ph = "F-ARC"
                            else:                                     # alineado -> recto
                                cmd = (0, spd, 0, 0); ph = "F-GOsl" if spd < FWD_SPEED else "F-A* "
                    elif blocked:                        # sin ruta y algo delante -> PARA (no embiste); replanifica
                        cmd = (0, 0, 0, 0); ph = "STOP "; nstop += 1
                        if nstop > 25:
                            cl = clear_ahead(cdp, +55); cr = clear_ahead(cdp, -55)
                            brk = {"ph": "TURN", "t0": now, "dir": -AV_TURN if cl >= cr else AV_TURN}; nstop = 0
                    else:                                # sin ruta/objetivo y libre -> avanza recto (replanifica)
                        nstop = 0; cmd = (0, FWD_SPEED, 0, 0); ph = "F-GO "; carrot = None

            line = (f"t={time.time()-t0:5.1f}/{secs:.0f} {ph} pos=({x:+.2f},{y:+.2f}) yaw={yaw:+6.1f} "
                    f"c0={sc(c0)} rear={sc(rear)} tgt={('(%+.1f,%+.1f)' % tgt) if tgt else '-'} "
                    f"vis={(vlbl+':'+vdist) if vb else '-'} expl={len(visited)} "
                    f"cmd=(lx={cmd[0]:+.2f},ly={cmd[1]:+.2f},rx={cmd[2]:+.2f})")
            lg.write(line + "\n"); lg.flush()
            if vision_on and now - vhealth_t > 3:
                with vlock:
                    vage = now - vision.get("ts", 0); vdtf = vision.get("dtf", -1)
                    vblk = vision.get("block"); vd = vision.get("dratio")
                lg.write(f"  VHEALTH dtf={vdtf}s age={vage:.1f}s block={vblk} dr={vd}\n")
                vhealth_t = now
            if time.time() - tprint > 0.4:
                print("  " + line); tprint = time.time()
            # --- publica estado para la ventana de mapa (modo viz) ---
            if vshare is not None:
                upd_vis = (now - viz_t > 0.5)
                if upd_vis:
                    vsnap = list(visited.keys()); viz_t = now
                with vlock:
                    cam_jpg = vision.get("jpg"); cam_dmet = vision.get("dmet", 999)
                vtxt = (f"{vlbl} {('%.2fm' % cam_dmet) if cam_dmet < 9 else ''}".strip()
                        + (" CERCA" if vclose else "")) if vb else "libre"
                with lock:
                    vshare["x"] = x; vshare["y"] = y; vshare["yaw"] = yaw; vshare["ph"] = ph
                    vshare["tgt"] = tgt; vshare["expl"] = len(visited); vshare["t"] = now - t0
                    vshare["col"] = ncol; vshare["carrot"] = carrot
                    vshare["cam"] = cam_jpg; vshare["vtxt"] = vtxt        # camara + veredicto para el panel
                    vshare["path"].append((x, y))
                    if len(vshare["path"]) > 4000:
                        del vshare["path"][:len(vshare["path"]) - 4000]
                    if upd_vis:
                        vshare["visited"] = vsnap                              # celdas F_CELL (cobertura)
                        vshare["obs"] = [(cx * OCELL, cy * OCELL) for (cx, cy) in oset]   # METROS (mapa activo fino)
                        vshare["cobs"] = [(c[0] * OCELL, c[1] * OCELL) for c, exp in cobs.items() if exp > now]  # camara (transitorio)
                        vshare["plan"] = list(plan_pts); vshare["infl"] = list(infl_cells)   # ya en METROS
            prev_fwd = (cmd[1] > 0.1)
            cdp.eval(set_cmd_js(*cmd))
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n [STOP por Ctrl+C]")
    finally:
        if stop_event is not None:
            stop_event.set()
        cdp.eval(STOP_JS); time.sleep(0.3); cdp.eval(STOP_JS)
        save_state()
        png = save_map_snapshot(set(visited.keys()), oset, trail, tgt, tag=time.strftime('%H:%M:%S'))
        print(f"STOP. Fin frontera. Cobertura: {len(visited)} celdas, mapa: {len(oset)} obstaculos activos.")
        print(f"  MAPA guardado -> {png} (+ map_latest.json)")
        lg.write("FIN\n"); lg.close()


def save_map_snapshot(visited, oset, trail, tgt=None, tag=""):
    """Guarda el mapa actual a PNG + JSON en la carpeta del proyecto (para revisarlo despues / que Claude
    lo lea): celdas exploradas, obstaculos activos, recorrido de odometria y objetivo."""
    base = os.path.dirname(LOGPATH)
    png = os.path.join(base, "map_latest.png"); js = os.path.join(base, "map_latest.json")
    try:
        import matplotlib; matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(8, 8))
        if visited:
            ax.scatter([c[0] * F_CELL for c in visited], [c[1] * F_CELL for c in visited],
                       s=60, c="#bfe3bf", marker="s", linewidths=0, label=f"explorado ({len(visited)})")
        if oset:
            ax.scatter([c[0] * OCELL for c in oset], [c[1] * OCELL for c in oset],
                       s=16, c="#c0392b", marker="s", linewidths=0, label=f"obstaculo ({len(oset)})")
        if trail:
            ax.plot([p[0] for p in trail], [p[1] for p in trail], "-", c="#34495e", lw=1.0, alpha=0.6)
            ax.plot([trail[-1][0]], [trail[-1][1]], "o", c="#2980b9", ms=10)
        if tgt:
            ax.plot([tgt[0]], [tgt[1]], "*", c="#f39c12", ms=18)
        ax.set_aspect("equal", adjustable="datalim"); ax.grid(True, alpha=0.2)
        ax.legend(loc="upper right", fontsize=8); ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
        ax.set_title(f"Mapa G1 {tag}")
        fig.savefig(png, dpi=80); plt.close(fig)
    except Exception as e:
        print("  (no pude guardar PNG del mapa:", repr(e), ")")
    try:
        import json as _json
        with open(js, "w") as f:
            _json.dump({"explored": len(visited), "n_obstacles": len(oset),
                        "visited_cells_F0.4": [list(c) for c in visited],
                        "obstacle_cells_O0.2": [list(c) for c in oset],
                        "trail_xy": [[round(p[0], 2), round(p[1], 2)] for p in trail[-1500:]],
                        "target": list(tgt) if tgt else None, "F_CELL": F_CELL, "OCELL": OCELL}, f)
    except Exception:
        pass
    return png


def _map_window(vshare, lock, stop_event, secs):
    """Ventana en vivo (hilo principal) del mapa que construye 'frontier': celdas exploradas,
    obstaculos, traza de odometria, robot+rumbo y frontera objetivo."""
    try:
        import matplotlib
        import matplotlib.pyplot as plt
    except Exception as e:
        print("!! matplotlib no disponible para la ventana:", repr(e))
        print("   instala con: pip install matplotlib  (el control sigue corriendo sin ventana)")
        # sin ventana: espera a que termine el hilo de control
        while not stop_event.is_set():
            time.sleep(0.3)
        return
    try:
        import numpy as _np
        from PIL import Image as _Image
        import base64 as _b64, io as _io
        _have_cam = True
    except Exception:
        _have_cam = False
    plt.ion()
    fig, (ax, axc) = plt.subplots(1, 2, figsize=(15, 7.6),
                                  gridspec_kw={"width_ratios": [1.05, 1]})
    try:
        fig.canvas.manager.set_window_title("G1 frontier — mapa + camara del robot")
    except Exception:
        pass
    fig.canvas.mpl_connect("close_event", lambda e: stop_event.set())
    print("Ventana (mapa + camara) abierta. Cierrala o Ctrl+C en la terminal para parar.")
    try:
        while not stop_event.is_set():
            with lock:
                x = vshare["x"]; y = vshare["y"]; yaw = vshare["yaw"]; ph = vshare["ph"]
                tgt = vshare["tgt"]; expl = vshare["expl"]; t = vshare["t"]; col = vshare.get("col", 0)
                vis = list(vshare["visited"]); obs = list(vshare["obs"]); path = list(vshare["path"])
                plan = list(vshare.get("plan", [])); infl = list(vshare.get("infl", []))
                cobs = list(vshare.get("cobs", [])); carrot = vshare.get("carrot")
                cam = vshare.get("cam"); vtxt = vshare.get("vtxt", "")
            ax.clear()
            if vis:
                ax.scatter([c[0] * F_CELL for c in vis], [c[1] * F_CELL for c in vis],
                           s=70, c="#bfe3bf", marker="s", linewidths=0, label="explorado")
            if infl:                                     # halo de inflado (costmap, en metros)
                ax.scatter([p[0] for p in infl], [p[1] for p in infl],
                           s=22, c="#f5b041", marker="s", linewidths=0, alpha=0.45, label="margen (costmap)")
            if obs:                                      # mapa de obstaculos FINO LiDAR (metros, OCELL=0.2)
                ax.scatter([p[0] for p in obs], [p[1] for p in obs],
                           s=22, c="#c0392b", marker="s", linewidths=0, label="obstaculo (LiDAR)")
            if cobs:                                      # capa de camara transitoria (solo A*)
                ax.scatter([p[0] for p in cobs], [p[1] for p in cobs],
                           s=22, c="#8e44ad", marker="s", linewidths=0, alpha=0.6, label="camara (transit.)")
            if path:
                ax.plot([p[0] for p in path], [p[1] for p in path], "-", c="#34495e", lw=1.0, alpha=0.55)
            if plan and len(plan) > 1:                   # ruta A*
                ax.plot([p[0] for p in plan], [p[1] for p in plan], "-", c="#1565c0", lw=2.2, label="ruta A*")
            if carrot:
                ax.plot([carrot[0]], [carrot[1]], "o", c="#00bcd4", ms=8)
            if tgt:
                ax.plot([tgt[0]], [tgt[1]], "*", c="#f39c12", ms=20, label="frontera")
            ax.plot([x], [y], "o", c="#2980b9", ms=11)
            ax.arrow(x, y, 0.32 * math.cos(math.radians(yaw)), 0.32 * math.sin(math.radians(yaw)),
                     head_width=0.13, head_length=0.13, fc="#2980b9", ec="#2980b9", length_includes_head=True)
            ax.set_aspect("equal", adjustable="datalim"); ax.grid(True, alpha=0.2)
            ax.set_xlabel("x odom (m)"); ax.set_ylabel("y odom (m)")
            ax.set_title(f"t={t:.0f}/{secs:.0f}s  fase={ph.strip()}  celdas={expl}  obst={len(obs)}  colis={col}")
            try:
                ax.legend(loc="upper right", fontsize=8)
            except Exception:
                pass
            # --- panel CAMARA (lo que ve el robot) ---
            axc.clear(); axc.axis("off")
            if _have_cam and cam and cam.startswith("data:image"):
                try:
                    img = _Image.open(_io.BytesIO(_b64.b64decode(cam.split(",", 1)[1])))
                    axc.imshow(_np.asarray(img))
                    col_t = "#c0392b" if "CERCA" in vtxt else "#2c3e50"
                    axc.set_title(f"Camara del robot  —  {vtxt}", color=col_t, fontsize=11)
                except Exception:
                    axc.set_title("Camara del robot (sin frame)", fontsize=11)
            else:
                axc.set_title("Camara del robot (esperando frame...)", fontsize=11)
            plt.pause(0.3)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        plt.ioff()
        try:
            plt.close(fig)
        except Exception:
            pass


def cmd_frontier_viz(secs):
    """Lanza 'frontier' con ventana en vivo (mapa + camara del robot): control en hilo de fondo,
    ventana en el principal. Ideal para grabar pantalla."""
    secs = max(5, min(600, secs))
    vshare = {"x": 0.0, "y": 0.0, "yaw": 0.0, "ph": "", "tgt": None, "expl": 0, "t": 0.0,
              "col": 0, "path": [], "visited": [], "obs": [], "cobs": [],
              "plan": [], "infl": [], "carrot": None, "cam": None, "vtxt": ""}
    lk = threading.Lock(); stop_event = threading.Event()
    th = threading.Thread(target=cmd_frontier,
                          kwargs=dict(secs=secs, vshare=vshare, lock=lk, stop_event=stop_event),
                          daemon=True)
    th.start()
    try:
        _map_window(vshare, lk, stop_event, secs)
    except KeyboardInterrupt:
        stop_event.set()
    finally:
        stop_event.set()
        th.join(timeout=6)
    print("Ventana cerrada. Fin.")


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
    elif cmd == "frontier":
        secs_f = 90.0
        for a in sys.argv[2:]:
            try:
                secs_f = float(a); break
            except ValueError:
                pass
        if "viz" in sys.argv or "map" in sys.argv:
            cmd_frontier_viz(secs_f)
        else:
            cmd_frontier(secs_f)
    elif cmd == "scan":
        cmd_scan()
    elif cmd == "vsee":
        cmd_vsee()
    elif cmd == "floorcal":
        cmd_floorcal(sys.argv[2] if len(sys.argv) > 2 else None)
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
    elif cmd == "imu":
        cmd_imu()
    elif cmd == "sniffnav":
        cmd_sniffnav()
    else:
        print("comando desconocido:", cmd); print(__doc__)


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print("Error:", repr(e))
        sys.exit(1)
