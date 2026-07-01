#!/usr/bin/env python3
"""
g1_goto.py  -  NAVEGACION A->B sobre un MAPA CARGADO en la app (relocalizacion).

Reusa todo g1_nav_v2 (A*, DWA, costmap, camara, contacto IMU). Flujo:
  1) reloccheck   -> con el mapa cargado y relocalizado en la app, ver que datos llegan (pose/nube/camara)
  2) waypoint A   -> conduces el robot al destino y Ctrl+C; guarda la ULTIMA pose como 'A' en waypoints.json
                     (a la vez acumula el mapa 2D de obstaculos en nav_map.json)
  3) (el mapa 2d se va guardando solo en waypoint; tambien 'sweep' para mapear sin guardar waypoint)
  4) goto         -> menu en vivo: pides A/B/C..., A*+DWA te lleva y para

PRE: app de Unitree con el MAPA CARGADO y el robot RELOCALIZADO (de pie, en modo nav), ios_webkit_debug_proxy.
"""
import sys
import os
import time
import json
import math
import threading
from collections import deque
import g1_nav_v2 as g                      # reusa conexion + A* + DWA + costmap + camara + helpers
try:
    import g1_perception                    # cliente del servidor GPU offboard (opcional; via G1_PERC=host:port)
except Exception:
    g1_perception = None
import g1_metrics                            # metricas SEI: clearance (percepcion) + progression (rendimiento)

WP_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "waypoints.json")
MAP_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "nav_map.json")
GOTO_LOG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "goto.log")

# --- nube 'location' (frame del MAPA, Z-up): idx0=x, idx1=y, idx2=altura. CONFIRMADO con reloc_cloud.json ---
HBAND_LO = float(os.environ.get("G1_HBAND_LO", "-0.5"))   # borde INFERIOR banda de altura (m) para OBSTACULOS.
                                 # SUBIDO de -0.9 a -0.5: a -0.9 el suelo (~-1.0) entraba como obstaculo al cabecear
                                 # la marcha -> cientos de celdas falsas. -0.5 lo evita. (override: G1_HBAND_LO)
HBAND_HI = float(os.environ.get("G1_HBAND_HI", "0.6"))    # borde SUPERIOR (excluye techo ~+1.3)
NAV_REACH = 0.35                 # m: se considera ALCANZADO el waypoint
NAV_OMAP_TTL = 60.0              # s: la nube es estatica; TTL medio purga obstaculos dinamicos (persona que pasa)
GATE_M = 0.6                     # m: si arrancas a > esto del waypoint mas cercano = relocalizacion dudosa (como la app)
AGGR_AFTER = 12.0                # s atascado sin ACERCARSE al destino -> activa modo AGRESIVO (cruza la puerta)
AGGR_ROBOT_R = 0.13              # m: holgura reducida en modo agresivo. Es el MINIMO de seguridad (no baja de aqui)
PERC_PERIOD = 0.3                # s entre consultas al servidor de percepcion GPU (depth->scan virtual de la mesa)
DOOR_CENTER = (os.environ.get("G1_DOOR_CENTER", "1") == "1")   # centrar izq/dcha en la puerta (idea de Renxi): strafe al lado mas libre
DOOR_BAL_TH = 0.22               # |clear_left - clear_right| (normalizado) para considerar el robot DESCENTRADO
DOOR_STRAFE = 0.34               # magnitud del strafe lateral (> deadzone ~0.3, si no el robot no se mueve)
DOOR_STRAFE_SIGN = int(os.environ.get("G1_STRAFE_SIGN", "1"))  # +1/-1: si centra al lado EQUIVOCADO, lanza con G1_STRAFE_SIGN=-1
DOOR_MIN_GOAL = 1.3              # m: por debajo de esta distancia a B NO hay puerta (es el goal con un mueble):
                                 # desactiva la maniobra de puerta y deja que el DWA rodee el obstaculo (si no, empujaba recto)
PERSIST_N = 3                    # filtro de ruido del laser: ventana de barridos recientes
PERSIST_K = 2                    # una celda que SOLO ve el laser (no esta en el mapa) cuenta si aparece en >=K de los ultimos N barridos
# --- Mapa de obstaculos con SCORE/DECAY (anti acumulacion de ruido) ---
# Antes: celda confirmada -> entra al instante y dura 60s (TTL). Con el robot parado en la puerta y el LiDAR
# de cabeza vibrando, el mapa = UNION de todo lo visto en 60s -> cientos de celdas falsas (obs 142->521 en 18s).
# Ahora: cada celda tiene un SCORE que sube al verse y baja si esta EN RANGO y no se ve. Con SC_MISS>=SC_HIT una
# celda solo se mantiene si se ve en la MAYORIA de barridos recientes: el ruido intermitente decae y desaparece;
# las paredes/mesa reales (vistas casi siempre) saturan al tope y se quedan. NO añade paredes fantasma.
OLDMAP   = (os.environ.get("G1_OLDMAP") == "1")            # =1 vuelve al mapa TTL antiguo (para A/B en el robot)
SC_HIT   = float(os.environ.get("G1_SC_HIT",   "1.0"))    # +score al ver la celda
SC_MISS  = float(os.environ.get("G1_SC_MISS",  "1.0"))    # -score si esta EN RANGO y no se ve (>=HIT => exige mayoria de barridos)
SC_CAP   = float(os.environ.get("G1_SC_CAP",   "6.0"))    # tope de score (da inercia a paredes reales ante oclusiones breves)
SC_OBST  = float(os.environ.get("G1_SC_OBST",  "2.0"))    # umbral de score para contar como obstaculo
SC_RANGE = float(os.environ.get("G1_SC_RANGE", "2.6"))    # m: solo se penaliza (decay) dentro de este radio (ventana de plan); fuera se conserva por TTL
# GATE DE ROTACION: al girar rapido la nube 'location' se proyecta con la pose RETRASADA -> los barridos se
# ensucian (laser_noise +65% girando, observado). No es fiable meterlos en el mapa. Si |yaw_rate|>YAW_GATE
# CONGELAMOS el mapa (ni inserta ni decae): usamos lo capturado yendo recto/lento. 0 = desactiva. (G1_YAWGATE)
YAW_GATE = float(os.environ.get("G1_YAWGATE", "30.0"))    # deg/s (la maniobra de puerta gira ~36 deg/s -> se congela)
DATASET_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dataset")


class RunRecorder:
    """Graba una travesia completa a un JSON estructurado en dataset/ (dataset-ready, mismo esquema para
    nuestra nav y el firmware): metadatos + trayectoria + eventos (colisiones) + snapshots del laser +
    metricas resumen. Apto para subir/comparar/entrenar (tesis: meta-cognicion firmware vs nuestro)."""

    def __init__(self, mode, label, goal, pcd=""):
        try:
            os.makedirs(DATASET_DIR, exist_ok=True)
        except Exception:
            pass
        self.t0 = time.time()
        self.mode = mode
        self.fname = os.path.join(DATASET_DIR, time.strftime("%Y%m%d_%H%M%S") + f"_{mode}_{label}.json")
        self.rec = {"schema": "g1_goto_run/v1", "mode": mode, "label": label,
                    "goal": {"x": goal[0], "y": goal[1]}, "pcd": pcd, "OCELL": g.OCELL,
                    "hband": [HBAND_LO, HBAND_HI], "started": time.strftime("%Y-%m-%d %H:%M:%S"),
                    "samples": [], "events": [], "laser_snapshots": [], "telemetry": [], "summary": {}}
        self._laser_t = 0.0
        self._telem_t = -9.0

    def sample(self, t, x, y, yaw, d, spd, c0, nobs, cmd=None, phase="", extra=None):
        rec = {"t": round(t, 2), "x": round(x, 3), "y": round(y, 3),
               "yaw": round(yaw, 1), "d": round(d, 3), "spd": round(spd, 3),
               "c0": round(c0, 2), "nobs": nobs, "phase": phase,
               "cmd": [round(float(v), 2) for v in cmd] if cmd else None}
        if extra:
            rec.update({k: v for k, v in extra.items() if v is not None})
        self.rec["samples"].append(rec)

    def event(self, kind, t, x, y, extra=None):
        e = {"kind": kind, "t": round(t, 2), "x": round(x, 3), "y": round(y, 3)}
        if extra:
            e.update(extra)
        self.rec["events"].append(e)

    def maybe_laser(self, t, pts, every=2.0):
        if t - self._laser_t >= every:
            self.rec["laser_snapshots"].append({"t": round(t, 2),
                                                "pts": [[round(a, 2), round(b, 2)] for a, b in pts]})
            self._laser_t = t

    def telem(self, t, row, every=1.0):
        """Pista de telemetria completa (bateria/cpu/motores/IMU) a ~1Hz, separada de las muestras de nav."""
        if row and t - self._telem_t >= every:
            self.rec["telemetry"].append(dict(t=round(t, 2), **row))
            self._telem_t = t

    def save_cloud(self, tag, pose, points):
        """Guarda una nube 3D CRUDA (todas las alturas) a un fichero aparte y la referencia en el run.
        Util en colisiones: con la nube 3D se ve si era una mesa (tablero a media altura + hueco debajo)."""
        if not points:
            return None
        fn = self.fname[:-5] + f"_{tag}.json"
        try:
            json.dump({"tag": tag, "pose": pose, "npts": len(points) // 3, "points": points,
                       "OCELL": g.OCELL, "hband": [HBAND_LO, HBAND_HI],
                       "frame": "map (idx0=x, idx1=y, idx2=altura)"}, open(fn, "w"))
            self.rec.setdefault("clouds", []).append(os.path.basename(fn))
            return fn
        except Exception:
            return None

    def save_cam(self, tag, jpg):
        """Guarda la foto de la camara (prueba visual de lo que habia, p.ej. una mesa) a un .jpg aparte."""
        if not jpg or not jpg.startswith("data:image"):
            return None
        import base64
        fn = self.fname[:-5] + f"_{tag}.jpg"
        try:
            with open(fn, "wb") as f:
                f.write(base64.b64decode(jpg.split(",", 1)[1]))
            self.rec.setdefault("cams", []).append(os.path.basename(fn))
            return fn
        except Exception:
            return None

    def finish(self, result, summary):
        self.rec["result"] = result
        self.rec["summary"] = summary
        self.rec["duration_s"] = round(time.time() - self.t0, 2)
        try:
            json.dump(self.rec, open(self.fname, "w"))
            print(f"  [dataset] travesia guardada -> {self.fname}")
        except Exception as e:
            print("  [dataset] no se pudo guardar:", repr(e))
        return self.fname

# Hook independiente: captura la pose de RELOCALIZACION (mapa cargado) de rt/slam_info (currentPose sobre el
# .pcd) y de slam_relocation/odom. NO depende del hook de mapeo (slam_mapping/odom).
RELOC_JS = r"""(function(){
  if(!window.__relocHook){ window.__relocHook=1;
    var jp=JSON.parse;
    JSON.parse=function(s){ var v=jp.apply(this,arguments);
      try{ if(v && v.topic){ var tp=''+v.topic;
        if(tp.indexOf('slam_info')>=0){
          var d=(typeof v.data==='string')?jp(v.data):v.data;
          if(d && d.data && d.data.currentPose){ var p=d.data.currentPose;
            window.__pose=[p.x,p.y,p.z,p.q_x,p.q_y,p.q_z,p.q_w]; window.__pose_t=Date.now();
            if(d.data.pcdName) window.__pcd=d.data.pcdName;
          }
        }
        if(tp.indexOf('slam_relocation/odom')>=0 && v.data && v.data.pose && v.data.pose.pose){
          var pp=v.data.pose.pose;
          window.__relocodom=[pp.position.x,pp.position.y,pp.position.z,
                              pp.orientation.x,pp.orientation.y,pp.orientation.z,pp.orientation.w];
          window.__relocodom_t=Date.now();
          if(v.data.pose.covariance) window.__reloccov=v.data.pose.covariance;   // 6x6 (xx,yy,..); el G1 la manda a 0
        }
      }}catch(e){}
      return v;
    };
  } return 1;
})()"""


# Diagnostico para ENCONTRAR la nube en vivo en modo operacion/relocalizacion (los "puntitos blancos").
# Hookea: (1) TODOS los mensajes que los Workers mandan a la app (tipo + campos + si traen array de puntos),
# (2) la estructura de los topics slam_relocation/points y mapping/points.
CLOUD_DEBUG_JS = r"""(function(){
  if(!window.__cloudDbg){ window.__cloudDbg=1; window.__msgtypes={}; window.__cloudsample=null;
    var seen=new WeakSet();
    var o=Worker.prototype.postMessage;
    Worker.prototype.postMessage=function(m){
      try{ if(!seen.has(this)){ seen.add(this);
        this.addEventListener('message',function(ev){ try{
          var d=ev.data;
          var t=(d&&d.type)?(''+d.type):(d&&d.constructor?d.constructor.name:typeof d);
          var rec=window.__msgtypes[t]||{n:0,keys:'',dkeys:'',pts:0};
          rec.n++;
          if(!rec.keys && d && typeof d==='object'){
            rec.keys=Object.keys(d).slice(0,8).join(',');
            var dd=d.data;
            if(dd && typeof dd==='object'){ rec.dkeys=Object.keys(dd).slice(0,8).join(',');
              var arr=dd.directOutput||dd.points||dd.cloud||dd.data;
              if(arr){ var n=arr.length||(arr.byteLength?arr.byteLength/4:Object.keys(arr).length); rec.pts=n;
                if(!window.__cloudsample && n>30){ window.__cloudsample={type:t, n:n,
                  head:Array.prototype.slice.call(arr,0,9)}; } }
            }
          }
          window.__msgtypes[t]=rec;
        }catch(e){} });
      } }catch(e){}
      return o.apply(this,arguments);
    };
    // tambien: estructura de los topics de puntos por JSON.parse
    var jp=JSON.parse;
    JSON.parse=function(s){ var v=jp.apply(this,arguments);
      try{ if(v && v.topic){ var tp=''+v.topic;
        if(tp.indexOf('points')>=0){ var rec=window.__msgtypes['JSON:'+tp]||{n:0,keys:'',dkeys:'',pts:0};
          rec.n++; if(!rec.keys){ rec.keys=Object.keys(v).join(','); if(v.data) rec.dkeys=(typeof v.data)+':'+(v.data.length||Object.keys(v.data).slice(0,6).join('|')); }
          window.__msgtypes['JSON:'+tp]=rec; }
      }}catch(e){}
      return v;
    };
  } return 1;
})()"""


def cmd_clouddebug():
    """Encuentra la NUBE en vivo en modo operacion (los puntitos blancos). Lanza con el mapa cargado y
    relocalizado, deja que se vean los puntos, y muestra que tipos de mensaje los llevan."""
    cdp = g.get_cdp()
    cdp.eval(CLOUD_DEBUG_JS)
    dbglog = os.path.join(os.path.dirname(os.path.abspath(__file__)), "clouddebug.log")
    print(">>> CLOUD-DEBUG. Con el mapa cargado y los puntitos visibles, mueve el robot un poco.")
    print(f"    Voy guardando el resumen en {dbglog}. Ctrl+C para el volcado final.\n")

    def dump(final=False):
        try:
            mt = json.loads(cdp.eval("JSON.stringify(window.__msgtypes||{})") or "{}")
            samp = cdp.eval("JSON.stringify(window.__cloudsample||null)")
            samp = json.loads(samp) if samp and samp != "null" else None
        except Exception:
            mt = {}; samp = None
        lines = [f"=== CLOUDDEBUG {time.strftime('%H:%M:%S')}{' FINAL' if final else ''} "
                 "(n=cuantos, pts=nº puntos) ==="]
        for t, r in sorted(mt.items(), key=lambda kv: -kv[1].get("n", 0)):
            lines.append(f"  n={r.get('n', 0):<6} pts={r.get('pts', 0):<8} tipo='{t}' "
                         f"campos=[{r.get('keys', '')}] data=[{r.get('dkeys', '')}]")
        if samp:
            lines.append(f"  >>> NUBE: type='{samp.get('type')}' n={samp.get('n')} primeros9={samp.get('head')}")
        else:
            lines.append("  (aun sin muestra de nube con pts>30)")
        txt = "\n".join(lines)
        try:
            with open(dbglog, "w") as f:
                f.write(txt + "\n")
        except Exception:
            pass
        return txt

    try:
        while True:
            txt = dump()
            print("\033[2J\033[H", end=""); print(txt)
            time.sleep(1.0)
    except KeyboardInterrupt:
        dump(final=True)
        print(f"\nFin clouddebug. Guardado en {dbglog}. Di 'mira el clouddebug' y lo leo.")


# Captura la NUBE EN VIVO de operacion/relocalizacion: mensaje worker type='location', data.points =
# array plano [x,y,z,...]. Guarda la ultima en window.__relocbuf.
# ROBUSTO: adjunta el listener (1) a cada worker al que la app hace postMessage Y (2) a cada worker NUEVO
# via el constructor de Worker -> ya no depende del timing (antes a veces salia nobs=0).
RELOC_CLOUD_JS = r"""(function(){
  function grab(ev){ try{
    var d=ev.data;
    if(d && d.type==='location' && d.data && d.data.points){
      var a=d.data.points;
      window.__relocbuf=(ArrayBuffer.isView(a))?Array.prototype.slice.call(a):Object.values(a);
      window.__relocbuf_t=Date.now();
    }
  }catch(e){} }
  function attach(w){ try{ if(w && !w.__rcAtt){ w.__rcAtt=1; w.addEventListener('message',grab); } }catch(e){} }
  if(!window.__relocCloudHook){ window.__relocCloudHook=1;
    if(!window.__relocbuf) window.__relocbuf=[]; window.__relocbuf_t=window.__relocbuf_t||0;
    // 1) cada worker al que la app postea
    var o=Worker.prototype.postMessage;
    Worker.prototype.postMessage=function(m){ attach(this); return o.apply(this,arguments); };
    // 2) cada worker al que la app ESCUCHA mensajes (engancha aunque ya exista, en cuanto re-escucha)
    var oae=Worker.prototype.addEventListener;
    Worker.prototype.addEventListener=function(type,fn,opt){ try{ if(type==='message') attach(this); }catch(e){} return oae.apply(this,arguments); };
    // 3) cada worker NUEVO (constructor) -> coge el que emite 'location' al arrancar nav
    try{ var OW=window.Worker;
      function W(a,b){ var w=new OW(a,b); attach(w); return w; }
      W.prototype=OW.prototype; window.Worker=W;
    }catch(e){}
  }
  return (window.__relocbuf||[]).length;
})()"""


# Busca el PATH PLANIFICADO del firmware: caza mensajes (worker o JSON-topic) que contengan un array con
# pinta de ruta (>=3 elementos {x,y} o [x,y]). Si aparece, lo tenemos.
PATHSNIFF_JS = r"""(function(){
  if(!window.__pathSniff){ window.__pathSniff=1; window.__pathmsgs={}; window.__pathcand=null;
    function looksPath(o){ try{
        if(Array.isArray(o) && o.length>=3){ var a=o[0];
          if(a && typeof a==='object' && ('x' in a) && ('y' in a)) return o.length;
          if(Array.isArray(a) && a.length>=2 && typeof a[0]==='number') return o.length;
        }}catch(e){} return 0; }
    function scan(d,dep){ if(dep>4||!d||typeof d!=='object') return 0;
      var n=looksPath(d); if(n){ if(!window.__pathcand) window.__pathcand={where:'root',n:n,sample:JSON.stringify(d).slice(0,400)}; return n; }
      for(var k in d){ try{ var m=looksPath(d[k]);
        if(m){ if(!window.__pathcand) window.__pathcand={where:k,n:m,sample:JSON.stringify(d[k]).slice(0,400)}; return m; }
        var mm=scan(d[k],dep+1); if(mm) return mm; }catch(e){} } return 0; }
    var seen=new WeakSet(); var o=Worker.prototype.postMessage;
    Worker.prototype.postMessage=function(m){ try{ if(!seen.has(this)){ seen.add(this);
      this.addEventListener('message',function(ev){ try{ var d=ev.data; var t=(d&&d.type)?(''+d.type):typeof d;
        var rec=window.__pathmsgs[t]||{n:0,pathlen:0,keys:''}; rec.n++;
        if(!rec.keys && d&&typeof d==='object') rec.keys=Object.keys(d).slice(0,8).join(',');
        var pl=scan(d,0); if(pl>rec.pathlen) rec.pathlen=pl; window.__pathmsgs[t]=rec;
      }catch(e){} }); } }catch(e){} return o.apply(this,arguments); };
    var jp=JSON.parse; JSON.parse=function(s){ var v=jp.apply(this,arguments);
      try{ if(v&&v.topic){ var tp=''+v.topic;
        if(/path|plan|traj|route|nav|key_info|waypoint|topo/i.test(tp)){
          var d=(typeof v.data==='string')?jp(v.data):v.data;
          var rec=window.__pathmsgs['JSON:'+tp]||{n:0,pathlen:0,keys:''}; rec.n++;
          if(!rec.keys && d && typeof d==='object') rec.keys=Object.keys(d).slice(0,8).join(',');
          var pl=scan(d,0); if(pl>rec.pathlen) rec.pathlen=pl; window.__pathmsgs['JSON:'+tp]=rec;
        }
      }}catch(e){} return v; };
  } return 1;
})()"""


# Sniffer PASIVO: TU lanzas la navegacion desde la app y capturamos TODO lo util:
#  - OUT: lo que la app ENVIA por el datachannel (su comando de nav real + lo que sea)
#  - WORKER IN: mensajes worker->app (laser y posible render de ruta)
#  - TOPICS: todos, marcando los que parecen ruta + candidato de path
APPSNIFF_JS = r"""(function(){
  if(!window.__appSniff){ window.__appSniff=1;
    window.__snOut={}; window.__snWk={}; window.__snTop={}; window.__snSamples=[]; window.__snPath=null;
    window.__snAll={parse:0,worker:0,send:0};
    function push(a,o){ a.push(o); if(a.length>300) a.shift(); }
    function looksPath(o){ try{ if(Array.isArray(o)&&o.length>=3){ var a=o[0];
      if(a&&typeof a==='object'&&('x'in a)&&('y'in a)) return o.length;
      if(Array.isArray(a)&&a.length>=2&&typeof a[0]==='number') return o.length; }}catch(e){} return 0; }
    function scan(d,dep){ if(dep>5||!d||typeof d!=='object') return 0; var n=looksPath(d);
      if(n){ if(!window.__snPath) window.__snPath={where:'root',n:n,sample:JSON.stringify(d).slice(0,600)}; return n; }
      for(var k in d){ try{ var m=looksPath(d[k]);
        if(m){ if(!window.__snPath) window.__snPath={where:k,n:m,sample:JSON.stringify(d[k]).slice(0,600)}; return m; }
        var mm=scan(d[k],dep+1); if(mm) return mm; }catch(e){} } return 0; }
    var S=RTCDataChannel.prototype.send;
    RTCDataChannel.prototype.send=function(d){ try{ window.__snAll.send++; if((this.label||'')==='data' && typeof d==='string'){
      var v=JSON.parse(d); var tp=(v&&v.topic)?(''+v.topic):'?'; var api='';
      try{ api=(v.data&&v.data.header&&v.data.header.identity)?v.data.header.identity.api_id:''; }catch(e){}
      var key=tp+(api!==''?(' api='+api):''); var rec=window.__snOut[key]||{n:0,sample:''}; rec.n++;
      if(!rec.sample) rec.sample=d.slice(0,600); window.__snOut[key]=rec;
      if(tp.indexOf('wirelesscontroller')<0) push(window.__snSamples,{dir:'OUT',topic:tp,api:''+api,t:Date.now(),raw:d.slice(0,900)});
    }}catch(e){} return S.apply(this,arguments); };
    var seen=new WeakSet(); var o=Worker.prototype.postMessage;
    Worker.prototype.postMessage=function(m){ try{ if(!seen.has(this)){ seen.add(this);
      this.addEventListener('message',function(ev){ try{ window.__snAll.worker++; var d=ev.data; var t=(d&&d.type)?(''+d.type):typeof d;
        var rec=window.__snWk[t]||{n:0,pathlen:0,keys:''}; rec.n++;
        if(!rec.keys&&d&&typeof d==='object') rec.keys=Object.keys(d).slice(0,8).join(',');
        var pl=scan(d,0); if(pl>rec.pathlen) rec.pathlen=pl; window.__snWk[t]=rec;
      }catch(e){} }); } }catch(e){} return o.apply(this,arguments); };
    var jp=JSON.parse; JSON.parse=function(s){ var v=jp.apply(this,arguments);
      try{ if(v&&v.topic){ window.__snAll.parse++; var tp=''+v.topic; var rec=window.__snTop[tp]||{n:0,pathlen:0}; rec.n++;
        if(/path|plan|traj|route|nav|key_info|topo|waypoint/i.test(tp)){ var d=(typeof v.data==='string')?jp(v.data):v.data; var pl=scan(d,0); if(pl>rec.pathlen) rec.pathlen=pl; }
        window.__snTop[tp]=rec; } }catch(e){} return v; };
  } return 1;
})()"""


def _all_inspectable_pages():
    """Lista (titulo, url, ws_url) de TODAS las paginas inspeccionables del WebView (no solo la mejor)."""
    import requests
    devs = requests.get(g.PROXY + "/json", timeout=5).json()
    if not devs:
        return []
    durl = devs[0]["url"]
    pages = requests.get(f"http://{durl}/json", timeout=5).json()
    return [(p.get("title", "?"), p.get("url", "?"), p.get("webSocketDebuggerUrl"))
            for p in pages if p.get("webSocketDebuggerUrl")]


def cmd_appsniff(secs=40):
    """PASIVO multi-pagina: se engancha a TODAS las paginas del WebView. TU navegas DESDE LA APP (ir a B)
    y captura TODO en la que tenga trafico (la activa). No envio nada yo."""
    pages = _all_inspectable_pages()
    if not pages:
        print("No hay paginas inspeccionables. ¿ios_webkit_debug_proxy + app en SLAM?"); return
    cdps = []
    for (ti, url, wsu) in pages:
        try:
            c = g.CDP(wsu)
            c.eval(DISABLE_DRV_JS); c.eval(APPSNIFF_JS); c.eval(RELOC_JS)
            cdps.append((ti, url, c))
            print(f"  enganchado: '{ti[:22]}'  {url[:46]}")
        except Exception as e:
            print(f"  (no enganche '{(ti or '')[:22]}': {repr(e)[:50]})")
    if not cdps:
        print("No pude engancharme a ninguna pagina."); return
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "appsniff.log")
    outj = os.path.join(os.path.dirname(os.path.abspath(__file__)), "appsniff.json")
    print(f"\n>>> APPSNIFF en {len(cdps)} paginas. EN LA APP: pon destino B y navega. Capturo ~{secs}s. Ctrl+C para parar.\n")

    def traffic(c):
        try:
            a = json.loads(c.eval("JSON.stringify(window.__snAll||{})") or "{}")
            return sum(a.values()), a
        except Exception:
            return 0, {}

    def best_cdp():
        bc = None; bn = -1; rows = []
        for (ti, url, c) in cdps:
            tot, a = traffic(c); rows.append((ti, tot, a))
            if tot > bn:
                bn = tot; bc = c
        return bc, bn, rows
    try:
        for i in range(int(secs * 2)):
            bc, bn, rows = best_cdp()
            L = [f"=== APPSNIFF {time.strftime('%H:%M:%S')}  ({len(cdps)} paginas) ==="]
            for ti, tot, a in rows:
                L.append(f"  pagina '{ti[:24]:<24}' trafico={tot:<6} (parse={a.get('parse',0)} worker={a.get('worker',0)} send={a.get('send',0)})")
            if bc and bn > 0:
                so = json.loads(bc.eval("JSON.stringify(window.__snOut||{})") or "{}")
                sw = json.loads(bc.eval("JSON.stringify(window.__snWk||{})") or "{}")
                cand = bc.eval("JSON.stringify(window.__snPath||null)")
                L.append("  -- OUT (la APP envia) --")
                for k, r in sorted(so.items(), key=lambda kv: -kv[1]["n"]):
                    L.append(f"     n={r['n']:<4} {k}")
                L.append("  -- WORKER (PATHLEN=puntos tipo ruta) --")
                for k, r in sorted(sw.items(), key=lambda kv: -kv[1].get("pathlen", 0)):
                    L.append(f"     n={r['n']:<4} PATHLEN={r.get('pathlen',0):<4} tipo='{k}' [{r.get('keys','')}]")
                L.append(f"  >>> CANDIDATO DE RUTA: {cand if cand and cand!='null' else '(ninguno aun)'}")
            else:
                L.append("  (sin trafico aun en NINGUNA pagina; navega desde la app)")
            with open(out, "w") as f:
                f.write("\n".join(L) + "\n")
            print("\033[2J\033[H", end=""); print("\n".join(L))
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            bc, bn, _ = best_cdp()
            if bc:
                json.dump({"out": json.loads(bc.eval("JSON.stringify(window.__snOut||{})") or "{}"),
                           "worker": json.loads(bc.eval("JSON.stringify(window.__snWk||{})") or "{}"),
                           "path_candidate": json.loads(bc.eval("JSON.stringify(window.__snPath||null)") or "null"),
                           "samples_out": json.loads(bc.eval("JSON.stringify(window.__snSamples||[])") or "[]")},
                          open(outj, "w"), indent=1)
        except Exception:
            pass
        print(f"\nFin appsniff. Guardado {out} + {outj}. Di 'mira el appsniff' y lo analizo.")


def cmd_pathsniff(label):
    """DESCUBRE si el firmware expone su PATH planificado. Envia el goal nativo a <label> y caza durante
    ~20s cualquier mensaje con pinta de ruta. El robot SE MUEVE (firmware). Mando en mano."""
    wps = _load_wps()
    if not label or label not in wps:
        print(f"uso: python g1_goto.py pathsniff <N>  (waypoints: {list(wps.keys())})"); return
    w = wps[label]
    cdp = g.get_cdp()
    cdp.eval(DISABLE_DRV_JS); cdp.eval(NATIVE_CAP_JS); cdp.eval(RELOC_JS); cdp.eval(PATHSNIFF_JS)
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "pathsniff.log")
    # espera dc
    for _ in range(30):
        if cdp.eval("!!window.__dc"):
            break
        time.sleep(0.3)
    cdp.eval(native_avoid_js(True)); r = cdp.eval(native_goal_js(w["x"], w["y"]))
    print(f">>> PATHSNIFF -> {label} ({w['x']:+.2f},{w['y']:+.2f}). Goal enviado: {r}. El robot se movera.")
    print(f"    Cazando mensajes con pinta de ruta ~20s. Ctrl+C para parar. Log -> {out}")
    try:
        for i in range(40):
            mm = json.loads(cdp.eval("JSON.stringify(window.__pathmsgs||{})") or "{}")
            cand = cdp.eval("JSON.stringify(window.__pathcand||null)")
            lines = [f"=== PATHSNIFF {time.strftime('%H:%M:%S')} (pathlen = nº de puntos tipo ruta) ==="]
            for t, rec in sorted(mm.items(), key=lambda kv: -kv[1].get("pathlen", 0)):
                lines.append(f"  n={rec.get('n',0):<5} PATHLEN={rec.get('pathlen',0):<5} tipo='{t}' campos=[{rec.get('keys','')}]")
            if cand and cand != "null":
                lines.append(f"  >>> CANDIDATO DE RUTA: {cand}")
            else:
                lines.append("  (todavia sin candidato de ruta)")
            with open(out, "w") as f:
                f.write("\n".join(lines) + "\n")
            print("\033[2J\033[H", end=""); print("\n".join(lines))
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        cdp.eval(native_cancel_js())
        print(f"\nFin pathsniff. Guardado en {out}. Di 'mira el pathsniff' y lo analizo.")


def cmd_cloudgrab():
    """Captura una nube 'location' en vivo + la pose, y la guarda en reloc_cloud.json para analizar el frame."""
    cdp = g.get_cdp()
    cdp.eval(RELOC_CLOUD_JS)
    cdp.eval(RELOC_JS)
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "reloc_cloud.json")
    print(">>> CLOUDGRAB. Con el mapa cargado y los puntos visibles, espero a capturar una nube...")
    try:
        for _ in range(40):
            src, p, pcd = read_pose(cdp)
            n = int(cdp.eval("(window.__relocbuf||[]).length") or 0)
            print(f"  pose={'si' if p else 'no'}  puntos_nube={n}", end="\r")
            if p and n > 100:
                buf = json.loads(cdp.eval("JSON.stringify(window.__relocbuf||[])") or "[]")
                json.dump({"pose_src": src, "pose": p, "pcd": pcd, "npts": n, "points": buf[:9000]},
                          open(out, "w"))
                print(f"\n  GUARDADO: {n} valores ({n // 3} puntos), pose=({p[0]:+.2f},{p[1]:+.2f}) -> {out}")
                print("  Di 'mira el reloc_cloud' y analizo el frame para montar la rejilla 2D.")
                return
            time.sleep(0.5)
        print("\n  No capture nube. ¿Se ven los puntos en la app? ¿mapa cargado?")
    except KeyboardInterrupt:
        print("\nCancelado.")


def _install(cdp):
    cdp.eval(g.INSTALL_JS)          # captura mapeo (odom+nube+driver teleop) + grid hook
    cdp.eval(RELOC_JS)              # + pose de relocalizacion
    cdp.eval(RELOC_CLOUD_JS)        # + nube en vivo de operacion (mensaje 'location')
    cdp.eval(HEALTH_JS)             # + errorCode reloc + telemetria (bateria, cpu, motores)
    cdp.eval(IMUFULL_JS)            # + IMU completa (accel/gyro/rpy/par de patas)


def get_live_cdp():
    """Conecta a TODAS las paginas del WebView y devuelve el CDP de la que tiene el SLAM VIVO (pose +
    nube 'location'). Resuelve el problema de que get_cdp() coja una pagina muerta cuando hay varias
    (era la causa de nobs=0). Si solo hay una pagina, la devuelve directamente."""
    try:
        pages = _all_inspectable_pages()
    except Exception:
        pages = []
    if len(pages) <= 1:
        return g.get_cdp()
    cands = []
    for (ti, url, wsu) in pages:
        try:
            c = g.CDP(wsu)
            c.eval(RELOC_JS); c.eval(RELOC_CLOUD_JS)      # sondas ligeras (pose + nube)
            cands.append((ti, c))
        except Exception:
            pass
    if not cands:
        return g.get_cdp()
    if len(cands) == 1:
        return cands[0][1]
    best = None; bestsc = -1
    end = time.time() + 4.0
    while time.time() < end and bestsc < 3:
        for ti, c in cands:
            try:
                sc = (2 if c.eval("!!window.__pose") else 0) + \
                     (1 if int(c.eval("(window.__relocbuf||[]).length") or 0) > 50 else 0)
            except Exception:
                sc = -1
            if sc > bestsc:
                bestsc = sc; best = c
        time.sleep(0.4)
    print(f"  pagina viva elegida (score {bestsc}/3: pose+nube)" + ("" if bestsc >= 2 else "  <-- OJO: poca señal"))
    return best or cands[0][1]


def read_pose(cdp):
    """Pose LOCALIZADA sobre el mapa cargado. Prioriza slam_info.currentPose (autoritativa en relocalizacion),
    luego slam_relocation/odom, luego slam_mapping/odom. Devuelve (src, [x,y,z,qx,qy,qz,qw]) o (None,None)."""
    try:
        s = cdp.eval("JSON.stringify({pose:window.__pose||null, reloc:window.__relocodom||null, "
                     "map:window.__odom||null, pcd:window.__pcd||'', pt:window.__pose_t||0, rt:window.__relocodom_t||0})")
        d = json.loads(s) if s else {}
    except Exception:
        return (None, None, "")
    pcd = d.get("pcd", "")
    if d.get("pose"):
        return ("slam_info", d["pose"], pcd)
    if d.get("reloc"):
        return ("reloc_odom", d["reloc"], pcd)
    if d.get("map"):
        return ("map_odom", d["map"], pcd)
    return (None, None, pcd)


def yaw_of(q):
    """yaw (deg) desde quaternion [.. qx,qy,qz,qw] (indices 3..6)."""
    qx, qy, qz, qw = q[3], q[4], q[5], q[6]
    return math.degrees(math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)))


def cmd_reloccheck():
    """PASO 1: con el mapa cargado y el robot relocalizado en la app, muestra que datos llegan."""
    cdp = get_live_cdp()
    _install(cdp)
    print(">>> RELOCCHECK. En la app: carga el mapa y RELOCALIZA el robot. Mueve el robot a mano y mira")
    print("    que la pose CAMBIA (= localizacion viva). Ctrl+C para salir.\n")
    try:
        while True:
            src, p, pcd = read_pose(cdp)
            try:
                extra = json.loads(cdp.eval(
                    "JSON.stringify({buf:(window.__buf||[]).length, grid:Object.keys(window.__grid||{}).length, "
                    "dc:!!window.__dc})") or "{}")
            except Exception:
                extra = {}
            if p:
                print(f"  POSE[{src}] x={p[0]:+.2f} y={p[1]:+.2f} yaw={yaw_of(p):+6.1f}  | "
                      f"nube={extra.get('buf', 0)} pts  grid={extra.get('grid', 0)} celdas  "
                      f"camara={'si' if extra.get('dc') else 'no'}  mapa='{pcd}'")
            else:
                print("  (sin pose localizada todavia; ¿mapa cargado y relocalizado en la app?)")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nFin reloccheck.")


def reloc_cells(cdp, pose=None):
    """Celdas OBSTACULO (OCELL, frame del mapa) desde la nube en vivo 'location' (window.__relocbuf),
    filtrando la banda de altura de torso. La nube ya esta en el frame de la pose -> sin conversiones.
    Si se pasa 'pose' (x,y), descarta el campo cercano (<NEAR_BLIND): el anillo fantasma del cabeceo."""
    try:
        s = cdp.eval("JSON.stringify(window.__relocbuf||[])")
        buf = json.loads(s) if s else []
    except Exception:
        return set()
    px = py = None
    if pose is not None:
        px, py = pose[0], pose[1]
    cells = set()
    for i in range(0, len(buf) - 2, 3):
        z = buf[i + 2]
        if z < HBAND_LO or z > HBAND_HI:
            continue
        cx, cy = buf[i], buf[i + 1]
        if px is not None and math.hypot(cx - px, cy - py) < g.NEAR_BLIND:
            continue
        cells.add((round(cx / g.OCELL), round(cy / g.OCELL)))
    return cells


# Salud/telemetria del robot (robot_data) + estado de relocalizacion (errorCode). El firmware NO da
# covarianza de pose (todo ceros), asi que la CONFIANZA de localizacion la estimamos nosotros (scan-to-map).
HEALTH_JS = r"""(function(){
  if(!window.__healthHook){ window.__healthHook=1;
    var jp=JSON.parse;
    JSON.parse=function(s){ var v=jp.apply(this,arguments);
      try{ if(v && v.topic && (''+v.topic).indexOf('slam_info')>=0){
        var d=(typeof v.data==='string')?jp(v.data):v.data;
        if(d){ if(d.type==='pos_info'){ window.__poseErr=d.errorCode; }
          if(d.type==='robot_data' && d.data){ var m=d.data; var me=0,mm=(m.motorError||[]);
            for(var i=0;i<mm.length;i++){ if(mm[i]) me++; }
            var mtm=0,mhi=-1,mtt=(m.motorTemp||[]); for(var j=0;j<mtt.length;j++){ if(mtt[j]>mtm){mtm=mtt[j];mhi=j;} }
            window.__health={bat:m.batteryPower, vol:m.batteryVol, amp:m.batteryAmp, batT:m.batteryTemp,
              cpuT:m.cpuTemp, cpuU:m.cpuUsage, cpuMem:m.cpuMemory, cpuFreq:m.cpuFrequency,
              motTmax:mtm, motThot:mhi, merr:me, motorTemp:mtt, motorError:mm,
              sport:m.sportMode, gait:m.gaitType, t:Date.now()}; }
        }
      }}catch(e){}
      return v;
    };
  } return 1;
})()"""


# IMU completa de rt/lf/lowstate (~15Hz): quaternion, giroscopo, acelerometro, rpy + par max de patas.
IMUFULL_JS = r"""(function(){
  if(!window.__imuFullHook){ window.__imuFullHook=1;
    var jp=JSON.parse;
    JSON.parse=function(s){ var v=jp.apply(this,arguments);
      try{ if(v && v.topic && (''+v.topic).indexOf('lowstate')>=0 && v.data && v.data.imu_state){
        var im=v.data.imu_state; var ms=v.data.motor_state||[]; var mt=0;
        for(var i=0;i<12&&i<ms.length;i++){ var tq=Math.abs(ms[i].tau_est||0); if(tq>mt) mt=tq; }
        window.__imufull={quat:im.quaternion, gyro:im.gyroscope, accel:im.accelerometer, rpy:im.rpy,
          legtau:mt, t:Date.now()};
      }}catch(e){}
      return v;
    };
  } return 1;
})()"""


def read_telemetry(cdp):
    """TODO lo util: errorCode reloc + robot_data (bateria/cpu/motores) + IMU (accel/gyro/rpy/par). Dict o {}."""
    try:
        s = cdp.eval("JSON.stringify({err:(window.__poseErr==null?null:window.__poseErr), "
                     "h:(window.__health||null), imu:(window.__imufull||null), cov:(window.__reloccov||null)})")
        return json.loads(s) if s else {}
    except Exception:
        return {}


def read_health(cdp):
    """Compat: solo errorCode + robot_data (sin IMU)."""
    t = read_telemetry(cdp)
    return {"err": t.get("err"), "h": t.get("h")}


def _telem_row(hh):
    """Aplana read_telemetry() a una fila de telemetria para el dataset (campos no nulos)."""
    h = dict(hh.get("h") or {}); im = hh.get("imu") or {}
    h.pop("t", None)
    row = dict(h)
    row["err"] = hh.get("err")
    if im:
        row["accel"] = im.get("accel"); row["gyro"] = im.get("gyro")
        row["rpy"] = im.get("rpy"); row["legtau"] = im.get("legtau"); row["quat"] = im.get("quat")
    cov = hh.get("cov")
    if cov and len(cov) >= 15:                # covarianza de pose (6x6). traza de posicion xx+yy+zz
        row["pose_cov"] = cov                 # completa (el G1 la manda a 0)
        row["pose_cov_trace"] = round(cov[0] + cov[7] + cov[14], 5)
    return {k: v for k, v in row.items() if v is not None}


def ref_points():
    """Puntos de pared 2D (frame G1) del mapa de referencia elegido por env G1_REFMAP:
       'summit' (DEFECTO) = mapa del Summit ALINEADO a A/B (bien orientado) -> summit/ref_map_g1.json;
       'g1'               = mapa propio del G1 (dataset/map_full.json) — OJO: puede salir rotado/desalineado."""
    choice = os.environ.get("G1_REFMAP", "summit").lower()
    here = os.path.dirname(os.path.abspath(__file__))
    if choice != "g1":                          # por defecto, el mapa Summit alineado (orientacion correcta)
        p = os.path.join(here, "summit", "ref_map_g1.json")
        try:
            if os.path.exists(p):
                pts = [(q[0], q[1]) for q in json.load(open(p)).get("points", [])]
                if pts:
                    return pts
        except Exception:
            pass
    # 'g1': mapa propio del G1 (puede estar rotado/desalineado)
    def _clip(pts):
        return [(a, b) for (a, b) in pts if -15 <= a <= 15 and -15 <= b <= 15]   # quita outliers de reloc
    p3 = os.path.join(DATASET_DIR, "map_full.json")
    try:
        if os.path.exists(p3):
            d = json.load(open(p3)); pts = []
            for q in d.get("points", []):
                if len(q) >= 3 and HBAND_LO <= q[2] <= HBAND_HI:
                    pts.append((q[0], q[1]))
                elif len(q) == 2:
                    pts.append((q[0], q[1]))
            pts = _clip(pts)
            if pts:
                return pts
    except Exception:
        pass
    try:
        d = json.load(open(MAP_FILE)); OC = d.get("OCELL", g.OCELL)
        return _clip([(c[0] * OC, c[1] * OC) for c in d.get("cells", [])])
    except Exception:
        return []


def load_ref_map():
    """Mapa de referencia (set de celdas OCELL, frame G1) para confianza/plan. Ver ref_points()."""
    return set((round(x / g.OCELL), round(y / g.OCELL)) for x, y in ref_points())


def frame_check(lg, x0, y0):
    """Compara la pose INICIAL con el waypoint mas cercano. Si arrancas FISICAMENTE en un waypoint pero la
    pose dice que estas lejos de su coordenada guardada => la relocalizacion de esta sesion difiere de la
    de cuando se capturaron los waypoints (deriva de frame). Devuelve dict para el dataset."""
    try:
        wps = json.load(open(WP_FILE))
    except Exception:
        wps = {}
    if not wps:
        return None
    near, w = min(wps.items(), key=lambda kv: math.hypot(kv[1]["x"] - x0, kv[1]["y"] - y0))
    off = math.hypot(w["x"] - x0, w["y"] - y0)
    msg = (f"FRAME-CHECK start=({x0:+.2f},{y0:+.2f}) wp_mas_cercano={near}({w['x']:+.2f},{w['y']:+.2f}) "
           f"offset={off:.2f}m")
    try:
        lg.write(msg + "\n")
    except Exception:
        pass
    if off < 0.4:
        print(f"  {msg}  -> frame OK (pose y waypoints alineados)")
    else:
        print(f"  {msg}\n  >>> OJO: arrancas lejos del waypoint mas cercano. Si fisicamente estas EN un "
              f"waypoint, la relocalizacion difiere de la captura -> re-captura A/B/C en ESTA sesion.")
    return {"start": [round(x0, 2), round(y0, 2)], "nearest_wp": near, "offset_m": round(off, 2)}


def match_score(live_cells, ref_cells):
    """Confianza de localizacion ESTIMADA: fraccion de celdas del laser en vivo que caen sobre (o junto a)
    una celda del mapa conocido. ~1 = bien localizado (el laser encaja con el mapa); bajo = deriva/duda.
    Es la auto-evaluacion meta-cognitiva (el robot no da covarianza)."""
    if not ref_cells or not live_cells:
        return None
    hit = 0
    for (cx, cy) in live_cells:
        if any((cx + dx, cy + dy) in ref_cells for dx in (-1, 0, 1) for dy in (-1, 0, 1)):
            hit += 1
    return round(hit / len(live_cells), 3)


def grab_cam(cdp):
    """Foto actual de la camara del robot (data:image base64) o None. En una colision = prueba VISUAL de
    lo que habia (p.ej. una mesa que el LiDAR no ve)."""
    try:
        j = cdp.eval(g.CAM_JS)
        return j if (j and isinstance(j, str) and j.startswith("data:image")) else None
    except Exception:
        return None


def cam_floor_clear(cam):
    """Decodifica el frame de camara y devuelve (frac_suelo_CENTRO, near_run) con la segmentacion de suelo
    de g1_nav_v2. frac alto + near_run bajo = camino DESPEJADO por delante. Util donde el LASER es RUIDOSO
    (puerta/mesa): la VISION confirma si se puede pasar. None si no hay frame."""
    if not cam or not isinstance(cam, str) or not cam.startswith("data:image"):
        return None, None
    try:
        import base64, io
        from PIL import Image
        img = Image.open(io.BytesIO(base64.b64decode(cam.split(",", 1)[1]))).convert("RGB")
        lf, cf, rf, refS, nrun, mcont = g.floor_free_bands(img)
        return cf, nrun
    except Exception:
        return None, None


def grab_full_cloud(cdp, cap=12000):
    """Nube 'location' CRUDA (todas las alturas, sin filtrar) -> lista plana [x,y,z,...]. Para guardar en
    una colision y poder ver despues si era una MESA (tablero a media altura con hueco debajo) u otro
    obstaculo invisible a la banda de torso del LiDAR."""
    try:
        s = cdp.eval("JSON.stringify(window.__relocbuf||[])")
        buf = json.loads(s) if s else []
    except Exception:
        return []
    return buf[:cap]


def clear_dir(x, y, yaw_deg, off_deg, obs_pts, maxd=2.5, cone=25.0):
    """Distancia (m) al obstaculo mas cercano en un cono de +-cone deg hacia (yaw+off). Sustituye a
    clear_ahead() (que leia la nube de MAPEO, no disponible en modo nav): aqui se calcula de obs_pts."""
    best = maxd
    aim = yaw_deg + off_deg
    for (ox, oy) in obs_pts:
        dx = ox - x; dy = oy - y; d = math.hypot(dx, dy)
        if d < 0.05 or d >= best:
            continue
        ang = abs((math.degrees(math.atan2(dy, dx)) - aim + 180) % 360 - 180)
        if ang < cone:
            best = d
    return best


def global_plan(sx, sy, gx, gy, oset):
    """A* GLOBAL de (sx,sy) -> (gx,gy). Es solo para VISUALIZAR/orientar, asi que usa el costmap SIN inflado
    (marca solo las celdas-pared como bloqueadas) para que pueda cruzar puertas estrechas. Si no halla ruta
    devuelve la RECTA, para que la ventana siempre muestre algo."""
    cm = {c: math.inf for c in oset}
    cells = g.astar((round(sx / g.OCELL), round(sy / g.OCELL)),
                    (round(gx / g.OCELL), round(gy / g.OCELL)), cm, margin=25)
    if cells and len(cells) > 1:
        return [(c[0] * g.OCELL, c[1] * g.OCELL) for c in cells]
    return [(sx, sy), (gx, gy)]    # fallback: recta origen->destino


def navigate_to(cdp, lg, wx, wy, label, vshare=None, lock=None, stop_event=None):
    """NAVEGA A->B sobre el mapa cargado: A* (firmware-like) + DWA local, obstaculos de la nube 'location',
    contacto por IMU/odom y desatasco (reusados del frontier explorer). Para al llegar. Ctrl+C aborta.
    Si se pasan vshare/lock/stop_event, publica el estado para la ventana en vivo (modo viz)."""
    print(f"\n>>> GOTO '{label}' -> ({wx:+.2f},{wy:+.2f}). Mando en mano (L2+B). Ctrl+C aborta.")
    lg.write(f"\n=== RUN ours '{label}' -> ({wx:+.2f},{wy:+.2f})  {time.strftime('%Y-%m-%d %H:%M:%S')} ===\n"); lg.flush()
    cdp.eval(g.LOWSTATE_JS)                       # contacto rapido por par/accel
    # espera pose + primera nube
    print("  Esperando pose localizada y primera nube...", end="", flush=True)
    for _ in range(30):
        src, p, _ = read_pose(cdp)
        n = int(cdp.eval("(window.__relocbuf||[]).length") or 0)
        if p and n > 50:
            break
        time.sleep(0.3)
    else:
        print(" sin datos. ¿Mapa cargado y robot RELOCALIZADO en la app?"); return False
    print(" ok.")
    # --- GATE de relocalizacion (como la app): pose inicial vs waypoint mas cercano ---
    fc0 = frame_check(lg, p[0], p[1])
    if fc0 and fc0["offset_m"] > GATE_M and os.environ.get("G1_NOGATE") != "1":
        print(f"\n  >>> RELOCALIZACION DUDOSA: arrancas a {fc0['offset_m']}m del waypoint {fc0['nearest_wp']}. NO navego.")
        print("      Re-localiza en la app (que los puntitos encajen con el mapa) y reintenta (override: G1_NOGATE=1).")
        lg.write("GATE-BLOCKED reloc dudosa\n")
        return False

    omap = {}                                     # celda OCELL -> ultimo instante visto (mapa ACTIVO de obstaculos; lo lee el resto del codigo)
    oscore = {}                                   # celda OCELL -> score de confianza (anti-ruido: sube al verse, baja al no verse)
    oseen  = {}                                   # celda OCELL -> ultimo instante visto (para el TTL de respaldo del score)
    yaw_prev = None; yaw_prev_t = None            # para estimar yaw_rate (gate de rotacion)
    yaw_rate = 0.0                                # deg/s (se recalcula cada tick)
    livehist = deque(maxlen=PERSIST_N)            # ultimos N barridos del laser (filtro de ruido por persistencia)
    colmap = set()                                # colisiones PERMANENTES (no re-chocar en el mismo sitio)
    plan_pts = []; plan_t = 0; carrot = None
    fhist = []; prev_fwd = False; recov = None; ncol = 0; last_col_t = -99; rside = 1
    low_t = 0; last_low = None; lt_base = []; ah_base = []
    dhist = []; brk = None; brk_cool = 0; nbrk = 0; nstop = 0
    pose_t = time.time(); last_pose = None; t0 = time.time(); tprint = 0
    trail = []
    # --- diagnostico: calibracion de giro (signo real vs comandado) + spin ---
    prev_yaw = None; prev_cmd = (0, 0, 0, 0); prev_lt = None
    spin_acc = 0.0; prog_pos = None; prog_t = t0; turncal = []; phcount = {}
    minc0 = 9.9
    rd = RunRecorder("ours", label, (wx, wy))
    refmap = load_ref_map(); health_t = 0; hh = {}; cloud_ok = False; cloud_warned = False
    gplan = []; gplan_t = 0; cam_t = 0; cam_jpg = None
    aggressive = (os.environ.get("G1_AGGRESSIVE") == "1")    # modo agresivo (forzable; si no, se activa al atascarse)
    best_d = 1e9; best_d_t = t0; ROBOT_R0 = g.ROBOT_R        # progreso hacia B + holgura normal (para restaurar)
    vis_center = None; vis_nearrun = None; vis_t = 0         # VISION (suelo despejado) para la puerta (laser ruidoso ahi)
    # --- PERCEPCION GPU offboard (opcional): depth -> scan virtual que VE la mesa invisible al LiDAR ---
    perc = g1_perception.make_client_from_env(g.OCELL) if g1_perception else None
    perc_worker = g1_perception.PerceptionWorker(perc) if perc else None       # consulta GPU en HILO APARTE (no congela el control)
    if perc_worker:
        perc_worker.start()
    perc_t = 0; perc_cells = set(); perc_dets = []; nperc = 0
    sei = g1_metrics.SEIMetrics()                            # clearance + progression por tick (las 2 metricas del tutor)
    sens = g1_metrics.SensingMonitor()                       # auto-evaluacion de sensado (ruido/fiabilidad) = feedback de capacidad
    m_clear = 0.0; m_prog = 0.0; m_rel = 1.0; m_cl = 0.0; m_cr = 0.0
    print(f"  mapa de referencia: {len(refmap)} celdas" + (" (sin mapa -> confianza N/A)" if not refmap else ""))
    try:
        while not (stop_event is not None and stop_event.is_set()):
            now = time.time()
            src, p, pcd = read_pose(cdp)
            if not p:
                cdp.eval(g.STOP_JS)
                if now - pose_t > 3.0:
                    print("\n  POSE PERDIDA (3s). STOP. Relocaliza en la app y reintenta."); return False
                time.sleep(0.2); continue
            x, y, yaw = p[0], p[1], yaw_of(p)         # yaw en GRADOS
            reloc_flag = False
            if last_pose is not None and math.hypot(x - last_pose[0], y - last_pose[1]) > 0.5:
                jd = math.hypot(x - last_pose[0], y - last_pose[1])     # >0.5m en un ciclo (~0.1s) = salto reloc
                reloc_flag = True
                rd.event("reloc_jump", now - t0, x, y, {"dist": round(jd, 2),
                                                        "from": [round(last_pose[0], 2), round(last_pose[1], 2)]})
                lg.write(f"RELOC-JUMP {jd:.2f}m de ({last_pose[0]:+.2f},{last_pose[1]:+.2f}) a ({x:+.2f},{y:+.2f})\n")
            if last_pose is None or abs(x - last_pose[0]) > 1e-4 or abs(y - last_pose[1]) > 1e-4:
                pose_t = now; last_pose = (x, y)
            if not trail:
                rd.rec["frame_check"] = fc0
            if not trail or math.hypot(x - trail[-1][0], y - trail[-1][1]) > 0.05:
                trail.append((x, y))

            d_goal = math.hypot(wx - x, wy - y)
            if d_goal < NAV_REACH:                    # --- LLEGADA ---
                cdp.eval(g.STOP_JS); time.sleep(0.2); cdp.eval(g.STOP_JS)
                print(f"\n  LLEGADO a '{label}' ({wx:+.2f},{wy:+.2f}); error={d_goal:.2f} m, colisiones={ncol}.")
                lg.write(f"REACHED {label} err={d_goal:.2f} ncol={ncol} {time.strftime('%Y-%m-%d %H:%M:%S')}\n"); lg.flush()
                T = now - t0; plen = _path_len(trail)
                straight = math.hypot(wx - trail[0][0], wy - trail[0][1]) if trail else 0.0
                rd.save_cloud("end", [round(x, 3), round(y, 3), round(yaw, 1)], grab_full_cloud(cdp))
                rd.finish("reached", {"time_s": round(T, 2), "path_m": round(plen, 2),
                                      "straight_m": round(straight, 2),
                                      "efficiency": round(straight / plen, 2) if plen > 0 else 0.0,
                                      "collisions": ncol, "c0min": round(minc0, 2),
                                      "perc_queries": nperc,
                                      "start": {"x": round(trail[0][0], 3), "y": round(trail[0][1], 3)} if trail else None})
                if vshare is not None:                # marca llegada en la ventana antes de salir
                    with lock:
                        vshare["ph"] = "LLEGADO"; vshare["x"] = x; vshare["y"] = y
                return True

            # --- OBSTACULOS de la nube 'location' (frame mapa) -> mapa con SCORE/DECAY (anti-ruido) ---
            live = reloc_cells(cdp)                   # celdas del barrido ACTUAL (laser en vivo)
            if live:
                cloud_ok = True
            elif not cloud_ok and not cloud_warned and now - t0 > 4.0:
                print("\n  [AVISO] no llega la nube 'location' -> NO puedo planificar (sin obstaculos).")
                print("          ¿se ven los PUNTITOS del laser en la app?")
                lg.write("NO-CLOUD warning\n"); cloud_warned = True
            # --- CONFIRMACION de ENTRADA (confiar mas en el mapa) ---
            # Una celda del MAPA estatico (pared conocida) se confirma YA; una celda que SOLO ve el laser
            # necesita verse en >=PERSIST_K de los ultimos PERSIST_N barridos (filtra el parpadeo de 1 barrido:
            # ruido del cabeceo o nube desplazada por un salto de reloc).
            livehist.append(live)
            confirmed = set()
            for c in live:
                if refmap and c in refmap:            # confirmado por el MAPA -> instantaneo (confiamos en el mapa)
                    confirmed.add(c)
                elif sum(1 for h in livehist if c in h) >= PERSIST_K:   # confirmado por PERSISTENCIA del laser
                    confirmed.add(c)
            confirmed = {c for c in confirmed         # descarta campo cercano (anillo fantasma del cabeceo)
                         if math.hypot(c[0] * g.OCELL - x, c[1] * g.OCELL - y) >= g.NEAR_BLIND}
            # --- PERCEPCION GPU (HILO APARTE): depth -> scan virtual (la MESA que el LiDAR no ve) + suelo despejado ---
            if perc_worker is not None and now - perc_t > PERC_PERIOD:
                perc_worker.submit(grab_cam(cdp), x, y, yaw)   # no bloquea: el hilo hace la consulta GPU
                perc_t = now
            if perc_worker is not None and perc_worker.latest is not None:
                res = perc_worker.latest                       # ultimo resultado disponible (puede ir 1-2 ticks por detras)
                nperc = perc_worker.n_ok
                perc_cells = res.cells
                perc_dets = res.detections or []
                if res.free_center is not None:                # VISION basada en depth/seg (mejor que la heuristica)
                    vis_center, vis_nearrun = res.free_center, (res.near_run or 0); vis_t = now
            vis_conf = {c for c in perc_cells         # obstaculos de vision (mesa) -> tambien pasan por el score
                        if math.hypot(c[0] * g.OCELL - x, c[1] * g.OCELL - y) >= g.NEAR_BLIND}
            seen_now = confirmed | vis_conf           # candidatos vistos AHORA (laser confirmado + vision)
            # --- GATE DE ROTACION: estima yaw_rate; si el giro es rapido, la nube va poco fiable ---
            if yaw_prev is not None and yaw_prev_t is not None and now > yaw_prev_t:
                yaw_rate = abs((yaw - yaw_prev + 180.0) % 360.0 - 180.0) / (now - yaw_prev_t)
            yaw_prev, yaw_prev_t = yaw, now
            turning_fast = (YAW_GATE > 0) and (yaw_rate > YAW_GATE)

            if OLDMAP:
                # --- MODO ANTIGUO (G1_OLDMAP=1): entra al instante y dura NAV_OMAP_TTL -> UNION de 60s (acumula ruido) ---
                for c in seen_now:
                    omap[c] = now
                omap = {c: t for c, t in omap.items() if now - t < NAV_OMAP_TTL}
            else:
                # --- SCORE/DECAY: obstaculo solo si se ve en la MAYORIA de barridos recientes ---
                # Solo se actualiza con barrido fresco (si cae la nube, no penalizamos -> no se borra el mapa).
                # El decay solo actua dentro de SC_RANGE (ventana de plan); fuera de rango se conserva por TTL
                # de respaldo (memoria de paredes ya pasadas, no se borran por dejar de verse al girar).
                if (live or seen_now) and not turning_fast:   # GATE: no actualizar el mapa mientras se gira rapido
                    for c in confirmed:               # laser confirmado: sube score; mapa estatico -> tope (instantaneo)
                        oscore[c] = SC_CAP if (refmap and c in refmap) else min(SC_CAP, oscore.get(c, 0.0) + SC_HIT)
                        oseen[c] = now
                    for c in vis_conf:                # vision: sube score (necesita ~2 frames -> filtra depth ruidoso)
                        if c not in confirmed:
                            oscore[c] = min(SC_CAP, oscore.get(c, 0.0) + SC_HIT)
                            oseen[c] = now
                    for c in list(oscore.keys()):     # decay + purga
                        if c in seen_now:
                            continue
                        if math.hypot(c[0] * g.OCELL - x, c[1] * g.OCELL - y) < SC_RANGE:
                            oscore[c] -= SC_MISS      # en rango y no visto -> ruido intermitente: penaliza
                        if oscore[c] <= 0.0 or now - oseen.get(c, now) >= NAV_OMAP_TTL:
                            oscore.pop(c, None); oseen.pop(c, None)
                # mapa ACTIVO = celdas con score suficiente (omap sigue siendo celda->instante para el resto del codigo)
                omap = {c: oseen[c] for c, s in oscore.items() if s >= SC_OBST}
            oset = set(omap.keys()) | colmap
            op = [(cx * g.OCELL, cy * g.OCELL) for (cx, cy) in oset
                  if abs(cx * g.OCELL - x) < 2.6 and abs(cy * g.OCELL - y) < 2.6]
            c0 = clear_dir(x, y, yaw, 0, op); minc0 = min(minc0, c0)
            mm = sei.update(now - t0, d_goal, c0)            # 2 metricas SEI: clearance (espacio libre) + progression (avance a B)
            m_clear, m_prog = mm["clearance"], mm["progression"]
            cl_left = clear_dir(x, y, yaw, +60, op); cl_right = clear_dir(x, y, yaw, -60, op)   # clearance IZQ/DCHA (Renxi: balancear)
            m_cl = min(1.0, cl_left / 1.5); m_cr = min(1.0, cl_right / 1.5)
            # VISION en zona estrecha (puerta/mesa): el laser ahi es ruidoso -> mira si la camara ve suelo despejado.
            # Si hay servidor de percepcion GPU, ya rellena vis_* arriba; esto es el FALLBACK heuristico (sin GPU).
            if perc is None and c0 < 1.1 and now - vis_t > 0.5:
                vis_center, vis_nearrun = cam_floor_clear(grab_cam(cdp)); vis_t = now
            cmd = None; ph = ""

            # --- CONTACTO (odom-stall fiable / IMU rapido por par-accel) ---
            if now - low_t > 0.2:
                lw = g.read_low(cdp)
                if lw:
                    last_low = (math.hypot(lw.get("ax", 0.0), lw.get("ay", 0.0)), lw.get("legtau", 0.0))
                low_t = now
            cur_ah, cur_lt = last_low if last_low else (None, None)
            if prev_fwd:
                fhist.append((now, x, y))
            fhist = [h for h in fhist if now - h[0] <= 2.0]
            mvd = math.hypot(x - fhist[0][1], y - fhist[0][2]) if len(fhist) >= 2 else 0.0
            if prev_fwd and cur_lt is not None and mvd > 0.10:
                lt_base.append(cur_lt); lt_base = lt_base[-40:]
                ah_base.append(cur_ah); ah_base = ah_base[-40:]
            contact = False; ctype = ""
            if recov is None and brk is None and now - last_col_t > 4.0:
                if len(fhist) >= 8 and now - fhist[0][0] >= 0.9 and mvd < 0.05:
                    contact = True; ctype = "odom"
                elif (g.IMU_CONTACT and prev_fwd and cur_lt is not None and len(fhist) >= 5
                      and now - fhist[0][0] >= 0.5 and mvd < 0.04):
                    bl = sorted(lt_base)[len(lt_base) // 2] if len(lt_base) >= 5 else 15.0
                    ba = sorted(ah_base)[len(ah_base) // 2] if len(ah_base) >= 5 else 1.5
                    if cur_lt > bl * 1.5 + 3.0 or cur_ah > ba + 1.8:
                        contact = True; ctype = "imu"
            if contact:
                ncol += 1; last_col_t = now
                yr = math.radians(yaw); fxx, fyy = math.cos(yr), math.sin(yr); pxx, pyy = -fyy, fxx
                for d in (0.30, 0.45):                     # marca PEQUEÑA (antes encerraba el paso de vuelta)
                    for L in (-0.1, 0.0, 0.1):
                        colmap.add((round((x + d * fxx + L * pxx) / g.OCELL),
                                    round((y + d * fyy + L * pyy) / g.OCELL)))
                cl = clear_dir(x, y, yaw, +55, op); cr = clear_dir(x, y, yaw, -55, op)
                rside = 1 if cl >= cr else -1
                recov = {"ph": "BACK", "t0": now}; fhist = []; plan_pts = []
                print(f"\n  COLISION #{ncol} [{ctype}] en ({x:+.2f},{y:+.2f}) -> marco y recupero.")
                lg.write(f"COLISION #{ncol} [{ctype}] pos=({x:+.2f},{y:+.2f}) yaw={yaw:+.0f}\n")
                rd.event("collision", now - t0, x, y, {"src": ctype})
                rd.save_cloud(f"col{ncol}", [round(x, 3), round(y, 3), round(yaw, 1)], grab_full_cloud(cdp))
                rd.save_cam(f"col{ncol}", grab_cam(cdp))

            # --- RECUPERACION: mini paso atras (si hay hueco detras) + pivota ---
            if recov is not None:
                el = now - recov["t0"]
                if recov["ph"] == "BACK":
                    rear = clear_dir(x, y, yaw, 180, op)
                    if el < 0.45 and rear > 0.6:
                        cmd = (0, -0.35, 0, 0); ph = "R-BACK"
                    else:
                        recov = {"ph": "TURN", "t0": now}; el = 0
                if recov is not None and recov["ph"] == "TURN":
                    if el < 1.3:
                        cmd = (0, 0, -g.AV_TURN if rside > 0 else g.AV_TURN, 0); ph = "R-TURN"
                    else:
                        recov = {"ph": "GO", "t0": now}; el = 0
                if recov is not None and recov["ph"] == "GO":
                    if el < 1.0 and c0 > g.EXP_FWD_MIN:
                        cmd = (0, g.FWD_SPEED, 0, 0); ph = "R-GO "
                    else:
                        recov = None

            # --- DESATASCO: sin avanzar STUCK_SEC -> mini atras + giro grande hacia el lado mas abierto ---
            dhist.append((now, x, y))
            dhist = [h for h in dhist if now - h[0] <= g.STUCK_SEC]
            if (recov is None and brk is None and now > brk_cool and len(dhist) >= 2
                    and now - dhist[0][0] >= g.STUCK_SEC * 0.9
                    and math.hypot(x - dhist[0][1], y - dhist[0][2]) < g.STUCK_DISP):
                cl = clear_dir(x, y, yaw, +55, op); cr = clear_dir(x, y, yaw, -55, op)
                nbrk += 1; brk = {"ph": "BACK", "t0": now, "dir": -g.AV_TURN if cl >= cr else g.AV_TURN}
                plan_pts = []
                print(f"\n  DESATASCO #{nbrk} en ({x:+.2f},{y:+.2f}).")
                lg.write(f"DESATASCO #{nbrk} pos=({x:+.2f},{y:+.2f})\n")
            if brk is not None:
                el = now - brk["t0"]
                if brk["ph"] == "BACK":
                    rear = clear_dir(x, y, yaw, 180, op)
                    if el < 0.45 and rear > 0.6:
                        cmd = (0, -0.35, 0, 0); ph = "BRK-BK"
                    else:
                        brk = {"ph": "TURN", "t0": now, "dir": brk["dir"]}; el = 0
                if brk is not None and brk["ph"] == "TURN":
                    if el < g.BRK_TURN_SEC:
                        cmd = (0, 0, brk["dir"], 0); ph = "BRK-TR"
                    else:
                        brk = None; brk_cool = now + 6.0; dhist = []; plan_pts = []

            # --- MODO AGRESIVO: si lleva atascado sin ACERCARSE a B, baja inflado y holgura (con minimo seguro) ---
            if d_goal < best_d - 0.15:
                best_d = d_goal; best_d_t = now
            if not aggressive and now - best_d_t > AGGR_AFTER:
                aggressive = True
                print(f"\n  >>> MODO AGRESIVO ON ({AGGR_AFTER:.0f}s sin acercarse a B): reduzco inflado y holgura "
                      f"(min seguridad {AGGR_ROBOT_R}m) para cruzar la puerta.")
                lg.write(f"AGGRESSIVE-ON t={now-t0:.0f}s d={d_goal:.2f}\n")
            g.ROBOT_R = AGGR_ROBOT_R if aggressive else ROBOT_R0   # holgura del DWA (min seguridad en agresivo)

            # --- PLAN A* + CONTROL LOCAL DWA (hacia el WAYPOINT, no una frontera) ---
            if cmd is None:
                if (not plan_pts) or (now - plan_t > g.PLAN_SEC):
                    # PLAN: 1) sobre el LASER VIVO (rapido y preciso para ACERCARSE, como antes);
                    #       2) si falla (puerta sellada/estrecha en el mapa vivo) -> sobre el MAPA DE
                    #          REFERENCIA limpio SIN inflado (que tiene la puerta abierta). DWA = seguridad.
                    scell = (round(x / g.OCELL), round(y / g.OCELL))
                    gcell = (round(wx / g.OCELL), round(wy / g.OCELL))
                    if aggressive:
                        # AGRESIVO: obstaculos RECIENTES (4s, no sella la puerta) + mapa limpio (puerta abierta),
                        # SIN colmap (las marcas de colision encierran) ni inflado. Rodea la mesa, cruza la puerta.
                        recent = {c for c, tt in omap.items() if now - tt < 4.0}
                        cm = {c: math.inf for c in (recent | (refmap or set()))}
                    else:
                        cm = g.build_costmap(oset)         # aproximacion rapida con el laser vivo (con seguridad)
                    cells_path = g.astar(scell, gcell, cm)
                    if not cells_path and refmap:          # ultimo recurso: solo el mapa limpio (siempre tiene la puerta)
                        cells_path = g.astar(scell, gcell, {c: math.inf for c in refmap})
                    plan_pts = [(c[0] * g.OCELL, c[1] * g.OCELL) for c in cells_path] if cells_path else []
                    plan_t = now
                    if not plan_pts:
                        lg.write(f"A*-FAIL goal=({wx:+.1f},{wy:+.1f}) d={d_goal:.1f} obs={len(oset)}\n")
                if plan_pts:
                    carrot = g.path_carrot(plan_pts, x, y)
                    # --- MANIOBRA DE PUERTA: SOLO cuando el robot YA esta en zona estrecha (c0 bajo) y hay un
                    #     cuello MUY cerca por delante. Asi NO se activa al inicio (en abierto va con DWA rapido). ---
                    door = None
                    if op and c0 < 0.9 and d_goal > DOOR_MIN_GOAL:   # cerca de B NO hay puerta: deja el DWA rodear
                        bc = 9.9; bi = -1
                        for i, p in enumerate(plan_pts):
                            dd = math.hypot(p[0] - x, p[1] - y)
                            if dd < 0.1 or dd > 0.9:        # cuello MUY cerca (no desde lejos)
                                continue
                            clr = min((p[0] - o[0]) ** 2 + (p[1] - o[1]) ** 2 for o in op) ** 0.5
                            if clr < bc:
                                bc = clr; bi = i
                        if bi >= 0 and bc < 0.42:           # vano estrecho justo delante = puerta
                            door = (bi, plan_pts[bi], bc)
                    if door is not None:
                        bi, dp, bc = door
                        a = plan_pts[max(0, bi - 2)]; b = plan_pts[min(len(plan_pts) - 1, bi + 2)]
                        ddir = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))   # eje de la puerta
                        he = (ddir - yaw + 180) % 360 - 180
                        # VISION manda en la puerta (el laser es ruidoso ahi): suelo despejado por delante?
                        vis_ok = (vis_center is not None and vis_center > 0.45 and (vis_nearrun or 0) < 8)
                        bal = m_cl - m_cr                   # >0 = mas libre a la IZQ ; <0 = mas libre a la DCHA
                        if abs(he) > 25:                    # 1) alinea el rumbo con el eje ANTES de entrar.
                            # Banda ANCHA (25, no 12): el eje de puerta 'ddir' tiembla porque el A* replanifica
                            # cada tick con el laser ruidoso; con banda estrecha + giro fijo 0.45 (no se puede
                            # bajar: hay deadzone ~0.3) el robot oscilaba sin parar (thrash). 25 lo tolera y deja
                            # de cazar el ruido; el centrado fino lo hace DOOR-CTR (strafe).
                            cmd = (0, 0, -g.AV_TURN if he > 0 else g.AV_TURN, 0); ph = "DOOR-AL"
                        elif DOOR_CENTER and abs(bal) > DOOR_BAL_TH and max(cl_left, cl_right) > 0.30:
                            # 2) DESCENTRADO -> strafe hacia el lado MAS LIBRE para entrar centrado (Renxi)
                            lx = DOOR_STRAFE_SIGN * (DOOR_STRAFE if bal > 0 else -DOOR_STRAFE)
                            cmd = (lx, 0, 0, 0); ph = "DOOR-CTR"
                        elif c0 > AGGR_ROBOT_R or vis_ok:   # 3) entra RECTO si el LASER o la VISION lo ven despejado
                            cmd = (0, 0.28, 0, 0); ph = "DOOR-GO" + ("v" if vis_ok and c0 <= AGGR_ROBOT_R else "")
                        else:
                            cmd = (0, 0, 0, 0); ph = "DOOR-WT"   # ni laser ni vision: espera (no fuerza)
                        nstop = 0
                    else:
                        _, lyc, rxc, _, lbl = g.dwa_step(x, y, yaw, carrot, op)
                        cmd = (0, lyc, rxc, 0); ph = lbl
                        if lyc == 0 and rxc == 0:
                            nstop += 1
                            if nstop > 12:                  # ~1.2s encajonado -> desatasco
                                cl = clear_dir(x, y, yaw, +55, op); cr = clear_dir(x, y, yaw, -55, op)
                                brk = {"ph": "BACK", "t0": now, "dir": -g.AV_TURN if cl >= cr else g.AV_TURN}
                                nstop = 0; plan_pts = []
                        else:
                            nstop = 0
                else:
                    # sin ruta A* -> orienta al goal y avanza si el frente esta despejado (busqueda simple)
                    bg = math.degrees(math.atan2(wy - y, wx - x))
                    be = (bg - yaw + 180) % 360 - 180
                    if abs(be) > 20:
                        cmd = (0, 0, -g.AV_TURN if be > 0 else g.AV_TURN, 0); ph = "SEEK-T"
                    elif c0 > g.EXP_FWD_MIN:
                        cmd = (0, g.FWD_SPEED, 0, 0); ph = "SEEK-F"
                    else:
                        nstop += 1; cmd = (0, 0, 0, 0); ph = "SEEK-S"
                        if nstop > 12:
                            cl = clear_dir(x, y, yaw, +55, op); cr = clear_dir(x, y, yaw, -55, op)
                            brk = {"ph": "BACK", "t0": now, "dir": -g.AV_TURN if cl >= cr else g.AV_TURN}
                            nstop = 0

            # --- DIAGNOSTICO: rumbos a objetivo y carrot ---
            bg = math.degrees(math.atan2(wy - y, wx - x))
            beg = (bg - yaw + 180) % 360 - 180                          # error de rumbo al OBJETIVO
            bce = None
            if carrot is not None:
                bc = math.degrees(math.atan2(carrot[1] - y, carrot[0] - x))
                bce = (bc - yaw + 180) % 360 - 180                      # error de rumbo al CARROT
            # --- DIAGNOSTICO: CALIBRACION DE GIRO (signo) — clave del "da mil vueltas" ---
            # compara el giro REAL medido (dyaw/dt) con el que el comando ANTERIOR deberia producir.
            # modelo del DWA: wz=-1.8*rx (rad/s) -> deg/s = -103*rx. Si el signo medido != esperado -> el
            # robot gira al REVES que el modelo -> nunca converge -> vueltas infinitas.
            if prev_lt is not None and prev_yaw is not None:
                dt = now - prev_lt
                if dt > 0.01:
                    dyaw = (yaw - prev_yaw + 180) % 360 - 180
                    yawrate = dyaw / dt
                    rxp = prev_cmd[2]; lyp = prev_cmd[1]
                    if abs(rxp) > 0.1 and abs(lyp) < 0.05:             # giro puro previo
                        exp = -103.0 * rxp                            # deg/s esperado por el modelo
                        ok = (yawrate * exp > 0) or abs(yawrate) < 5
                        turncal.append((rxp, yawrate))
                        lg.write(f"  TURN-CAL rx={rxp:+.2f} esperado={exp:+.0f}deg/s medido={yawrate:+.0f}deg/s "
                                 f"{'OK' if ok else '>>> SIGNO INVERTIDO <<<'}\n")
                    spin_acc += abs(dyaw)
            prev_yaw = yaw; prev_lt = now; prev_cmd = cmd
            # progreso real (desplazamiento): si avanza, resetea el acumulador de giro
            if prog_pos is None or math.hypot(x - prog_pos[0], y - prog_pos[1]) > 0.15:
                prog_pos = (x, y); prog_t = now; spin_acc = 0.0
            phcount[ph.strip()] = phcount.get(ph.strip(), 0) + 1
            if spin_acc > 540 and now - prog_t > 4.0:                  # >1.5 vueltas sin avanzar 15cm
                lg.write(f"  SPIN!! girado {spin_acc:.0f}deg sin avanzar en {now-prog_t:.0f}s | yaw={yaw:+.0f} "
                         f"goal_err={beg:+.0f} carrot_err={(bce if bce is not None else 0):+.0f} c0={c0:.2f} "
                         f"plan={len(plan_pts)} fases={phcount}\n"); lg.flush()
                spin_acc = 0.0

            if aggressive:
                ph = "AGR-" + ph.strip()
            line = (f"t={now-t0:5.1f} {ph} pos=({x:+.2f},{y:+.2f}) yaw={yaw:+6.1f} d={d_goal:.2f} "
                    f"goal_err={beg:+.0f} carrot_err={(bce if bce is not None else 0):+4.0f} "
                    f"c0={c0:.2f} clear={m_clear:.2f} prog={m_prog:.2f} rel={m_rel:.2f} bal={m_cl-m_cr:+.2f} "
                    f"obs={len(oset)} obsc={len(oscore)} yr={yaw_rate:3.0f}{'*' if (YAW_GATE>0 and yaw_rate>YAW_GATE) else ' '} plan={len(plan_pts)} cmd=(lx={cmd[0]:+.2f},ly={cmd[1]:+.2f},rx={cmd[2]:+.2f})")
            lg.write(line + "\n"); lg.flush()
            if now - health_t > 1.0:
                hh = read_telemetry(cdp); health_t = now
                rd.telem(now - t0, _telem_row(hh))
            loc = match_score(live, refmap)
            ss2 = sens.update(now - t0, live, c0, loc, reloc_flag)   # auto-evaluacion: ruido/fiabilidad de sensado
            m_rel = ss2["reliability"]
            h = hh.get("h") or {}
            rd.sample(now - t0, x, y, yaw, d_goal, math.hypot(x - prog_pos[0], y - prog_pos[1]) if prog_pos else 0.0,
                      c0, len(oset), cmd=cmd, phase=ph.strip(),
                      extra={"err": hh.get("err"), "bat": h.get("bat"), "cpuT": h.get("cpuT"),
                             "merr": h.get("merr"), "loc_match": loc,
                             "clearance": mm["clearance"], "clearance_m": mm["clearance_m"],
                             "progression": mm["progression"], "progress_rate": mm["progress_rate"],
                             "reliability": ss2["reliability"], "laser_noise": ss2["laser_noise"],
                             "loc_conf": ss2["loc_conf"], "c0_std": ss2["c0_std"],
                             "scan_churn": ss2["scan_churn"], "reloc_rate10s": ss2["reloc_rate10s"],
                             "perc_n": len(perc_cells),                       # nº de celdas-obstaculo que aporto la VISION este tick
                             "clear_left": round(m_cl, 3), "clear_right": round(m_cr, 3),   # clearance lateral (Renxi: balance)
                             "clearL_m": round(cl_left, 2), "clearR_m": round(cl_right, 2),
                             "balance": round(m_cl - m_cr, 3),                # +izq libre / -dcha libre (0 = centrado)
                             "dets": ([[d.get("label"), round(d.get("conf", 0), 2),
                                        d.get("bearing_deg"), d.get("range_m")] for d in perc_dets] or None)})
            rd.maybe_laser(now - t0, op)
            if now - tprint > 0.4:
                print("  " + line); tprint = now

            # --- PLAN GLOBAL origen->destino (para verlo completo en la ventana) ---
            if vshare is not None and now - gplan_t > 3.0 and trail:
                gplan = global_plan(trail[0][0], trail[0][1], wx, wy, oset or refmap)
                gplan_t = now
            # --- publica estado para la ventana en vivo ---
            if vshare is not None:
                if now - cam_t > 0.5:
                    cam_jpg = grab_cam(cdp); cam_t = now
                with lock:
                    vshare["x"] = x; vshare["y"] = y; vshare["yaw"] = yaw; vshare["ph"] = ph
                    vshare["d"] = d_goal; vshare["col"] = ncol; vshare["t"] = now - t0
                    vshare["goal"] = (wx, wy); vshare["carrot"] = carrot; vshare["cam"] = cam_jpg
                    vshare["obs"] = [(cx * g.OCELL, cy * g.OCELL) for (cx, cy) in oset]      # mapa acumulado (m)
                    vshare["laser"] = [(cx * g.OCELL, cy * g.OCELL) for (cx, cy) in live]    # barrido en vivo (m)
                    vshare["plan"] = list(plan_pts)                                          # ruta A* local (m)
                    vshare["gplan"] = list(gplan)                                            # PLAN GLOBAL origen->destino
                    vshare["trail"] = list(trail)                                            # odometria recorrida
                    vshare["clear"] = m_clear; vshare["prog"] = m_prog; vshare["rel"] = m_rel  # 3 metricas (valor actual)
                    vshare["mhist"] = sei.history()                                          # historia (t,clearance,progression)
                    vshare["shist"] = sens.history()                                         # historia (t,reliability,noise,loc_conf)

            prev_fwd = (cmd[1] > 0.1)
            cdp.eval(g.set_cmd_js(*cmd))
            time.sleep(0.1)
        rd.finish("aborted", {"time_s": round(time.time() - t0, 2), "path_m": round(_path_len(trail), 2),
                              "collisions": ncol, "c0min": round(minc0, 2)})
        return False                                  # salida por cierre de ventana (stop_event)
    except KeyboardInterrupt:
        print(f"\n  [ABORTADO '{label}']"); lg.write(f"ABORT {label} {time.strftime('%Y-%m-%d %H:%M:%S')}\n"); lg.flush()
        rd.finish("aborted", {"time_s": round(time.time() - t0, 2), "path_m": round(_path_len(trail), 2),
                              "collisions": ncol, "c0min": round(minc0, 2)})
        return False
    finally:
        cdp.eval(g.STOP_JS); time.sleep(0.2); cdp.eval(g.STOP_JS)
        g.ROBOT_R = locals().get("ROBOT_R0", g.ROBOT_R)    # restaura la holgura normal del DWA
        # resumen de la calibracion de giro: ¿el robot gira en el sentido que el modelo cree?
        tc = locals().get("turncal", [])
        if tc:
            pos = [yr for rx, yr in tc if rx > 0]; neg = [yr for rx, yr in tc if rx < 0]
            mp = sorted(pos)[len(pos) // 2] if pos else None
            mn = sorted(neg)[len(neg) // 2] if neg else None
            verdict = "OK (modelo correcto)"
            if mp is not None and mp > 5:   verdict = ">>> SIGNO INVERTIDO: rx>0 deberia BAJAR yaw y lo SUBE <<<"
            if mn is not None and mn < -5:  verdict = ">>> SIGNO INVERTIDO: rx<0 deberia SUBIR yaw y lo BAJA <<<"
            ph = locals().get("phcount", {})
            lg.write(f"TURN-CAL-RESUMEN rx>0 -> medido~{mp}deg/s (modelo<0) ; rx<0 -> medido~{mn}deg/s (modelo>0) "
                     f":: {verdict}\n")
            lg.write(f"FASES {ph}\n")
            lg.flush()
        if stop_event is not None:
            stop_event.set()


def cmd_goto(label=None):
    """PASO 4: navega a un waypoint guardado. Con argumento (A/B/...) va una vez; sin argumento, menu en
    vivo: escribes la etiqueta y te lleva, al llegar pide otra. 'q' para salir."""
    wps = _load_wps()
    if not wps:
        print("Sin waypoints. Captura primero: python g1_goto.py waypoint A"); return
    cdp = g.get_cdp()
    _install(cdp)
    lg = open(GOTO_LOG, "a")
    lg.write(f"\n=== GOTO {time.strftime('%Y-%m-%d %H:%M:%S')} ===\n")

    def go(lbl):
        lbl = (lbl or "").strip()
        if lbl not in wps:
            print(f"  '{lbl}' no existe. Waypoints: {list(wps.keys())}"); return
        w = wps[lbl]
        navigate_to(cdp, lg, w["x"], w["y"], lbl)

    try:
        if label:
            go(label); return
        print(f"Waypoints disponibles: {', '.join(wps.keys())}")
        print("Escribe una etiqueta (A/B/...) y Enter para ir; 'q' para salir.")
        while True:
            try:
                sel = input("goto> ").strip()
            except (EOFError, KeyboardInterrupt):
                break
            if sel.lower() in ("q", "quit", "exit", ""):
                break
            go(sel)
    finally:
        cdp.eval(g.STOP_JS); time.sleep(0.2); cdp.eval(g.STOP_JS)
        lg.write("FIN\n"); lg.close()
        print("\nFin goto.")


# Hook de DESCUBRIMIENTO: guarda las estructuras CRUDAS completas de slam_info (por 'type') y
# slam_relocation/odom, para ver si traen covarianza / score / confianza de localizacion.
POSEDUMP_JS = r"""(function(){
  if(!window.__poseDump){ window.__poseDump=1; window.__sirawTypes={}; window.__reloraw=null;
    var jp=JSON.parse;
    JSON.parse=function(s){ var v=jp.apply(this,arguments);
      try{ if(v && v.topic){ var tp=''+v.topic;
        if(tp.indexOf('slam_info')>=0){ var d=(typeof v.data==='string')?jp(v.data):v.data;
          var ty=(d&&d.type)?(''+d.type):'?'; window.__sirawTypes[ty]=d; }
        if(tp.indexOf('slam_relocation/odom')>=0){ window.__reloraw=v.data; }
      }}catch(e){}
      return v;
    };
  } return 1;
})()"""


def cmd_posedump():
    """DESCUBRE si la pose trae COVARIANZA / score / confianza. Vuelca las estructuras crudas de
    slam_info y slam_relocation/odom a dataset/ y resalta cualquier campo de incertidumbre."""
    cdp = g.get_cdp()
    cdp.eval(POSEDUMP_JS)
    print(">>> POSEDUMP. Robot RELOCALIZADO. Recojo ~6s las estructuras crudas de pose...")
    time.sleep(6)
    try:
        si = json.loads(cdp.eval("JSON.stringify(window.__sirawTypes||{})") or "{}")
        rel = json.loads(cdp.eval("JSON.stringify(window.__reloraw||null)") or "null")
    except Exception:
        si = {}; rel = None
    try:
        os.makedirs(DATASET_DIR, exist_ok=True)
    except Exception:
        pass
    fn = os.path.join(DATASET_DIR, time.strftime("%Y%m%d_%H%M%S") + "_posedump.json")
    try:
        json.dump({"slam_info_by_type": si, "slam_relocation_odom": rel}, open(fn, "w"), indent=1)
    except Exception:
        pass
    UNC = ("cov", "score", "conf", "reliab", "status", "quality", "valid", "uncert", "error", "std")
    print("\n--- slam_info ---")
    for ty, d in si.items():
        keys = list(d.keys()) if isinstance(d, dict) else type(d).__name__
        print(f"  type={ty}: {keys}")
        if isinstance(d, dict):
            for k in d:
                if any(w in k.lower() for w in UNC):
                    print(f"     >> {k} = {d[k]}")
    print("--- slam_relocation/odom ---")
    if isinstance(rel, dict):
        print("  claves:", list(rel.keys()))
        po = rel.get("pose")
        if isinstance(po, dict):
            print("  pose.claves:", list(po.keys()))
            if "covariance" in po:
                print("  >> pose.covariance =", po["covariance"])
    else:
        print("  (no llego slam_relocation/odom; quizas solo slam_info en este modo)")
    print(f"\nGuardado {os.path.basename(fn)} en dataset/. Di 'mira el posedump' y te digo qué confianza hay.")


# Captura el MAPA CARGADO: cuando la app carga el .pcd para relocalizar, lo decodifica y renderiza en el
# WebView. Este hook guarda la nube MAS GRANDE que pase por los workers (= el mapa entero, decenas de
# miles de pts, frente a los ~1-3k del laser en vivo 'location'). Asi bajamos el mapa SIN getBigFile.
MAPGRAB_JS = r"""(function(){
  if(!window.__mapHook){ window.__mapHook=1; window.__mapbuf=[]; window.__mapinfo={n:0,type:'',t:0};
    var seen=new WeakSet();
    var o=Worker.prototype.postMessage;
    Worker.prototype.postMessage=function(m){
      try{ if(!seen.has(this)){ seen.add(this);
        this.addEventListener('message',function(ev){ try{
          var d=ev.data; if(!d||typeof d!=='object') return;
          var ty=(d.type!=null?(''+d.type):''); var dd=d.data; var arr=null;
          if(dd&&typeof dd==='object'){ arr=dd.directOutput||dd.points||dd.cloud||dd.positions||dd.data; }
          if(!arr && (d.points||d.positions)) arr=d.points||d.positions;
          if(arr){ var n=arr.length||(arr.byteLength?arr.byteLength/4:0);
            if(n>window.__mapinfo.n){                          // guarda la nube MAS GRANDE = mapa cargado
              var a=(ArrayBuffer.isView(arr))?Array.prototype.slice.call(arr):Object.values(arr);
              window.__mapbuf=a.slice(0,600000);
              window.__mapinfo={n:n,type:ty,t:Date.now()};
            }
          }
        }catch(e){} });
      } }catch(e){}
      return o.apply(this,arguments);
    };
  } return JSON.stringify(window.__mapinfo);
})()"""


def cmd_mapgrab(secs=30):
    """Descarga el MAPA CARGADO desde el WebView (sin getBigFile): captura la nube mas grande que renderiza
    la app al cargar el .pcd. Con el mapa cargado, MUEVE/ROTA la vista del mapa en la app para que se
    redibuje. Guarda dataset/map_loaded.json."""
    cdp = g.get_cdp()
    cdp.eval(MAPGRAB_JS)
    print(">>> MAPGRAB. En la app: mapa CARGADO. Mueve/rota la VISTA del mapa (o re-localiza) para que se")
    print(f"    redibuje. Capturo la nube MAS GRANDE durante {secs}s. Ctrl+C para fijar antes.\n")
    t0 = time.time()
    try:
        while time.time() - t0 < secs:
            info = json.loads(cdp.eval(MAPGRAB_JS) or "{}")
            n = info.get("n", 0)
            print(f"  mapa max: {n // 3 if n else 0} puntos (type='{info.get('type', '')}')   "
                  f"t={time.time()-t0:.0f}/{secs}s", end="\r")
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    buf = json.loads(cdp.eval("JSON.stringify(window.__mapbuf||[])") or "[]")
    info = json.loads(cdp.eval(MAPGRAB_JS) or "{}")
    if len(buf) < 300:
        print(f"\n  Solo {len(buf)//3} puntos. ¿El mapa esta cargado/visible? Prueba a mover la vista del mapa.")
        return
    try:
        os.makedirs(DATASET_DIR, exist_ok=True)
    except Exception:
        pass
    fn = os.path.join(DATASET_DIR, "map_loaded.json")
    try:
        json.dump({"source": "app_loaded_map", "msg_type": info.get("type", ""),
                   "npts": len(buf) // 3, "points": buf}, open(fn, "w"))
        print(f"\n  MAPA CARGADO capturado: {len(buf)//3} puntos (msg type='{info.get('type','')}') -> {fn}")
        print("  Di 'mira el map_loaded' y detecto el frame (Y-up vs Z-up) + lo dibujo con waypoints y paths.")
    except Exception as e:
        print("\n  no se pudo guardar:", repr(e))


def cmd_buildmap(secs=40, force_loc=False):
    """Reconstruye el MAPA 3D del entorno. Dos fuentes segun el modo:
      - OPERACION/relocalizacion: acumula la nube 'location' (frame mapa Z-up, ANCLADO al .pcd, = frame de
        los waypoints, SIN drift). Es la fiable para validar paths. Mueve/gira el robot DESPACIO.
      - MAPEO (#/newSlam): nube densa en window.__buf (Three.js Y-up); rapida pero su frame DERIVA al
        conducir -> no encaja con A/B. Solo como vista rapida.
    'force_loc'=True obliga a usar 'location' (operacion) aunque haya __buf. Guarda dataset/map_full.json."""
    cdp = g.get_cdp()
    _install(cdp)
    time.sleep(1.0)
    nloc = int(cdp.eval("(window.__relocbuf||[]).length") or 0)
    nbuf = int(cdp.eval("(window.__buf||[]).length") or 0)
    acc = {}; src = ""

    if not force_loc and nloc < 100 and nbuf > 3000:
        # --- MODO MAPEO: nube densa en window.__buf (Three.js Y-up), va ACUMULANDO al conducir ---
        src = "mapping __buf (Y-up -> map)"
        print(f">>> BUILDMAP modo mapeo: CONDUCE el robot DESPACIO por las DOS habitaciones.")
        print(f"    window.__buf acumula (empezo en {nbuf//3} pts). Guardo al llegar a {secs}s o con Ctrl+C.")
        t0 = time.time()
        try:
            while time.time() - t0 < secs:
                n = int(cdp.eval("(window.__buf||[]).length") or 0)
                print(f"  __buf acumulado: {n//3} puntos   t={time.time()-t0:.0f}/{secs}s   "
                      f"(Ctrl+C para guardar)", end="\r")
                time.sleep(0.5)
        except KeyboardInterrupt:
            print("\n  Ctrl+C -> fijo el mapa con lo acumulado.")
        buf = json.loads(cdp.eval("JSON.stringify((window.__buf||[]).slice(0,800000))") or "[]")
        for i in range(0, len(buf) - 2, 3):
            X = buf[i]; H = buf[i + 1]; Z = buf[i + 2]          # idx0, idx1=altura, idx2
            k = (round(X / 0.05), round((-Z) / 0.05), round(H / 0.05))   # map: x=idx0, y=-idx2, z=idx1
            acc[k] = acc.get(k, 0) + 1
        minhits = 1
    else:
        # --- MODO OPERACION: acumula 'location' (frame mapa Z-up) ---
        src = "location (Z-up, map frame)"
        print(f">>> BUILDMAP {secs}s (modo operacion): conduce DESPACIO, ATRAVIESA la puerta a la habitacion B.")
        print(f"    Filtros: campo cercano (<{g.NEAR_BLIND}m), SALTOS de relocalizacion (frame descartado) y")
        print("    PERSISTENCIA (un voxel debe verse en varios frames -> mata el rastro de persona/dinamico).")
        if nloc < 100:
            print("  AVISO: no llega nube 'location'. ¿Mapa cargado y RELOCALIZADO (modo operation, como en benchmark)?")
        t0 = time.time(); nf = 0; jumps = 0; fi = 0; prevp = None
        try:
            while time.time() - t0 < secs:
                src2, p, _ = read_pose(cdp)
                px, py = (p[0], p[1]) if p else (None, None)
                if p and prevp is not None and math.hypot(px - prevp[0], py - prevp[1]) > 0.5:
                    jumps += 1; prevp = (px, py)             # salto de pose (imposible andando) = glitch reloc
                    print(f"  [reloc-JUMP #{jumps}] salto de pose -> descarto frame        ", end="\r")
                    time.sleep(0.3); continue                # los puntos irian a un sitio equivocado
                if p:
                    prevp = (px, py)
                buf = grab_full_cloud(cdp, cap=20000)
                fr = set()
                for i in range(0, len(buf) - 2, 3):
                    xx, yy, zz = buf[i], buf[i + 1], buf[i + 2]
                    if abs(zz) > 2.0:
                        continue                            # altura imposible = outlier
                    if px is not None and math.hypot(xx - px, yy - py) < g.NEAR_BLIND:
                        nf += 1; continue                   # anillo fantasma (cuerpo/suelo) junto al robot
                    fr.add((round(xx / 0.05), round(yy / 0.05), round(zz / 0.05)))
                for k in fr:
                    acc[k] = acc.get(k, 0) + 1              # cuenta FRAMES distintos en que aparece el voxel
                fi += 1
                print(f"  voxels={len(acc)} frames={fi}  (cercano {nf}, saltos {jumps})  t={time.time()-t0:.0f}/{secs}s   ", end="\r")
                time.sleep(0.3)
        except KeyboardInterrupt:
            pass
        minhits = 3        # PERSISTENCIA: solo voxels vistos en >=3 frames distintos (estatico). Persona/ruido cae.
        print(f"\n  {jumps} frames descartados por salto de relocalizacion; {fi} frames usados.")

    pts = [[k[0] * 0.05, k[1] * 0.05, k[2] * 0.05] for k, c in acc.items() if c >= minhits]
    try:
        os.makedirs(DATASET_DIR, exist_ok=True)
    except Exception:
        pass
    fn = os.path.join(DATASET_DIR, "map_full.json")
    try:
        json.dump({"frame": "map idx0=x,idx1=y,idx2=altura", "src": src, "voxel": 0.05,
                   "hband_obstac": [HBAND_LO, HBAND_HI], "npts": len(pts), "points": pts}, open(fn, "w"))
        print(f"\n  Mapa 3D: {len(pts)} voxels (fuente: {src}) -> {fn}")
        print("  Di 'mira el map_full' y lo dibujo con A/B + valido los paths. (Si es modo mapeo, verifico que A/B encajan.)")
    except Exception as e:
        print("\n  no se pudo guardar:", repr(e))


def cmd_tablecheck():
    """Captura AHORA la nube 3D completa + foto de la camara + pose, y muestra un histograma de ALTURA
    delante del robot, para ver si hay una MESA u otro obstaculo invisible al LiDAR de banda de torso.
    Coloca el robot MIRANDO al sitio del choque (a ~0.5-1 m)."""
    import collections
    cdp = g.get_cdp()
    _install(cdp)
    print(">>> TABLECHECK. Robot MIRANDO al obstaculo (mesa) a ~0.5-1 m. Capturo nube 3D + foto...")
    for _ in range(40):
        src, p, _ = read_pose(cdp)
        n = int(cdp.eval("(window.__relocbuf||[]).length") or 0)
        if p and n > 100:
            break
        time.sleep(0.3)
    else:
        print(" sin nube/pose. ¿Mapa cargado y relocalizado?"); return
    try:
        os.makedirs(DATASET_DIR, exist_ok=True)
    except Exception:
        pass
    base = os.path.join(DATASET_DIR, time.strftime("%Y%m%d_%H%M%S") + "_tablecheck")
    buf = grab_full_cloud(cdp); jpg = grab_cam(cdp)
    saved = []
    try:
        json.dump({"pose": p, "yaw": round(yaw_of(p), 1), "npts": len(buf) // 3, "points": buf,
                   "frame": "map idx0=x,idx1=y,idx2=altura"}, open(base + ".json", "w"))
        saved.append(os.path.basename(base + ".json"))
    except Exception as e:
        print("  no se pudo guardar la nube:", repr(e))
    if jpg:
        import base64
        try:
            with open(base + ".jpg", "wb") as f:
                f.write(base64.b64decode(jpg.split(",", 1)[1]))
            saved.append(os.path.basename(base + ".jpg"))
        except Exception:
            pass
    x, y = p[0], p[1]; yaw = yaw_of(p)
    h = collections.Counter(); nf = 0
    for i in range(0, len(buf) - 2, 3):
        px, py, pz = buf[i], buf[i + 1], buf[i + 2]
        dd = math.hypot(px - x, py - y)
        if dd < 0.1 or dd > 2.0:
            continue
        ang = abs((math.degrees(math.atan2(py - y, px - x)) - yaw + 180) % 360 - 180)
        if ang > 30:
            continue
        nf += 1; h[round(pz * 2) / 2] += 1
    print(f"\nFRENTE del robot (<2 m, cono ±30°): {nf} puntos. Histograma de ALTURA (idx2):")
    print("  (suelo ~ -1.3/-1.0 | torso/paredes ~ -0.5..+0.5 | techo ~ +1.3)")
    for k in sorted(h):
        print(f"  z~{k:+.1f}: {'#' * max(1, h[k] // 2)} {h[k]}")
    print(f"\nGuardado en dataset/: {', '.join(saved)}")
    print("Di 'mira el tablecheck' y analizo la nube + te enseño la foto.")


def cmd_turntest():
    """DIAGNOSTICO del SIGNO DE GIRO (causa tipica del 'da mil vueltas'). Gira el robot en el sitio a un
    lado y a otro midiendo el yaw REAL de slam_info, y comprueba si coincide con el modelo del DWA
    (wz=-1.8*rx -> rx>0 BAJA el yaw, rx<0 lo SUBE). ESPACIO LIBRE + mando en mano (L2+B)."""
    cdp = get_live_cdp()
    _install(cdp)
    print(">>> TURN-TEST. Espacio libre alrededor; mando en mano. Voy a girar el robot en el sitio.\n")
    for _ in range(20):
        src, p, _ = read_pose(cdp)
        if p:
            break
        time.sleep(0.3)
    else:
        print("Sin pose. ¿Mapa cargado y robot relocalizado?"); return

    def yaw_now():
        s, q, _ = read_pose(cdp)
        return yaw_of(q) if q else None

    def spin(rx, secs, name):
        y0 = yaw_now(); t0 = time.time(); acc = 0.0; prev = y0
        print(f"  girando {name} (rx={rx:+.2f}) {secs}s...")
        while time.time() - t0 < secs:
            cdp.eval(g.set_cmd_js(0, 0, rx, 0))
            time.sleep(0.1)
            yn = yaw_now()
            if yn is not None and prev is not None:
                acc += (yn - prev + 180) % 360 - 180; prev = yn
        cdp.eval(g.STOP_JS); time.sleep(0.5)
        rate = acc / secs
        exp = -103.0 * rx
        ok = (rate * exp > 0)
        print(f"    -> yaw cambio {acc:+.0f}deg ({rate:+.0f}deg/s). modelo esperaba {exp:+.0f}deg/s. "
              f"{'OK' if ok else '>>> INVERTIDO <<<'}")
        return rate, exp, ok

    try:
        r1 = spin(+0.35, 2.5, "DERECHA-modelo(yaw baja)")
        time.sleep(0.5)
        r2 = spin(-0.35, 2.5, "IZQUIERDA-modelo(yaw sube)")
        cdp.eval(g.STOP_JS)
        inverted = (not r1[2]) and (not r2[2])
        print("\n  VEREDICTO:", ">>> SIGNO DE GIRO INVERTIDO: hay que invertir rx en el control <<<"
              if inverted else ("OK: el modelo del DWA coincide con el giro real (el spin viene de otra cosa)"
                                 if r1[2] and r2[2] else "MIXTO/RUIDOSO: repite con mas espacio y robot quieto al inicio"))
        with open(GOTO_LOG, "a") as lg:
            lg.write(f"\n=== TURNTEST {time.strftime('%H:%M:%S')} ===\n")
            lg.write(f"  rx=+0.35 -> {r1[0]:+.0f}deg/s (exp {r1[1]:+.0f}) ok={r1[2]}\n")
            lg.write(f"  rx=-0.35 -> {r2[0]:+.0f}deg/s (exp {r2[1]:+.0f}) ok={r2[2]}\n")
            lg.write(f"  INVERTIDO={inverted}\n")
        print(f"\n  (guardado en {GOTO_LOG})")
    except KeyboardInterrupt:
        cdp.eval(g.STOP_JS); print("\n  cancelado.")
    finally:
        cdp.eval(g.STOP_JS); time.sleep(0.2); cdp.eval(g.STOP_JS)


def _load_refmap_points():
    """Puntos de pared del mapa de referencia (frame G1) para pintar de FONDO. Mapa elegido por G1_REFMAP
    ('g1' por defecto = mapa propio del G1; 'summit' = mapa del Summit alineado)."""
    return ref_points()


def _goto_window(vshare, lock, stop_event, label, wps):
    """Ventana en vivo (hilo principal), DOS paneles:
       izq = mapa cargado (fondo) + plan global + ruta + recorrido + robot;  der = LASER en vivo (robot-centrico)."""
    try:
        import matplotlib
        import matplotlib.pyplot as plt
    except Exception as e:
        print("!! matplotlib no disponible para la ventana:", repr(e))
        while not stop_event.is_set():
            time.sleep(0.3)
        return
    refmap = _load_refmap_points()
    try:
        import base64 as _b64, io as _io
        from PIL import Image as _Image
        _have_cam = True
    except Exception:
        _have_cam = False
    plt.ion()
    fig = plt.figure(figsize=(17, 9.2))
    gs = fig.add_gridspec(3, 2, width_ratios=[1.45, 1], height_ratios=[1, 1, 0.7])
    ax = fig.add_subplot(gs[:, 0])      # mapa+plan (izq, alto completo)
    axl = fig.add_subplot(gs[0, 1])     # laser (arriba dcha)
    axc = fig.add_subplot(gs[1, 1])     # camara (medio dcha)
    axm = fig.add_subplot(gs[2, 1])     # METRICAS clearance+progression (abajo dcha)
    try:
        fig.canvas.manager.set_window_title(f"G1 {label} — mapa+plan | laser | camara | metricas")
    except Exception:
        pass
    closed = {"v": False}
    def _on_close(e):
        closed["v"] = True; stop_event.set()      # cerrar la ventana PARA el robot si aun navega
    fig.canvas.mpl_connect("close_event", _on_close)
    print("Ventana abierta. Al TERMINAR la navegacion la dejo ABIERTA para que la revises; cierrala tu cuando quieras.")
    print("(cerrar la ventana o Ctrl+C durante la navegacion PARA el robot.)")
    try:
        while not closed["v"]:                     # sigue viva aunque la navegacion termine (stop_event)
            with lock:
                x = vshare["x"]; y = vshare["y"]; yaw = vshare["yaw"]; ph = vshare["ph"]
                d = vshare.get("d", 0); col = vshare.get("col", 0); t = vshare.get("t", 0)
                goal = vshare.get("goal"); carrot = vshare.get("carrot")
                obs = list(vshare.get("obs", [])); laser = list(vshare.get("laser", []))
                plan = list(vshare.get("plan", [])); trail = list(vshare.get("trail", []))
                gplan = list(vshare.get("gplan", [])); cam = vshare.get("cam")
                mhist = list(vshare.get("mhist", [])); m_clear = vshare.get("clear", 0); m_prog = vshare.get("prog", 0)
                shist = list(vshare.get("shist", [])); m_rel = vshare.get("rel", 1.0)
            # ===================== PANEL IZQ: mapa cargado + plan =====================
            ax.clear()
            if refmap:                               # FONDO = mapa real (Summit en frame G1) = paredes/puerta
                ax.scatter([p[0] for p in refmap], [p[1] for p in refmap],
                           s=4, c="#9aa6b2", marker="s", linewidths=0, alpha=0.55, label="mapa cargado (paredes)")
            for k, w in wps.items():
                ax.plot([w["x"]], [w["y"]], "s", c="#cfcfcf", ms=6)
                ax.annotate(k, (w["x"], w["y"]), fontsize=9, color="#777")
            if obs:                                  # obstaculos del laser acumulados (TTL)
                ax.scatter([p[0] for p in obs], [p[1] for p in obs],
                           s=14, c="#c0392b", marker="s", linewidths=0, alpha=0.5, label="obstaculos (laser)")
            if trail and len(trail) > 1:
                ax.plot([p[0] for p in trail], [p[1] for p in trail], "-", c="#34495e", lw=1.2, alpha=0.8, label="recorrido")
            if gplan and len(gplan) > 1:             # PLAN GLOBAL (verde)
                ax.plot([p[0] for p in gplan], [p[1] for p in gplan], "-", c="#00d000", lw=3.2, alpha=0.95,
                        label="PLAN GLOBAL (A* origen->destino)")
            if plan and len(plan) > 1:
                ax.plot([p[0] for p in plan], [p[1] for p in plan], "-", c="#1565c0", lw=1.8, label="ruta A* local")
            if carrot:
                ax.plot([carrot[0]], [carrot[1]], "o", c="#00bcd4", ms=8)
            if goal:
                ax.plot([goal[0]], [goal[1]], "*", c="#f39c12", ms=22, label=f"objetivo {label}")
            ax.plot([x], [y], "o", c="#2980b9", ms=11)
            ax.arrow(x, y, 0.4 * math.cos(math.radians(yaw)), 0.4 * math.sin(math.radians(yaw)),
                     head_width=0.16, head_length=0.16, fc="#2980b9", ec="#2980b9", length_includes_head=True)
            ax.set_aspect("equal"); ax.grid(True, alpha=0.2)
            ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
            done_sfx = "   [TERMINADO — cierra la ventana cuando quieras]" if stop_event.is_set() else ""
            ax.set_title(f"{label}  t={t:.0f}s  {ph.strip()}  dist={d:.2f}m  colis={col}{done_sfx}",
                         color=("#1a7d3c" if stop_event.is_set() else "black"))
            # CENTRA la vista en la accion (robot+waypoints+plan+recorrido), ignora el ruido lejano del mapa
            vw = [(x, y)] + ([goal] if goal else []) + [(w["x"], w["y"]) for w in wps.values()] + trail + gplan
            if vw:
                vx = [p[0] for p in vw]; vy = [p[1] for p in vw]
                cx = (min(vx) + max(vx)) / 2; cy = (min(vy) + max(vy)) / 2
                half = max(max(vx) - min(vx), max(vy) - min(vy)) / 2 + 1.5
                ax.set_xlim(cx - half, cx + half); ax.set_ylim(cy - half, cy + half)
            try:
                ax.legend(loc="upper right", fontsize=7)
            except Exception:
                pass
            # ===================== PANEL DER: laser en vivo en FRAME DEL ROBOT (delante = ARRIBA) =====
            axl.clear()
            if laser:                                  # rota los puntos para que el rumbo del robot apunte hacia +y
                a = math.radians(90 - yaw); ca, sa = math.cos(a), math.sin(a)
                lx = [ca * (p[0] - x) - sa * (p[1] - y) for p in laser]
                ly = [sa * (p[0] - x) + ca * (p[1] - y) for p in laser]
                axl.scatter(lx, ly, s=10, c="#16a085", marker="o", linewidths=0)
            for rr in (1, 2):                          # anillos de distancia
                axl.add_artist(plt.Circle((0, 0), rr, fill=False, color="#445", lw=0.6, alpha=0.6))
            axl.plot(0, 0, "o", c="#2980b9", ms=9)
            axl.arrow(0, 0, 0, 0.5, head_width=0.18, fc="#2980b9", ec="#2980b9")   # robot mira ARRIBA
            axl.annotate("delante", (0, 1.0), ha="center", fontsize=8, color="#2980b9")
            axl.set_xlim(-3, 3); axl.set_ylim(-3, 3); axl.set_aspect("equal")
            axl.grid(True, alpha=0.2); axl.set_title(f"LASER en vivo — frame robot  ({len(laser)} pts)")
            axl.set_xlabel("izq/dcha (m)"); axl.set_ylabel("delante (m)")
            # ===================== PANEL CAMARA (abajo dcha) =====================
            axc.clear(); axc.axis("off")
            if _have_cam and cam and isinstance(cam, str) and cam.startswith("data:image"):
                try:
                    imgc = _Image.open(_io.BytesIO(_b64.b64decode(cam.split(",", 1)[1])))
                    axc.imshow(imgc); axc.set_title("camara en vivo", fontsize=10)
                except Exception:
                    axc.set_title("camara (frame invalido)", fontsize=10)
            else:
                axc.set_title("camara (esperando frame...)", fontsize=10)
            # ===================== PANEL METRICAS: clearance + progression (las 2 del tutor) =====================
            axm.clear()
            if mhist:
                tt = [h[0] for h in mhist]; cc = [h[1] for h in mhist]; pp = [h[2] for h in mhist]
                axm.plot(tt, cc, "-", c="#1565c0", lw=1.6, label="clearance")
                axm.plot(tt, pp, "-", c="#e67e22", lw=1.6, label="progression")
                axm.fill_between(tt, cc, alpha=0.08, color="#1565c0")
                if tt[-1] - tt[0] > 40:                # ventana movil de ~40s
                    axm.set_xlim(tt[-1] - 40, tt[-1])
            if shist:                                  # fiabilidad de sensado (auto-evaluacion)
                st_t = [h[0] for h in shist]; st_r = [h[1] for h in shist]
                axm.plot(st_t, st_r, "-", c="#2ca02c", lw=1.6, label="sensing reliab.")
                if st_t[-1] - st_t[0] > 40:
                    axm.set_xlim(st_t[-1] - 40, st_t[-1])
            if mhist or shist:
                axm.legend(loc="upper left", fontsize=7, ncol=3)
            axm.set_ylim(-0.02, 1.05); axm.grid(True, alpha=0.3)
            axm.set_xlabel("t (s)")
            axm.set_title(f"clearance={m_clear:.2f}  progression={m_prog:.2f}  sensing={m_rel:.2f}", fontsize=10)
            plt.pause(0.25)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        plt.ioff()
        try:
            plt.close(fig)
        except Exception:
            pass


def cmd_goto_viz(label):
    """goto a un waypoint CON ventana en vivo (mapa + laser + odometria + ruta A*). El control corre en
    un hilo de fondo y la ventana en el principal. Una sola travesia (no menu)."""
    if not label:
        print("uso: python g1_goto.py gotoviz <N>   (ej: B)"); return
    wps = _load_wps()
    if label not in wps:
        print(f"'{label}' no existe. Waypoints: {list(wps.keys())}"); return
    w = wps[label]
    vshare = {"x": 0.0, "y": 0.0, "yaw": 0.0, "ph": "", "d": 0.0, "col": 0, "t": 0.0,
              "goal": (w["x"], w["y"]), "carrot": None, "obs": [], "laser": [], "plan": [], "gplan": [], "trail": [], "cam": None}
    lk = threading.Lock(); stop_event = threading.Event()

    def control():
        try:
            cdp = get_live_cdp()
            _install(cdp)
            lg = open(GOTO_LOG, "a")
            lg.write(f"\n=== GOTOVIZ {label} {time.strftime('%Y-%m-%d %H:%M:%S')} ===\n")
            navigate_to(cdp, lg, w["x"], w["y"], label, vshare=vshare, lock=lk, stop_event=stop_event)
            lg.write("FIN\n"); lg.close()
        except Exception as e:
            print("Error en control:", repr(e)); stop_event.set()

    th = threading.Thread(target=control, daemon=True)
    th.start()
    try:
        _goto_window(vshare, lk, stop_event, label, wps)
    except KeyboardInterrupt:
        stop_event.set()
    finally:
        stop_event.set()
        th.join(timeout=6)
    print("Ventana cerrada. Fin goto.")


# =========================== BENCHMARK: navegacion NATIVA del firmware ===========================
# El firmware conduce (anyPointNavigation 1102, sniffeado); nosotros SOLO registramos los mismos
# metricas que nuestra nav (tiempo, recorrido, colisiones, laser, odometria) para comparar.
BENCH_LOG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "goto_native.log")

# apaga NUESTRO driver (el setInterval que envia rt/wirelesscontroller) para no pelear con el firmware
DISABLE_DRV_JS = ("(function(){if(window.__drv){clearInterval(window.__drv);window.__drv=null;}"
                  "window.__cmd={lx:0,ly:0,rx:0,ry:0};return 'drv-off';})()")

# captura SOLO el datachannel (sin arrancar driver), para poder enviar el goal nativo
NATIVE_CAP_JS = r"""(function(){
  if(!window.__dcHook){ window.__dcHook=1;
    var S=RTCDataChannel.prototype.send;
    RTCDataChannel.prototype.send=function(d){ try{ if((this.label||'')==='data') window.__dc=this; }catch(e){} return S.apply(this,arguments); };
  }
  return !!window.__dc;
})()"""


def _native_req_js(api_id, parameter_js, topic="rt/api/slam_operate/request"):
    return ("(function(){if(!window.__dc)return 'nodc';var id=Math.floor(Math.random()*1e9);"
            "var par=%s;"
            "var msg={type:'req',topic:'%s',data:{header:{identity:{id:id,api_id:%d}},parameter:par}};"
            "try{window.__dc.send(JSON.stringify(msg));return 'sent';}catch(e){return 'err:'+e;}})()"
            % (parameter_js, topic, api_id))


def native_goal_js(x, y):
    """anyPointNavigation 1102 -> destino (x,y) en frame del mapa (q todo 0 = sin restriccion de rumbo)."""
    par = ("JSON.stringify({data:{targetPose:{x:%g,y:%g,z:0,q_x:0,q_y:0,q_z:0,q_w:0},mode:1}})" % (x, y))
    return _native_req_js(1102, par)


def native_avoid_js(enable):
    """obstacles_avoid 1001 ON/OFF (que el firmware esquive = benchmark justo)."""
    par = "JSON.stringify({data:{enable:%s}})" % ("true" if enable else "false")
    return _native_req_js(1001, par, topic="rt/api/obstacles_avoid/request")


def native_cancel_js():
    """closeNavControlTask 1203 -> para la navegacion del firmware (seguridad al acabar/abortar)."""
    return _native_req_js(1203, "''")


def _path_len(trail):
    return sum(math.hypot(trail[i + 1][0] - trail[i][0], trail[i + 1][1] - trail[i][1])
               for i in range(len(trail) - 1)) if len(trail) > 1 else 0.0


def benchmark_run(cdp, lg, wx, wy, label, vshare=None, lock=None, stop_event=None):
    """BENCHMARK: lanza la navegacion NATIVA del firmware al waypoint y registra (pasivo) las mismas
    metricas que nuestra nav, para comparar. El firmware conduce; nosotros NO enviamos velocidad."""
    print(f"\n>>> BENCHMARK NATIVO -> '{label}' ({wx:+.2f},{wy:+.2f}). El FIRMWARE conduce; yo registro.")
    cdp.eval(DISABLE_DRV_JS)                       # no pelear con el firmware
    cdp.eval(g.LOWSTATE_JS); cdp.eval(RELOC_JS); cdp.eval(RELOC_CLOUD_JS); cdp.eval(NATIVE_CAP_JS)
    cdp.eval(HEALTH_JS); cdp.eval(IMUFULL_JS)
    refmap = load_ref_map()                        # mapa conocido para estimar confianza de localizacion
    print(f"  mapa de referencia: {len(refmap)} celdas" + (" (sin mapa -> confianza N/A)" if not refmap else ""))
    print("  Esperando pose + datachannel...", end="", flush=True)
    for _ in range(40):
        src, p, _ = read_pose(cdp)
        dc = cdp.eval("!!window.__dc")
        if p and dc:
            break
        time.sleep(0.3)
    else:
        print(" sin pose/datachannel. ¿Mapa cargado y robot relocalizado?"); return False
    print(" ok.")
    # --- GATE de relocalizacion (como la app): si la pose inicial esta lejos del waypoint mas cercano,
    #     la relocalizacion es dudosa -> NO navegamos (evita ir a un sitio equivocado). Override G1_NOGATE=1.
    fc = frame_check(lg, p[0], p[1])
    if fc and fc["offset_m"] > GATE_M and os.environ.get("G1_NOGATE") != "1":
        print(f"\n  >>> RELOCALIZACION DUDOSA: arrancas a {fc['offset_m']}m del waypoint {fc['nearest_wp']}.")
        print("      NO envio el goal. Re-localiza en la app (que los puntitos encajen con el mapa) y reintenta.")
        print("      (si de verdad arrancas lejos de un waypoint a proposito: G1_NOGATE=1 ...)")
        lg.write("GATE-BLOCKED reloc dudosa\n")
        return False
    cdp.eval(native_avoid_js(True))               # esquiva del firmware ON
    r = cdp.eval(native_goal_js(wx, wy))          # GOAL nativo (1102)
    print(f"  Goal nativo (1102) enviado: {r}.  Mando en mano (L2+B) por seguridad.")
    lg.write(f"NATIVE-GOAL {label} ({wx:+.3f},{wy:+.3f}) send={r}\n")
    rd = RunRecorder("native", label, (wx, wy))
    rd.rec["frame_check"] = fc

    t0 = time.time(); tprint = 0; trail = []; poshist = []
    low_t = 0; last_low = None; lt_base = []; ah_base = []; ncol = 0; last_col_t = -99
    minc0 = 9.9; stall_t = t0; last_movepos = None; pose_t = time.time()
    health_t = 0; hh = {}; jprev = None; cloud_ok = False; cloud_warned = False
    omap = {}; gplan = []; gplan_t = 0; start_xy = None      # mapa acumulado + plan global (solo viz/comparacion)
    cam_t = 0; cam_jpg = None
    try:
        while not (stop_event is not None and stop_event.is_set()):
            now = time.time()
            src, p, pcd = read_pose(cdp)
            if not p:
                if now - pose_t > 4.0:
                    print("\n  POSE PERDIDA (4s)."); break
                time.sleep(0.15); continue
            pose_t = now
            x, y, yaw = p[0], p[1], yaw_of(p)
            if jprev is not None and math.hypot(x - jprev[0], y - jprev[1]) > 0.5:
                jd = math.hypot(x - jprev[0], y - jprev[1])
                rd.event("reloc_jump", now - t0, x, y, {"dist": round(jd, 2),
                                                        "from": [round(jprev[0], 2), round(jprev[1], 2)]})
                lg.write(f"RELOC-JUMP {jd:.2f}m\n")
            jprev = (x, y)
            if not trail or math.hypot(x - trail[-1][0], y - trail[-1][1]) > 0.05:
                trail.append((x, y))
            poshist.append((now, x, y)); poshist = [h for h in poshist if now - h[0] <= 0.8]
            spd = (math.hypot(x - poshist[0][1], y - poshist[0][2]) / max(1e-3, now - poshist[0][0])
                   if len(poshist) >= 2 else 0.0)
            d_goal = math.hypot(wx - x, wy - y)
            if d_goal < NAV_REACH:                # --- LLEGADA ---
                cdp.eval(native_cancel_js())
                T = now - t0; plen = _path_len(trail)
                straight = math.hypot(wx - trail[0][0], wy - trail[0][1]) if trail else 0.0
                eff = (straight / plen) if plen > 0 else 0.0
                print(f"\n  LLEGADO (NATIVO) a '{label}' en {T:.1f}s | recorrido={plen:.2f}m recto={straight:.2f}m "
                      f"efic={eff:.2f} | colis={ncol} c0min={minc0:.2f}")
                lg.write(f"NATIVE-REACHED {label} t={T:.1f}s path={plen:.2f}m straight={straight:.2f}m "
                         f"eff={eff:.2f} ncol={ncol} c0min={minc0:.2f}\n"); lg.flush()
                rd.save_cloud("end", [round(x, 3), round(y, 3), round(yaw, 1)], grab_full_cloud(cdp))
                rd.save_cam("end", grab_cam(cdp))
                rd.finish("reached", {"time_s": round(T, 2), "path_m": round(plen, 2),
                                      "straight_m": round(straight, 2), "efficiency": round(eff, 2),
                                      "collisions": ncol, "c0min": round(minc0, 2),
                                      "start": {"x": round(trail[0][0], 3), "y": round(trail[0][1], 3)} if trail else None})
                if vshare is not None:
                    with lock:
                        vshare["ph"] = "LLEGADO"
                return True

            live = reloc_cells(cdp, (x, y))       # laser en vivo (mismo metodo que el nuestro -> comparable)
            if live:
                cloud_ok = True
            elif not cloud_ok and not cloud_warned and now - t0 > 4.0:
                print("\n  [AVISO] no llega la nube 'location' (nobs=0) -> dataset sin laser/loc_match.")
                print("          ¿se ven los PUNTITOS del laser en la app? Si no, el robot no la publica.")
                lg.write("NO-CLOUD warning (no 'location' stream)\n"); cloud_warned = True
            op = [(cx * g.OCELL, cy * g.OCELL) for (cx, cy) in live
                  if abs(cx * g.OCELL - x) < 2.6 and abs(cy * g.OCELL - y) < 2.6]
            c0 = clear_dir(x, y, yaw, 0, op); minc0 = min(minc0, c0)
            # mapa acumulado + PLAN GLOBAL (nuestro A* origen->destino) solo para ver/comparar en la ventana
            if start_xy is None:
                start_xy = (x, y)
            for c in live:
                omap[c] = now
            omap = {c: tt for c, tt in omap.items() if now - tt < NAV_OMAP_TTL}
            if vshare is not None and now - gplan_t > 3.0:
                obs_plan = set(omap.keys()) or refmap          # mapa vivo, o el de referencia, o (vacio)=recta
                gplan = global_plan(start_xy[0], start_xy[1], wx, wy, obs_plan)
                gplan_t = now

            if now - low_t > 0.2:                  # contacto por IMU/par (mismo detector)
                lw = g.read_low(cdp)
                if lw:
                    last_low = (math.hypot(lw.get("ax", 0.0), lw.get("ay", 0.0)), lw.get("legtau", 0.0))
                low_t = now
            cur_ah, cur_lt = last_low if last_low else (None, None)
            if spd > 0.06 and cur_lt is not None:
                lt_base.append(cur_lt); lt_base = lt_base[-40:]
                ah_base.append(cur_ah); ah_base = ah_base[-40:]
            if now - last_col_t > 4.0 and cur_lt is not None and len(lt_base) >= 5:
                bl = sorted(lt_base)[len(lt_base) // 2]; ba = sorted(ah_base)[len(ah_base) // 2]
                if cur_lt > bl * 1.5 + 3.0 or cur_ah > ba + 1.8:
                    ncol += 1; last_col_t = now
                    print(f"\n  CONTACTO #{ncol} [imu] (nativo) en ({x:+.2f},{y:+.2f}).")
                    lg.write(f"NATIVE-CONTACT #{ncol} pos=({x:+.2f},{y:+.2f}) legtau={cur_lt:.1f}\n")
                    rd.event("collision", now - t0, x, y, {"src": "imu", "legtau": round(cur_lt, 1)})
                    rd.save_cloud(f"col{ncol}", [round(x, 3), round(y, 3), round(yaw, 1)], grab_full_cloud(cdp))
                    rd.save_cam(f"col{ncol}", grab_cam(cdp))
            if last_movepos is None or math.hypot(x - last_movepos[0], y - last_movepos[1]) > 0.1:
                last_movepos = (x, y); stall_t = now
            stalled = now - stall_t > 3.0

            if now - health_t > 1.0:
                hh = read_telemetry(cdp); health_t = now
                rd.telem(now - t0, _telem_row(hh))
            loc = match_score(live, refmap)
            h = hh.get("h") or {}
            line = (f"t={now-t0:5.1f} NATIVO pos=({x:+.2f},{y:+.2f}) yaw={yaw:+6.1f} d={d_goal:.2f} "
                    f"spd={spd:.2f} c0={c0:.2f} loc={loc if loc is not None else '-'} bat={h.get('bat','-')} "
                    f"nobs={len(op)} col={ncol}{' STALL' if stalled else ''}")
            lg.write(line + "\n"); lg.flush()
            rd.sample(now - t0, x, y, yaw, d_goal, spd, c0, len(op), phase="NATIVO",
                      extra={"err": hh.get("err"), "bat": h.get("bat"), "cpuT": h.get("cpuT"),
                             "merr": h.get("merr"), "loc_match": loc})
            rd.maybe_laser(now - t0, op)
            if now - tprint > 0.4:
                print("  " + line); tprint = now
            if vshare is not None:
                if now - cam_t > 0.5:
                    cam_jpg = grab_cam(cdp); cam_t = now
                with lock:
                    vshare["x"] = x; vshare["y"] = y; vshare["yaw"] = yaw; vshare["ph"] = "NATIVO"
                    vshare["d"] = d_goal; vshare["col"] = ncol; vshare["t"] = now - t0
                    vshare["goal"] = (wx, wy); vshare["carrot"] = None; vshare["plan"] = []
                    vshare["cam"] = cam_jpg
                    vshare["gplan"] = list(gplan)                                     # plan global (nuestro A*, referencia)
                    vshare["obs"] = [(cx * g.OCELL, cy * g.OCELL) for (cx, cy) in omap]   # mapa acumulado (m)
                    vshare["laser"] = [(cx * g.OCELL, cy * g.OCELL) for (cx, cy) in live]
                    vshare["trail"] = list(trail)
            time.sleep(0.1)
        if "x" in dir():
            rd.save_cloud("end", [round(x, 3), round(y, 3), round(yaw, 1)], grab_full_cloud(cdp))
        rd.finish("aborted", {"time_s": round(time.time() - t0, 2), "path_m": round(_path_len(trail), 2),
                              "collisions": ncol, "c0min": round(minc0, 2)})
        return False
    except KeyboardInterrupt:
        print(f"\n  [ABORTADO benchmark '{label}']"); lg.write(f"NATIVE-ABORT {label}\n")
        if "x" in dir():
            rd.save_cloud("end", [round(x, 3), round(y, 3), round(yaw, 1)], grab_full_cloud(cdp))
        rd.finish("aborted", {"time_s": round(time.time() - t0, 2), "path_m": round(_path_len(trail), 2),
                              "collisions": ncol, "c0min": round(minc0, 2)})
        return False
    finally:
        cdp.eval(native_cancel_js()); time.sleep(0.2); cdp.eval(native_cancel_js())
        if stop_event is not None:
            stop_event.set()


def cmd_benchmark(label, viz=False):
    """Lanza la navegacion NATIVA del firmware a un waypoint y registra metricas en goto_native.log
    (benchmark para comparar con nuestra nav). 'viz' abre la ventana en vivo."""
    if not label:
        print("uso: python g1_goto.py benchmark <N> [viz]"); return
    wps = _load_wps()
    if label not in wps:
        print(f"'{label}' no existe. Waypoints: {list(wps.keys())}"); return
    w = wps[label]
    if not viz:
        cdp = get_live_cdp()
        lg = open(BENCH_LOG, "a")
        lg.write(f"\n=== BENCHMARK NATIVO {label} {time.strftime('%Y-%m-%d %H:%M:%S')} ===\n")
        try:
            benchmark_run(cdp, lg, w["x"], w["y"], label)
        finally:
            lg.write("FIN\n"); lg.close()
            print(f"\nFin benchmark. Log -> {BENCH_LOG}")
        return
    vshare = {"x": 0.0, "y": 0.0, "yaw": 0.0, "ph": "", "d": 0.0, "col": 0, "t": 0.0,
              "goal": (w["x"], w["y"]), "carrot": None, "obs": [], "laser": [], "plan": [], "gplan": [], "trail": [], "cam": None}
    lk = threading.Lock(); stop_event = threading.Event()

    def control():
        try:
            cdp = get_live_cdp()
            lg = open(BENCH_LOG, "a")
            lg.write(f"\n=== BENCHMARK NATIVO {label} {time.strftime('%Y-%m-%d %H:%M:%S')} ===\n")
            benchmark_run(cdp, lg, w["x"], w["y"], label, vshare=vshare, lock=lk, stop_event=stop_event)
            lg.write("FIN\n"); lg.close()
        except Exception as e:
            print("Error en benchmark:", repr(e)); stop_event.set()

    th = threading.Thread(target=control, daemon=True); th.start()
    try:
        _goto_window(vshare, lk, stop_event, label + " (NATIVO)", wps)
    except KeyboardInterrupt:
        stop_event.set()
    finally:
        stop_event.set(); th.join(timeout=6)
    print(f"Ventana cerrada. Fin benchmark. Log -> {BENCH_LOG}")


def _load_wps():
    try:
        return json.load(open(WP_FILE))
    except Exception:
        return {}


def _save_map(cdp, pose=None):
    """Acumula el mapa de obstaculos a nav_map.json desde la nube 'location' (frame del MAPA, celdas OCELL),
    filtrando el campo cercano (anillo fantasma) con la pose. El mapa es siempre el mismo: se va completando."""
    try:
        prev = set(tuple(c) for c in json.load(open(MAP_FILE)).get("cells", []))
    except Exception:
        prev = set()
    prev |= reloc_cells(cdp, pose)
    try:
        json.dump({"cells": [list(c) for c in prev], "OCELL": g.OCELL,
                   "frame": "map", "hband": [HBAND_LO, HBAND_HI]}, open(MAP_FILE, "w"))
    except Exception:
        pass
    return len(prev)


def cmd_waypoint(label):
    """PASO 2: conduce el robot al destino (con la app o teleop) y Ctrl+C -> guarda la ULTIMA pose como 'label'.
    Mientras, acumula el mapa de obstaculos en nav_map.json."""
    if not label:
        print("uso: python g1_goto.py waypoint <NOMBRE>   (ej: A, B, cocina...)"); return
    cdp = g.get_cdp()
    _install(cdp)
    print(f">>> WAYPOINT '{label}'. Lleva el robot al destino (app/teleop). Cuando este EN el punto, Ctrl+C.")
    print("    (voy mostrando la pose y acumulando el mapa). \n")
    last = None
    try:
        while True:
            src, p, pcd = read_pose(cdp)
            if p:
                last = (src, p, pcd)
                ncells = _save_map(cdp, p)
                print(f"  [{src}] x={p[0]:+.2f} y={p[1]:+.2f} yaw={yaw_of(p):+6.1f}  mapa={ncells} celdas", end="\r")
            time.sleep(0.3)
    except KeyboardInterrupt:
        if not last:
            print("\n!! No capture pose. ¿Mapa cargado y relocalizado?"); return
        src, p, pcd = last
        wps = _load_wps()
        wps[label] = {"x": round(p[0], 3), "y": round(p[1], 3), "yaw": round(yaw_of(p), 1),
                      "src": src, "pcd": pcd, "t": time.strftime("%Y-%m-%d %H:%M:%S")}
        json.dump(wps, open(WP_FILE, "w"), indent=2)
        print(f"\n\n  WAYPOINT '{label}' guardado: x={p[0]:+.3f} y={p[1]:+.3f} (fuente {src}, mapa '{pcd}')")
        print(f"  Total waypoints: {list(wps.keys())}  -> {WP_FILE}")


def cmd_listwp():
    wps = _load_wps()
    if not wps:
        print("Sin waypoints. Captura con: python g1_goto.py waypoint A"); return
    print("Waypoints guardados:")
    for k, v in wps.items():
        print(f"  {k}: x={v['x']:+.2f} y={v['y']:+.2f} yaw={v.get('yaw', 0):+.0f}  ({v.get('src')}, {v.get('pcd', '')})")


def cmd_noisecheck(secs=20):
    """Mide el RUIDO REAL de los sensores con el robot QUIETO (sin conducir): jitter del laser, deriva de
    pose, confianza de localizacion + bateria/temperatura/salud de motor por-junta. Es el 'feedback del
    robot sobre su propia capacidad' (Renxi). Guarda dataset/<ts>_noise.json + .png."""
    cdp = get_live_cdp()
    if not cdp:
        print("No hay pagina viva del WebView. ¿proxy + app + relocalizado?"); return
    cdp.eval(g.LOWSTATE_JS); cdp.eval(HEALTH_JS)
    print("Esperando pose + nube...", end="", flush=True)
    for _ in range(30):
        src, p, _ = read_pose(cdp); n = int(cdp.eval("(window.__relocbuf||[]).length") or 0)
        if p and n > 30:
            break
        time.sleep(0.3)
    else:
        print(" sin datos. ¿Mapa cargado y robot RELOCALIZADO?"); return
    print(" ok.")
    print(f"\n>>> MANTÉN EL ROBOT QUIETO {secs}s (de pie, sin conducir). Midiendo ruido de sensores...")
    refmap = load_ref_map(); t0 = time.time(); rows = []
    sens = g1_metrics.SensingMonitor()
    while time.time() - t0 < secs:
        src, p, _ = read_pose(cdp)
        if not p:
            time.sleep(0.1); continue
        x, y, yaw = p[0], p[1], yaw_of(p)
        live = reloc_cells(cdp)
        op = [(cx * g.OCELL, cy * g.OCELL) for (cx, cy) in set(live)]
        c0 = clear_dir(x, y, yaw, 0, op)
        loc = match_score(live, refmap) if refmap else None
        sv = sens.update(time.time() - t0, live, c0, loc, False)
        rows.append({"t": round(time.time() - t0, 2), "x": round(x, 4), "y": round(y, 4), "yaw": round(yaw, 2),
                     "n": len(live), "c0": round(c0, 3), "loc": loc,
                     "reliability": sv["reliability"], "laser_noise": sv["laser_noise"],
                     "c0_std": sv["c0_std"], "scan_churn": sv["scan_churn"]})
        time.sleep(0.1)
    hh = read_telemetry(cdp); H = hh.get("h") or {}
    import statistics as _st
    xs = [r["x"] for r in rows]; ys = [r["y"] for r in rows]; ns = [r["n"] for r in rows]
    c0s = [r["c0"] for r in rows]; locs = [r["loc"] for r in rows if r["loc"] is not None]
    drift = (_st.pstdev(xs) ** 2 + _st.pstdev(ys) ** 2) ** 0.5 if len(xs) > 1 else 0.0
    summary = {"secs": secs, "ticks": len(rows), "pose_drift_cm": round(drift * 100, 2),
               "count_mean": round(_st.mean(ns), 1) if ns else 0,
               "count_std": round(_st.pstdev(ns), 1) if len(ns) > 1 else 0,
               "c0_mean": round(_st.mean(c0s), 3) if c0s else 0,
               "c0_std_m": round(_st.pstdev(c0s), 3) if len(c0s) > 1 else 0,
               "loc_mean": round(_st.mean(locs), 3) if locs else None,
               "loc_std": round(_st.pstdev(locs), 3) if len(locs) > 1 else None,
               "battery_pct": H.get("bat"), "batT": H.get("batT"), "cpuT": H.get("cpuT"),
               "motTmax": H.get("motTmax"), "motThot_idx": H.get("motThot"), "merr": H.get("merr")}
    os.makedirs(DATASET_DIR, exist_ok=True)
    base = os.path.join(DATASET_DIR, time.strftime("%Y%m%d_%H%M%S") + "_noise")
    json.dump({"schema": "g1_noise/v1", "summary": summary, "rows": rows,
               "motorTemp": H.get("motorTemp"), "motorError": H.get("motorError")}, open(base + ".json", "w"))
    print("\n=== RUIDO DE SENSORES (robot quieto) = capacidad de sensado del robot ===")
    for k, v in summary.items():
        print(f"  {k}: {v}")
    print(f"  -> {base}.json")
    try:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        t = [r["t"] for r in rows]
        loc_al = [(r["loc"] if r["loc"] is not None else float("nan")) for r in rows]
        rel_al = [r["reliability"] for r in rows]
        fig, ax = plt.subplots(2, 2, figsize=(12, 8))
        ax[0, 0].plot(t, ns, c="#16a085"); ax[0, 0].set_title(f"laser point count (std={summary['count_std']})")
        ax[0, 0].set_xlabel("s")
        ax[0, 1].plot(t, c0s, c="#1565c0"); ax[0, 1].set_title(f"forward clearance c0 (noise std={summary['c0_std_m']} m)")
        ax[0, 1].set_xlabel("s")
        sc = ax[1, 0].scatter(xs, ys, s=10, c=t, cmap="viridis")
        ax[1, 0].set_title(f"pose drift while still (std={summary['pose_drift_cm']} cm)")
        ax[1, 0].set_aspect("equal"); ax[1, 0].set_xlabel("x (m)"); ax[1, 0].set_ylabel("y (m)")
        ax[1, 1].plot(t, loc_al, c="#8e44ad", label="loc_match")
        ax[1, 1].plot(t, rel_al, c="#2ca02c", label="reliability")
        ax[1, 1].set_ylim(0, 1.05); ax[1, 1].set_title("localisation conf. / sensing reliability")
        ax[1, 1].set_xlabel("s"); ax[1, 1].legend(fontsize=8)
        fig.suptitle(f"G1 sensor self-capacity (still {secs}s)   battery={summary['battery_pct']}%   "
                     f"motTmax={summary['motTmax']}C   drift={summary['pose_drift_cm']}cm")
        fig.tight_layout(); fig.savefig(base + ".png", dpi=100)
        print(f"  -> {base}.png")
    except Exception as e:
        print("  (grafica omitida:", e, ")")


def main():
    if len(sys.argv) < 2:
        print(__doc__); return
    c = sys.argv[1]
    if c == "reloccheck":
        cmd_reloccheck()
    elif c == "clouddebug":
        cmd_clouddebug()
    elif c == "cloudgrab":
        cmd_cloudgrab()
    elif c == "pathsniff":
        cmd_pathsniff(sys.argv[2] if len(sys.argv) > 2 else None)
    elif c == "appsniff":
        cmd_appsniff(int(sys.argv[2]) if len(sys.argv) > 2 and sys.argv[2].isdigit() else 40)
    elif c == "waypoint":
        cmd_waypoint(sys.argv[2] if len(sys.argv) > 2 else None)
    elif c == "listwp":
        cmd_listwp()
    elif c == "noisecheck":
        cmd_noisecheck(int(sys.argv[2]) if len(sys.argv) > 2 and sys.argv[2].isdigit() else 20)
    elif c == "turntest":
        cmd_turntest()
    elif c == "tablecheck":
        cmd_tablecheck()
    elif c == "posedump":
        cmd_posedump()
    elif c == "buildmap":
        secs = next((int(a) for a in sys.argv[2:] if a.isdigit()), 40)
        cmd_buildmap(secs, force_loc=("loc" in sys.argv[2:] or "op" in sys.argv[2:]))
    elif c == "mapgrab":
        cmd_mapgrab(int(sys.argv[2]) if len(sys.argv) > 2 else 30)
    elif c in ("benchmark", "native"):
        label = sys.argv[2] if len(sys.argv) > 2 else None
        cmd_benchmark(label, viz=("viz" in sys.argv[2:]))
    elif c == "gotoviz":
        cmd_goto_viz(sys.argv[2] if len(sys.argv) > 2 else None)
    elif c == "goto":
        label = sys.argv[2] if len(sys.argv) > 2 else None
        if label and "viz" in sys.argv[2:]:          # 'goto B viz' -> con ventana
            cmd_goto_viz(label)
        else:
            cmd_goto(label)
    else:
        print("comandos: reloccheck | clouddebug | cloudgrab | waypoint <N> | listwp | goto [N] | gotoviz <N>")
        print(__doc__)


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print("Error:", repr(e)); sys.exit(1)
