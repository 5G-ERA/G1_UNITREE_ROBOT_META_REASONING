#!/usr/bin/env python3
"""
g1_inject_teleop.py  -  OPCION C: mover el G1 inyectando teleop por el DATACHANNEL de la app

En vez de abrir una 2a sesion WebRTC (imposible: el robot solo admite una, y la tiene la app),
nos metemos DENTRO de la WebView de la app por el inspector USB (ios-webkit-debug-proxy),
enganchamos su RTCDataChannel y enviamos por EL MISMO canal mensajes rt/wirelesscontroller.
Asi podemos tener a la vez el laser/SLAM (lo pinta la app) y el control (lo inyectamos nosotros).

COMO FUNCIONA
  - Engancha RTCDataChannel.prototype.send -> captura el canal vivo (window.__dc) y una
    plantilla real de un mensaje 'wirelesscontroller' (window.__wc) cuando la app envia uno.
  - Instala un DRIVER EN-PAGINA a 20Hz: lee window.__cmd {lx,ly,rx,ry} y lo envia por __dc.
    Lleva HOMBRE-MUERTO: si el comando no se refresca en 600ms, manda ceros (para de moverse).
  - Desde Python solo refrescamos window.__cmd; el envio a 20Hz ocurre dentro de la pagina
    (no depende de la latencia del inspector).

PRE
  1) ios_webkit_debug_proxy corriendo en otra terminal (expone localhost:9221/9222).
  2) iPhone con la app Unitree conectada al robot. Inspector Web de Safari CERRADO para esa pagina.
  3) pip install websocket-client requests
  4) >>> SEGURIDAD <<< robot DE PIE en modo marcha (como con el mando), ESPACIO LIBRE o grua,
     MANDO EN LA MANO como kill-switch, valores BAJOS y rafagas CORTAS. Empieza por 'sniff'.

USO
  # 0) comprobar que enganchamos el canal y capturar la plantilla (NO mueve el robot):
  python g1_inject_teleop.py sniff
  #    si dice tpl:false, mueve un PELIN el joystick en pantalla de la app para que envie uno.

  # 1) micro-movimientos controlados (val 0..1, segundos). Empieza muy bajo:
  python g1_inject_teleop.py forward 0.12 0.8
  python g1_inject_teleop.py back    0.12 0.8
  python g1_inject_teleop.py turnL   0.15 0.8
  python g1_inject_teleop.py turnR   0.15 0.8
  python g1_inject_teleop.py strafeL 0.12 0.8
  python g1_inject_teleop.py strafeR 0.12 0.8

  # 2) conduccion interactiva (teclas): W/S adelante-atras, A/D giro, Q/E lateral, ESPACIO stop, X salir
  python g1_inject_teleop.py drive
"""
import json, sys, time, math
import requests
import websocket  # websocket-client

PROXY = "http://localhost:9221"

# -------- JS que se inyecta en la pagina de la app --------
INSTALL_JS = r"""
(function(){
  // 1) hook del datachannel: captura el canal vivo y una plantilla real de wirelesscontroller
  if(!window.__dcHook){
    window.__dcHook = 1; window.__wcCount = 0;
    var S = RTCDataChannel.prototype.send;
    RTCDataChannel.prototype.send = function(d){
      try{
        window.__dc = this;
        if(typeof d === 'string' && d.indexOf('wirelesscontroller') >= 0){
          window.__wc = d; window.__wcCount++;
        }
      }catch(e){}
      return S.apply(this, arguments);
    };
  }
  // 2) driver en-pagina a 20Hz con hombre-muerto
  if(!window.__drv){
    window.__cmd = {lx:0,ly:0,rx:0,ry:0}; window.__cmdTs = 0; window.__sent = 0;
    window.__mkMsg = function(c){
      var msg = null;
      if(window.__wc){ try{ msg = JSON.parse(window.__wc); }catch(e){ msg = null; } }
      if(!msg){ msg = {type:'msg', topic:'rt/wirelesscontroller', data:{}}; }
      var dStr = false;
      if(typeof msg.data === 'string'){ try{ msg.data = JSON.parse(msg.data); dStr = true; }catch(e){} }
      var ok = (function set(o){
        if(o && typeof o === 'object'){
          if(('ly' in o) || ('lx' in o)){ o.lx=c.lx; o.ly=c.ly; o.rx=c.rx; o.ry=c.ry; return true; }
          for(var k in o){ if(set(o[k])) return true; }
        }
        return false;
      })(msg);
      if(!ok){ msg.data = {lx:c.lx, ly:c.ly, rx:c.rx, ry:c.ry}; }
      if(dStr){ msg.data = JSON.stringify(msg.data); }
      return JSON.stringify(msg);
    };
    window.__drv = setInterval(function(){
      if(!window.__dc) return;
      var c = window.__cmd || {lx:0,ly:0,rx:0,ry:0};
      if(Date.now() - (window.__cmdTs || 0) > 600){ c = {lx:0,ly:0,rx:0,ry:0}; }  // hombre-muerto
      try{ window.__dc.send(window.__mkMsg(c)); window.__sent++; }catch(e){}
    }, 50);
  }
  return JSON.stringify({dc: !!window.__dc, tpl: !!window.__wc, wc: window.__wcCount||0});
})();
"""

STATUS_JS = "JSON.stringify({dc:!!window.__dc, tpl:!!window.__wc, wc:window.__wcCount||0, sent:window.__sent||0, sample:(window.__wc||null)})"

# -------- modo CAPTURE: ver el mensaje REAL de la app (sin enviar nada nosotros) --------
CAPTURE_JS = r"""
(function(){
  // mata el driver inyectado previo (mandaba ceros y ensuciaria la captura)
  if(window.__drv){ try{ clearInterval(window.__drv); }catch(e){} window.__drv = null; }
  window.__wc = null;
  if(!window.__capHook){
    window.__capHook = 1;
    window.__chans = {};   // label -> {count, last, rs}
    window.__moves = [];   // mensajes que parecen de movimiento
    var S = RTCDataChannel.prototype.send;
    RTCDataChannel.prototype.send = function(d){
      try{
        var lbl = this.label || '(nolabel)';
        var r = window.__chans[lbl] || (window.__chans[lbl] = {count:0, last:null, rs:''});
        r.count++; r.rs = this.readyState;
        if(typeof d === 'string'){
          r.last = d.slice(0,300);
          if(d.indexOf('wirelesscontroller')>=0 || d.indexOf('"lx"')>=0 || d.indexOf('"ly"')>=0 || d.indexOf('Move')>=0){
            window.__moves.push(lbl + ' | ' + d.slice(0,300));
            if(window.__moves.length > 30) window.__moves.shift();
          }
        } else { r.last = '[bin ' + ((d && d.byteLength) || '?') + ']'; }
      }catch(e){}
      return S.apply(this, arguments);
    };
  }
  return 'cap-installed';
})();
"""

CAPSTAT_JS = "(function(){var m=window.__moves||[];window.__moves=[];return JSON.stringify({chans:window.__chans||{},moves:m});})()"


def set_cmd_js(lx, ly, rx, ry):
    return ("(function(){window.__cmd={lx:%g,ly:%g,rx:%g,ry:%g};window.__cmdTs=Date.now();return 'ok';})()"
            % (lx, ly, rx, ry))


STOP_JS = "(function(){window.__cmd={lx:0,ly:0,rx:0,ry:0};window.__cmdTs=Date.now();return 'stop';})()"


# -------- descubrimiento + cliente CDP (igual que g1_inspector_bridge) --------
def discover_ws():
    devs = requests.get(PROXY + "/json", timeout=5).json()
    if not devs:
        raise RuntimeError("No hay dispositivos. ¿iPhone conectado y ios_webkit_debug_proxy corriendo?")
    durl = devs[0]["url"]
    pages = requests.get(f"http://{durl}/json", timeout=5).json()
    if not pages:
        raise RuntimeError("No hay paginas inspeccionables. ¿App abierta y conectada? ¿Inspector de Safari CERRADO?")
    # prefiere la app (B2App / puerto 8084 / slam); si no, la primera
    def score(p):
        u = (p.get("url", "") + " " + p.get("title", "")).lower()
        return ("b2app" in u) * 4 + ("8084" in u) * 3 + ("slam" in u) * 2 + 1
    page = sorted(pages, key=score, reverse=True)[0]
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

    def connect_setup(self):
        try:
            self.call("Runtime.enable")
        except Exception:
            pass
        return self.eval(INSTALL_JS)


# -------- acciones --------
AXES = {
    "forward": (0, 1, 0, 0), "back": (0, -1, 0, 0),
    "strafeL": (-1, 0, 0, 0), "strafeR": (1, 0, 0, 0),
    "turnL": (0, 0, -1, 0), "turnR": (0, 0, 1, 0),
}  # signo de ejes por verificar con pruebas reales


def get_cdp(setup=True):
    url = discover_ws()
    print("Inspector WS:", url)
    cdp = CDP(url)
    if setup:
        print("Instalando hook+driver:", cdp.connect_setup())
    else:
        try:
            cdp.call("Runtime.enable")
        except Exception:
            pass
    return cdp


def cmd_capture():
    cdp = get_cdp(setup=False)
    print("Instalando captura (sin enviar nada):", cdp.eval(CAPTURE_JS))
    print("\n>>> AHORA mueve el robot con el JOYSTICK DE LA APP (poco, espacio libre, mando a mano).")
    print("    Observa qué canal/mensaje aparece en 'MOV'. Ctrl+C para terminar.\n")
    try:
        while True:
            st = json.loads(cdp.eval(CAPSTAT_JS))
            chans = st.get("chans", {})
            if chans:
                desc = "  ".join(f"{k}(n={v['count']},rs={v['rs']})" for k, v in chans.items())
                print("Canales:", desc)
            for mv in st.get("moves", []):
                print("  MOV>>", mv)
            time.sleep(0.6)
    except KeyboardInterrupt:
        print("\nFin captura.")


def cmd_sniff():
    cdp = get_cdp()
    print("\nSniff 8s. Si tpl:false, mueve UN PELIN el joystick de la app para capturar una plantilla.")
    for _ in range(16):
        st = json.loads(cdp.eval(STATUS_JS))
        print(f"  dc:{st['dc']} tpl:{st['tpl']} wc:{st['wc']} sent:{st['sent']}")
        if st.get("sample"):
            print("  PLANTILLA wirelesscontroller capturada:\n   ", st["sample"][:300])
            break
        time.sleep(0.5)
    print("\nListo. Si dc:true ya podemos inyectar. Si tpl:true, mejor aun (usa el envoltorio exacto).")


def burst(lx, ly, rx, ry, secs):
    cdp = get_cdp()
    st = json.loads(cdp.eval(STATUS_JS))
    print(f"Estado: dc:{st['dc']} tpl:{st['tpl']}")
    if not st["dc"]:
        print("!! No hay datachannel capturado todavia. Mueve un pelin el joystick de la app y reintenta.")
        return
    if not st["tpl"]:
        print("(aviso) sin plantilla real; usare envoltorio estandar {type:'msg',topic:'rt/wirelesscontroller',data:{...}}.")
    print(f">>> MOVIENDO {secs:.1f}s  lx={lx:+.2f} ly={ly:+.2f} rx={rx:+.2f} ry={ry:+.2f}  (Ctrl+C = stop)")
    t0 = time.time()
    try:
        while time.time() - t0 < secs:
            cdp.eval(set_cmd_js(lx, ly, rx, ry))   # refresca el target ~10Hz (hombre-muerto 600ms)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print(" [interrumpido]")
    finally:
        cdp.eval(STOP_JS)
        time.sleep(0.3)
        cdp.eval(STOP_JS)
        print("STOP enviado (ceros).")


def cmd_drive():
    import termios, tty, select
    cdp = get_cdp()
    print("\nDRIVE: W/S adelante-atras, A/D giro, Q/E lateral, ESPACIO=stop, X=salir.")
    print("Valor fijo bajo (0.15). Manten el MANDO listo como kill-switch.\n")
    V = 0.15
    keymap = {
        "w": (0, V, 0, 0), "s": (0, -V, 0, 0),
        "a": (0, 0, -V, 0), "d": (0, 0, V, 0),
        "q": (-V, 0, 0, 0), "e": (V, 0, 0, 0),
        " ": (0, 0, 0, 0),
    }
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        cur = (0, 0, 0, 0)
        last = 0
        while True:
            r, _, _ = select.select([sys.stdin], [], [], 0.1)
            if r:
                ch = sys.stdin.read(1).lower()
                if ch == "x":
                    break
                if ch in keymap:
                    cur = keymap[ch]
                    print(f"  -> lx={cur[0]:+.2f} ly={cur[1]:+.2f} rx={cur[2]:+.2f} ry={cur[3]:+.2f}")
            # refresca el comando (hombre-muerto en pagina a 600ms)
            if time.time() - last > 0.1:
                cdp.eval(set_cmd_js(*cur))
                last = time.time()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        cdp.eval(STOP_JS); time.sleep(0.3); cdp.eval(STOP_JS)
        print("\nSTOP enviado. Fin.")


def main():
    if len(sys.argv) < 2:
        print(__doc__); return
    cmd = sys.argv[1]
    if cmd == "sniff":
        cmd_sniff()
    elif cmd == "capture":
        cmd_capture()
    elif cmd == "drive":
        cmd_drive()
    elif cmd in AXES:
        val = float(sys.argv[2]) if len(sys.argv) > 2 else 0.4
        secs = float(sys.argv[3]) if len(sys.argv) > 3 else 0.8
        val = max(0.0, min(0.6, val))      # tope de seguridad (app usa ~0.5-0.73; hay deadzone ~0.3)
        secs = max(0.1, min(3.0, secs))
        sx, sy, sr, se = AXES[cmd]
        burst(sx * val, sy * val, sr * val, se * val, secs)
    else:
        print("comando desconocido:", cmd); print(__doc__)


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print("Error:", repr(e))
        sys.exit(1)
