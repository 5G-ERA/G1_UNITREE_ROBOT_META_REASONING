#!/usr/bin/env python3
"""
g1_floorcolor.py — deteccion de obstaculos por COLOR DE MOQUETA (rama feature/floor-color-vision).

Idea (Adrian, 2026-07-02): la moqueta del lab es siempre del mismo azul-gris. Todo lo que NO es
moqueta en la mitad inferior de la imagen es potencialmente obstaculo. Complementa (NO sustituye)
a depth+YOLO del perception_server:
  - ve la MESA LiDAR-ciega y los CABLES (sin clase YOLO, demasiado finos para el depth) a coste CPU;
  - free_center por color = tercera opinion para la compuerta DOOR-GO (laser | vision | color).

Validado offline con crashes/ (2026-07-02):
  - moqueta pura (crash_01_151142)  -> 97.4% moqueta, free_center=1.00
  - armario+moqueta (crash_01_090100)-> perfil detecta el mueble (cols a 0.0), moqueta 62%
  - contra la mesa (crash_02_090115) -> 1.2% moqueta, free_center=0.00  (el "NO VAYAS" que falto)

Diseno:
  - Modelo HSV robusto (mediana+MAD) calibrado con fotos de moqueta pura. La moqueta es de
    SATURACION BAJA -> el hue es poco fiable (MAD enorme): discriminan S y V; H solo como canal debil.
  - Limitacion conocida: cambios fuertes de iluminacion/balance de blancos desplazan V ->
    recalibrar con `calib` si cambia la luz del lab. Sombras marcadas pueden dar falso obstaculo
    (conservador: prefiere falso obstaculo a falso libre).

Uso:
  python g1_floorcolor.py calib ref1.jpg [ref2.jpg ...]     # crea floorcolor_calib.json
  python g1_floorcolor.py test carpeta_de_jpgs/             # mascaras + metricas -> floorcolor_out/
  (como modulo)  fc = FloorColor.load(); mask = fc.mask(img); free = fc.free_center(mask)
"""
from __future__ import annotations
import json
import os
import sys
import glob

import numpy as np
import cv2

CALIB_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "floorcolor_calib.json")
# multiplicadores k*MAD por canal (H, S, V). MAD minimo por canal para no degenerar (moqueta muy uniforme).
K_HSV = (4.0, 4.0, 5.0)
MAD_MIN = (2.0, 4.0, 8.0)


class FloorColor:
    """Modelo MULTI-MODO: la misma moqueta cambia por completo con la exposicion de la camara
    (oficina iluminada: gris lavado S~11; pasillo oscuro: azul saturado S~91-101, medido 2026-07-02
    en crash_03_162351). Un pixel es moqueta si encaja con CUALQUIER modo."""

    def __init__(self, modes):
        # modes: lista de (med, mad) para modos estadisticos, o dicts {"h","dh","s":[lo,hi],"v":[lo,hi]}
        # para modos con LIMITES EXPLICITOS (data-driven, ej. moqueta oscura del pasillo).
        self.modes = []
        for m in modes:
            if isinstance(m, dict):
                self.modes.append(m)
            else:
                med, mad = m
                self.modes.append((np.asarray(med, dtype=np.float32),
                                   np.maximum(np.asarray(mad, dtype=np.float32), MAD_MIN)))

    @property
    def med(self):
        return self.modes[0][0] if not isinstance(self.modes[0], dict) else None

    @property
    def mad(self):
        return self.modes[0][1] if not isinstance(self.modes[0], dict) else None

    # ---------- calibracion ----------
    @classmethod
    def calibrate(cls, images):
        """images: lista de BGR (np.ndarray) de MOQUETA PURA. Cada imagen -> UN modo (mediana+MAD).
        Pasa una imagen por condicion de luz (oficina, pasillo oscuro, ...)."""
        modes = []
        for im in images:
            px = cv2.cvtColor(im, cv2.COLOR_BGR2HSV).reshape(-1, 3).astype(np.float32)
            med = np.median(px, axis=0)
            mad = np.median(np.abs(px - med), axis=0)
            modes.append((med, mad))
        return cls(modes)

    def save(self, path=CALIB_FILE):
        out = []
        for m in self.modes:
            if isinstance(m, dict):
                out.append(m)
            else:
                out.append({"med": m[0].tolist(), "mad": m[1].tolist()})
        json.dump({"modes": out, "k": list(K_HSV),
                   "note": "HSV cv2 (H 0-180); un modo por condicion de luz (med/mad o limites explicitos)"},
                  open(path, "w"), indent=1)

    @classmethod
    def load(cls, path=CALIB_FILE):
        j = json.load(open(path))
        if "modes" in j:
            return cls([m if "h" in m else (m["med"], m["mad"]) for m in j["modes"]])
        return cls([(j["med"], j["mad"])])          # calibracion antigua de un solo modo

    # ---------- inferencia ----------
    def mask(self, img_bgr):
        """255 = moqueta (suelo libre), 0 = NO moqueta (potencial obstaculo). OR de todos los modos."""
        hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV).astype(np.float32)
        acc = np.zeros(hsv.shape[:2], dtype=bool)
        for m in self.modes:
            if isinstance(m, dict):                 # modo con limites explicitos
                dh = np.abs(hsv[..., 0] - m["h"]); dh = np.minimum(dh, 180.0 - dh)
                acc |= ((dh <= m["dh"]) &
                        (hsv[..., 1] >= m["s"][0]) & (hsv[..., 1] <= m["s"][1]) &
                        (hsv[..., 2] >= m["v"][0]) & (hsv[..., 2] <= m["v"][1]))
                continue
            med, mad = m
            dh = np.abs(hsv[..., 0] - med[0]); dh = np.minimum(dh, 180.0 - dh)   # hue circular
            ds = np.abs(hsv[..., 1] - med[1])
            dv = np.abs(hsv[..., 2] - med[2])
            acc |= ((dh <= K_HSV[0] * mad[0]) & (ds <= K_HSV[1] * mad[1]) & (dv <= K_HSV[2] * mad[2]))
        m = acc.astype(np.uint8) * 255
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))    # quita sal-pimienta
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))   # cierra poros de la moqueta
        return m

    def free_profile(self, mask, ncols=16):
        """Por columna: fraccion de imagen (desde ABAJO) con moqueta CONTINUA, 0..1.
        Mismo contrato que floor_free_bands / free_center del perception_server."""
        h, w = mask.shape
        out = []
        for c in range(ncols):
            x0, x1 = int(c * w / ncols), int((c + 1) * w / ncols)
            col = mask[:, x0:x1].mean(axis=1) > 128
            run = 0
            for r in range(h - 1, -1, -1):
                if col[r]:
                    run += 1
                else:
                    break
            out.append(run / h)
        return out

    def free_center(self, mask, ncols=16):
        """Fraccion de suelo libre CONTINUO delante del robot (media de las 6 columnas centrales)."""
        p = self.free_profile(mask, ncols)
        lo = ncols // 2 - 3
        return float(np.mean(p[lo:lo + 6]))

    def near_run(self, mask, ncols=16, near_frac=0.18):
        """Nº de columnas cuya banda INFERIOR (cerca del robot) NO es moqueta = obstaculo pegado."""
        h, w = mask.shape
        y0 = int(h * (1.0 - near_frac))
        n = 0
        for c in range(ncols):
            x0, x1 = int(c * w / ncols), int((c + 1) * w / ncols)
            if (mask[y0:, x0:x1] > 128).mean() < 0.5:
                n += 1
        return n

    # ---------- DETECTOR DE PUERTA (sin entrenamiento) ----------
    # Puerta del lab = corredor de MOQUETA que se adentra en la imagen, flanqueado por
    # VERTICALES BLANCAS (marco/hoja: S baja, V alta). Devuelve el rumbo del centro del vano,
    # estable frente al 'ddir' del A* (que tiembla con el laser ruidoso y hacia oscilar DOOR-AL).
    def find_door(self, img_bgr, hfov_deg=56.0, ncols=32, depth_th=0.45, white_v=150, white_s=60,
                  min_width_frac=0.12):
        """Devuelve dict {bearing_deg, width_frac, depth, left_white, right_white, score} o None.
        bearing_deg: + = puerta a la IZQUIERDA del eje optico (convencion bearing estandar).
        LECCION (crash_03_162351): la RENDIJA entre la hoja abierta y el marco esta flanqueada de
        blanco por ambos lados y ganaba al vano real -> se exige ANCHO MINIMO PASABLE
        (min_width_frac ~0.12 = ~4/32 cols) y el score pondera depth*sqrt(width). Los flancos
        blancos son bonus/etiqueta, no requisito de la banda (el pasillo oscuro puede no tenerlos
        pegados); el requisito de contexto es que HAYA columnas blancas en la imagen (estamos
        ante marco/hoja) o un flanco blanco directo."""
        mask = self.mask(img_bgr)
        h, w = mask.shape
        prof = self.free_profile(mask, ncols)                      # profundidad de moqueta por columna
        hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        whitec = []
        for c in range(ncols):                                     # columna "blanca": marco/pared/hoja
            x0, x1 = int(c * w / ncols), int((c + 1) * w / ncols)
            col = hsv[:, x0:x1]
            whitec.append(((col[..., 1] < white_s) & (col[..., 2] > white_v)).mean() > 0.35)
        min_cols = max(2, int(round(min_width_frac * ncols)))
        WIN = 4                                                    # ventana de busqueda de flanco blanco
        best = None
        c = 0
        while c < ncols:
            if prof[c] >= depth_th:
                c0 = c
                while c < ncols and prof[c] >= depth_th:
                    c += 1
                band = (c0, c - 1)
                nb = band[1] - band[0] + 1
                if nb < min_cols:                                  # rendija impasable: descartar
                    continue
                depth = float(np.mean(prof[c0:c]))
                lw = any(whitec[max(0, c0 - WIN):c0])
                rw = any(whitec[c:min(ncols, c + WIN)])
                wf = nb / ncols
                score = depth * (wf ** 0.5) + 0.15 * (lw + rw)     # ANCHO manda; blanco es bonus
                if best is None or score > best["score"]:
                    best = {"_band": band, "depth": round(depth, 2),
                            "left_white": lw, "right_white": rw, "score": round(score, 2)}
            else:
                c += 1
        if best is None:
            return None
        band = best.pop("_band")
        # --- REFINADO de bordes (feedback Adrian 2026-07-02: las flechas caian en los marcos) ---
        # topext[c] = hasta donde SUBE la moqueta en la columna (1 = borde superior). Por el VANO el
        # pasillo continua y la moqueta llega alta; delante de un MARCO/pared se corta en su base.
        topext = []
        for cc in range(ncols):
            x0, x1 = int(cc * w / ncols), int((cc + 1) * w / ncols)
            colm = (mask[:, x0:x1] > 128).mean(axis=1) > 0.5
            rows = np.where(colm)[0]
            topext.append(1.0 - rows.min() / h if len(rows) else 0.0)
        core = [cc for cc in range(band[0], band[1] + 1) if topext[cc] >= 0.75]
        if not core:
            core = list(range(band[0], band[1] + 1))
        b0, b1 = min(core), max(core)
        # recorte de bordes: fuera columnas cuyo topext queda claramente bajo el p90 del nucleo
        # (moqueta que muere en la base del marco). p90 y no max: un pico aislado no debe mandar.
        ref = float(np.percentile([topext[cc] for cc in range(b0, b1 + 1)], 90)) - 0.04
        while b1 > b0 and topext[b1] < ref:
            b1 -= 1
        while b0 < b1 and topext[b0] < ref:
            b0 += 1
        # extension SOLO sobre columnas con suelo TAPADO abajo (prof<th) pero pasillo visible arriba
        # (la hoja de la puerta oculta la base; la moqueta del otro lado asoma por encima).
        while b0 - 1 >= 0 and prof[b0 - 1] < depth_th and topext[b0 - 1] >= 0.62:
            b0 -= 1
        while b1 + 1 < ncols and prof[b1 + 1] < depth_th and topext[b1 + 1] >= 0.62:
            b1 += 1
        # SNAP a cantos VERTICALES (Sobel, mitad superior): el canto de la hoja/marco marca el borde
        # fisico del vano con precision de pixel; topext solo con precision de ~1 columna.
        gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        gx = np.abs(cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3))
        estr = []
        for cc in range(ncols):
            x0, x1 = int(cc * w / ncols), int((cc + 1) * w / ncols)
            estr.append(float(gx[:int(h * 0.65), x0:x1].mean()))
        emax = max(estr) or 1.0
        estr = [v / emax for v in estr]
        SNAP = 3
        for cc in range(max(0, b0 - SNAP), min(ncols, b0 + SNAP + 1)):     # borde IZQ -> canto mas fuerte cercano
            if estr[cc] > 0.5 and (estr[b0] <= 0.5 or estr[cc] > estr[b0]):
                b0 = cc
        for cc in range(max(0, b1 - SNAP), min(ncols, b1 + SNAP + 1)):     # borde DCH
            if estr[cc] > 0.5 and (estr[b1] <= 0.5 or estr[cc] > estr[b1]):
                b1 = cc
        if b1 <= b0:                                                       # seguridad: nunca invertir el vano
            b0, b1 = min(b0, b1), max(b0, b1) if b1 > b0 else (b0, b0 + 1)
        center = (b0 + b1 + 1) / 2.0 / ncols                    # 0..1
        best.update({"bearing_deg": round((0.5 - center) * hfov_deg, 1),
                     "left_edge_deg": round((0.5 - b0 / ncols) * hfov_deg, 1),
                     "right_edge_deg": round((0.5 - (b1 + 1) / ncols) * hfov_deg, 1),
                     "width_frac": round((b1 - b0 + 1) / ncols, 2)})
        # contexto de puerta: flanco blanco directo O suficientes columnas blancas en la imagen
        if best["left_white"] or best["right_white"] or sum(whitec) >= 3:
            return best
        return None


# ---------------- CLI ----------------
def _cli():
    if len(sys.argv) >= 3 and sys.argv[1] == "calib":
        imgs = [cv2.imread(p) for p in sys.argv[2:]]
        imgs = [i for i in imgs if i is not None]
        if not imgs:
            print("no pude leer las imagenes de referencia"); return 1
        fc = FloorColor.calibrate(imgs)
        fc.save()
        print(f"calibrado con {len(imgs)} imagen(es): med={fc.med.round(1)} mad={fc.mad.round(1)}")
        print(f"-> {CALIB_FILE}")
        return 0
    if len(sys.argv) >= 3 and sys.argv[1] == "test":
        fc = FloorColor.load()
        outdir = "floorcolor_out"; os.makedirs(outdir, exist_ok=True)
        files = sorted(sum((glob.glob(os.path.join(sys.argv[2], e)) for e in ("*.jpg", "*.png")), []))
        for f in files:
            img = cv2.imread(f)
            if img is None:
                continue
            m = fc.mask(img)
            vis = img.copy()
            vis[m == 0] = (vis[m == 0] * 0.3 + np.array([0, 0, 180]) * 0.7).astype(np.uint8)
            vis[m > 0] = (vis[m > 0] * 0.7 + np.array([0, 120, 0]) * 0.3).astype(np.uint8)
            cv2.imwrite(os.path.join(outdir, os.path.basename(f) + ".mask.png"), np.hstack([img, vis]))
            print(f"{os.path.basename(f):40s} moqueta={(m > 0).mean() * 100:5.1f}%  "
                  f"free_center={fc.free_center(m):.2f}  near_run={fc.near_run(m)}")
        print(f"-> visualizaciones en {outdir}/")
        return 0
    print(__doc__)
    return 1


if __name__ == "__main__":
    sys.exit(_cli())
