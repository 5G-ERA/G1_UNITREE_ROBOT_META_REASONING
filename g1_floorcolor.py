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
    def __init__(self, med, mad):
        self.med = np.asarray(med, dtype=np.float32)
        self.mad = np.maximum(np.asarray(mad, dtype=np.float32), MAD_MIN)

    # ---------- calibracion ----------
    @classmethod
    def calibrate(cls, images):
        """images: lista de BGR (np.ndarray) de MOQUETA PURA. Mediana+MAD sobre todos los pixeles."""
        px = np.vstack([cv2.cvtColor(im, cv2.COLOR_BGR2HSV).reshape(-1, 3) for im in images]).astype(np.float32)
        med = np.median(px, axis=0)
        mad = np.median(np.abs(px - med), axis=0)
        return cls(med, mad)

    def save(self, path=CALIB_FILE):
        json.dump({"med": self.med.tolist(), "mad": self.mad.tolist(),
                   "k": list(K_HSV), "note": "HSV cv2 (H 0-180); moqueta lab"}, open(path, "w"), indent=1)

    @classmethod
    def load(cls, path=CALIB_FILE):
        j = json.load(open(path))
        return cls(j["med"], j["mad"])

    # ---------- inferencia ----------
    def mask(self, img_bgr):
        """255 = moqueta (suelo libre), 0 = NO moqueta (potencial obstaculo). Morfologia anti-ruido."""
        hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV).astype(np.float32)
        dh = np.abs(hsv[..., 0] - self.med[0]); dh = np.minimum(dh, 180.0 - dh)   # hue circular
        ds = np.abs(hsv[..., 1] - self.med[1])
        dv = np.abs(hsv[..., 2] - self.med[2])
        m = ((dh <= K_HSV[0] * self.mad[0]) &
             (ds <= K_HSV[1] * self.mad[1]) &
             (dv <= K_HSV[2] * self.mad[2])).astype(np.uint8) * 255
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
