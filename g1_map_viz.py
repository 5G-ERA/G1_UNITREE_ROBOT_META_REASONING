#!/usr/bin/env python3
"""
g1_map_viz.py  -  Visualiza un mapa de puntos del G1 (nube laser exportada desde la WebView)

Lee un JSON con la lista de puntos [[x,y,z], ...] (exportado desde la consola del inspector con
copy(JSON.stringify(Object.values(window.__map)))) y lo dibuja: vista en planta coloreada por
altura, mas una vista 3D opcional.

Requisitos: pip install matplotlib numpy
USO:
  python g1_map_viz.py ~/g1/map.json
  python g1_map_viz.py ~/g1/map.json 3d     # añade vista 3D
"""
import json, sys, os
import numpy as np
import matplotlib
for _bk in ("MacOSX", "TkAgg", "QtAgg"):
    try: matplotlib.use(_bk); break
    except Exception: continue
import matplotlib.pyplot as plt


def main():
    if len(sys.argv) < 2:
        print("uso: python g1_map_viz.py <archivo.json> [3d]"); sys.exit(1)
    path = os.path.expanduser(sys.argv[1])
    pts = json.load(open(path))
    arr = np.array(pts, dtype=float)
    if arr.ndim != 2 or arr.shape[1] < 3:
        print("formato inesperado; esperaba lista de [x,y,z]"); sys.exit(1)
    print(f"{len(arr)} puntos. x[{arr[:,0].min():.2f},{arr[:,0].max():.2f}] "
          f"y[{arr[:,1].min():.2f},{arr[:,1].max():.2f}] z[{arr[:,2].min():.2f},{arr[:,2].max():.2f}]")

    if len(sys.argv) > 2 and sys.argv[2] == "3d":
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection="3d")
        ax.scatter(arr[:, 0], arr[:, 1], arr[:, 2], s=1, c=arr[:, 2], cmap="viridis")
        ax.set_xlabel("x"); ax.set_ylabel("y"); ax.set_zlabel("z")
        ax.set_title(f"G1 mapa LiDAR 3D — {len(arr)} puntos")
    else:
        fig, ax = plt.subplots(figsize=(10, 10))
        sc = ax.scatter(arr[:, 0], arr[:, 1], s=2, c=arr[:, 2], cmap="viridis")
        plt.colorbar(sc, label="altura z (m)")
        ax.set_aspect("equal"); ax.grid(True, alpha=0.3)
        ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
        ax.set_title(f"G1 mapa LiDAR (planta) — {len(arr)} puntos")
    plt.show()


if __name__ == "__main__":
    main()
