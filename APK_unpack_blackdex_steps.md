# Desempaquetar Unitree Explore (Baidu shield) con emulador + BlackDex

Objetivo: sacar los **nombres de topic reales** del LiDAR/SLAM del G1, que están en el
dex cifrado del APK (blindaje `libbaiduprotect.so`). Se desempaqueta en runtime con
BlackDex sobre un emulador Android en tu Mac M4.

APK: `/Users/adrianlendinezibanez/unitree_webrtc_connect/examples/g1/Unitree_Explore.apk`

---

## PARTE 1 — Android Studio + emulador (una vez)

1. Descarga **Android Studio**: https://developer.android.com/studio  → instálalo.
2. Ábrelo. Arriba a la derecha (o More Actions) → **Virtual Device Manager**.
3. **Create Device** → elige p.ej. **Pixel 6** → Next.
4. En la imagen del sistema elige **API 33** (Tiramisu), variante **arm64-v8a**.
   - IMPORTANTE: coge una imagen **"Google APIs"** (NO "Google Play"), así puedes
     sideload sin restricciones. Descárgala (Download) → Next → Finish.
5. Pulsa ▶ para arrancar el emulador. Déjalo abierto.

## PARTE 2 — Comprobar adb

`adb` viene con Android Studio. En Terminal:
```bash
export PATH="$HOME/Library/Android/sdk/platform-tools:$PATH"
adb devices
```
Debe listar 1 emulador (algo como `emulator-5554   device`). Si sale vacío, espera a
que arranque del todo el emulador y repite.

## PARTE 3 — Instalar la app Unitree y BlackDex

1. Instala el APK de Unitree en el emulador:
```bash
adb install "/Users/adrianlendinezibanez/unitree_webrtc_connect/examples/g1/Unitree_Explore.apk"
```
(Si da error de ABI, es que la imagen no es arm64 → recrea el emulador en arm64.)

2. Descarga **BlackDex** (APK) desde su GitHub oficial:
   https://github.com/CodingGay/BlackDex/releases  → baja el `BlackDex-x.x.x.apk`.

3. Instálalo en el emulador:
```bash
adb install ~/Downloads/BlackDex-*.apk
```

## PARTE 4 — Desempaquetar

1. En el emulador, abre la app **Unitree Explore** una vez (déjala llegar a la pantalla
   principal aunque dé errores por no tener robot) y luego déjala en segundo plano.
2. Abre **BlackDex** en el emulador.
3. En su lista, selecciona **Unitree Explore** (o `com.unitree...`).
4. Pulsa el botón de **dump/desempaquetar**. Cuando termine, BlackDex **muestra la ruta**
   donde dejó los `.dex` (apúntala; suele ser algo como
   `/storage/emulated/0/Android/data/com.junkfood.../files/` o `/sdcard/BlackDex/...`).

## PARTE 5 — Sacar los dex al Mac y buscar los topics

1. Lista y copia lo desempaquetado (ajusta la ruta a la que mostró BlackDex):
```bash
adb shell ls -R /sdcard/Android/data/ | grep -i black     # localiza la carpeta
adb pull "<RUTA_QUE_MOSTRO_BLACKDEX>" ~/Downloads/unitree_dump
```

2. Busca los topics reales en los dex desempaquetados:
```bash
cd ~/Downloads/unitree_dump
grep -rhao -E "rt/[a-z_]+/[a-zA-Z0-9_/]+|utlidar/[a-zA-Z0-9_]+|voxel_map[a-zA-Z0-9_]*|qt_command|qt_add_node|uslam/[a-zA-Z0-9_/]+|lio_sam[a-zA-Z0-9_/]*|grid_map|cloud_world[a-zA-Z0-9_]*|/switch" . | sort -u
```

3. (Opcional, para leer la lógica del SLAM) decompila un dex con jadx:
```bash
brew install jadx
jadx -d ~/Downloads/unitree_src ~/Downloads/unitree_dump/*.dex
grep -rniE "utlidar|voxel_map|qt_command|uslam|disableTrafficSaving|traffic_saving|switch" ~/Downloads/unitree_src | head -60
```

---

## Qué me pegas

- La salida del **grep del Paso 5.2** (los topics `rt/...` reales).
- Si jadx funciona, los matches del 5.3 (cómo enciende el LiDAR / arranca mapping).

Con el nombre real del topic monto el stream → `.pcd` → `slam_toolbox`.

## Si algo se atasca
- Emulador lento/no arranca: en Android Studio, usa imagen arm64 y dale 4+ GB RAM.
- BlackDex no lista la app: ábrela una vez antes; si tiene varios procesos, dump del
  principal.
- Alternativa a BlackDex: `frida-dexdump` (necesita frida-server en el emulador).
