# Plan: descompilar el APK de Unitree para sacar los topics/comandos de SLAM del G1

Objetivo: extraer del APK de la app de Unitree (la que SÍ hace SLAM en tu G1 Air) los
**nombres de topic reales** y el **comando exacto** de empezar/guardar mapa, para
replicarlos por WebRTC. Es ingeniería inversa para interoperar con tu propio robot.

Contexto: la búsqueda en comunidad confirmó que los nombres de topic que usamos son los
del Go2 y que el SLAM "de PC" se hace con `rt/utlidar/voxel_map` + slam_toolbox; pero en
tu G1 Air no llega nada por esos nombres. El APK es la fuente de verdad de qué topics y
comandos usa la app en TU firmware.

---

## 0. Antes del APK (2 comprobaciones rápidas que pueden ahorrarlo)

1. **Discord de RoboVerse / legion1581**: preguntar directamente "G1 Air: ¿streamea
   `rt/utlidar/voxel_map` por WebRTC? ¿topic/condición?". Ahí están los que mantienen la
   librería y tienen G1.
2. **Ejecutar el ejemplo `lidar_stream.py`** de la librería (está orientado a Go2, pero
   revela la secuencia correcta de habilitación de LiDAR; quizá usa `voxel_map_compressed`
   o un orden distinto al que probamos):
   ```bash
   find ~/unitree_webrtc_connect/examples -iname "*lidar*"
   ```

---

## 1. Conseguir el APK

- App: "Unitree" / "Unitree Explore" / "Unitree Go" (la que usas para el G1).
- Obtén el `.apk` de tu propio dispositivo Android (método limpio): 
  ```bash
  # con el movil conectado por USB y depuracion activada:
  adb shell pm list packages | grep -i unitree     # encuentra el package name
  adb shell pm path <com.unitree.xxx>               # ruta del apk en el movil
  adb pull /data/app/.../base.apk  unitree.apk      # descargalo
  ```
- Alternativa: descargar el APK desde un mirror de confianza (APKMirror/APKPure) si no
  tienes Android. Verifica que la versión coincide con la que te funciona el SLAM.

## 2. Descompilar con jadx

```bash
# instalar jadx (macOS)
brew install jadx

# decompilar a fuentes Java + recursos
jadx -d unitree_src unitree.apk
# o GUI:
jadx-gui unitree.apk
```

## 3. Qué buscar (grep dentro de unitree_src/)

```bash
cd unitree_src

# topics de lidar / slam / mapa (nombres REALES del firmware)
grep -rni "utlidar\|voxel_map\|uslam\|qt_command\|qt_add_node\|grid_map\|cloud_map" . | grep -i "rt/"

# comando de mapeo: start / stop / save / relocation
grep -rniE "client_command|start_?map|stop_?map|save_?map|mapping|relocat" . | head -50

# estructura del mensaje QtCommand (campos)
grep -rni "QtCommand\|qt_command" .

# habilitacion del lidar / traffic saving
grep -rni "traffic_saving\|disable_traffic\|lidar.*switch\|utlidar/switch" .

# como construye el request (api_id) para SLAM/navegacion
grep -rniE "api_id|rt/api/.*request" . | grep -iE "slam|nav|map|lidar|uslam" | head
```

## 4. Qué nos llevamos de ahí

- **Nombres de topic exactos** del LiDAR y del SLAM en tu firmware (para suscribirnos).
- **Payload exacto** del comando start/stop/save mapping (el `data`/`api_id`/campos).
- La **secuencia de habilitación** (orden de switch lidar + traffic saving + subscribe).

Con eso, convierto los scripts que ya tienes (`capture_slam_data.py` / `slam_command_probe.py`)
en el flujo definitivo: arrancar mapping → conducir → guardar mapa → descargar a `.pcd`/`.pgm`.

## 5. Notas legales

Descompilar el APK de un producto que posees, para interoperar con tu propio robot y para
investigación personal, es la práctica estándar de esta comunidad (de ahí salieron las
claves y métodos que usa la librería). No publiques binarios ni secretos de terceros; usa
los hallazgos solo para tu integración.

---

## Estado para retomar

- LiDAR + `unitree_slam` activos a bordo (confirmado). SLAM no alcanzable por WebRTC con
  nombres/método del Go2. Ver `README_G1Air_SLAM_findings.md`.
- Siguiente acción concreta: §0 (Discord + lidar_stream.py) y si no, §1–§4 (APK).
