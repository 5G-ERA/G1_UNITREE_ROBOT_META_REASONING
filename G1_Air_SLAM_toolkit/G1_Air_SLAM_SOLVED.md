# G1 Air — SLAM por WebRTC desde Python: RESUELTO ✅

Cómo controlar el SLAM y recibir datos de mapeo del **Unitree G1 Air** desde un PC, por
WebRTC, sin EDU ni ROS2 a bordo. Resuelto en jun 2026 por ingeniería inversa de la app
Unitree Explore.

> Resultado probado: `startBuildMap` devuelve `code=0 "Successfully started mapping."` y
> el robot emite `rt/slam_info` y `rt/unitree/slam_mapping/odom` (~10 Hz) al PC. La nube
> `rt/unitree/slam_mapping/points` empieza a llegar al **conducir** el robot.

---

## 1. La clave del misterio

El G1 **no** usa los topics del Go2 (`rt/uslam/*`, `rt/utlidar/voxel_map`). Por eso fallaron
todos los intentos previos. El G1 tiene su **propia API de SLAM**:

| Qué | Topic |
|---|---|
| Comando SLAM (request/response) | `rt/api/slam_operate/request` |
| Nube del mapeo | `rt/unitree/slam_mapping/points` |
| Odometría del mapeo | `rt/unitree/slam_mapping/odom` |
| Nube/odom de relocalización | `rt/unitree/slam_relocation/points` · `.../odom` |
| Info del SLAM | `rt/slam_info` · `rt/slam_key_info` |
| LiDAR crudo (opcional) | `rt/utlidar/voxel_map` · `..._compressed` · switch `rt/utlidar/switch` |
| Grid map | `rt/mapping/grid_map` |

## 2. Cómo se manda un comando

Request publicado en `rt/api/slam_operate/request` (tipo `req`). Con la librería
`legion1581/unitree_webrtc_connect`:

```python
await ps.publish_request_new("rt/api/slam_operate/request",
    {"api_id": 1801, "parameter": {"data": {"slam_type": "indoor"}}})
```
Internamente equivale al envoltorio de la app:
`{header:{identity:{id, api_id}}, parameter: json({data: <data>})}`.

## 3. api_id por operación

| Operación | api_id | data |
|---|---|---|
| Empezar mapeo | **1801** | `{"slam_type":"indoor"}` |
| Terminar + guardar | **1802** | `{"address":"/unitree/data/unitree_slam/<nombre>.pcd"}` |
| Cancelar mapeo | **1803** | — |
| Iniciar relocalización | **1804** | `{x,y,z,q_x,q_y,q_z,q_w, address:".../<nombre>.pcd"}` |
| Cerrar relocalización | **1805** | — |
| Navegar a punto | **1102** | `{"targetPose":{x,y,z,q_x,q_y,q_z,q_w}, "mode":1}` |
| Pausar / reanudar nav | 1201 / 1202 | — |
| Cerrar tarea de nav | 1203 | — |
| Cerrar todo | 1901 | — |
| Nav circular multipunto+ | 1932 | `{version:1, topoAddress, nodeList, cyclesNum}` |
| Escribir/leer fichero grande (mapa) | 1933 / 1934 | `{address:"/unitree/data/unitree_slam/...", ...}` |

(Operaciones de nodos/topología usan otro constructor `command_` por `rt/qt_command`:
saveNodeEdgeToMap=16, loadTopMap=17, singlePointNavigation=9, multiPointCircular=10,
returnCharge=12, returnToStartPoint=15.)

## 4. Formato de los mensajes recibidos

- `rt/unitree/slam_mapping/odom`: nav_msgs/Odometry-like → `data.pose.pose.{position, orientation}`, `frame_id:"map"`.
- `rt/slam_info`: JSON `{type:"robot_data", sec, nanosec, errorCode, info, data:{motorTemp:[...], ...}}`.
- `rt/unitree/slam_mapping/points`: la app lo pasa tal cual a un worker (`{type:"newMap", data}`).
  Formato binario por confirmar (se captura crudo en `~/g1/slam_map/` para analizar y convertir a `.pcd`/PointCloud2).

## 5. Flujo de mapeo (probado)

1. **App del móvil CERRADA** (sesión WebRTC única). Mac en el AP del robot (`192.168.12.x`).
2. Robot **de pie** con el mando, zona despejada o en grúa, mando a mano como parada.
3. Arrancar y mapear:
   ```bash
   cd ~/unitree_webrtc_connect && source .venv/bin/activate
   python slam_g1_mapping.py map
   ```
4. **Conduce el robot despacio** por toda la zona → empieza a llegar `slam_mapping/points`.
5. `Ctrl+C` → te pregunta nombre → envía `endBuildMap` (1802) → mapa guardado en el robot
   en `/unitree/data/unitree_slam/<nombre>.pcd`.
6. Bajar el `.pcd` al PC: operación `getBigFile` (api_id 1934) — pendiente de script.

Comandos sueltos: `start`, `save <nombre>`, `cancel`, `listen` (ver §scripts).

## 6. Cómo se descubrió (para reproducir/auditar)

1. El G1 Air es WebRTC-only; la lib de terceros solo veía telemetría, no la nube.
2. La app Explore (`com.unitree.b2dog`) está blindada con Baidu (`libbaiduprotect.so`).
3. Se desempaquetó con **frida-dexdump** sobre un emulador Android arm64 (Android Studio),
   con `frida-server` y `adb root` (BlackDex falló por detección de entorno).
4. El SLAM es un **WebView** (`SlamWebActivity`, web app Vue+Three.js en `assets/dist`,
   workers cifrados).
5. Se lanzó la pantalla sin robot:
   `adb shell am start -n com.unitree.b2dog/com.unitree.godog.ui.activity.web.SlamWebActivity`
   y se inspeccionó el WebView con **chrome://inspect** (DevTools → Sources), donde el código
   ya descifrado reveló los topics y los `api_id` (`genParam_*`, `publishReqNewForSlam`).

## 7. Scripts (en este toolkit)

| Script | Para qué |
|---|---|
| `slam_g1_mapping.py` | **Principal**: arrancar/parar/guardar mapeo + recibir nube/odom |
| `dump_services.py` | Lista los servicios del robot (rt/servicestate) |
| `query_api.py` | Interroga la API request/response (imprime respuestas/códigos) |
| `discover_slam_api.py` | Descubre api-topics existentes + formato motion_switcher |
| `g1_lidar_stream.py` | Intento de LiDAR crudo (utlidar/voxel) — referencia |
| `capture_slam_data.py` / `probe_slam_lidar.py` / `lidar_topic_sweep.py` | Diagnóstico/exploración |

## 8. Pendiente / siguiente

- Confirmar el **formato binario** de `slam_mapping/points` (capturar crudo conduciendo) y
  convertirlo a `.pcd` / `sensor_msgs/PointCloud2`.
- Script para **bajar el `.pcd`** del robot (api_id 1934, chunked).
- Navegación: probar `1102` (anyPointNavigation) con una pose sobre un mapa relocalizado.

## Notas legales / seguridad
- Ingeniería inversa hecha sobre tu propia app/robot, para interoperar, **sin redistribuir**
  el APK/dex/claves. Mantener privado.
- Cualquier prueba con el robot: de pie por mando, zona despejada/grúa, mando como parada.
