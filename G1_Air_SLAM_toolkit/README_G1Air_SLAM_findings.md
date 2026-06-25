# G1 Air — SLAM por WebRTC: notas de campo

Registro de la investigación para controlar el SLAM/navegación de un **Unitree G1 Air**
desde un PC por WebRTC (librería `legion1581/unitree_webrtc_connect`, modo LocalAP).
Fecha: junio 2026. Estado: **SLAM no alcanzable por este canal con lo conocido.**

---

## 1. Hardware y acceso (confirmado)

El robot es un **G1 Air** (gama base), no EDU:

- Sin ordenador interno de desarrollo (no Jetson), sin Ethernet de desarrollo, sin SSH.
- Único acceso: WiFi AP, subred `192.168.12.0/24`, robot en `192.168.12.1`.
- Señalización WebRTC en `http://192.168.12.1:8081` (ruta `/offer`).
- Firmware en ruta legacy (`data2===2`, clave AES estática) — conecta sin clave por dispositivo.
- NO aplica el camino ROS2 Humble + `unitree_ros2` + FAST-LIO del EDU: no hay dónde correrlo a bordo.

### Restricción operativa clave

El robot admite **una sola sesión WebRTC**. La app y un script propio **no** pueden
coexistir por el AP local. Para usar el script hay que cerrar la app.

---

## 2. Servicios a bordo (vía rt/servicestate / robot_state api 1003)

La unidad **SÍ tiene LiDAR y SLAM**, y los servicios están ACTIVOS:

| Servicio | status | versión |
|---|---|---|
| `unitree_slam` | 1 (activo) | 1.0.2.1 |
| `lidar_driver` | 1 (activo) | 1.0.0.5 |
| `ai_sport` | 0 | 8.5.1.1 |
| `motion_switcher` | 0 | 1.0.0.1 |
| `webrtc_bridge` | 0 (protect) | 1.0.8.10 |
| `robot_state` | 0 (protect) | 1.2.3.6 |
| ... (27 servicios en total) | | |

**Importante:** `status=1` significa "servicio registrado/vivo", NO "sesión de mapeo activa".
Que `unitree_slam` esté arriba no implica que esté mapeando ni que acepte comandos de terceros.
No arrancamos nada: solo leímos que ya estaba vivo.

---

## 3. Qué SÍ funciona por WebRTC (whitelist alcanzable)

El canal request/response funciona. Confirmado alcanzable:

- **Telemetría**: `rt/lf/lowstate` (LOW_STATE, ~1Hz), `rt/lf/sportmodestate` (LF_SPORT_MOD_STATE, ~15Hz).
- **Estado**: `rt/servicestate`; `rt/api/robot_state/request` (api 1003 = lista servicios, code 0); `rt/api/config/request`.
- **motion_switcher** (`rt/api/motion_switcher/request`):
  - `1001` GET modo → `{"form":"0","name":"ai"}`
  - `1005` GET silent → `{"silent":false}`
  - `1007` GET modo (igual que 1001)
  - `1002` SELECT modo → siempre `7002` con los nombres probados (ai/normal/advanced/navigation/slam/mapping), dict o string.
- **obstacles_avoid** (`rt/api/obstacles_avoid/request`): SWITCH_SET 1001 `{enable:bool}`, SWITCH_GET 1002, MOVE 1003 `{x,y,yaw,mode:0}`.
- Sport (`rt/api/sport/request`): GetState 1034 → `3203` (servicio sport no disponible; cuadra con `ai_sport` status 0).

### Códigos de estado observados
`0`=OK · `7001`=falta parámetro · `7002`=parámetro inválido · `3203`/`3204`=api/servicio no disponible · `8201`=config no disponible · `7404 FSM_UNAVAILABLE`=robot tumbado/FSM dormido.

---

## 4. Qué NO funciona: el SLAM

Probado exhaustivamente, **nada del SLAM/LiDAR es alcanzable**:

- Topics de comando SLAM (`rt/uslam/client_command`, `rt/qt_command`): **ni ack, ni log, ni error** ante 12 payloads candidatos.
- Topics de datos (`rt/utlidar/*`, `rt/uslam/frontend/*`, `rt/uslam/cloud_map`, `rt/mapping/grid_map`, `rt/utlidar/robot_pose`): **0 mensajes**, incluso tras `disableTrafficSaving(True)` (que el robot ACK-eó con `-> True`).
- NO existe ningún api-topic de SLAM: `rt/api/{uslam,slam,navigation,nav,mapping,gridmap,lidar,localization,map,qt}/request` → todos **timeout** (no existen).
- `ULIDAR_SWITCH` (`rt/utlidar/switch`) con `{"data":True}`: sin efecto observable.

### Por qué no podemos ni "descubrir" los nombres
El WebRTC de Unitree solo reenvía un topic si te suscribes a su **nombre exacto**; el robot
no empuja topics no solicitados. Un "catch-all" no revela nombres desconocidos. Por eso quedan
dos hipótesis indistinguibles desde aquí:

1. Los nombres de topic SLAM son **distintos** en este firmware (los del Go2 no valen), o
2. El `webrtc_bridge` los **bloquea** a clientes de terceros.

### Hipótesis descartada: AES
El AES de la librería (`unitree_auth.py`) cifra solo el **handshake de señalización (SDP)**,
no es control de acceso a topics. Autenticarse no cambia la whitelist. (Además
`unitree-fetch-aes-key` exige credenciales de cuenta Unitree.)

---

## 5. Conclusión

- **El SLAM existe y está vivo a bordo, pero no se puede arrancar ni leer desde el PC** con
  la librería WebRTC de terceros y los nombres/comandos conocidos.
- La **app** controla el SLAM por una vía privilegiada distinta (probablemente nombres de topic
  propios y/o forwarding que el bridge solo concede a la app).
- SLAM programático "de verdad" requeriría acceso de desarrollador a bordo (SSH a los binarios
  `unitree_slam`), que el Air no expone — territorio EDU.

## 5b. CONCLUSIÓN DEFINITIVA tras ingeniería inversa del APK (jun 2026)

Se desempaquetó la app Unitree Explore (`com.unitree.b2dog`), blindada con Baidu
(`libbaiduprotect.so`), mediante frida-dexdump sobre un emulador Android arm64 (BlackDex
falló por detección de entorno). Hallazgos del código descifrado:

- La capa WebRTC nativa (`com.unitree.webrtc.data.*`) solo contiene beans de **estado/
  telemetría**: `G1DataBean`, `G1ImuState`, `G1MotorState`, `BmsState`, `LidarStateBean`
  (= salud del LiDAR), `UwbStateBean`, etc. + comandos (`DogApiId`, `DogCmdConstant`,
  `SendGo2Req`, `TopicSubscribe`, `WebRTCConstant`). **No hay bean de nube/voxel/mapa.**
- Único topic LiDAR por WebRTC: `rt/utlidar/lidar_state` (solo estado).
- El **SLAM es un WebView**: `com.unitree.godog.ui.activity.web.SlamWebActivity` carga una
  web app empaquetada en `assets/dist/` (Vue + Three.js, `three.worker.js` para la nube 3D),
  servida en `localhost:17979`, que habla con el robot por `192.168.12.1:8081/offer` y
  `:9991/con_check` vía un puente JS. En ese JS minificado **no aparecen "voxel"/"utlidar"/
  "lidar"** → la nube no se referencia como topic ROS reutilizable.

**Veredicto:** en el G1 Air la nube del LiDAR / mapa SLAM NO se expone como topic WebRTC a
clientes de terceros (a diferencia del Go2, que sí publica `rt/utlidar/voxel_map`). El camino
"stream voxel_map → slam_toolbox en el PC" NO es viable en este hardware. La SLAM/navegación
solo está disponible dentro de la app (WebView), y el mapa se queda en el robot.

Vías abiertas: (a) inspeccionar a fondo el JS de `assets/dist` / la red del WebView para ver
qué pide exactamente (avanzado, incierto); (b) preguntar en Discord RoboVerse/legion1581;
(c) usar la app para SLAM y programar lo alcanzable por WebRTC (obstacle_avoid + teleop +
telemetría).

---

## 6. Próximos pasos posibles

1. **Descompilar el APK de Unitree** (`jadx`) y buscar `rt/uslam`, `rt/utlidar`, `QtCommand`,
   `client_command` → revela los nombres de topic reales y cómo construye la app el comando de
   empezar/guardar mapa. Es la vía con opción real. (La comunidad ya descompila el apk; la propia
   `unitree_auth.py` referencia `AESGCMUtil.keyBytes` del apk.)
2. **Comunidad legion1581** (Discord/GitHub issues): preguntar por comandos SLAM del G1.
3. **Pragmático**: usar la app para SLAM (el mapa vive en el robot) y dedicar el código a lo
   alcanzable: navegación **reactiva** (`obstacles_avoid` MOVE/SWITCH) + teleop + telemetría.

---

## 7. Scripts de la investigación (carpeta del proyecto)

| Script | Para qué |
|---|---|
| `probe_slam_lidar.py` | Probe pasivo de topics LiDAR/SLAM (+ `--lidar-on`) |
| `dump_services.py` | Vuelca la lista completa de servicios del robot |
| `slam_command_probe.py` | Envía comandos candidatos de start-mapping y escucha feedback |
| `query_api.py` | Interroga la API request/response (imprime respuestas/códigos) |
| `discover_slam_api.py` | Descubre api-topics existentes (RESP vs timeout) + formato motion_switcher |
| `lidar_traffic_test.py` | LiDAR con `disableTrafficSaving(True)` + catch-all de topics |
| `capture_slam_data.py` | Captura/vuelca datos de mapping mientras la app mapea (limitado por sesión única) |

### Seguridad
Cualquier prueba de movimiento: robot en zona despejada o en grúa, mando físico a mano como
parada de emergencia. Arrancar el robot con **los brazos rectos hacia abajo**. Antes de cualquier
`Move`, ponerlo de pie con el mando (si no, los comandos devuelven `FSM_UNAVAILABLE`).
