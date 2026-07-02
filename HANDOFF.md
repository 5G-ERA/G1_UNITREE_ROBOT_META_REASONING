# HANDOFF — G1 ROBOT (navegación A→B + evaluación DCE/meta-razonamiento)

Documento de traspaso para continuar el trabajo en otra cuenta de Claude (misma máquina).
**Cómo usarlo:** abre la carpeta `G1 ROBOT` en la nueva cuenta y pídele a Claude que lea este archivo primero.

_Última actualización: 2026-07-02 (sesión Claude Cowork; ver sección 8). Autor del contexto: Adrian (dev RPA + doctorando en robótica: metacognición y analogías)._

---

## 1. Qué es el robot

- **Unitree G1 "Air"**: humanoide de consumo. **Solo WebRTC** vía la app de iPhone (no hay ROS2/EDU nativo).
- Se controla "pinchando" la sesión web del WebView: `ios_webkit_debug_proxy` (puerto 9221) + CDP por USB, e **inyectando** `rt/wirelesscontroller {lx,ly,rx,ry}` a 20 Hz.
- **Zona muerta ~0.3**: por debajo de |0.3| en un stick el robot NO se mueve. La app usa 0.5–0.73. Todo comando útil va por encima de 0.3.
- **Láser en vivo** = nube `location` (frame del MAPA, Z-up: idx0=x, idx1=y, idx2=altura). Es **muy ruidoso**: LiDAR en la cabeza de un bípedo (vibra con la marcha) proyectado por la pose de relocalización.
- `loc_match` = solape scan-vs-mapa (confianza de localización estimada por nosotros; el firmware no da covarianza).

## 2. Repo y ramas

- GitHub: `https://github.com/5G-ERA/G1_UNITREE_ROBOT_META_REASONING`
- `master` → `origin/main` = versión estable de navegación.
- `feature/escmr-meta-reasoning` = meta-razonador DCE (lanzar con `G1_META=1`).
- `feature/fsm-baseline` = baseline FSM (lanzar con `G1_FSM=1`).

**Flujo git (IMPORTANTE):** la carpeta del Mac es el repo de trabajo; **Adrian hace el push él mismo** (`git push origin HEAD:main`). El robot corre desde un **Ubuntu** que es otro clon (allí se hace `git pull`). Rechazos non-fast-forward frecuentes porque las runs se pushean desde Ubuntu → resolver con `git pull --no-rebase` antes del push. Claude **no** mete credenciales de GitHub: solo commitea y da el comando de push.

## 3. Archivos principales

- `g1_goto.py` — navegación A→B en vivo: A* (tipo firmware) + DWA local, obstáculos de la nube `location`, maniobra de puerta, métricas. **Es el fichero de control.**
- `g1_nav_v2.py` (importado como `g`) — conexión, A*, DWA, costmap, cámara, helpers. `AV_TURN=0.45`, `OCELL=0.2`, `NEAR_BLIND=0.60`.
- `perception_server.py` + `g1_perception.py` — servidor GPU offboard (YOLO + depth) que ve la **mesa invisible al LiDAR** en la puerta. Cliente async (`PerceptionWorker`) para no congelar el control.
- `g1_metrics.py` — métricas SEI: clearance (espacio libre) + progression (avance a B) + sensing reliability.
- `g1_meta.py` (rama escmr) — `MetaController`, analogías (efficient_nav / cautious_nav / payload_sensitive / human_aware), `build_reasoner(ablation)`.
- `g1_fsm.py` (rama fsm) — `FSMBaseline` (árbol de decisión por umbrales).
- `calibrate_cam.py` — calibración de cámara (modo `wall`, sin ajedrez).
- `plot_metrics.py` / `plot_health.py` / `plot_trajectory.py` / `organize_run.py` / `summarize_runs.py` — análisis de runs (dataset/ → `runs_summary.csv`).
- `waypoints.json` — A=(0.99,0.57), B=(−4.73,3.04), C=(−0.03,−1.49); pcd `Qw_20260625`.

## 4. Cómo lanzar una run (con visión ON — necesario para la puerta)

En el **Ubuntu**, tras `git pull`:

```bash
# Terminal 1: servidor de percepción (calibración correcta para frame de 320px)
source venv/bin/activate
python perception_server.py --host 0.0.0.0 --port 8008 \
    --fx 300 --fy 300 --cx 160 --cy 120 --cam-h 1.10 --cam-pitch -10

# Terminal 2: navegación con visión enchufada
G1_PERC=127.0.0.1:8008 python g1_goto.py gotoviz B
```
Al arrancar debe imprimir `[perc] ... -> OK`. En el log, `perc_n > 0` = la visión aporta.

**Requisitos del robot:** relocalizado en la app, batería OK, espacio despejado, kill switch a mano. Pruebas con **agua** (no café).

## 5. Estado ACTUAL de la navegación (dónde estamos)

**RESUELTO — la puerta se cruza.** Era el muro real durante muchas runs. El combo que lo desatascó:
- Banda de alineación de puerta ensanchada 12°→25° (mata la oscilación de yaw; no se puede bajar el giro por la zona muerta) — commit `92030c7`.
- **Visión ON**: el LiDAR ve la mesa/marco del umbral como pared (c0 baja a ~0.09); la visión abre la compuerta de avance (`DOOR-GOv`). Runs `134458` y `135819` cruzaron y llegaron a **0.52 m y 1.12 m de B**.

**Aplicado, PENDIENTE de validar en robot** (2 cambios, uno por fallo observado):
1. `1858520` — **último metro**: por debajo de `DOOR_MIN_GOAL=1.3 m` se desactiva la maniobra de puerta (cerca de B no hay puerta, es el goal con un mueble) para que el DWA **rodee** el obstáculo en vez de empujar recto. (Run 135819 se clavaba a 1.5 m contra una mesa a 13 cm.)
2. `45e5e69` — **filtro de ruido del láser / confiar más en el mapa**: una celda del mapa estático (pared conocida) entra al instante; una celda que **solo** ve el láser necesita aparecer en ≥2 de los últimos 3 barridos (`PERSIST_K=2`, `PERSIST_N=3`). El ruido y la nube desplazada por saltos de reloc parpadean 1 barrido → se descartan. **No añade paredes fantasma** (eso rompió el movimiento antes), solo quita ruido. La visión no se filtra.

**PENDIENTE — fix 2 (siguiente):** guardia anti-divergencia de reloc. En la run `134458` la relocalización explotó (78 `reloc_jump`, `path_m=582 m`, posición final a 538 m). Falta re-añadir el guardia que para (STOP) cuando la reloc salta repetidamente y no integra el salto. (Se fue en el rollback a la versión de ayer.)

**Qué mirar en el próximo log:** `obs=` que no crezca solo (ruido acumulado); cerca de B que rodee y no se clave; `DOOR-GOv` en la puerta; si la reloc salta, que NO aparezcan paredes nuevas de golpe.

## 6. Gotchas aprendidos (no repetir)

- **NO hacer el mapa autoritario para AÑADIR paredes.** El `map_full.json` del G1 está desalineado (el waypoint A cae sobre una pared ~0.08 m) → metía paredes fantasma en el arranque y el robot no se movía. Confiar en el mapa solo para **rechazar ruido**, nunca para inventar obstáculos. Por defecto `G1_REFMAP="summit"` (el mapa alineado).
- **Un cambio cada vez y validar en el robot** antes del siguiente. Meter varios cambios juntos nos costó un día (un fix tapaba el fallo del otro).
- Cámara: para frame de 320 px usar `cx=160 cy=120 fx=300 fy=300` (NO 320/240, que es de 640 px).
- BrokenPipe en el server: cliente async + warmup del modelo + el server ignora BrokenPipe.
- YOLO falsos positivos → `--det-conf 0.45`.
- Ventana cv2 no va en SSH/headless → endpoints `/debug.jpg` y `/debug.mjpg` en el navegador.

## 7. Contexto de investigación (el "para qué")

Paper del tutor sobre **DCA/DCE** (Decentralised Capability Abstraction/Ecosystem). `ESCMR` (`ExperienceScopedCapabilityMetaReasoner`) = forma runtime del Cap.5 de la tesis (razonamiento por analogías): vectores de atención, zonas semánticas de QoE, tensión de desplegabilidad, creencia/plausibilidad Dempster-Shafer, decisiones keep/switch/fallback/help/insufficient.

**Evaluación buscada:** DCE (meta-razonador) vs FSM baseline + ablaciones, en el robot real, cruzando la puerta con distintas condiciones (payload sin tapa, humano cerca, batería baja). Métricas instrumentadas: clearance, progression, sensing reliability, spill ground-truth (un humano marca con Enter cuando se derrama agua), colisiones, salud por articulación. `summarize_runs.py` junta `dataset/*.json` en `runs_summary.csv` (rellenar a mano `condition` y `notes`).

---

## 8. Sesión 2026-07-02 — hallazgos y estado (MEMORIA: leer antes de tocar nada)

### ⚡ TL;DR — ESTADO ACTUAL (si solo lees una cosa, lee esto)

- **Funciona**: A→B llega en 72–84 s, 0 colisiones, sin modo agresivo (run 130231). Nube láser a
  ~2.2 Hz. Canal de moqueta VIVO en el server (`G1_FLOORCOLOR=1`, verificar `floorcolor=ON` en el
  PERC-TEST del arranque). Strafe con signo corregido (mapeo gamepad: lx>0=DERECHA; default -1).
- **Bugs cerrados hoy (con mediciones)**: dedup de barrido · clamp 0.4→0.7 (NEAR_BLIND se comía los
  avisos) · signo del strafe · anti-jaula (clamp solo central+alto; visión por score, sin bypass).
- **PENDIENTE de validar en robot (en orden, UNA por run)**: ① B→A con fix anti-jaula
  ② `G1_HARDGUARD=1` (paredes no negociables — idea de Renxi) ③ `G1_HBAND_LO=-0.7` (objetos bajos).
- **Herramientas**: `python autopsy.py dataset/<run>.json` → informe HTML completo (trayectoria,
  timelines, eventos, fotos). `summarize_runs.py` → CSV comparativo. Ventana live: server `--debug`
  + navegador en `http://IP:8008/`.
- **Reglas operativas**: git pull en el Ubuntu ANTES de lanzar · push siempre `origin HEAD:main` ·
  un cambio por run · nunca subir PERSIST_K · el color nunca resta obstáculos (fusión por unión).

### 8.1 Cambios aplicados a `g1_goto.py` (compilan; PENDIENTES de validar en robot)

1. **Dedup de barrido fresco**: `reloc_cells.fresh` (hash del buffer `__relocbuf`). La persistencia y el
   score/decay solo votan con nube NUEVA — antes el mismo barrido leído 2-3 ticks se autoconfirmaba solo
   en el filtro 2-de-3. `SCAN-STALE` en el log si la nube se congela ~3 s.
2. **Diagnóstico en vivo** (solo observación): log `nz=` (laser_noise) `flt=` (fracción del barrido rechazada
   por persistencia) `dmap=+A/-R` (churn del mapa activo) `shz=` (Hz real del topic location). Summary por
   run: `laser_noise_mean/max, filt_rej_mean, scan_hz, stale_pct, gated_pct, safer_inserts, map_adds/dels,
   obs_max, reloc_jumps, tick_ms_p95`. `summarize_runs.py` con esas columnas (runs viejas en blanco).
3. **Guardia anti-divergencia de reloc** (fix 2 del handoff, re-añadido): ≥4 saltos/10 s → STOP + aborta +
   nube postmortem `_relocdiv`. Validado offline vs run 134458: paraba a 0.9 s del inicio de la divergencia
   (los 78 saltos fueron TODOS a partir de t=273.9 s, uno por tick; 44 m→573 m de path en los últimos 26 s).
   0 falsos positivos en runs sanas. `G1_RELOCGUARD=0` desactiva; `G1_RELOC_N/WIN` ajustan.
4. **Gate DURO de visión**: al arrancar, test real frame→`/perceive`; sin YOLO verificado NO navega
   (`PERC-GATE BLOCKED`). Override consciente `G1_NOVIS=1`. Motivo: colisiones del 07-01 (164306/164456)
   fueron con c0=2.50 y perc_n=0 (visión apagada + gate v1 por yaw medido congelaba el mapa andando recto).
5. **`PERSIST_N/K` por entorno** (`G1_PERSIST_N/K`, defaults 3/2) para A/B en campo sin editar código.

### 8.2 Investigación del laser noise (2026-07-02, con fuentes)

- El Mid-360 usa **escaneo NO repetitivo** (Livox): cada frame muestrea direcciones DISTINTAS. El parpadeo
  celda-a-celda es INHERENTE al sensor, no solo ruido de marcha. ⇒ integrar barridos (persistencia/score)
  es la forma correcta de consumir un Livox. **REGLA: nunca subir PERSIST_K** (retrasa obstáculos finos
  reales que también parpadean); si el ruido gana, subir N manteniendo K=2.
- Precisión del sensor (≤2 cm @10 m, <0.15°) es sub-celda: las celdas falsas vienen de la MARCHA BÍPEDA
  (pitch/roll de cabeza + pose de reloc retrasada), confirmado por literatura de humanoides 2025.
- Sin timestamps por punto ni IMU vía WebRTC no se puede hacer deskew "de libro" (FAST-LIO): el filtrado
  temporal a posteriori es LA única familia de soluciones disponible. Nuestro score hit/miss ≈ occupancy
  grid log-odds (Thrun) y el decay sin raycasting ≈ Spatio-Temporal Voxel Layer de Nav2 — práctica estándar.
- Livox: detección NO garantizada a 0.1–1 m en superficies oscuras/pulidas/finas → la mesa LiDAR-ciega
  tiene explicación de fábrica. El Mid-360 del G1 va además montado físicamente invertido (repo deepglint).

### 8.3 Visión por color de moqueta (`g1_floorcolor.py` + `floorcolor_calib.json`, en main SIN conectar)

- Moqueta del lab azul-gris uniforme. Modelo HSV mediana+MAD (S y V discriminan; hue casi ruido en baja
  saturación). Calibrado con `crash_01_151142` (moqueta pura).
- **Validado con las 148 imágenes de crashes/**: 73% BLOQUEADO en el momento del choque (mediana
  free_center=0.01). De los 40 "libre": 15 = serie `1512xx` (obstáculo FUERA del FOV de la cámara),
  ~19 colisiones laterales (vista frontal genuinamente libre), ~6 borderline con el obstáculo YA marcado
  en rojo. **0 fallos claros del clasificador**; sombras apenas dan falsos rojos (lado conservador).
- **Umbrales validados para integrarlo**: veto con `free_center < 0.45` o `near_run ≥ 5`.
- Ve CABLES y muebles que YOLO no reconoce, a coste CPU. Es COMPLEMENTO de depth+YOLO, no sustituto.
- Hallazgo para la tesis: 15 choques repetidos con vista limpia ⇒ ningún sensor de cabeza cubre el campo
  cercano bajo ⇒ la detección de contacto (IMU) es una capability de pleno derecho para el DCE.
- Plan: tras validar la run de hoy, integrarlo en `perception_server.py` tras flag `G1_FLOORCOLOR=1`
  (default OFF) para A/B limpio con/sin color → ablación extra para el paper.
- **MULTI-MODO (2026-07-02 tarde)**: la MISMA moqueta cambia con la exposición de la cámara: oficina
  iluminada = gris lavado (S~11), pasillo oscuro = azul saturado (H105 S~91 V~86, medido en
  crash_03_162351). Con 1 modo el color VETABA el cruce de la puerta (falso bloqueado). Calibración
  final: modo1 med/mad + modo2 límites explícitos H105±6, S[35,140], V[64,120]. Validación: 98/148
  bloqueadas (66%), puerta free=0.63 ✓, mesa free=0.00 ✓.
- **LÍMITE ESTRUCTURAL descubierto**: bajo la mesa hay moqueta real en sombra → el color la ve como
  suelo (correcto cromáticamente). Los VOLADIZOS (mesa) son trabajo de depth+YOLO. REGLA DE FUSIÓN:
  obstáculo si CUALQUIER canal lo dice (unión); el color nunca resta obstáculos de otros canales.
- **DETECTOR DE PUERTA `find_door()`** (sin entrenamiento): corredor de moqueta profunda flanqueado
  por verticales BLANCAS (marco S<60 V>150) → bearing_deg del vano + width_frac. Probado en
  crash_03_162351: detecta con ambos flancos. Uso previsto: rumbo ESTABLE para DOOR-AL (el `ddir`
  del A* tiembla con el láser y hacía oscilar la alineación). Entrenar YOLO con las 10 fotos de
  iPhone (images_iphone/door): NO merece la pena (dataset mínimo + salto de dominio); si se quiere
  aprendizaje, etiquetar frames de la cámara del robot y fine-tune yolov8n en el Ubuntu.

### 8.4-bis VALIDADO EN ROBOT (run 20260702_113327, 11:33)

**PRIMERA LLEGADA LIMPIA A B**: reached en 71.8 s, 13.2 m (eficiencia 0.48), **0 colisiones,
0 saltos de reloc**, c0min=0.33. Ayer el mejor intento no llegó (0.84 m a falta, 300 s, 30.6 m, 1 col).
Diagnósticos clave: **scan_hz=2.17 / stale_pct=30.5%** → la nube refresca MÁS LENTO que el tick:
el dedup era CRÍTICO (cada barrido votaba ~1.4x sin él). filt_rej=8% (filtro suave con votación
honesta), obs_max=421 (vs 788 ayer), dmap quieto +6.5/-1.9 (residual, aceptable), gated_pct=27%
(vigilar: umbral de alarma 30%), tick_ms_p95=347. Visión activa (179 queries; la mesa entró por
depth+SAFE_R=234 inserciones, YOLO nunca dijo "table" → otro argumento pro-floorcolor).
Combo dedup+score/decay+gate-rx+SAFE_R+gate de visión: **VALIDADO**.

### 8.4-ter Run 114603 (11:46) + INTEGRACIÓN FLOORCOLOR

Run 2: reached en 142 s / 20.5 m / **1 colisión** (t=9.1 s, cajonera de madera, roce lateral derecho):
c0=0.7 (mapa decía hueco), **perc_n=0 y dets=None** — ni depth ni YOLO la vieron (sin clase para
cajoneras). El canal de COLOR la veía entera (6/16 columnas derechas a 0.0). gated_pct subió a 35.7%
(>30%: vigilar RX_GATE). scan_hz≈2.2 confirmado otra vez.

**INTEGRADO el canal de color en `perception_server.py`** tras `G1_FLOORCOLOR=1` (default OFF, A/B):
- `color_to_scan()`: por columna, fila donde acaba la moqueta continua = base del obstáculo →
  proyección al suelo → [bearing, range]; obstáculo tocando el borde inferior → clamp 0.4 m.
- Bandas no-moqueta FINAS con moqueta encima (umbral de madera de la puerta, cinta amarilla) se
  SALTAN: son marcas planas, no obstáculos (sin esto el umbral tapaba el vano con fantasmas a 0.4 m).
- FUSIÓN POR UNIÓN: el color solo añade puntos al scan; nunca resta. `door` (find_door) y `color_pts`
  van en la respuesta (el cliente actual los ignora; DOOR-AL por visión = mejora futura).
- Validado offline: cajonera 11 pts ✓ (habría evitado el choque), puerta limpia (solo base de la
  hoja) ✓, mesa 48 pts ✓, moqueta pura 0 falsos ✓.
- OJO para revisar algún día: en `depth_to_scan` la fórmula de altura con cam_pitch=-10 parece
  inflar la altura con la distancia (¿convención de signo invertida?). El canal de color usa
  abs(pitch) y es inmune. Puede explicar el perc_n bajo a 1-2 m.

**A/B**: misma run B, servidor con y sin `G1_FLOORCOLOR=1` → comparar collisions, path_m, time_s,
perc_n medio y comportamiento en puerta (runs_summary.csv). Es la ablación extra del paper.

### 8.5 CIERRE DEL DÍA 2026-07-02 (8 runs) — estado FINAL VALIDADO

Run final **130231**: reached 84 s / 13.0 m / eficiencia 0.49 / **0 colisiones / SIN modo agresivo**
(primera vez que cruza la puerta con holgura normal) / gated 43%→21%. Tres bugs de campo cerrados
HOY con mediciones:
1. **Dedup de barrido** (la nube location refresca a ~2.2 Hz < tick; sin dedup cada barrido votaba ~1.4x).
2. **NEAR_CLAMP 0.4→0.7** (los avisos cercanos del canal de color morían en el anillo NEAR_BLIND=0.6).
3. **DOOR_STRAFE_SIGN default -1**: el mapeo físico de lx es tipo gamepad (lx>0 = DERECHA), medido en
   3 runs con ambos signos (123933: 46 órdenes izq → 38 cm dcha; 125209: ambos signos consistentes;
   130231 con fix: 37/48 strafes hacia el lado libre). DOOR-CTR llevaba centrando HACIA el obstáculo.
   Verificable en vivo con STRAFE-CAL / STRAFE-CAL-RESUMEN en goto.log.
OJO al orden operativo que costó una run: en el Ubuntu **git pull ANTES de lanzar** (la 125209 corrió
con código viejo y pareció refutar el fix del strafe).

### 8.6 Bug de la JAULA (run 130524, B→A atascada) — arreglado

Con el canal de color VIVO, en la sala abarrotada de B el robot se ENJAULO solo: clamp de 0.7 m en
todo el FOV + pivotar buscando ruta = anillo de celdas sinteticas alrededor (holgura ~0.6-0.7 m EN
TODAS las direcciones, medido) → A* sellado → SEEK → mas pivoteo. Fix doble: (1) el clamp del server
solo en columnas CENTRALES (±30°) y con obstruccion ALTA (≥35% de la columna) — es para "me lo como
andando", no para clutter lateral que el laser ya ve; (2) la VISION ya no salta el gate por SAFE_R:
pasa por el score normal (+1/frame, ~1 s para entrar; mientras PIVOTA no inserta nada → jaula
imposible). Solo el laser confirmado mantiene el bypass de seguridad. Regresion offline OK
(escritorio/pared frontales siguen detectandose; vano y moqueta pura sin fantasmas).

### 8.7 Instrumentación total + capa de confianza (tarde 2026-07-02)

- **Telemetría completa** por tick en samples: canal de color (color_pts/carpet_pct/color_near/
  color_rmin/door_b), plan (carrot/goal_err/carrot_err/plan_n=0 si A* sin ruta), confianza
  (c0_hard/n_hard). Eventos al dataset: astar_fail, aggressive_on. PELÍCULA: frame cada 3 s
  (G1_FILM, tNNNs.jpg). Colisiones: pre-frames t−1/−2/−3 s + omap_near en el evento.
- **Capa de ALTA CONFIANZA (Renxi)**: hard_set = refmap confirmado | score saturado | colmap.
  HARD-GUARD tras `G1_HARDGUARD=1` (default OFF): <0.45 m de pared → lento; <0.22 m → corta avance,
  incluso en agresivo. Lo blando sigue negociable (la puerta necesita 0.13).
- **Objetos BAJOS**: ciegos para el láser por HBAND_LO=-0.5 (anti-suelo). Los cubren depth (0.10 m+)
  y el canal de moqueta (a ras de suelo). A/B pendiente: `G1_HBAND_LO=-0.7` con el stack anti-ruido.
- Overlay live: rojo 60 % (sobre muebles oscuros el 28 % desaparecía), verde 22 %.
- ORDEN DE VALIDACIÓN: ① B→A anti-jaula ② G1_HARDGUARD=1 ③ G1_HBAND_LO=-0.7. Una cosa por run.

### 8.4 Próximos pasos (en orden)

1. **Prueba goto B** en el Ubuntu (percepción ON; el gate ya lo exige). Mirar en el log, en este orden:
   `shz` (si <3 Hz el dedup era crítico), `stale_pct`, `flt` (pasillo vs puerta), `dmap` con robot parado
   (→ +0/-0), `gated_pct` (si >30% afinar `RX_GATE`), `nz`/`rel` al girar, y cerca de B que RODEE.
2. **Análisis post-run**: comparar vs runs del 07-01 con `summarize_runs.py` (columnas nuevas).
3. **Integrar floorcolor** en perception_server tras `G1_FLOORCOLOR=1` y repetir la run → comparativa.
4. Investigar la serie `1512xx` (¿qué golpeaba en x+1.20,y+0.19 invisible a la cámara?).

_Fin del traspaso. Para continuar: valida en robot los cambios de la sección 8.1 y sigue con 8.4._
