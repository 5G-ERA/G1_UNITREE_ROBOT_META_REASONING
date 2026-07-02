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

### 8.4 Próximos pasos (en orden)

1. **Prueba goto B** en el Ubuntu (percepción ON; el gate ya lo exige). Mirar en el log, en este orden:
   `shz` (si <3 Hz el dedup era crítico), `stale_pct`, `flt` (pasillo vs puerta), `dmap` con robot parado
   (→ +0/-0), `gated_pct` (si >30% afinar `RX_GATE`), `nz`/`rel` al girar, y cerca de B que RODEE.
2. **Análisis post-run**: comparar vs runs del 07-01 con `summarize_runs.py` (columnas nuevas).
3. **Integrar floorcolor** en perception_server tras `G1_FLOORCOLOR=1` y repetir la run → comparativa.
4. Investigar la serie `1512xx` (¿qué golpeaba en x+1.20,y+0.19 invisible a la cámara?).

_Fin del traspaso. Para continuar: valida en robot los cambios de la sección 8.1 y sigue con 8.4._
