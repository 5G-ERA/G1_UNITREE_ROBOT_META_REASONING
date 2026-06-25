# Guía: Unitree G1 + ROS2 Humble para navegación

Guía práctica de extremo a extremo: encender el robot, configurar la red, instalar ROS2 Humble + `unitree_ros2`, y montar el stack de navegación.

> Aviso de seguridad: el G1 es un humanoide bípedo. Para las primeras pruebas usa un soporte/grúa de suspensión o espacio amplio despejado. Ten siempre el mando a mano para entrar en `damping` (caída controlada) ante cualquier fallo.

---

## 0. Topología de red del G1

El G1 trae un switch interno. Direcciones por defecto en la subred `192.168.123.x/24`:

| Dispositivo | IP |
|---|---|
| PC de control de movimiento (RockChip / PC1) | `192.168.123.161` |
| PC de desarrollo interno (Jetson Orin / PC2) | `192.168.123.164` |
| LiDAR Livox Mid-360 | `192.168.123.120` |
| **Tu PC de desarrollo** (recomendado) | `192.168.123.99` |

Tu portátil va en `192.168.123.99`, máscara `255.255.255.0`.

---

## 1. Encendido

1. Batería cargada e insertada y enclavada.
2. Coloca el robot con **los brazos rectos hacia abajo** antes de encender (una postura distinta afecta al control por ROS2 al arrancar).
3. Botón de encendido: **una pulsación corta + una pulsación larga** (mantener) hasta que arranque. Espera a que los ventiladores y los motores entren en estado de fijación (`damping`/`lock`).
4. Con el mando: secuencia para llevarlo a posición de pie y a modo locomoción (típicamente `L2+A` → `L2+↑` → `Start`; confirma en el manual de tu unidad, varía por firmware).
5. Si está suspendido en grúa, comprueba que toca el suelo correctamente antes de pasar a locomoción.

---

## 2. Instalar ROS2 Humble (Ubuntu 22.04) en tu máquina

`unitree_ros2` está probado y recomendado en **Ubuntu 22.04 + Humble**.

```bash
# Locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Repos ROS2
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Instalar
sudo apt update
sudo apt install -y ros-humble-desktop ros-dev-tools
```

> Alternativas si no tienes Ubuntu 22.04 nativo: el repo incluye `.devcontainer` (Dockerfile) y es usable vía Dev Container de VSCode o Codespaces.

---

## 3. Instalar el paquete `unitree_ros2`

Unitree SDK2 comunica sobre **CycloneDDS**, que también es un RMW de ROS2 → los mensajes ROS2 hablan directamente con el robot sin envolver el SDK.

```bash
sudo apt install -y ros-humble-rmw-cyclonedds-cpp ros-humble-rosidl-generator-dds-idl libyaml-cpp-dev

git clone https://github.com/unitreerobotics/unitree_ros2
cd unitree_ros2/cyclonedds_ws/src
git clone https://github.com/ros2/rmw_cyclonedds -b humble
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
cd ..
colcon build --packages-select cyclonedds   # en Humble este paso suele poder omitirse
source /opt/ros/humble/setup.bash
colcon build   # compila unitree_go, unitree_api, unitree_hg, etc.
```

> Importante: al compilar `cyclonedds` el entorno de ROS2 **no** debe estar sourceado. Si tienes `source /opt/ros/humble/setup.bash` en tu `~/.bashrc`, coméntalo durante ese paso. Después ya lo sourceas para compilar los paquetes unitree.

---

## 4. Configurar la conexión

### 4.1 IP de tu PC
Conecta el cable Ethernet al switch del G1. Identifica la interfaz:

```bash
ifconfig   # p.ej. enp3s0
```

Pon esa interfaz en modo manual: dirección `192.168.123.99`, máscara `255.255.255.0`. Verifica:

```bash
ping 192.168.123.161   # PC de control de movimiento
```

### 4.2 Apuntar CycloneDDS a tu interfaz
Edita `~/unitree_ros2/setup.sh` y cambia el nombre de la interfaz (`enp3s0` → la tuya):

```bash
#!/bin/bash
source /opt/ros/humble/setup.bash
source $HOME/unitree_ros2/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
    <NetworkInterface name="enp3s0" priority="default" multicast="default" />
</Interfaces></General></Domain></CycloneDDS>'
```

Para trabajar **sin robot** (simulación/local) usa `setup_local.sh` (interfaz `lo`).

### 4.3 Probar la conexión
Reinicia el PC (recomendado tras configurar la red) y:

```bash
source ~/unitree_ros2/setup.sh
ros2 topic list
```

Deberías ver los topics del robot. Para el G1 (serie HG) lee el estado con:

```bash
ros2 topic echo /lowstate           # estado de motores/IMU/batería (read_low_state_hg)
ros2 topic echo /wirelesscontroller # estado del mando
```

---

## 5. Control de movimiento del G1

El G1 (familia humanoide HG) **no usa la API de `sport_mode` de los cuadrúpedos**. El control de alto nivel se hace con el **Loco Client** del `unitree_sdk2` / `unitree_sdk2_python`:

- **Alto nivel (locomoción)**: caminar/equilibrio mediante comandos de velocidad. La interfaz clave para navegación es `Move(vx, vy, vyaw)` — exactamente lo que necesitas para enchufar un `cmd_vel`.
- **Bajo nivel**: control articular (posición/velocidad/par) vía topic `/lowcmd` con `unitree_hg::msg::LowCmd` (ejemplos `g1_low_level_example`).

Instala el SDK Python (cómodo para prototipar dado que usas Python):

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2_python
cd unitree_sdk2_python
pip install -e .
```

Ejemplo de alto nivel (esquema): inicializas el canal DDS en la interfaz de red, creas el `LocoClient`, y envías velocidades:

```python
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

ChannelFactoryInitialize(0, "enp3s0")   # tu interfaz
client = LocoClient()
client.Init(); client.SetTimeout(10.0)

client.Move(0.3, 0.0, 0.0)   # vx=0.3 m/s adelante
# ...
client.StopMove()
```

> Verifica los nombres exactos de método/clase contra tu versión del SDK; la API de `g1_loco_client` evoluciona entre releases.

---

## 6. Stack de navegación (SLAM + Nav2)

Esquema típico para navegación autónoma del G1:

```
Livox Mid-360 ──► livox_ros_driver2 ──► PointCloud2 + IMU
        │
        ▼
  SLAM / Odometría (FAST-LIO2 o Point-LIO)  ──►  /odom + mapa
        │
        ▼
        Nav2 (planner + controller)  ──►  /cmd_vel
        │
        ▼
  Puente cmd_vel → LocoClient.Move(vx, vy, vyaw)
```

Componentes:

1. **Driver del LiDAR**: `livox_ros_driver2`. En el G1 el **Mid-360 va montado boca abajo**, así que necesitas un driver con la extrínseca/IMU corregidas (hay forks específicos para G1). Publica la nube de puntos e IMU.
2. **SLAM / localización**:
   - Mapeo: `FAST-LIO2` o `Point-LIO` (LIO basado en LiDAR-IMU).
   - Localización sobre mapa previo: `FAST_LIO_LOCALIZATION_HUMANOID` (pensado para G1) evita la deriva acumulada usando un mapa offline.
3. **Nav2**: configura `costmaps` (footprint del humanoide ~ círculo de radio del torso), planificador global y controlador local que emita `/cmd_vel`. Existe un issue abierto en `ros-navigation/navigation2` con configuraciones mínimas recomendadas para G1/GO2/B2.
4. **Puente cmd_vel → robot**: un nodo que suscribe `/cmd_vel` (Twist) y llama a `LocoClient.Move(linear.x, linear.y, angular.z)`.

Proyectos de referencia que ya integran esto para el G1:

- **`hucebot/g1pilot`** — paquete ROS2 con driver Livox, locomoción y stack de navegación.
- **`deepglint/FAST_LIO_LOCALIZATION_HUMANOID`** — localización LiDAR específica para humanoides tipo G1.
- Driver de Unitree de la comunidad documentado en **docs.quadruped.de** (G1 ROS2 driver) con MOLA para odometría/planificación.

---

## 7. Orden recomendado de puesta en marcha

1. PC con Humble + `unitree_ros2` compilado y `setup.sh` apuntando a tu interfaz.
2. PC en `192.168.123.99`, `ping 192.168.123.161` OK.
3. Encender G1 (brazos abajo), pasar a modo locomoción con el mando.
4. `ros2 topic list` y `echo /lowstate` para confirmar comunicación.
5. Probar movimiento de alto nivel con `LocoClient.Move` a baja velocidad (con grúa).
6. Lanzar driver Livox → comprobar nube en RViz2 (`Fixed Frame` correcto).
7. Lanzar SLAM/LIO → ver `/odom` y el mapa.
8. Lanzar Nav2 + puente `cmd_vel` → enviar un goal en RViz2.

---

## Fuentes

- [unitree_ros2 (GitHub, README)](https://github.com/unitreerobotics/unitree_ros2/blob/master/README.md)
- [G1 SDK Development Guide (Unitree)](https://support.unitree.com/home/en/G1_developer)
- [G1 ROS2 driver / pre-requisitos (quadruped.de)](https://docs.quadruped.de/projects/g1/html/g1_ros2_driver.html)
- [SLAM and Navigation Services Interface (Unitree)](https://support.unitree.com/home/en/developer/SLAM%20and%20Navigation_service)
- [Unitree G1 Startup Guide (RoboStore)](https://robostore.com/blogs/news/unitree-g1-startup-guide-essential-steps-for-setup-calibration-and-control)
- [hucebot/g1pilot (GitHub)](https://github.com/hucebot/g1pilot)
- [deepglint/FAST_LIO_LOCALIZATION_HUMANOID (GitHub)](https://github.com/deepglint/FAST_LIO_LOCALIZATION_HUMANOID)
- [Nav2 config mínima para G1/GO2/B2 (issue #5512)](https://github.com/ros-navigation/navigation2/issues/5512)
