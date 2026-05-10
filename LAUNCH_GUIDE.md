# Guía de arranque — Wind Tower Inspection

Esta guía arranca la simulación, las herramientas manuales actuales y el primer monitor MVP de inspección. La operación autónoma completa por calles axiales con indexado angular todavía está pendiente.

Regla metodológica vigente: durante una futura inspección autónoma, el virador no debe girar continuamente mientras el robot avanza. El giro del tubo debe hacerse solo en el estado `INDEX_TUBE`, cuando `safe_to_index_tube == true`.

## Requisitos previos (una vez por sesión)

### 1. Conectar mando PS5 DualSense por USB

En **PowerShell de Windows como administrador**:

```powershell
usbipd list
# Anota el BUSID del DualSense (aparece como "Sony..." o "DualSense Wireless...")
usbipd bind --busid 1-3         # solo la primera vez
usbipd attach --wsl --busid 1-3 # repetir en cada sesión
```

Verificar en WSL2:

```bash
ls /dev/input/event*   # debe aparecer al menos un event*
```

### 2. Activar entorno GPU + ROS 2

```bash
ai-on   # alias que activa GPU, ROS2 Jazzy y el workspace
```

El alias `ai-on` exporta:

```bash
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
export GALLIUM_DRIVER=d3d12
export PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH
source /opt/ros/jazzy/setup.bash
source ~/ROS2_wind_tower_inspection/ros2_ws/install/setup.bash
```

## Arranque de la simulación

### Terminal 1 — Simulación Gazebo + RViz

```bash
cd ~/ROS2_wind_tower_inspection/ros2_ws && ai-on
ros2 launch wind_tower_bringup simulation.launch.py
```

Espera a que aparezcan los mensajes de controladores activos:

```text
[INFO] platform_velocity_controller: Configured and activated
[INFO] arm_0_joint_trajectory_controller: Configured and activated
```

El robot aparece dentro del tubo. La GPU RTX 4070 se usa automáticamente si `ai-on` está bien configurado.

Opciones disponibles:

```bash
# Sin RViz (más ligero)
ros2 launch wind_tower_bringup simulation.launch.py rviz:=false

# Posición de spawn personalizada
ros2 launch wind_tower_bringup simulation.launch.py x:=0.0 y:=-5.0 z:=0.3
```

### Terminal 2 — Mando DualSense

```bash
cd ~/ROS2_wind_tower_inspection/ros2_ws && ai-on
ros2 run wind_tower_bringup dualsense_joy
```

### Terminal 3 — Teleop brazo + virador

```bash
cd ~/ROS2_wind_tower_inspection/ros2_ws && ai-on
ros2 run wind_tower_bringup ps5_teleop
```

## Controles del mando DualSense

| Control | Acción |
|---------|--------|
| **L2** + stick izquierdo | Mover Husky (adelante/atrás/giro) |
| **L2** + stick derecho X | Rotar base brazo UR5e (joint 0) |
| **L2** + stick derecho Y | Subir/bajar hombro UR5e (joint 1) |
| **X (Cruz)** | Girar virador + |
| **Círculo** | Solicitar `STOP` a la máquina de estados |
| **Cuadrado** | Girar virador − |
| **Triángulo** | Solicitar `START_AUTO` a la máquina de estados |

Velocidades:

- Virador: 0.15 rad/s ≈ 8.6°/s (ajustable en `ps5_teleop.py` → `TURNER_VEL`).
- Brazo: 0.05 rad/tick a 10Hz → ~0.5 rad/s máx (ajustable en `MAX_JOINT_VEL`).
- Husky: `simulation.launch.py` limita automáticamente `teleop_twist_joy_node` a 0.15 m/s y 0.08 rad/s en modo normal para pruebas de inspección.

Uso correcto en pruebas de la nueva metodología:

- Para simular una calle axial, deja el tubo parado y mueve el Husky en dirección axial.
- Para simular indexado, gira el virador en pequeños incrementos y vuelve a detenerlo.
- No mezcles giro continuo del tubo con avance axial si estás generando datos de cobertura para la estrategia nueva.
- Si lanzas `inspection.launch.py`, Triángulo inicia la misión autónoma y el teleop queda bloqueado mientras `/inspection/autonomous_active=true`.

## Verificar el virador

```bash
# Ver ángulo en tiempo real
ros2 topic echo /turner/angle_deg

# Verificar frecuencia del bridge Gazebo
ros2 topic hz /turner/joint_state

# Enviar comando manual de prueba
ros2 topic pub /turner/cmd_vel std_msgs/msg/Float64 "{data: 0.1}" --once
```

Señales relevantes:

| Topic | Uso |
|---|---|
| `/turner/joint_state` | Encoder simulado del virador |
| `/turner/angle` | `θ_tube` acumulado en radianes |
| `/turner/angle_deg` | Diagnóstico en grados |
| `/turner/cmd_vel` | Comando de velocidad angular |

En el sistema autónomo futuro, `tube_indexing_controller.py` deberá cerrar el lazo con `/turner/angle` y no depender de tiempo abierto.

## Verificar el LiDAR

```bash
# Comprobar que llegan puntos
ros2 topic hz /velodyne_points

# Ver en RViz: añadir display PointCloud2 → topic /velodyne_points
```

El LiDAR 3D se usa para geometría del cilindro, detección de obstáculos/bushings, pared, cobertura y seguridad. No sustituye al encoder del virador para `θ_tube`.

## Verificar cámara RGB de inspección

La cámara RGB simulada está montada en el TCP del UR5e (`arm_0_tool0`) mediante `platform.extras.urdf`. La iluminación controlada se simula con dos luces rasantes en el cabezal. Esto sirve para validar encuadre y pipeline de imagen; la detección visual de defectos sigue pendiente.

```bash
# Imagen RGB simulada directa desde Gazebo/ros_gz_image
ros2 topic hz /robot/sensors/inspection_camera/image

# Alias objetivo para futuros nodos de inspección; actualmente EN DESARROLLO
ros2 topic hz /inspection/camera/image_raw
ros2 topic echo /inspection/camera/camera_info --once

# Ver en RViz/rqt_image_view: usar primero /robot/sensors/inspection_camera/image
```

Si no aparecen estos topics, comprueba que `inspection_camera_bridge` está activo:

```bash
ros2 node list | grep inspection_image_bridge
ros2 node list | grep inspection_image_relay
ros2 node list | grep inspection_camera_info_bridge
ros2 node list | grep bridge
```

## Verificar IMU y EKF

`simulation.launch.py` lanza `imu_bridge` y el EKF de Clearpath publica `/robot/platform/odom/filtered`.

```bash
# IMU simulada
ros2 topic hz /robot/sensors/imu_0/data
ros2 topic echo /robot/sensors/imu_0/data --once

# EKF
ros2 topic hz /robot/platform/odom/filtered
ros2 topic echo /robot/platform/odom/filtered --once
```

La IMU es obligatoria para la nueva metodología. Debe permitir estimar roll/pitch respecto a gravedad, `α_robot` y si el eje Z del robot está alineado con la aceleración de la gravedad.

## Monitor de estabilidad MVP

Con `cylinder_localizer` corriendo para publicar `/cylinder_fit/stats`, lanza:

```bash
cd ~/ROS2_wind_tower_inspection/ros2_ws && ai-on
ros2 run wind_tower_inspection_behaviour stability_monitor
```

Verifica:

```bash
ros2 topic echo /inspection/stability
ros2 topic echo /inspection/bottom_lane_locked
ros2 topic echo /inspection/safe_to_scan
ros2 topic echo /inspection/safe_to_index_tube
```

Comportamiento validado:

- Robot parado y estable: `bottom_lane_locked=true`, `safe_to_scan=true`, `safe_to_index_tube=true`.
- Avance axial lento: `bottom_lane_locked=true`, `safe_to_scan=true`, `safe_to_index_tube=false`.

## Mapa cilíndrico MVP

Este nodo no mueve el robot ni gira el tubo. Solo registra cobertura observada y cobertura nominal en una malla `(x, θ)` usando odometría, ángulo del virador y flags de estabilidad.

```bash
cd ~/ROS2_wind_tower_inspection/ros2_ws && ai-on
ros2 run wind_tower_inspection_behaviour cylindrical_map --ros-args \
  --params-file install/wind_tower_inspection_behaviour/share/wind_tower_inspection_behaviour/config/inspection_params.yaml
```

Verifica:

```bash
ros2 topic echo /inspection/cylindrical_pose --once --full-length
ros2 topic echo /inspection/coverage_status --once --full-length
ros2 topic echo /inspection/cylindrical_map_stats --once --full-length
```

Para una prueba manual de calle axial:

- Mantén el tubo parado.
- Lanza `stability_monitor`.
- Avanza despacio con el Husky.
- La cobertura nominal solo debe crecer si `bottom_lane_locked=true` y `safe_to_scan=true`.

## Máquina de estados MVP

Este nodo sí manda movimiento. Para la primera prueba no pulses el mando mientras esté activo, porque puede competir con los comandos autónomos de la base. La base se detiene durante `INDEX_TUBE`; el giro del tubo no cuenta como cobertura nominal.

Terminales mínimos:

1. Simulación completa.
2. Launch de inspección.

Arranque recomendado sin movimiento automático:

```bash
cd ~/ROS2_wind_tower_inspection/ros2_ws && ai-on
ros2 launch wind_tower_inspection_behaviour inspection.launch.py
```

Esto levanta:

- `cylinder_localizer`
- `dualsense_joy`
- `ps5_teleop`
- `stability_monitor`
- `cylindrical_map`
- `state_machine` en `IDLE`

Durante el arranque, `stability_monitor` calibra la IMU tomando las primeras muestras como referencia de generatriz inferior. No muevas el robot durante aproximadamente el primer segundo. En `/inspection/stability` verás `imu_calibration.calibrated=true` cuando la referencia esté lista.

Con el mando:

- `Triángulo`: publica `START_AUTO` en `/inspection/mission_command`.
- `Círculo`: publica `STOP` en `/inspection/mission_command`.
- Al empezar la misión, `/inspection/autonomous_active=true`.
- Mientras `autonomous_active=true`, `dualsense_joy` publica Joy neutro y bloquea los controles manuales de Clearpath; `ps5_teleop` ignora brazo y virador manual. Círculo queda disponible como parada de misión.

Lanzamiento conservador con arranque automático:

```bash
cd ~/ROS2_wind_tower_inspection/ros2_ws && ai-on
ros2 launch wind_tower_inspection_behaviour inspection.launch.py state_machine_auto_start:=true
```

Verifica estado y comandos:

```bash
ros2 topic echo /inspection/state_text
ros2 topic echo /inspection/mission_status --once --full-length
ros2 topic echo /inspection/autonomous_active
ros2 topic echo /robot/platform/cmd_vel
ros2 topic echo /turner/cmd_vel
```

La máquina de estados considera que llega al final de calle por distancia axial relativa, no por una coordenada absoluta: guarda la posición al arrancar la calle y termina cuando `lane_progress_m >= lane_length_m`. Para depuración rápida el perfil actual de prueba usa `lane_length_m=1.0`, `axial_axis=x`, `lane_delta_theta_deg=5.0`, `turner_speed_rad_s=0.02` y `publish_rate_hz=30.0`.

Al terminar la calle entra en `WAIT_SAFE_TO_INDEX` si la base todavía no está suficientemente parada. Cuando `safe_to_index_tube=true`, pasa a `INDEX_TUBE` y publica velocidad en `/turner/cmd_vel`.

Si el eje axial de tu simulación resulta ser `y`, lanza:

```bash
ros2 launch wind_tower_inspection_behaviour inspection.launch.py \
  state_machine_axial_axis:=y
```

Para hacer una prueba corta de indexado sin recorrer 30m:

```bash
ros2 launch wind_tower_inspection_behaviour inspection.launch.py \
  state_machine_lane_length_m:=1.0 \
  state_machine_lane_delta_theta_deg:=5.0 \
  state_machine_axial_speed_mps:=0.05 \
  state_machine_turner_speed_rad_s:=0.02 \
  state_machine_publish_rate_hz:=30.0
```

Limitación actual: si el robot se desvía o pierde `safe_to_scan`, la misión entra en recuperación y realineación a bottom lane; todavía no hay bypass de obstáculos completo ni manejo avanzado de defectos.

## Prototipos LiDAR disponibles

Estos nodos existen en `wind_tower_bringup`, pero no sustituyen todavía a la arquitectura final de inspección:

```bash
ros2 run wind_tower_bringup cylinder_localizer
ros2 run wind_tower_bringup map_accumulator
```

| Nodo | Estado | Salidas |
|---|---|---|
| `cylinder_localizer` | Prototipo | `/robot_in_tube`, `/cylinder_fit/wall_points`, `/cylinder_fit/stats` |
| `map_accumulator` | Prototipo | `/map_cloud`, `/map_cloud/stats` |

Estos prototipos ayudan a aprender y validar geometría LiDAR, pero el nodo final debe ser `cylindrical_map_node.py` en `wind_tower_inspection_behaviour`, con malla `(x, θ)` y estados de cobertura.

## `inspection.launch.py`

Ya existe el launcher autónomo de misión. Debe arrancar progresivamente:

```text
stability_monitor_node
cylindrical_map_node
state_machine_node
bottom_lane_controller
tube_indexing_controller
image_capture_manager_node
report_generator_node
```

No debe arrancar Nav2 completo como dependencia base del MVP. Si se añade Nav2, debe ser como herramienta concreta y justificada, por ejemplo Collision Monitor.

## Compilar tras cambios

```bash
cd ~/ROS2_wind_tower_inspection/ros2_ws && ai-on

# Compilar todo
colcon build && source install/setup.bash

# Compilar paquetes concretos (más rápido)
colcon build --packages-select wind_tower_bringup wind_tower_simulation
source install/setup.bash
```

Para recompilar también el paquete de comportamiento de inspección:

```bash
colcon build --packages-select wind_tower_bringup wind_tower_simulation wind_tower_inspection_behaviour
source install/setup.bash
```

## Solución de problemas

**Mando no detectado**

```bash
ls /dev/input/event*   # si no aparece, reconectar USB
# En PowerShell admin:
usbipd attach --wsl --busid X-X
```

**Gazebo usa llvmpipe en vez de GPU**

```bash
glxinfo | grep renderer   # debe mostrar "D3D12 (NVIDIA...)"
# Si no: verificar que ai-on está activo
```

**Error `No module named 'apt'`**

```bash
export PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH
```

**El robot no aparece en Gazebo**

- Esperar más tiempo; el spawn tarda ~15-20s.
- Verificar que `wind_tower_bringup` y `wind_tower_simulation` están compilados.

**El tubo no gira sobre su eje (gira sobre vértice)**

- Verificar que `wind_tower_simulation` está compilado con la corrección del joint pose `(4.0, 4.0, 0)`.
- Ejecutar `colcon build --packages-select wind_tower_simulation`.

**El botón X no mueve el virador**

- Verificar mapeo de botones: `ros2 topic echo /robot/joy_teleop/joy`.
- X (Cruz) debe activar `buttons[1]`, no `buttons[0]`.
- Verificar que `ps5_teleop` está corriendo: `ros2 node list | grep teleop`.

**RTAB-Map: "Did not receive data since 5 seconds"**

```bash
# Verificar que llegan puntos LiDAR
ros2 topic hz /velodyne_points
# Si no llegan, verificar que lidar3d_bridge está activo
ros2 node list | grep bridge
```

**La cobertura parece incoherente**

- Comprueba que no estás moviendo el virador de forma continua durante una calle axial.
- Verifica `/turner/angle` y el modo de operación usado para marcar celdas.
- Durante esquiva, la cobertura debe marcarse como `bypass_mode`, no como inspección normal.

---

## Onboarding para nuevos compañeros — primer uso en WSL2

### Requisitos de sistema

| Requisito | Versión |
|---|---|
| Windows 11 + WSL2 | Ubuntu 24.04 LTS |
| ROS 2 | Jazzy |
| Gazebo | Harmonic (gz-sim 8.x) |
| GPU (opcional pero recomendado) | NVIDIA con driver ≥ 535 + Mesa D3D12 |
| usbipd-win | 4.x (para mando DualSense) |

### 1. Clonar el repositorio en WSL2

```bash
cd ~
git clone https://github.com/<org>/ROS2_wind_tower_inspection.git
cd ROS2_wind_tower_inspection
```

### 2. Instalar dependencias ROS 2

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

Si falta algún paquete de Clearpath:

```bash
sudo apt install ros-jazzy-clearpath-gz ros-jazzy-robot-localization \
     ros-jazzy-teleop-twist-joy ros-jazzy-ros-gz-sim
```

### 3. Compilar el workspace

```bash
cd ~/ROS2_wind_tower_inspection/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

> `--symlink-install` hace que los cambios en ficheros Python (`.py`, `.yaml`, `.launch.py`) sean
> visibles **sin recompilar**. Solo hay que recompilar si cambias un paquete C++ o añades nuevos
> ejecutables en `setup.py`.

Si solo cambias código Python de un paquete:

```bash
# No hace falta recompilar; el symlink ya apunta al fuente.
# Solo relanza el nodo.
```

Si añades un nuevo nodo Python (entrada en `setup.py`) o cambias C++:

```bash
colcon build --packages-select <nombre_paquete>
```

### 4. Activar el entorno en cada terminal nueva

```bash
source /opt/ros/jazzy/setup.bash
source ~/ROS2_wind_tower_inspection/ros2_ws/install/setup.bash
```

Para no repetirlo, añade estas líneas a `~/.bashrc` o crea el alias `ai-on`:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/ROS2_wind_tower_inspection/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### 5. Activar GPU NVIDIA en WSL2 (opcional, necesario para Gazebo fluido)

```bash
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
export GALLIUM_DRIVER=d3d12
export PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH
```

Añádelo al alias `ai-on` en `~/.bashrc` para no repetirlo.

### 6. Conectar mando DualSense desde Windows

En **PowerShell de Windows (administrador)**:

```powershell
# Primera vez: listar y enlazar
usbipd list
usbipd bind --busid <BUSID>   # p.ej. 2-4

# Cada sesión: adjuntar a WSL
usbipd attach --wsl --busid <BUSID>
```

Verificar en WSL2:

```bash
ls /dev/input/event*    # debe aparecer event0 o similar
python3 -c "import evdev; print([d.name for d in map(evdev.InputDevice, evdev.list_devices())])"
```

### 7. Workflow de desarrollo en branch

```bash
# Crear tu branch desde main
git checkout main && git pull
git checkout -b feature/mi-feature

# Trabajar, compilar y probar
colcon build --packages-select <paquete>
source install/setup.bash
ros2 launch wind_tower_bringup simulation.launch.py

# Commit y push
git add <ficheros>
git commit -m "feat: descripción"
git push -u origin feature/mi-feature
```

Ver `ARCHITECTURE.md` para entender la estructura de nodos, topics y flujo de datos antes de empezar.

---

## Estrategia de branches del equipo

```
main
├── feature/inspection-behaviour   — máquina de estados, control axial, indexado
├── feature/manipulation           — gripper + UR5e picking de objetos en el tubo
├── feature/simulation-objects     — cubos/cilindros en el mundo Gazebo (SDF)
└── feature/fixed-camera-pipeline  — cámara fija + detección visual de objetos
```

**Reglas:**
- Cada branch parte de `main` actualizado.
- PR a `main` solo cuando la feature es funcional y no rompe `simulation.launch.py`.
- Los ficheros de simulación (`wind_tower_world.sdf`, `robot.yaml`) son propiedad de `main`; coordinar antes de modificarlos desde varias branches.
- El paquete `wind_tower_inspection_behaviour` es de `feature/inspection-behaviour`; los demás no deben modificarlo directamente.
- Añadir objetos al mundo SDF se hace en `feature/simulation-objects` y se mergea antes de que `feature/manipulation` empiece a referenciarlos.
