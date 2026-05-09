# PROJECT STATE — Wind Tower Inspection

Última actualización: 2026-05-09

## Entorno

| Componente | Versión / Detalle |
|-----------|------------------|
| OS | Ubuntu 24.04 LTS (WSL2) |
| ROS 2 | Jazzy |
| Gazebo | Harmonic (gz sim v8.x) |
| GPU | RTX 4070 Laptop — Mesa D3D12 (`GALLIUM_DRIVER=d3d12`) |
| Workspace | `~/ROS2_wind_tower_inspection/ros2_ws` |

## Cambio metodológico vigente

La metodología de referencia ya no es una pasada helicoidal continua. El método objetivo es:

**Inspección por calles axiales discretas con indexado angular del tubo.**

Esto implica:

| Antes | Ahora |
|---|---|
| Virador girando continuamente mientras el robot avanza | Tubo parado durante cada calle axial |
| Cobertura inferida por hélice ideal | Cobertura verificable en malla `(x, θ)` |
| Control principal de velocidades helicoidales | Máquina de estados + estabilidad + indexado |
| ICP potencialmente tentador para rotación | `θ_tubo` viene del encoder del virador |
| Esquiva como detalle secundario | Esquiva con modo especial y retorno obligatorio |

No debe documentarse “100% garantizado” sin métrica de cobertura y evidencias por celda.

## Estado de paquetes

### `wind_tower_description` (ament_cmake) — FUNCIONAL

- `meshes/TRAMO_TORRE.STL` — tubo Ø8m × 30m, en mm, eje Z axial.
- Centro sección STL: X=4m, Y=4m en local; joint pose corregida.

### `wind_tower_simulation` (ament_cmake) — FUNCIONAL

- `worlds/wind_tower_world.sdf` — nave 30×60×18m + tubo dinámico rotante.
- Tubo: `<static>false</static>`, joint revolute `turner_joint`.
- Joint pose en frame `tube_link`: `(4.0, 4.0, 0)` para eje por centro del cilindro.
- Plugins: JointController (`cmd_vel`) + JointStatePublisher.
- Parámetros virador: damping=50.0, friction=10.0.

### `wind_tower_bringup` (ament_python) — FUNCIONAL / EN VALIDACIÓN

Nodos instalados:

| Nodo | Estado | Función |
|---|---|---|
| `dualsense_joy.py` | Funcional | Driver PS5 vía evdev |
| `ps5_teleop.py` | Funcional | Teleop brazo UR5e + control virador |
| `turner_node.py` | Funcional | Publica `/turner/angle` y `/turner/angle_deg` a 20Hz |
| `tf_static_relay.py` | Funcional | Relay TRANSIENT_LOCAL para TF estáticos |
| `scan_gate_node.py` | Disponible | Filtro sectorial LaserScan |
| `cylinder_localizer_node.py` | Prototipo | Localización en tubo por ajuste de cilindro al LiDAR |
| `map_accumulator_node.py` | Prototipo | Acumulador de nube en frame canónico del tubo |

Launchers:

| Launcher | Estado | Contenido |
|---|---|---|
| `simulation.launch.py` | Funcional | Gazebo, robot_spawn, bridges, TF relays, `turner_node`, bridge IMU, EKF y límites teleop de inspección |
| `rtabmap.launch.py` | Secundario | RTAB-Map con loop closure desactivado |
| `slam.launch.py` | No usado actualmente | SLAM Toolbox |

Config:

- `robot.yaml` — Husky A200 + UR5e + Velodyne VLP-16 + IMU declarada.
- Clearpath publica odometría filtrada en `/robot/platform/odom/filtered`; en las pruebas actuales `/robot/ekf_node` usa `/robot/platform/odom`.
- Symlink desde `~/clearpath/robot.yaml`.

### `wind_tower_inspection_behaviour` — MVP INICIAL

Paquete creado para la lógica autónoma de inspección por calles axiales.

- `stability_monitor_node.py` — implementado y validado como MVP; publica `bottom_lane_locked`, `safe_to_scan`, `safe_to_index_tube` y resumen `/inspection/stability`.
- `inspection_state_machine_node.py` — pendiente.
- `bottom_lane_controller.py`
- `tube_indexing_controller.py`
- `cylindrical_map_node.py`
- `bushing_detector_node.py`
- `local_inspection_controller.py`
- `anomaly_detector_node.py`
- `report_generator_node.py`

El antiguo `coverage_controller` helicoidal no debe implementarse como controlador de pasada continua. Si se mantiene el concepto, debe ser `coverage_manager` de malla `(x, θ)`.

## Funcionando y verificado

- [x] Simulación Gazebo arranca completa con robot dentro del tubo.
- [x] Velodyne VLP-16 publica `/velodyne_points` (`PointCloud2`, 20Hz según documentación previa).
- [x] Virador rota el tubo correctamente sobre su eje axial.
- [x] `/turner/angle_deg` refleja el ángulo real desde Gazebo.
- [x] Botón X del mando gira virador +.
- [x] Brazo UR5e controlable con stick derecho + L2.
- [x] Husky controlable con stick izquierdo + L2.

## Configurado pero pendiente de validar

- [ ] Botón Cuadrado del mando para virador − figura como pendiente de verificación en el mapeo.
- [ ] IMU simulada: `robot.yaml`, bridge y EKF están configurados, pero falta registrar prueba de `/robot/sensors/imu_0/data`.
- [x] Odometría filtrada Clearpath disponible en `/robot/platform/odom/filtered` (~43Hz en prueba).
- [ ] `cylinder_localizer_node.py`: existe como prototipo, falta validar robustez con movimiento real del robot.
- [ ] `map_accumulator_node.py`: existe como prototipo de nube acumulada, no sustituye todavía a `cylindrical_map_node.py`.
- [ ] Cámara RGB industrial e iluminación controlada: pendientes.
- [ ] Máquina de estados de inspección: pendiente.
- [x] `stability_monitor_node.py`: MVP validado en simulación.
- [x] Teleop Clearpath limitado automáticamente por `simulation.launch.py` a 0.15 m/s normal y 0.30 m/s turbo para pruebas de inspección.

## Mapeo de botones PS5 DualSense verificado

```text
axes[2]    = stick derecho X → shoulder_pan brazo
axes[3]    = stick derecho Y → shoulder_lift brazo
axes[4]    = L2 (-1 sin pulsar, +1 pulsado) → deadman brazo
buttons[1] = Cruz (X)      → virador +   ← VERIFICADO
buttons[3] = Cuadrado      → virador -   ← PENDIENTE verificar
```

## Sensores y señales clave

| Señal | Estado | Uso en metodología nueva |
|---|---|---|
| `/turner/angle` | Funcional | `θ_tubo` acumulado para indexado y malla |
| `/turner/angle_deg` | Funcional | Diagnóstico humano |
| `/turner/joint_state` | Funcional | Encoder simulado del virador |
| `/turner/cmd_vel` | Funcional | Comando de giro; futuro uso solo en `INDEX_TUBE` |
| `/velodyne_points` | Funcional | Geometría, obstáculos, bushings, seguridad |
| `/robot/platform/odom` | Configurado | Fuente de `x` para EKF |
| `/robot/sensors/imu_0/data` | Configurado, validar | Gravedad, roll/pitch, `α_robot` |
| `/robot/platform/odom/filtered` | Verificado | Odometría filtrada Clearpath para `x` y velocidad |
| `/inspection/bottom_lane_locked` | Verificado | True si IMU y geometría local cumplen umbrales |
| `/inspection/safe_to_scan` | Verificado | True durante avance axial lento y estable |
| `/inspection/safe_to_index_tube` | Verificado | True solo con robot estable y prácticamente parado |
| Cámara RGB | Pendiente | Inspección superficial |
| Iluminación controlada | Pendiente | Repetibilidad visual |

## Reglas técnicas de operación objetivo

Durante `AXIAL_SCAN`:

- `bottom_lane_locked == true`.
- `safe_to_scan == true`.
- Roll/pitch dentro de umbral.
- Heading alineado con eje axial del tubo.
- El tubo no gira.

Durante `BYPASS_OBSTACLE`:

- Se permite `α_robot` distinto de cero.
- Se reduce velocidad.
- No se marca cobertura como inspección normal.
- Se registra modo especial.
- Se obliga a `RETURN_TO_BOTTOM_LANE`.

Durante `INDEX_TUBE`:

- `safe_to_index_tube == true`.
- El virador gira `Δθ_real`.
- Se verifica de nuevo `bottom_lane_locked`.
- Solo entonces se inicia la siguiente calle.

## Fixes críticos permanentes

### `gz_ros2_control` — fix WSL2

- Archivo: `ros2_ws/src/gz_ros2_control/gz_ros2_control/src/gz_ros2_control_plugin.cpp`.
- Bug: null-pointer dereference en destructor.
- Fix: null checks con `RCLCPP_WARN` y `continue`.
- Sin este fix: segfault al arrancar Gazebo.

### Clearpath generator

- `parent: top_plate_link` para el LiDAR.
- No usar `top_plate_default_mount`; no existe en PACS.
- `generate:=true` por defecto; no usar `generate:=false` porque `bool('false') == True` en Python.
- El xacro generado va a `/home/dmore/clearpath/robot.urdf.xacro`; no está en git.

### LiDAR bridge

- Topic Gazebo correcto: `/robot/sensors/lidar3d_0/scan/points` (`PointCloudPacked`).
- Topic ROS remapeado: `/velodyne_points`.
- No usar `/scan` para VLP-16 3D; `/scan` es `LaserScan`.

### Tubo — simetría cilíndrica

- ICP no puede distinguir de forma fiable rotación pura en una sección circular.
- `θ_tubo` debe venir del encoder del virador.
- RTAB-Map debe mantenerse como herramienta secundaria; no fuente primaria de rotación.

## Variables de entorno (`ai-on`)

```bash
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
export GALLIUM_DRIVER=d3d12
export PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH
source /opt/ros/jazzy/setup.bash
source ~/ROS2_wind_tower_inspection/ros2_ws/install/setup.bash
```

## Próximos pasos técnicos

1. Implementar `inspection_state_machine_node.py` con estados discretos mínimos.
2. Implementar `tube_indexing_controller.py` usando `/turner/angle` como realimentación.
3. Convertir el prototipo de nube acumulada en `cylindrical_map_node.py` con celdas `(x, θ)` y estados de cobertura.
4. Añadir cámara RGB industrial e iluminación controlada al modelo.

## Controladores ROS 2 activos

```text
/robot/platform_velocity_controller      ← Husky diff drive
/robot/joint_state_broadcaster           ← estados joints
/robot/arm_0_joint_trajectory_controller ← brazo UR5e
```
