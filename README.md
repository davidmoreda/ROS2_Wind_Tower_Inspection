# ROS2 Wind Tower Inspection

Sistema ROS 2 Jazzy + Gazebo Harmonic para inspección del interior de un tramo de torre eólica (cilindro horizontal Ø8 m × 30 m) con Husky A200, brazo UR5e, virador del tubo, IMU y LiDAR Velodyne VLP-16.

Proyecto en fase de **simulación y MVP**. Hay una base funcional de simulación + autonomía limitada por calles axiales. Nav2 y la navegación sobre cilindro desplegado **todavía no están implementados**; viven en arquitectura objetivo.

---

## 1. Estado actual

| Componente | Estado | Evidencia |
|---|---|---|
| Mundo Gazebo (nave + tubo rotante) | **IMPLEMENTADO** | `ros2_ws/src/wind_tower_simulation/worlds/wind_tower_world.sdf` |
| Spawn Husky + UR5e + VLP-16 + IMU | **IMPLEMENTADO** | `simulation.launch.py` + `clearpath_gz/robot_spawn` |
| Bridges Gazebo↔ROS (LiDAR, IMU, virador, cámara) | **IMPLEMENTADO** | `simulation.launch.py:87-198` |
| Teleop DualSense PS5 (Husky + UR5e + virador) | **IMPLEMENTADO** | `dualsense_joy`, `ps5_teleop` |
| Encoder virador → `/turner/angle` | **IMPLEMENTADO** | `turner_node` |
| Monitor de estabilidad (`bottom_lane_locked`, `safe_to_scan`, `safe_to_index_tube`) | **IMPLEMENTADO** | `stability_monitor_node.py` |
| Localizador de cilindro (LiDAR fit) | **IMPLEMENTADO PARCIALMENTE** | `cylinder_localizer_node.py` (prototipo) |
| Inspección autónoma por calles axiales con indexado angular (MVP) | **IMPLEMENTADO PARCIALMENTE** | `state_machine_node.py` (sin bypass / local_inspection / report aún) |
| Cobertura pasiva en malla `(x, θ)` | **IMPLEMENTADO** | `cylindrical_map_node.py` |
| **Navegación Nav2 sobre cilindro desplegado** | **PROPUESTO / NO IMPLEMENTADO** | `docs/architecture/NAV2_CYLINDRICAL_NAVIGATION.md` |
| Captura de imágenes por distancia + metadatos | **PROPUESTO / NO IMPLEMENTADO** | — |
| Generación de informe de inspección | **PROPUESTO / NO IMPLEMENTADO** | — |
| SLAM (`slam_toolbox`) | **RETIRADO** | Eliminado en BUILD 2026-05-12 (recuperable vía `git log`) |
| RTAB-Map | **AUXILIAR / experimental** | `rtabmap.launch.py` (loop closure desactivado por simetría del cilindro) |

Para detalle completo ver:

- [`docs/architecture/CURRENT_ARCHITECTURE.md`](docs/architecture/CURRENT_ARCHITECTURE.md) — qué existe hoy.
- [`docs/architecture/TARGET_ARCHITECTURE.md`](docs/architecture/TARGET_ARCHITECTURE.md) — hacia dónde vamos.
- [`docs/audit/BUILD_INPUT.md`](docs/audit/BUILD_INPUT.md) — auditoría completa con evidencia.

---

## 2. Paquetes

| Paquete | Tipo build | Propósito |
|---|---|---|
| `wind_tower_description` | ament_cmake | Mesh STL del tubo + URDF del cabezal cámara/luces |
| `wind_tower_simulation` | ament_cmake | Mundo Gazebo `wind_tower_world.sdf` |
| `wind_tower_bringup` | ament_python | Launchers, bridges, teleop, virador, utilidades LiDAR |
| `wind_tower_inspection_behaviour` | ament_python | Lógica autónoma: state_machine, stability_monitor, cylindrical_map |
| `gz_ros2_control` (fork) | upstream | Fix WSL2 null-pointer crítico para Gazebo |

---

## 3. Launchers principales

| Launcher | Paquete | Estado | Comando |
|---|---|---|---|
| `simulation.launch.py` | `wind_tower_bringup` | **CENTRAL** | `ros2 launch wind_tower_bringup simulation.launch.py` |
| `inspection.launch.py` | `wind_tower_inspection_behaviour` | **CENTRAL** | `ros2 launch wind_tower_inspection_behaviour inspection.launch.py` |
| `rtabmap.launch.py` | `wind_tower_bringup` | AUXILIAR | (experimentos LiDAR puros) |

Detalle en [`docs/operation/LAUNCHERS_REFERENCE.md`](docs/operation/LAUNCHERS_REFERENCE.md).

---

## 4. Cómo arrancar

Pasos completos en [`docs/operation/HOW_TO_LAUNCH.md`](docs/operation/HOW_TO_LAUNCH.md). Versión mínima:

```bash
# Una vez por sesión
cd ~/ROS2_wind_tower_inspection/ros2_ws && ai-on
colcon build --packages-select wind_tower_simulation wind_tower_description wind_tower_bringup wind_tower_inspection_behaviour
source install/setup.bash

# Terminal 1 — simulación + bridges
ros2 launch wind_tower_bringup simulation.launch.py

# Terminal 2 — misión de inspección (queda en IDLE por defecto)
ros2 launch wind_tower_inspection_behaviour inspection.launch.py
# Pulsar Triángulo en el DualSense → START_AUTO
```

---

## 5. Concepto de inspección

Metodología vigente: **inspección por calles axiales discretas con indexado angular del tubo** (NO helicoidal).

1. Tubo parado en `θ_tube = θ_i`.
2. Robot alineado con la generatriz inferior.
3. Avance axial por una "calle" (lane).
4. Final de calle → `INDEX_TUBE` gira el virador `Δθ`.
5. Robot vuelve a la generatriz inferior y empieza la siguiente calle.
6. Repetir hasta acumular `360° + solape`.

Cobertura verificable sobre malla `(x, θ)`. Detalle en `docs/architecture/CURRENT_ARCHITECTURE.md` §8 y en `PROJECT_PLAN.md` (roadmap completo).

---

## 6. Flujo de desarrollo

Guía completa: [`docs/development/DEVELOPMENT_GUIDE.md`](docs/development/DEVELOPMENT_GUIDE.md).

Ramas mínimas:

```bash
git checkout -b feat/<descripcion>      # nueva funcionalidad
git checkout -b fix/<descripcion>       # bug fix
git checkout -b docs/<descripcion>      # documentación
git checkout -b exp/<descripcion>       # experimento aislado
```

Antes de mergear a `main`:

- `colcon build` limpio.
- `simulation.launch.py` + `inspection.launch.py` arrancan sin errores.
- Cualquier nodo/launcher nuevo está documentado en `docs/operation/`.

---

## 7. Documentación

| Tema | Archivo |
|---|---|
| Cómo arrancar | [`docs/operation/HOW_TO_LAUNCH.md`](docs/operation/HOW_TO_LAUNCH.md) |
| Tabla de launchers con estado | [`docs/operation/LAUNCHERS_REFERENCE.md`](docs/operation/LAUNCHERS_REFERENCE.md) |
| Tabla de nodos con estado | [`docs/operation/NODES_REFERENCE.md`](docs/operation/NODES_REFERENCE.md) |
| Topics y frames TF reales | [`docs/operation/TOPICS_AND_FRAMES.md`](docs/operation/TOPICS_AND_FRAMES.md) |
| Arquitectura actual (lo que existe) | [`docs/architecture/CURRENT_ARCHITECTURE.md`](docs/architecture/CURRENT_ARCHITECTURE.md) |
| Arquitectura objetivo (propuesta) | [`docs/architecture/TARGET_ARCHITECTURE.md`](docs/architecture/TARGET_ARCHITECTURE.md) |
| Nav2 sobre cilindro desplegado (técnico) | [`docs/architecture/NAV2_CYLINDRICAL_NAVIGATION.md`](docs/architecture/NAV2_CYLINDRICAL_NAVIGATION.md) |
| Guía de desarrollo | [`docs/development/DEVELOPMENT_GUIDE.md`](docs/development/DEVELOPMENT_GUIDE.md) |
| Auditoría completa | [`docs/audit/BUILD_INPUT.md`](docs/audit/BUILD_INPUT.md) |
| Resumen de la sesión BUILD | [`docs/audit/BUILD_SUMMARY.md`](docs/audit/BUILD_SUMMARY.md) |

---

## 8. Advertencias importantes

- **SLAM retirado**: `slam.launch.py` + `slam_toolbox.yaml` eliminados (dependían de `/scan` LaserScan que el pipeline activo no produce). Recuperables vía `git log`.
- **`rtabmap.launch.py` es AUXILIAR**: experimentos LiDAR puros, no fuente de `θ_tube` (eso viene del encoder del virador, decisión arquitectónica por simetría del cilindro).
- **`scan_gate` y `map_accumulator`** retirados: eran prototipos sin uso en ningún launcher.
- **EKF propio retirado**: `simulation.launch.py` ya no lanza `ekf_filter_node`. La odometría real es `/robot/platform/odom/filtered` (EKF interno de Clearpath).
- **WSL2 + RTX 4070**: requiere `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA` y `GALLIUM_DRIVER=d3d12`. Alias `ai-on` los exporta.
- **`gz_ros2_control` es un fork**: sin el fix WSL2 hay segfault al arrancar Gazebo. No sustituir por upstream.
- **Mensajes custom (`BottomLaneState.msg`, etc.)** son PROPUESTOS / NO IMPLEMENTADOS. Hoy todo viaja como `std_msgs/String` JSON.

---

## 9. Soporte

- Para depurar la misión autónoma: `python3 tools/debug/capture_inspection_debug.py --duration 90`.
- Para auditar el repo de nuevo: regenerar `docs/audit/` ejecutando el flujo PLAN.
