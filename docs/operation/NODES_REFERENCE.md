# NODES_REFERENCE — Tabla canónica de nodos

> Fuente de verdad sobre qué nodo existe, dónde y cuál es su estado.

## Convención de estado

- **ACTIVO**: existe y se lanza desde un launcher central.
- **PROTOTIPO INTEGRADO**: existe, se lanza, está marcado como en validación.
- **PROTOTIPO NO INTEGRADO**: existe (entry point en `setup.py`) pero ningún launcher lo arranca.
- **LEGACY PROBABLE**: existe pero su pipeline asociado no funciona contra el flujo actual.
- **PROPUESTO / NO IMPLEMENTADO**: documentado en roadmap; no hay archivo de código.
- **NO VERIFICADO**: no se pudo confirmar.

---

## Paquete `wind_tower_bringup`

| Nodo | Archivo | Lanzado por | Estado | Tópicos clave |
|---|---|---|---|---|
| `turner_node` | `turner_node.py` | `simulation.launch.py:217` | **ACTIVO** | Sub: `/turner/cmd_vel`, `/turner/joint_state`. Pub: `/turner/angle`, `/turner/angle_deg` |
| `tf_static_relay` | `tf_static_relay.py` | `simulation.launch.py:110` | **ACTIVO** | Relay TRANSIENT_LOCAL para TF estáticos |
| `dualsense_joy` | `dualsense_joy.py` | `inspection.launch.py:458` | **ACTIVO** | Pub: `/robot/joy_teleop/joy`, `/inspection/emergency_stop` |
| `ps5_teleop` | `ps5_teleop.py` | `inspection.launch.py:468` | **ACTIVO** | Pub: trayectorias UR5e, `/turner/cmd_vel` (manual), `/inspection/mission_command` |
| `cylinder_localizer` | `cylinder_localizer_node.py` | `inspection.launch.py:451` | **PROTOTIPO INTEGRADO** | Sub: `/velodyne_points`. Pub: `/robot_in_tube`, `/cylinder_fit/stats`, `/cylinder_fit/wall_points` |

**Retirado en BUILD 2026-05-12** (eliminado del repo; recuperable vía `git log`):

- `scan_gate_node.py` — sin uso en ningún launcher.
- `map_accumulator_node.py` — reemplazado conceptualmente por `cylindrical_map_node.py`.

**Retirado del launcher** `simulation.launch.py`:

- `ekf_filter_node` (paquete `robot_localization`) — redundante con el EKF interno de Clearpath.

## Paquete `wind_tower_inspection_behaviour`

| Nodo | Archivo | Lanzado por | Estado | Tópicos clave |
|---|---|---|---|---|
| `stability_monitor` | `stability_monitor_node.py` | `inspection.launch.py:479` | **ACTIVO** | Sub: IMU, odom/filtered, cylinder_fit/stats, turner/angle. Pub: `/inspection/bottom_lane_locked`, `/safe_to_scan`, `/safe_to_index_tube`, `/stability` |
| `cylindrical_map` | `cylindrical_map_node.py` | `inspection.launch.py:493` | **ACTIVO** (MVP de cobertura pasiva) | Sub: odom/filtered, turner/angle, flags estabilidad, state_text. Pub: `/inspection/cylindrical_pose`, `/coverage_status`, `/cylindrical_map_stats` |
| `state_machine` | `state_machine_node.py` | `inspection.launch.py:501` | **PROTOTIPO INTEGRADO** (MVP en validación; sin bypass/local_inspection) | Sub: odom/filtered, stability, turner/angle, flags, mission_command. Pub: `/robot/platform/cmd_vel`, `/turner/cmd_vel`, `/inspection/{state, state_text, current_lane, mission_status, autonomous_active}` |

## Paquete `wind_tower_perception`

| Nodo | Archivo | Lanzado por | Estado | Tópicos clave |
|---|---|---|---|---|
| `defect_detector` | `detector_node.py` | `perception.launch.py` | **PROTOTIPO INTEGRADO** (HoughCircles activo, backend YOLO listo a la espera de pesos) | Sub: `/inspection/camera/image_raw`. Pub: `/inspection/detections/raw` (`vision_msgs/Detection2DArray`), `/inspection/detections/text` (JSON), `/inspection/detections/image_annotated` |
| `image_capture` | `image_capture_node.py` | `perception.launch.py` | **PROTOTIPO INTEGRADO** | Sub: `/inspection/camera/image_raw`, `/inspection/detections/text`, `/inspection/cylindrical_pose`, `/inspection/state_text`. Escribe `~/wind_tower_inspections/run_*/` con `frames/*.jpg + *.json`, `detections.ndjson`, `manifest.json` |
| `defect_mapper` | `defect_mapper_node.py` | `perception.launch.py` | **PROTOTIPO INTEGRADO** | Sub: `/inspection/detections/raw`, `/inspection/camera/camera_info`, `/inspection/cylindrical_pose`, `/turner/angle`, TF `world → camera`. Pub: `/inspection/defects/cylindrical` (JSON por frame), `/inspection/defects/cumulative` (JSON acumulado clustered por proximidad) |
| `synthetic_capture` | `synthetic_capture_node.py` | `perception.launch.py` (off por defecto, `use_synthetic_capture:=true`) | **HERRAMIENTA** (solo para generar dataset YOLO) | Sub: cámara + `camera_info` + TF. Carga `defects_ground_truth.yaml` (output de `generate_synthetic_world`). Escribe dataset YOLO en `images/{train,val}/`, `labels/{train,val}/` |

Scripts (entry points en `setup.py`, **no** son nodos):

| Script | Archivo | Función |
|---|---|---|
| `generate_synthetic_world` | `scripts/generate_synthetic_world.py` | Inyecta defectos esféricos aleatorios en una copia de `wind_tower_world.sdf` y produce el YAML de ground truth |
| `train_yolo` | `scripts/train_yolo.py` | Wrapper de `ultralytics.YOLO.train` con defaults del proyecto |
| `generate_inspection_report` | `scripts/generate_inspection_report.py` | Lee `detections.ndjson` de un run, agrupa por proximidad, llama a la API de Claude y escribe `report/inspection_report.md` |

## Nodos PROPUESTOS / NO IMPLEMENTADOS

Roadmap futuro. No existen archivos correspondientes.

| Nombre propuesto | Función prevista | Estado |
|---|---|---|
| `bottom_lane_controller` | Mantener generatriz inferior | PROPUESTO |
| `tube_indexing_controller` | Indexado por `Δθ` con verificación | PROPUESTO |
| `obstacle_manager_node` | Detección/clasificación obstáculos | PROPUESTO |
| `bypass_manager_node` | Bypass conservador | PROPUESTO |
| `local_inspection_controller` | UR5e con base parada | FUTURO |
| `cylindrical_odom_node` | Odom + TF en `cyl_map` | PROPUESTO (Nav2 cilindro) |
| `cylindrical_lidar_projector` | Proyecta LiDAR al plano desplegado | PROPUESTO (Nav2 cilindro) |
| `cylindrical_cmd_vel_adapter` | Reparte cmd_vel entre base + turner | PROPUESTO (Nav2 cilindro) |

---

## Mapeo nodo ↔ entry point ↔ launcher

Evidencia primaria:

- `ros2_ws/src/wind_tower_bringup/setup.py:35-44`
- `ros2_ws/src/wind_tower_inspection_behaviour/setup.py:33-39`
- `ros2_ws/src/wind_tower_perception/setup.py:33-49`
- `ros2_ws/src/wind_tower_bringup/launch/simulation.launch.py`
- `ros2_ws/src/wind_tower_inspection_behaviour/launch/inspection.launch.py`
- `ros2_ws/src/wind_tower_perception/launch/perception.launch.py`
