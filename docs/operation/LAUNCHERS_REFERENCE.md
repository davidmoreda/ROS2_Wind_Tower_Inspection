# LAUNCHERS_REFERENCE — Tabla canónica de launchers

> Fuente de verdad sobre qué launcher hace qué. Si un launcher no figura aquí o tiene estado distinto, esta tabla manda.

## Convención de estado

- **CENTRAL**: parte del flujo MVP activo, debe lanzarse en operación normal.
- **AUXILIAR**: experimental, se lanza para pruebas comparativas, no es parte del MVP.
- **DEMO / TEST**: demo upstream o ejemplo.
- **LEGACY PROBABLE**: presente pero no funcional contra el pipeline actual; pendiente de mover o eliminar tras confirmación.
- **NO VERIFICADO**: no se pudo confirmar el estado.

---

## Tabla

| Launcher | Paquete | Qué arranca | Configs | Estado | Comando |
|---|---|---|---|---|---|
| `simulation.launch.py` | `wind_tower_bringup` | Gazebo + mundo + spawn Clearpath (Husky+UR5e+VLP-16+IMU) + bridges (clock, turner, LiDAR, IMU, cámara) + `turner_node` + `tf_static_relay` + `ekf_filter_node` (ver nota EKF) + límites teleop | `wind_tower_bringup/config/ekf.yaml` | **CENTRAL** | `ros2 launch wind_tower_bringup simulation.launch.py` |
| `inspection.launch.py` | `wind_tower_inspection_behaviour` | `cylinder_localizer`, `dualsense_joy`, `ps5_teleop`, `stability_monitor`, `cylindrical_map`, `state_machine` | `inspection_params.yaml`, `stability_monitor.yaml`, `state_machine.yaml` | **CENTRAL** | `ros2 launch wind_tower_inspection_behaviour inspection.launch.py` |
| `perception.launch.py` | `wind_tower_perception` | Static TF `world→odom`, `defect_detector`, `image_capture`, `defect_mapper`, opcionalmente `synthetic_capture` | `perception_params.yaml`, `synthetic_dataset.yaml` | **CENTRAL** | `ros2 launch wind_tower_perception perception.launch.py` |
| Launchers en `gz_ros2_control_demos/launch/*` | `gz_ros2_control_demos` | Demos upstream | varias | **DEMO / TEST** (no se usan en este proyecto) | (ver upstream) |

**Eliminado del repo** en BUILD 2026-05-12: `slam.launch.py` y `slam_toolbox.yaml` (dependían de `/scan` que el pipeline no publica). Recuperables con `git log --all -- ros2_ws/src/wind_tower_bringup/launch/slam.launch.py`.

---

## Notas operativas por launcher

### `simulation.launch.py`

- Debe lanzarse **antes** que `inspection.launch.py`.
- Requiere `~/clearpath/robot.yaml` (Clearpath generator externo).
- Aplica una `TimerAction` de 20 s tras arrancar para limitar la velocidad del teleop Clearpath (0.15 m/s normal, 0.30 m/s turbo).
- Args útiles: `rviz:=false` para arrancar sin RViz; `x`, `y`, `z`, `yaw` para posición de spawn.
- Odometría: el flujo real consume `/robot/platform/odom/filtered` del EKF interno de Clearpath. El antiguo `ekf_filter_node` propio se retiró (ver `docs/audit/BUILD_SUMMARY.md`).

### `inspection.launch.py`

- ≈60 `DeclareLaunchArgument` permiten override por CLI de gains y umbrales del state_machine.
- `state_machine_auto_start:=false` por defecto. La misión se arranca pulsando **Triángulo** en el DualSense o publicando `/inspection/mission_command "data: START_AUTO"`.
- Cuando `/inspection/autonomous_active=true`, `ps5_teleop` deja el turner en 0 y bloquea el teleop manual.
- Perfil corto de pruebas (referencia): `state_machine_lane_length_m:=1.0 state_machine_lane_delta_theta_deg:=5.0 state_machine_axial_speed_mps:=0.05 state_machine_turner_speed_rad_s:=0.02 state_machine_publish_rate_hz:=30.0`.

### `perception.launch.py`

- Se lanza **después** de `simulation.launch.py` (necesita la cámara y la TF tree del robot).
- Publica un TF estático `world → odom` derivado del spawn por defecto (`spawn_x:=0.0 spawn_y:=-10.0 spawn_z:=0.3 spawn_yaw:=1.5708`). Si cambias el spawn en `simulation.launch.py`, pásalo también aquí.
- Por defecto el detector usa **HoughCircles** (cero entrenamiento). Para activar YOLO entrenado: `backend:=yolo yolo_model_path:=/ruta/a/best.pt`.
- Para generar dataset sintético: arranca primero Gazebo con un mundo generado por `generate_synthetic_world` y después `ros2 launch wind_tower_perception perception.launch.py use_synthetic_capture:=true ground_truth_path:=~/wind_tower_synthetic/defects_ground_truth.yaml`.
- Los `frames + sidecars + detections.ndjson` se escriben en `~/wind_tower_inspections/run_*/` (configurable via `output_root`).
- El generador de informe se invoca **fuera** de ROS: `python3 -m wind_tower_perception.scripts.generate_inspection_report --run-dir ~/wind_tower_inspections/run_YYYYMMDD_HHMMSS`. Requiere `ANTHROPIC_API_KEY` y `pip install anthropic`.

### `slam.launch.py` — RETIRADO

Eliminado en BUILD 2026-05-12. Dependía de `/scan` (LaserScan) que el pipeline activo no publica. Recuperable con `git log --all -- ros2_ws/src/wind_tower_bringup/launch/slam.launch.py`.

---

## Cómo añadir un launcher nuevo

1. Crearlo en `launch/<nombre>.launch.py` del paquete correspondiente.
2. Registrarlo en `setup.py` dentro de `data_files` para que `colcon` lo instale.
3. Documentarlo en esta tabla con su estado.
4. Si es CENTRAL, actualizar también [HOW_TO_LAUNCH.md](HOW_TO_LAUNCH.md).
5. Si es AUXILIAR o LEGACY, añadir un comentario claro en la cabecera del archivo `.launch.py` indicando su estado.
