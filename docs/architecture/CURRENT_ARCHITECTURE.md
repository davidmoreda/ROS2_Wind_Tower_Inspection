# CURRENT_ARCHITECTURE — Arquitectura implementada (2026-05-12)

> **Fuente de verdad para "qué existe hoy en código".** Para arquitectura objetivo ver [TARGET_ARCHITECTURE.md](TARGET_ARCHITECTURE.md). Para el roadmap histórico ver `../legacy/docs/PROJECT_PLAN.md`.

Solo se documenta aquí lo que está realmente implementado y se ejecuta en el flujo actual. Cualquier propuesta vive en `TARGET_ARCHITECTURE.md`.

---

## 1. Paquetes ROS 2 reales

| Paquete | Build | Estado | Ruta |
|---|---|---|---|
| `wind_tower_description` | ament_cmake | IMPLEMENTADO (solo meshes + xacro) | `ros2_ws/src/wind_tower_description/` |
| `wind_tower_simulation` | ament_cmake | IMPLEMENTADO (mundo SDF) | `ros2_ws/src/wind_tower_simulation/` |
| `wind_tower_bringup` | ament_python | IMPLEMENTADO | `ros2_ws/src/wind_tower_bringup/` |
| `wind_tower_inspection_behaviour` | ament_python | IMPLEMENTADO PARCIALMENTE (validación de misión larga en curso) | `ros2_ws/src/wind_tower_inspection_behaviour/` |
| `gz_ros2_control` (fork con fix WSL2) | upstream | IMPLEMENTADO | `ros2_ws/src/gz_ros2_control/` |

---

## 2. Launchers centrales

| Launcher | Paquete | Estado | Comando |
|---|---|---|---|
| `simulation.launch.py` | `wind_tower_bringup` | CENTRAL | `ros2 launch wind_tower_bringup simulation.launch.py` |
| `inspection.launch.py` | `wind_tower_inspection_behaviour` | CENTRAL | `ros2 launch wind_tower_inspection_behaviour inspection.launch.py` |
| `rtabmap.launch.py` | `wind_tower_bringup` | AUXILIAR / experimental | `ros2 launch wind_tower_bringup rtabmap.launch.py` |

(`slam.launch.py` y `slam_toolbox.yaml` eliminados en BUILD 2026-05-12; recuperables vía `git log`.)

Detalle completo en [../operation/LAUNCHERS_REFERENCE.md](../operation/LAUNCHERS_REFERENCE.md).

---

## 3. Nodos reales

### `wind_tower_bringup`

| Nodo | Archivo | Estado | Lanzado por |
|---|---|---|---|
| `turner_node` | `turner_node.py` | IMPLEMENTADO | `simulation.launch.py` |
| `tf_static_relay` | `tf_static_relay.py` | IMPLEMENTADO | `simulation.launch.py` |
| `dualsense_joy` | `dualsense_joy.py` | IMPLEMENTADO | `inspection.launch.py` |
| `ps5_teleop` | `ps5_teleop.py` | IMPLEMENTADO | `inspection.launch.py` |
| `cylinder_localizer` | `cylinder_localizer_node.py` | IMPLEMENTADO PARCIALMENTE (prototipo en validación) | `inspection.launch.py` |

(`scan_gate_node.py` y `map_accumulator_node.py` eliminados en BUILD 2026-05-12; recuperables vía `git log`.)

### `wind_tower_inspection_behaviour`

| Nodo | Archivo | Estado | Lanzado por |
|---|---|---|---|
| `stability_monitor` | `stability_monitor_node.py` | IMPLEMENTADO | `inspection.launch.py` |
| `cylindrical_map` | `cylindrical_map_node.py` | IMPLEMENTADO (MVP de cobertura pasiva) | `inspection.launch.py` |
| `state_machine` | `state_machine_node.py` | IMPLEMENTADO PARCIALMENTE (sin bypass/local_inspection todavía) | `inspection.launch.py` |

Detalle completo en [../operation/NODES_REFERENCE.md](../operation/NODES_REFERENCE.md).

---

## 4. Flujo de datos actual (evidencia de código)

```text
[Gazebo Harmonic]──► wind_tower_world.sdf
  │
  ├─ Husky A200 + UR5e + Velodyne VLP-16 (clearpath_gz/robot_spawn)
  └─ turner_joint (revolute) + JointController/JointStatePublisher plugins
        │
        ▼
[ros_gz_bridge / image_bridge / topic_tools relays]
  │
  │  /turner/cmd_vel       (Float64)
  │  /turner/joint_state   (JointState)
  │  /velodyne_points      (PointCloud2 20 Hz)
  │  /robot/sensors/inspection_camera/image
  │  /inspection/camera/image_raw   (relay)
  │  /inspection/camera/camera_info
  │  /robot/sensors/imu_0/data      (Imu 100 Hz)
  │  /clock
  │  /tf, /robot_description (relays desde namespace /robot)
  │
  ├──► turner_node       ──► /turner/angle, /turner/angle_deg
  ├──► tf_static_relay   (re-publica TF estáticos transient_local)
  └──► cylinder_localizer──► /robot_in_tube, /cylinder_fit/{stats,wall_points}

[PS5 DualSense]
  └──► dualsense_joy ──► /robot/joy_teleop/joy
         │
         └──► ps5_teleop ──► trayectorias UR5e
                            /turner/cmd_vel  (solo manual; bloqueado en autonomía)
                            /inspection/mission_command (START/STOP)

[Clearpath EKF interno] ──► /robot/platform/odom/filtered (≈43 Hz)
  │
  ▼
[stability_monitor] ◄── /robot/sensors/imu_0/data
                    ◄── /robot/platform/odom/filtered
                    ◄── /cylinder_fit/stats
                    ◄── /turner/angle
   ──► /inspection/bottom_lane_locked   (Bool)
   ──► /inspection/safe_to_scan         (Bool)
   ──► /inspection/safe_to_index_tube   (Bool)
   ──► /inspection/stability            (String JSON)

[cylindrical_map]   ◄── /robot/platform/odom/filtered
                    ◄── /turner/angle
                    ◄── /inspection/{bottom_lane_locked, safe_to_scan, state_text}
   ──► /inspection/cylindrical_pose
   ──► /inspection/coverage_status
   ──► /inspection/cylindrical_map_stats

[state_machine]     ◄── /robot/platform/odom/filtered
                    ◄── /inspection/stability
                    ◄── /inspection/{bottom_lane_locked, safe_to_scan, safe_to_index_tube}
                    ◄── /turner/angle
                    ◄── /inspection/mission_command
   ──► /robot/platform/cmd_vel  (TwistStamped, base Husky)
   ──► /turner/cmd_vel          (Float64, solo en INDEX_TUBE)
   ──► /inspection/state, /state_text, /current_lane, /mission_status
   ──► /inspection/autonomous_active  (bloquea teleop manual)
```

Evidencia clave:
- `ros2_ws/src/wind_tower_bringup/launch/simulation.launch.py:78-304`
- `ros2_ws/src/wind_tower_inspection_behaviour/launch/inspection.launch.py:451-591`

---

## 5. Topics y frames

Ver tabla canónica en [../operation/TOPICS_AND_FRAMES.md](../operation/TOPICS_AND_FRAMES.md).

---

## 6. Partes incompletas / a verificar

| Componente | Detalle | Evidencia |
|---|---|---|
| `cylinder_localizer` | Prototipo; necesita robustez bajo movimiento real | `cylinder_localizer_node.py` |
| Alias `/inspection/camera/image_raw` | Relay existe en `simulation.launch.py`; verificar consumo en runtime | `simulation.launch.py` |
| `state_machine` — estados de bypass/local_inspection/report | No implementados (FUTURO en roadmap). El MVP cubre AXIAL_SCAN, INDEX_TUBE, ROTATE_TO_TANGENTIAL, ROTATE_TO_AXIAL, ALIGN_TO_BOTTOM_LANE, VERIFY_*. | `state_machine_node.py` |
| Mensajes custom (`BottomLaneState.msg`, `CylindricalPose.msg`, …) | PROPUESTOS en roadmap; NO existe paquete `wind_tower_msgs`. Hoy todo es `std_msgs/String` con JSON. | grep negativo `find ros2_ws/src -name "*.msg"` |
| Captura de imágenes por distancia | Pipeline `image_capture_manager_node` no implementado | — |
| Informe de inspección | `report_generator_node` no implementado | — |

---

## 7. Limitaciones actuales conocidas

1. **No hay Nav2** en este sistema. La navegación la realiza un control PI propio dentro de `state_machine_node.py`. Ver `TARGET_ARCHITECTURE.md` para la dirección objetivo.
2. **No hay frame `cyl_map`** ni odometría cilíndrica desplegada. La "malla `(x, θ)`" actual es solo registro de cobertura pasiva, no espacio de navegación.
3. **SLAM no se usa** en el flujo MVP. `slam.launch.py` se eliminó (dependía de `/scan` no publicado). `rtabmap.launch.py` es auxiliar con loop closure desactivado y no fuente de `θ_tube`.
4. **`θ_tube` viene exclusivamente del encoder del virador** (decisión arquitectónica por simetría del cilindro: ICP no distingue rotación pura en sección circular; RTAB-Map secundario).
5. **Mando PS5 DualSense vía `evdev` directo** — requiere usbipd attach desde Windows y `/dev/input/event*` accesible.
6. **WSL2 / RTX 4070** — Requiere `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA` y `GALLIUM_DRIVER=d3d12`; alias `ai-on` los exporta. Sin ellos no hay aceleración GPU.
7. **`gz_ros2_control` es un fork** con fix de null-pointer en WSL2 (null checks con `RCLCPP_WARN` y `continue` en el destructor del plugin); sin ese fix segfault al arrancar Gazebo.

---

## 8. Cómo se relaciona esta arquitectura con la metodología

Metodología MVP (calles axiales discretas con indexado angular):

```
IDLE
  └─ START_AUTO →
VERIFY_BOTTOM_LOCK
  └─ bottom_lane_locked=true →
AXIAL_SCAN
  ├─ avance lento por la calle
  ├─ cobertura nominal solo si safe_to_scan=true
  └─ lane_progress_m >= lane_length_m →
WAIT_SAFE_TO_INDEX
  └─ safe_to_index_tube=true →
ROTATE_TO_TANGENTIAL → INDEX_TUBE (turner gira Δθ) → ROTATE_TO_AXIAL → ALIGN_TO_BOTTOM_LANE → VERIFY_INDEXED_POSITION
  └─ siguiente calle
FINISH cuando rotación acumulada ≥ 360° + solape
```

Esto es lo que orquesta `state_machine_node.py`. Los estados `OBSTACLE_DETECTED`, `LOCAL_INSPECTION`, `BYPASS_OBSTACLE`, `RETURN_TO_BOTTOM_LANE`, `ERROR_RECOVERY` están **propuestos** pero no implementados todavía.
