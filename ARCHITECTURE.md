# Arquitectura del Sistema — Wind Tower Inspection

**Stack:** ROS 2 Jazzy · Gazebo Harmonic · Husky A200 + UR5e · Velodyne VLP-16 · WSL2

---

## Vista general de paquetes

```
ros2_ws/src/
├── wind_tower_description/      (ament_cmake)  — meshes STL + URDF cámara/luces
├── wind_tower_simulation/       (ament_cmake)  — mundo Gazebo (.sdf)
├── wind_tower_bringup/          (ament_python) — launchers + nodos de soporte
├── wind_tower_inspection_behaviour/ (ament_python) — lógica autónoma de misión
└── gz_ros2_control/             (fork local)   — fix null-pointer WSL2
```

---

## Launchers

```
simulation.launch.py          inspection.launch.py
(wind_tower_bringup)          (wind_tower_inspection_behaviour)
        │                               │
        ├─ Gazebo Harmonic              ├─ cylinder_localizer
        ├─ robot_spawn (Clearpath)      ├─ dualsense_joy
        ├─ Clearpath stack             ├─ ps5_teleop
        │   ├─ platform_velocity_ctrl  ├─ stability_monitor
        │   ├─ joint_state_broadcaster ├─ cylindrical_map
        │   └─ arm_0_joint_traj_ctrl   └─ state_machine
        ├─ gz_bridge (LiDAR/IMU/...)
        ├─ tf_static_relay
        ├─ turner_node
        └─ robot_localization (EKF)
```

---

## Nodos y ficheros fuente

### wind_tower_bringup

```
dualsense_joy.py
  Nodo: dualsense_joy
  Lee el mando PS5 vía evdev (sin joydev) y publica Joy estándar.
  Sub: /dev/input/event0 (evdev directo)
  Pub: /robot/joy_teleop/joy  [sensor_msgs/Joy]
       /inspection/emergency_stop  [std_msgs/Bool]  (botón OPTIONS)

turner_node.py
  Nodo: turner_node
  Integra /turner/cmd_vel o lee joint_state del bridge para calcular
  el ángulo acumulado del tubo.
  Sub: /turner/cmd_vel       [std_msgs/Float64]
       /turner/joint_state   [sensor_msgs/JointState]  (desde Gazebo)
  Pub: /turner/angle         [std_msgs/Float64]  — θ_tube en rad
       /turner/angle_deg     [std_msgs/Float64]  — θ_tube en grados

ps5_teleop.py
  Nodo: ps5_teleop
  Traduce Joy a comandos de brazo UR5e y virador.
  Durante autonomía NO publica al turner (cede el canal a state_machine).
  Sub: /robot/joy_teleop/joy        [sensor_msgs/Joy]
       /inspection/autonomous_active [std_msgs/Bool]
  Pub: /robot/arm_0_joint_trajectory_controller/joint_trajectory
                                     [trajectory_msgs/JointTrajectory]
       /turner/cmd_vel               [std_msgs/Float64]  (solo manual)
       /inspection/mission_command   [std_msgs/String]   (START/STOP)

cylinder_localizer_node.py
  Nodo: cylinder_localizer
  Ajusta un círculo a la sección transversal LiDAR para estimar
  posición del robot dentro del tubo.
  Sub: /velodyne_points   [sensor_msgs/PointCloud2]
  Pub: /robot_in_tube     [geometry_msgs/PoseStamped]
       /cylinder_fit/wall_points  [sensor_msgs/PointCloud2]
       /cylinder_fit/stats        [std_msgs/String]  — JSON con fit_r, rms, wall_pts

tf_static_relay.py
  Nodo: tf_static_relay
  Relay TRANSIENT_LOCAL para TF estáticos que necesitan re-publicarse.

scan_gate_node.py   (disponible, no en uso activo)
  Filtro sectorial de LaserScan — recorta ángulos fuera de rango.

map_accumulator_node.py  (prototipo, no en uso activo)
  Acumula nube de puntos en un frame canónico del tubo.
```

### wind_tower_inspection_behaviour

```
stability_monitor_node.py
  Nodo: stability_monitor
  Convierte IMU + odometría + geometría LiDAR en flags de seguridad.
  Sub: /robot/sensors/imu_0/data        [sensor_msgs/Imu]        100 Hz
       /robot/platform/odom/filtered    [nav_msgs/Odometry]       43 Hz
       /cylinder_fit/stats              [std_msgs/String]         20 Hz
       /turner/angle                    [std_msgs/Float64]        20 Hz
  Pub: /inspection/bottom_lane_locked   [std_msgs/Bool]           10 Hz
       /inspection/safe_to_scan         [std_msgs/Bool]           10 Hz
       /inspection/safe_to_index_tube   [std_msgs/Bool]           10 Hz
       /inspection/stability            [std_msgs/String]  JSON   10 Hz

cylindrical_map_node.py
  Nodo: cylindrical_map
  Nodo pasivo. Registra cobertura observada/nominal en malla (x, θ).
  No mueve el robot ni el tubo.
  Sub: /robot/platform/odom/filtered    [nav_msgs/Odometry]
       /turner/angle                    [std_msgs/Float64]
       /inspection/bottom_lane_locked   [std_msgs/Bool]
       /inspection/safe_to_scan         [std_msgs/Bool]
       /inspection/state_text           [std_msgs/String]
  Pub: /inspection/cylindrical_pose     [std_msgs/String]  JSON
       /inspection/coverage_status      [std_msgs/String]  JSON
       /inspection/cylindrical_map_stats [std_msgs/String] JSON

state_machine_node.py
  Nodo: state_machine
  Orquesta la misión completa: avance axial, giro tangencial,
  indexado del tubo con compensación de ruedas, realineación.
  Sub: /robot/platform/odom/filtered    [nav_msgs/Odometry]
       /inspection/stability            [std_msgs/String]
       /turner/angle                    [std_msgs/Float64]
       /inspection/bottom_lane_locked   [std_msgs/Bool]
       /inspection/safe_to_scan         [std_msgs/Bool]
       /inspection/safe_to_index_tube   [std_msgs/Bool]
       /inspection/mission_command      [std_msgs/String]
  Pub: /robot/platform/cmd_vel          [geometry_msgs/TwistStamped]
       /turner/cmd_vel                  [std_msgs/Float64]  (solo durante INDEX_TUBE)
       /inspection/state                [std_msgs/String]  JSON completo
       /inspection/state_text           [std_msgs/String]  nombre de estado
       /inspection/current_lane         [std_msgs/String]  JSON lane info
       /inspection/mission_status       [std_msgs/String]  JSON (=state)
       /inspection/autonomous_active    [std_msgs/Bool]
```

---

## Flujo de información — diagrama completo

```
                        HARDWARE / SIMULACIÓN GAZEBO
┌──────────────────────────────────────────────────────────────────┐
│  Husky A200          UR5e arm         Velodyne VLP-16    IMU     │
│  (diff drive)        (6 DOF)          (3D LiDAR)         │       │
│      │                  │                  │              │       │
│  Clearpath stack    arm_0_joint_traj   gz_bridge      gz_bridge  │
└──────┼──────────────────┼──────────────────┼──────────────┼──────┘
       │                  │                  │              │
       ▼                  ▼                  ▼              ▼
 /robot/platform/   /robot/arm_0_...    /velodyne_points  /robot/sensors/
  odom (raw)        joint_trajectory    PointCloud2        imu_0/data
       │                                    │              │
       ▼                                    │              │
  robot_localization                        │              │
  (EKF Clearpath)                           │              │
       │                                    │              │
       ▼                                    │              │
 /robot/platform/                           │              │
  odom/filtered ─────────────────────┐      │              │
                                     │      │              │
                         ┌───────────┼──────┘              │
                         │           │                     │
                         ▼           ▼                     ▼
              ┌─────────────────┐  ┌───────────────────────────────┐
              │ cylinder_       │  │      stability_monitor        │
              │ localizer       │  │                               │
              │                 │  │  IMU calibration (50 muestras)│
              │  LiDAR → fit    │  │  imu_ok: |roll|<5° |pitch|<5°│
              │  círculo 2D     │  │           |lateral|<5°        │
              │  → posición en  │  │  geometry_ok: wall_pts>300    │
              │    tubo (x, φ)  │  │               fit_rms<0.35    │
              └────────┬────────┘  │  scan_ok: speed<0.20 m/s     │
                       │           │  index_ok: speed<0.02 m/s     │
                       │           │            yaw_rate<0.10 r/s  │
                       ▼           └──────────────┬────────────────┘
              /robot_in_tube                       │
              /cylinder_fit/                       ├─► /inspection/bottom_lane_locked
               wall_points                         ├─► /inspection/safe_to_scan
              /cylinder_fit/stats ─────────────────► /inspection/safe_to_index_tube
                       │                           └─► /inspection/stability (JSON)
                       │
                       │         ┌─────────── MANDO PS5 ───────────────┐
                       │         │                                      │
                       │   dualsense_joy ──► /robot/joy_teleop/joy     │
                       │         │                    │                 │
                       │         │                    ▼                 │
                       │         │              ps5_teleop              │
                       │         │              ├─ L2+stick→ UR5e traj │
                       │         │              ├─ X → turner manual   │
                       │         │              ├─ △ → START_AUTO      │
                       │         │              └─ ○ → STOP            │
                       │         │                    │                 │
                       │         │   /inspection/     │                 │
                       │         │   mission_command◄─┘                 │
                       │         └──────────────────────────────────────┘
                       │
                       │   ┌──────────────────────────────────────────────────┐
                       │   │                  turner_node                     │
                       │   │  /turner/joint_state (gz_bridge) ──► integra θ  │
                       │   │  /turner/cmd_vel ──────────────────► integra θ  │
                       │   │  Publica θ acumulado                             │
                       │   └──────┬───────────────────────────────────────────┘
                       │          │
                       │          ▼
                       │   /turner/angle ────────────────────────────────────┐
                       │   /turner/angle_deg                                 │
                       │                                                     │
                       ▼                                                     ▼
              ┌──────────────────┐         ┌───────────────────────────────────────┐
              │ cylindrical_map  │         │          state_machine                │
              │                  │         │                                       │
              │  Malla (x, θ)    │◄────────│  /inspection/state_text              │
              │  observed_map    │         │                                       │
              │  nominal_map     │         │  IDLE                                 │
              │                  │         │   │  START_AUTO / auto_start          │
              │  Solo registra   │         │   ▼                                   │
              │  cobertura si    │         │  VERIFY_BOTTOM_LOCK ──────────────────┤
              │  state==AXIAL_   │         │   │ bottom_lane_locked=T              │
              │  SCAN y          │         │   ▼                                   │
              │  safe_to_scan=T  │         │  AXIAL_SCAN ── safe_to_scan=F ──►     │
              │                  │         │   │  avanza 0.05 m/s                 │
              └──────────────────┘         │   │  PI heading + lateral IMU        │
              /inspection/                 │   │  lane_finished ──────────────────►│
               cylindrical_pose           │   ▼                                   │
              /inspection/                │  WAIT_SAFE_TO_INDEX                   │
               coverage_status            │   │  safe_to_index=T                  │
              /inspection/                │   ▼                                   │
               cylindrical_map_stats      │  ROTATE_TO_TANGENTIAL                │
                                          │   │  gira base ±90°                  │
                                          │   │  PI yaw                          │
                                          │   ▼                                   │
                                          │  INDEX_TUBE ◄── solo aquí publica    │
                                          │   │  turner a /turner/cmd_vel         │
                                          │   │  Δθ=5° @ 0.02 rad/s              │
                                          │   │  ruedas compensan: v=R·ω          │
                                          │   │  ★ _index_motion_committed        │
                                          │   │    evita re-gate speed            │
                                          │   ▼                                   │
                                          │  ROTATE_TO_AXIAL                     │
                                          │   │  gira base 180° (lawnmower)      │
                                          │   ▼                                   │
                                          │  ALIGN_TO_BOTTOM_LANE                │
                                          │   │  v=0.02 m/s + PI lateral IMU     │
                                          │   │  espera lock_hold_s=1s           │
                                          │   ▼                                   │
                                          │  VERIFY_INDEXED_POSITION             │
                                          │   │  espera settle_time_s=2s         │
                                          │   ▼                                   │
                                          │  AXIAL_SCAN (siguiente calle) ──────►│
                                          │                                       │
                                          │  FINISH (rotation_done >= 360°)      │
                                          │  ERROR_RECOVERY                      │
                                          └───────────────────────────────────────┘
                                                   │            │
                                                   ▼            ▼
                                        /robot/platform/   /turner/cmd_vel
                                          cmd_vel            (Float64)
                                        (TwistStamped)          │
                                                │               ▼
                                                ▼          Gazebo Joint
                                         Husky diff        Controller
                                          drive             (tubo gira)
```

---

## Topics — tabla de referencia rápida

| Topic | Tipo | Hz | Productor | Consumidores |
|---|---|---|---|---|
| `/robot/joy_teleop/joy` | `Joy` | event | dualsense_joy | ps5_teleop |
| `/robot/platform/odom` | `Odometry` | ~50 | Clearpath EKF | robot_localization |
| `/robot/platform/odom/filtered` | `Odometry` | ~43 | robot_localization | stability_monitor, cylindrical_map, state_machine |
| `/robot/sensors/imu_0/data` | `Imu` | 100 | Gazebo bridge | stability_monitor |
| `/velodyne_points` | `PointCloud2` | 20 | Gazebo bridge | cylinder_localizer |
| `/robot/sensors/inspection_camera/image` | `Image` | 16 | Gazebo bridge | — (futuro image_capture) |
| `/turner/joint_state` | `JointState` | 20 | Gazebo bridge | turner_node |
| `/turner/cmd_vel` | `Float64` | 30/10 | state_machine (auto) / ps5_teleop (manual) | turner_node, Gazebo JointController |
| `/turner/angle` | `Float64` | 20 | turner_node | stability_monitor, cylindrical_map, state_machine |
| `/turner/angle_deg` | `Float64` | 20 | turner_node | debug / humano |
| `/robot_in_tube` | `PoseStamped` | 20 | cylinder_localizer | — (futuro) |
| `/cylinder_fit/stats` | `String` | 20 | cylinder_localizer | stability_monitor |
| `/cylinder_fit/wall_points` | `PointCloud2` | 20 | cylinder_localizer | RViz debug |
| `/inspection/bottom_lane_locked` | `Bool` | 10 | stability_monitor | state_machine, cylindrical_map |
| `/inspection/safe_to_scan` | `Bool` | 10 | stability_monitor | state_machine, cylindrical_map |
| `/inspection/safe_to_index_tube` | `Bool` | 10 | stability_monitor | state_machine |
| `/inspection/stability` | `String` (JSON) | 10 | stability_monitor | state_machine, debug |
| `/inspection/mission_command` | `String` | event | ps5_teleop | state_machine |
| `/inspection/autonomous_active` | `Bool` | 30 | state_machine | ps5_teleop (bloqueo teleop) |
| `/inspection/state` | `String` (JSON) | 30 | state_machine | cylindrical_map, debug |
| `/inspection/state_text` | `String` | 30 | state_machine | cylindrical_map |
| `/inspection/current_lane` | `String` (JSON) | 30 | state_machine | debug |
| `/inspection/cylindrical_pose` | `String` (JSON) | 10 | cylindrical_map | debug |
| `/inspection/coverage_status` | `String` (JSON) | 10 | cylindrical_map | debug / futuro report |
| `/robot/platform/cmd_vel` | `TwistStamped` | 30 | state_machine (auto) / Clearpath teleop (manual) | Husky diff drive |
| `/robot/arm_0_joint_trajectory_controller/joint_trajectory` | `JointTrajectory` | event | ps5_teleop | UR5e |

---

## Flujo de una calle axial completa

```
Operador: △ (Triángulo)
      │
      ▼
ps5_teleop ──► /inspection/mission_command = "START_AUTO"
      │
      ▼
state_machine: IDLE → VERIFY_BOTTOM_LOCK
  ├─ bottom_lane_locked=F → ALIGN_TO_BOTTOM_LANE
  │    └─ publica cmd_vel (v=0.02, ω=f(IMU lateral)) hasta lock hold 1s
  └─ bottom_lane_locked=T → AXIAL_SCAN
       │
       │  publica /inspection/autonomous_active=True
       │  ps5_teleop deja de publicar al turner
       │
       ▼  (avanza a 0.05 m/s · PI heading)
      AXIAL_SCAN ──────────────────────────────────────────────────────┐
       │  safe_to_scan=T → cylindrical_map marca nominal coverage       │
       │  cada tick: cmd_vel.linear.x=0.05, angular.z=PI(heading)      │
       │  safe_to_scan=F → RETURN_TO_BOTTOM_LANE → ALIGN              │
       │  lane_finished (1m recorrido) → siguiente                      │
       ▼                                                                │
      WAIT_SAFE_TO_INDEX                                               │
       │  espera: speed<0.02 m/s, yaw_rate<0.10 r/s                   │
       ▼                                                                │
      ROTATE_TO_TANGENTIAL (±90°)                                      │
       │  cmd_vel: linear=0, angular=PI(yaw_error)                     │
       ▼                                                                │
      INDEX_TUBE (Δθ=5°)                                               │
       │  /turner/cmd_vel = 0.02 rad/s  → tubo gira                   │
       │  cmd_vel.linear.x = R·ω = 3.925·0.02 = 0.078 m/s            │
       │  cmd_vel.angular.z = PI(tangential yaw)                       │
       │  _index_motion_committed=True → no re-gate por speed          │
       │  solo aborta si bottom_lane_locked=F                          │
       ▼                                                                │
      ROTATE_TO_AXIAL (180° lawnmower)                                 │
       │  gira la base para afrontar la siguiente calle al revés       │
       ▼                                                                │
      ALIGN_TO_BOTTOM_LANE                                             │
       │  realinea en generatriz inferior (v=0.02 + PI lateral)        │
       ▼                                                                │
      VERIFY_INDEXED_POSITION                                           │
       │  settle_time=2s + safe_to_scan=T                              │
       └──────────────────────────────────────────────────────────────► AXIAL_SCAN
                                                                  (calle siguiente,
                                                                   dirección opuesta)
                                      rotation_done >= 360° → FINISH
```

---

## Reglas de cobertura nominal

```
  AXIAL_SCAN tick
       │
       ├─ state == AXIAL_SCAN ?  ─── NO ──► NO marca cobertura
       │         YES
       ├─ bottom_lane_locked ?    ─── NO ──► NO marca cobertura
       │         YES
       ├─ safe_to_scan ?          ─── NO ──► NO marca cobertura
       │         YES
       └──────────────────────────────────► MARCA observed_map + nominal_scan_map
                                            celda (x_robot, θ_tube)
```

---

## Capas de la malla cilíndrica (cylindrical_map_node)

```
  malla (x, θ)  ←  x_step (defecto 0.1m)  ×  theta_step (defecto 5°)

  observed_map    — toda celda vista por LiDAR/robot (cualquier estado)
  nominal_map     — solo celdas durante AXIAL_SCAN con safe_to_scan=T
  confidence_map  — (pendiente) calidad de observación por celda
  mode_map        — (pendiente) axial / bypass / local / manual
  obstacle_map    — (pendiente)
  anomaly_map     — (pendiente)
```

---

## Seguridad: quién manda al turner

```
  autonomous_active = False (IDLE / teleop manual)
       └─► ps5_teleop publica /turner/cmd_vel según botón X/□

  autonomous_active = True (cualquier estado activo)
       └─► ps5_teleop hace return sin publicar
           └─► state_machine es el ÚNICO publisher
                ├─ INDEX_TUBE  → publica 0.02 rad/s (o negativo)
                └─ resto       → publica 0.0 (stop) o nada
```

---

## Nodos pendientes de implementar

| Nodo | Fase | Responsabilidad |
|---|---|---|
| `bottom_lane_controller.py` | 9F | PID heading+lateral dedicado para AXIAL_SCAN |
| `obstacle_manager_node.py` | 9G | Detección LiDAR frontal, evento OBSTACLE_DETECTED |
| `tube_indexing_controller.py` | 9H | Giro Δθ verificado antes/después |
| `image_capture_manager_node.py` | 9I | Captura por Δx=0.10m + metadatos JSON |
| `bypass_manager_node.py` | 9J | Esquiva conservadora sin cobertura nominal |
| `local_inspection_controller.py` | 9L | UR5e + cámara + iluminación en ROI |
| `report_generator_node.py` | 9M | Informe JSON/Markdown de la misión |
