# TOPICS_AND_FRAMES — Topics y frames TF reales

> Solo se listan topics y frames que existen en el flujo activo (`simulation.launch.py` + `inspection.launch.py`). Para topics propuestos en arquitecturas futuras ver `../architecture/TARGET_ARCHITECTURE.md`.

---

## 1. Topics — categorías

### Simulación / bridges (origen: Gazebo o stack Clearpath)

| Topic | Tipo | Hz aprox | Productor | Consumidores conocidos |
|---|---|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | sim | clock_bridge | todos (use_sim_time) |
| `/tf` | `tf2_msgs/TFMessage` | varia | tf_relay (relay desde `/robot/tf`) | todo |
| `/tf_static` | `tf2_msgs/TFMessage` | latched | tf_static_relay + Clearpath | todo |
| `/robot_description` | `std_msgs/String` | latched | relay desde `/robot/robot_description` | RViz, controllers |
| `/robot/platform/odom` | `nav_msgs/Odometry` | ~50 | Clearpath | EKF |
| `/robot/platform/odom/filtered` | `nav_msgs/Odometry` | ~43 | Clearpath EKF interno | stability_monitor, cylindrical_map, state_machine |
| `/robot/sensors/imu_0/data` | `sensor_msgs/Imu` | 100 | gz bridge | stability_monitor |
| `/velodyne_points` | `sensor_msgs/PointCloud2` | 20 | gz bridge (remap desde `/robot/sensors/lidar3d_0/scan/points`) | cylinder_localizer |
| `/robot/sensors/inspection_camera/image` | `sensor_msgs/Image` | ~16 | ros_gz_image | inspection_image_relay |
| `/inspection/camera/image_raw` | `sensor_msgs/Image` | ~16 | relay desde el anterior | futuro image_capture |
| `/inspection/camera/camera_info` | `sensor_msgs/CameraInfo` | ~16 | gz bridge (remap) | futuro |

### Virador (turner)

| Topic | Tipo | Productor | Consumidores |
|---|---|---|---|
| `/turner/cmd_vel` | `std_msgs/Float64` | state_machine (auto) o ps5_teleop (manual) | turner_cmd_bridge → Gazebo JointController |
| `/turner/joint_state` | `sensor_msgs/JointState` | turner_state_bridge | turner_node |
| `/turner/angle` | `std_msgs/Float64` (rad acumulado) | turner_node | stability_monitor, cylindrical_map, state_machine |
| `/turner/angle_deg` | `std_msgs/Float64` (grados) | turner_node | debug humano |

### Cilindro / localización

| Topic | Tipo | Productor | Consumidores |
|---|---|---|---|
| `/robot_in_tube` | `geometry_msgs/PoseStamped` | cylinder_localizer | — (futuro) |
| `/cylinder_fit/wall_points` | `sensor_msgs/PointCloud2` | cylinder_localizer | RViz debug |
| `/cylinder_fit/stats` | `std_msgs/String` (JSON) | cylinder_localizer | stability_monitor |

### Inspección (lógica)

| Topic | Tipo | Productor | Consumidores |
|---|---|---|---|
| `/inspection/bottom_lane_locked` | `std_msgs/Bool` | stability_monitor | state_machine, cylindrical_map |
| `/inspection/safe_to_scan` | `std_msgs/Bool` | stability_monitor | state_machine, cylindrical_map |
| `/inspection/safe_to_index_tube` | `std_msgs/Bool` | stability_monitor | state_machine |
| `/inspection/stability` | `std_msgs/String` (JSON) | stability_monitor | state_machine, debug |
| `/inspection/state` | `std_msgs/String` (JSON) | state_machine | debug, cylindrical_map |
| `/inspection/state_text` | `std_msgs/String` | state_machine | cylindrical_map |
| `/inspection/current_lane` | `std_msgs/String` (JSON) | state_machine | debug |
| `/inspection/mission_status` | `std_msgs/String` (JSON) | state_machine | debug |
| `/inspection/autonomous_active` | `std_msgs/Bool` | state_machine | ps5_teleop (bloqueo manual) |
| `/inspection/mission_command` | `std_msgs/String` | ps5_teleop | state_machine |
| `/inspection/emergency_stop` | `std_msgs/Bool` | dualsense_joy | — (sin consumidor automático todavía) |
| `/inspection/cylindrical_pose` | `std_msgs/String` (JSON) | cylindrical_map | debug |
| `/inspection/coverage_status` | `std_msgs/String` (JSON) | cylindrical_map | debug, futuro report |
| `/inspection/cylindrical_map_stats` | `std_msgs/String` (JSON) | cylindrical_map | debug |

### Comando al robot

| Topic | Tipo | Productor | Consumidores |
|---|---|---|---|
| `/robot/platform/cmd_vel` | `geometry_msgs/TwistStamped` | state_machine (auto) o Clearpath teleop (manual) | Husky diff drive |
| `/robot/arm_0_joint_trajectory_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | ps5_teleop | UR5e |

---

## 2. Frames TF

Los frames se generan principalmente por Clearpath + robot_localization + el plugin de Gazebo. Frames clave:

| Frame | Quién lo produce | Notas |
|---|---|---|
| `world` o `wind_tower_world` | Gazebo | Frame mundo de simulación |
| `map` | EKF Clearpath / robot_localization | Frame mapa estándar (si el EKF está activo) |
| `odom` | EKF | Frame odom estándar |
| `base_link` | URDF Clearpath | Frame del cuerpo del robot |
| `*_link` UR5e | URDF UR5e | Cinemática del brazo |
| `velodyne` / `lidar3d_0_*` | URDF + Gazebo plugin | Frame del LiDAR |
| `tube_link` | Mundo SDF | Frame del cilindro (rota con `turner_joint`) |

**No existe** ningún frame `cyl_map` ni equivalente para el plano cilíndrico desplegado. Su creación es parte de la arquitectura objetivo (ver `TARGET_ARCHITECTURE.md`).

---

## 3. Cómo verificar en runtime

Con la simulación arrancada:

```bash
ros2 topic list                    # ver todos los topics activos
ros2 topic info /inspection/state  # productor y tipo
ros2 topic hz /turner/angle        # frecuencia real
ros2 topic echo /inspection/stability --once
ros2 run tf2_tools view_frames     # genera frames.pdf
ros2 run tf2_ros tf2_echo base_link velodyne
```
