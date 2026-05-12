# LAUNCH_GRAPH — Mapa real de ejecución

Fase PLAN. Solo describe lo que arranca cada launcher en el estado actual del repo.

---

## 1. Tabla resumen

| Launcher | Estado | Nodos que arranca | Configs cargadas | Incluye otros launchers |
|---|---|---|---|---|
| `wind_tower_bringup/launch/simulation.launch.py` | **ACTIVO / CENTRAL** | clock_bridge, turner_cmd_bridge, turner_state_bridge, lidar3d_bridge, inspection_image_bridge, inspection_image_relay, inspection_camera_info_bridge, robot_description_relay, tf_relay, `tf_static_relay` (propio), `turner_node` (propio), imu_bridge, `ekf_filter_node` (robot_localization) | `wind_tower_bringup/config/ekf.yaml` | `ros_gz_sim/launch/gz_sim.launch.py`, `clearpath_gz/launch/robot_spawn.launch.py` |
| `wind_tower_inspection_behaviour/launch/inspection.launch.py` | **ACTIVO / CENTRAL** | `cylinder_localizer`, `dualsense_joy`, `ps5_teleop`, `stability_monitor`, `cylindrical_map`, `state_machine` | `inspection_params.yaml`, `stability_monitor.yaml`, `state_machine.yaml` | Ninguno (no incluye `simulation.launch.py`) |
| `wind_tower_bringup/launch/slam.launch.py` | **LEGACY PROBABLE** | `slam_toolbox/async_slam_toolbox_node`, lifecycle configure+activate (TimerAction) | `wind_tower_bringup/config/slam_toolbox.yaml` | Ninguno |
| `wind_tower_bringup/launch/rtabmap.launch.py` | **AUXILIAR / experimental** | `rtabmap_odom/icp_odometry`, `rtabmap_slam/rtabmap`, `rtabmap_viz/rtabmap_viz` | parámetros inline | Ninguno |

Los launchers de `gz_ros2_control_demos/launch/*` se ignoran: pertenecen al fork upstream y no se invocan en este proyecto.

---

## 2. `simulation.launch.py` — detalle

Evidencia: `ros2_ws/src/wind_tower_bringup/launch/simulation.launch.py`

```
simulation.launch.py
├── SetEnvironmentVariable PYTHONPATH                                       (L59)
├── SetEnvironmentVariable GZ_SIM_RESOURCE_PATH                             (L69)
├── IncludeLaunchDescription  ros_gz_sim/gz_sim.launch.py                   (L78)
│       └── carga wind_tower_world.sdf
├── Node ros_gz_bridge/parameter_bridge   clock_bridge   (/clock)          (L87)
├── Node ros_gz_bridge/parameter_bridge   turner_cmd_bridge                 (L118)
│       /turner/cmd_vel  →  gz JointController
├── Node ros_gz_bridge/parameter_bridge   turner_state_bridge               (L133)
│       gz JointState  →  /turner/joint_state
├── Node ros_gz_bridge/parameter_bridge   lidar3d_bridge                    (L149)
│       /robot/sensors/lidar3d_0/scan/points → /velodyne_points
├── Node ros_gz_image/image_bridge        inspection_image_bridge           (L165)
├── Node topic_tools/relay                inspection_image_relay            (L175)
├── Node ros_gz_bridge/parameter_bridge   inspection_camera_info_bridge     (L186)
├── Node topic_tools/relay                robot_description_relay           (L95)
├── Node topic_tools/relay                tf_relay  /robot/tf → /tf         (L103)
├── Node wind_tower_bringup/tf_static_relay                                 (L110)
├── Node wind_tower_bringup/turner_node                                     (L217)
├── Node ros_gz_bridge/parameter_bridge   imu_bridge                        (L225)
├── Node robot_localization/ekf_node      ekf_filter_node                   (L237)
│       parameters: config/ekf.yaml
├── IncludeLaunchDescription  clearpath_gz/robot_spawn.launch.py            (L201)
│       └── arranca toda la pila Clearpath (controllers UR5e + Husky + RViz)
└── TimerAction(20s)  inspection_teleop_limits                              (L251)
        4× ros2 param set en /robot/teleop_twist_joy_node
```

Notas:

- Es el único launcher que carga el mundo SDF y el robot.
- No arranca `scan_gate` ni `map_accumulator` ni nodos de inspección.
- El `ekf_node` propio convive con el EKF de Clearpath (`/robot/platform/odom/filtered`). El comentario L236 indica que en operación se usa la odometría Clearpath, no la de este EKF. Posible redundancia (ver `DOCS_CONSISTENCY.md`).

---

## 3. `inspection.launch.py` — detalle

Evidencia: `ros2_ws/src/wind_tower_inspection_behaviour/launch/inspection.launch.py`

```
inspection.launch.py
├── DeclareLaunchArgument × 60 aprox  (todos los gains y umbrales)        (L139-449)
├── Node wind_tower_bringup/cylinder_localizer    if use_cylinder_localizer (L451)
├── Node wind_tower_bringup/dualsense_joy         if use_ps5               (L458)
├── Node wind_tower_bringup/ps5_teleop            if use_ps5               (L468)
├── Node wind_tower_inspection_behaviour/stability_monitor  if use_stability_monitor (L479)
│       params: stability_monitor.yaml + overrides
├── Node wind_tower_inspection_behaviour/cylindrical_map    if use_cylindrical_map   (L493)
│       params: inspection_params.yaml
└── Node wind_tower_inspection_behaviour/state_machine      if use_state_machine     (L501)
        params: state_machine.yaml + ~50 overrides desde launch args
```

Notas:

- No incluye `simulation.launch.py`. El operador debe lanzar `simulation.launch.py` antes en otra terminal (ver `LAUNCH_GUIDE.md`).
- Por defecto `state_machine_auto_start=false`. Se arranca con el botón Triángulo PS5 o publicando `/inspection/mission_command START_AUTO`.
- Cantidad masiva de `DeclareLaunchArgument` (≈60) que duplican defaults presentes ya en `state_machine.yaml`. Es una decisión consciente para permitir override por CLI; se puede simplificar en BUILD, pero no se debe romper la interfaz operativa actual.

---

## 4. `slam.launch.py` — detalle (LEGACY PROBABLE)

Evidencia: `ros2_ws/src/wind_tower_bringup/launch/slam.launch.py`

```
slam.launch.py
├── Node slam_toolbox/async_slam_toolbox_node       (L20)
│       parameters: config/slam_toolbox.yaml, use_sim_time:true
├── TimerAction(5s) → ros2 lifecycle set configure  (L28)
└── TimerAction(9s) → ros2 lifecycle set activate   (L34)
```

Problemas verificables:

- El comentario L3-L4 dice "scan_gate (en simulation.launch.py) garantiza que /scan...". **Falso a fecha actual**: `simulation.launch.py` NO lanza `scan_gate`, y el LiDAR del pipeline activo es `/velodyne_points` (PointCloud2 3D), no `/scan` (LaserScan 2D).
- `slam_toolbox` necesita `/scan` (LaserScan). Sin un nodo que convierta de PointCloud2 a LaserScan, este launcher no puede mapear nada hoy.
- `slam_toolbox.yaml` se instala pero no se referencia en docs operativas.

Clasificación: `LEGACY PROBABLE`. No borrar; mover a legacy tras confirmación humana (ver `CLEANUP_PLAN.md`).

---

## 5. `rtabmap.launch.py` — detalle (AUXILIAR)

Evidencia: `ros2_ws/src/wind_tower_bringup/launch/rtabmap.launch.py`

```
rtabmap.launch.py
├── Node rtabmap_odom/icp_odometry              (L16)
│       remap: scan_cloud → /velodyne_points
├── Node rtabmap_slam/rtabmap                   (L41)
│       remap: scan_cloud → /velodyne_points, odom → /odom
└── Node rtabmap_viz/rtabmap_viz                (L78)
```

Estado:

- Consume `/velodyne_points` (existe) y publica odom ICP. Loop closure explícitamente desactivado por simetría del cilindro (comentario L4).
- Se documenta como "secundario" en `PROJECT_STATE.md:68` y `PROJECT_PLAN.md:115`.
- Decisión arquitectónica explícita (`PROJECT_STATE.md:221-225`): el ángulo del tubo NO debe venir de ICP, sino del encoder del virador. Por tanto rtabmap.launch.py **no es parte del MVP**.

Clasificación: `AUXILIAR`. Útil para experimentos LiDAR puros. No mover a legacy todavía: depende de si se quiere conservar para experimentos comparativos.

---

## 6. Topics/remappings clave (de los launchers activos)

| Topic ROS | Origen / mapeo | Fuente |
|---|---|---|
| `/clock` | gz_bridge | simulation.launch.py:87 |
| `/robot_description` | relay desde `/robot/robot_description` | simulation.launch.py:95 |
| `/tf` | relay desde `/robot/tf` | simulation.launch.py:103 |
| `/turner/cmd_vel` | bridge → gz JointController | simulation.launch.py:118 |
| `/turner/joint_state` | bridge desde gz | simulation.launch.py:133 |
| `/velodyne_points` | bridge desde gz `/robot/sensors/lidar3d_0/scan/points` | simulation.launch.py:149 |
| `/robot/sensors/inspection_camera/image` | image_bridge | simulation.launch.py:165 |
| `/inspection/camera/image_raw` | relay desde imagen inspección | simulation.launch.py:175 |
| `/inspection/camera/camera_info` | bridge | simulation.launch.py:186 |
| `/robot/sensors/imu_0/data` | imu_bridge | simulation.launch.py:225 |

---

## 7. Conclusión de mapa real

- **Fuente de verdad operativa**: `simulation.launch.py` + `inspection.launch.py`.
- **Sin uso en flujo MVP**: `slam.launch.py` (roto vs pipeline actual), `rtabmap.launch.py` (experimental aislado).
- **Sin uso en ningún launcher**: nodos `scan_gate`, `map_accumulator` aunque sigan instalados como entry points.
