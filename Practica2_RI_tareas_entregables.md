# Practica 2 RI - Tareas y entregables

Resumen operativo del documento `Practica2_RI_compressed.pdf` (Navegacion Autonoma con NAV2 en ROS 2 Humble).

## Nota sobre el alcance de esta rama

Esta rama `practica-2` contiene el repositorio completo del TFM de inspeccion de torres eolicas. La memoria y el video de Practica 2 solo cubren el subconjunto que aporta a la rubrica de NAV2 (mundo + SLAM + EKF + AMCL + costmaps + planner/controller + TF + validacion en Gazebo). El resto del codigo (`wind_tower_arm_control`, `wind_tower_perception`, `wind_tower_inspection_behaviour` con BT/voz/langchain) forma parte de otras practicas del proyecto y no se entrega en el scope de la Practica 2 aunque coexista en la rama.

## Robot entregado

Igual que en Practica 1, se utiliza el **Husky A200 de Clearpath** en lugar del Tracer AgileX sugerido en algunos ejemplos. Razon: continuidad con el resto del TFM. La configuracion del robot vive en `ros2_ws/src/wind_tower_bringup/config/robot.yaml` y se sincroniza a `~/clearpath/robot.yaml` al lanzar la simulacion.

## Objetivo general (PDF §1 y §2)

Construir el stack completo de navegacion autonoma en ROS 2 Humble usando NAV2:

- Mapeo del entorno con SLAM Toolbox.
- Fusion sensorial IMU + odometria con filtro EKF (`robot_localization`).
- Localizacion global probabilistica con AMCL.
- Planificacion global (NavFn / Smac) y control local (DWB / Regulated Pure Pursuit).
- Mapas de coste local y global (static, obstacle, inflation layers).
- Validacion completa en Gazebo Sim con envio de metas desde RViz y mision con waypoints.

## Resultado final esperado

Lanzando los launchers de `wind_tower_bringup`:

- `simulation.launch.py` arranca Gazebo + Husky + bridges.
- `slam.launch.py` mapea el entorno y guarda el mapa.
- `navigation_amcl.launch.py` carga el mapa, activa AMCL y el stack NAV2, recibe metas en RViz y mueve el robot evitando obstaculos.
- `mission_navigator` (nodo del paquete) lee `waypoints.yaml` y ejecuta una secuencia de metas.

## 1. Configuracion de launchers (PDF §3)

### 1.1 Estructura del paquete `wind_tower_bringup`

```
wind_tower_bringup/
|-- launch/
|   |-- simulation.launch.py        # Gazebo + Husky + bridges (de Practica 1)
|   |-- slam.launch.py              # SLAM Toolbox sync + RViz con slam.rviz
|   |-- navigation.launch.py        # NAV2 sin AMCL (con EKF + slam online)
|   |-- navigation_amcl.launch.py   # NAV2 + AMCL + mapa existente + behaviour layer (TFM)
|-- config/
|   |-- robot.yaml                  # Husky + sensores (Clearpath)
|   |-- ekf_map.yaml                # robot_localization
|   |-- amcl_params.yaml            # AMCL
|   |-- nav2_params.yaml            # planner + controller + costmaps + smoother
|   |-- slam_params.yaml            # SLAM Toolbox
|   |-- pointcloud_to_laserscan.yaml
|   |-- waypoints.yaml              # Misiones predefinidas
|   |-- navigation.rviz             # RViz para navegacion
|   |-- slam.rviz                   # RViz para SLAM
ros2_ws/maps/
|-- wind_tower.pgm + .yaml + .posegraph + .data    # Mapa pre-generado
```

### 1.2 Buenas practicas seguidas

- `DeclareLaunchArgument` para `use_sim_time`, ruta del mapa, ruta de params.
- `get_package_share_directory()` para rutas relativas.
- `use_sim_time: true` en todos los nodos.
- `lifecycle_manager` con `autostart: true` para activar AMCL/Planner/Controller/BT en orden.
- `set_cyclonedds_uri` apuntando a `tools/cyclonedds.xml` para subir el limite de DDS participants (Clearpath + NAV2 superan los 64 slots por defecto).
- Relays de `/robot/tf` y `/robot/robot_description` a sus equivalentes raiz (`/tf`, `/robot_description`) por la coexistencia de namespace Clearpath con el stack NAV2.

### 1.3 Errores comunes a evitar (PDF §3.7)

- No activar `lifecycle_manager` -> nodos en `unconfigured`.
- No declarar `use_sim_time` -> TF se desfasa con respecto a `/clock`.
- Conflictos de namespace entre Clearpath (`/robot/...`) y NAV2 (`/`).
- Rutas a YAML incorrectas en `share/<paquete>/config/`.

Comando util para diagnosticar lifecycle:

```bash
ros2 lifecycle list
ros2 lifecycle set /controller_server activate
```

## 2. LIDAR e IMU (PDF §4)

### 2.1 LIDAR

- Topico ROS: `/scan` (`sensor_msgs/msg/LaserScan`).
- En el Husky el LIDAR 2D nativo seria el Hokuyo, pero el TFM usa el Velodyne VLP-16 3D montado en `top_plate_link` y se convierte a 2D mediante `pointcloud_to_laserscan` (config en `pointcloud_to_laserscan.yaml`).
- Parametros criticos del sensor revisados: `frame_id`, `angle_min/max`, `angle_increment`, `range_min/max`, `update_rate`.
- TF estatica relevante: `base_link -> velodyne` y de ahi a `velodyne_link`.

### 2.2 IMU

- Topico ROS: `/imu/data` (`sensor_msgs/msg/Imu`).
- Sensor declarado en `robot.yaml` (Clearpath): `phidgets_spatial` sobre `base_link`.
- Mide aceleracion lineal, velocidad angular y orientacion (cuaternion).
- Sus covarianzas deben estar bien definidas para que el EKF las pondere correctamente.

### 2.3 Fusion EKF (PDF §4.6)

Vector de estado para robot diferencial planar:

```
x = [x, y, theta, vx, omega]
```

Salida del EKF: TF `odom -> base_link` continua y suave + topico `/odometry/filtered`.

`ekf_map.yaml` declara:

- `frequency: 30.0`
- `sensor_timeout: 0.1`
- `two_d_mode: true`
- `publish_tf: true`
- `world_frame: odom`
- Matrices `_config` que definen que componentes se fusionan de odometria de las ruedas y de IMU.
- Matriz diagonal de covarianza de proceso ajustada a las dinamicas del Husky (incluyendo varianza Z baja porque el robot es planar).

Interaccion con AMCL:

- EKF produce `odom -> base_link` (estimacion local continua).
- AMCL produce `map -> odom` (correccion global probabilistica).
- Ambos transformaciones se componen sin pisarse gracias al desacople de frames.

## 3. AMCL (PDF §5)

### 3.1 Concepto

Filtro de particulas que aproxima la posterior `p(x_t | z_1:t, u_1:t)` con N particulas; cada particula es una hipotesis de pose. Numero ajustado dinamicamente (KLD-sampling) entre `min_particles` y `max_particles`.

### 3.2 Configuracion (`amcl_params.yaml`)

Valores clave en uso:

```yaml
amcl:
  ros__parameters:
    use_sim_time: true
    min_particles: 500
    max_particles: 2000
    alpha1: 0.2  # ruido rotacion por rotacion
    alpha2: 0.2  # ruido rotacion por traslacion
    alpha3: 0.2  # ruido traslacion por traslacion
    alpha4: 0.2  # ruido traslacion por rotacion
    laser_z_hit: 0.9
    laser_sigma_hit: 0.2
    update_min_d: 0.25
    update_min_a: 0.2
```

### 3.3 Flujo de operacion en RViz

1. `map_server` publica el mapa estatico.
2. `2D Pose Estimate` -> nube de particulas dispersa.
3. Tras unos movimientos, las particulas convergen.
4. AMCL publica `map -> odom`.
5. `Nav2 Goal` envia metas.

## 4. Mapas y costmaps (PDF §6)

### 4.1 Mapa estatico

- `wind_tower.yaml` con resolucion, origen y umbrales.
- `wind_tower.pgm` imagen de ocupacion.
- `wind_tower.posegraph` y `wind_tower.data` son artefactos del SLAM Toolbox.
- Publicado por `nav2_map_server`.

### 4.2 Costmaps

- **Global costmap** en `map` frame con `static_layer + obstacle_layer + inflation_layer`.
- **Local costmap** en `odom` frame, rolling window, con `obstacle_layer + inflation_layer`.
- Inflacion exponencial alrededor de obstaculos: `c(d) = c_max * exp(-lambda * d)`.

## 5. Planner y controller (PDF §7)

### 5.1 Planner global

Configurado `NavFn` (Dijkstra) en `nav2_params.yaml`. Adecuado para Husky diferencial.

### 5.2 Controller local: tuning iterativo (relevante para la memoria)

El controller en uso es `RegulatedPurePursuitController`. La rampa de 13° del mundo nos obligo a iterar la configuracion en dos fases:

**Hipotesis 1 (descartada):** "Subir velocidad y aceleracion para vencer la pendiente".

- `desired_linear_vel: 0.5`, `max_accel: 2.5`, `max_velocity: 0.6`.
- Resultado: el robot patinaba en los giros sobre la rampa, AMCL perdia coherencia y el odom saltaba.

**Hipotesis 2 (actual):** "Bajar velocidad y aceleracion, priorizar suavidad".

- `lookahead_dist: 0.9 -> 1.2`, `max_lookahead_dist: 1.5 -> 2.0`, `min_lookahead_dist: 0.4 -> 0.6` (anti-bandazos: horizonte mas largo amortigua el seguimiento del path).
- `rotate_to_heading_angular_vel: 0.7 -> 0.5`, `max_angular_accel: 2.0 -> 1.2` (suavidad angular).
- `rotate_to_heading_min_angle: 45° -> 60°` (menos giros in-situ innecesarios).
- `velocity_smoother`: `max_accel: 2.5 -> 0.75` lineal y `1.0 -> 0.8` angular para evitar slip en rampa.
- `min_approach_linear_velocity: 0.15 -> 0.06` y `approach_velocity_scaling_dist: 0.6 -> 0.9` para aproximacion suave que no derrape al frenar.

Lecciones aprendidas para la memoria:

1. El problema principal en la rampa no era falta de potencia sino tracción. Mas velocidad amplifica el slip.
2. El `RegulatedPurePursuitController` es muy sensible al horizonte de lookahead: 0.4 m amplifica ruido, 1.2 m suaviza.
3. La aceleracion del `velocity_smoother` debe ser coherente con el resto de la cadena (smoother -> twist_mux Clearpath -> diff-drive). Si una etapa satura, otras no compensan.

### 5.3 Controller alternativo

DWB Controller queda configurado en `nav2_params.yaml` como backup. Evalua trayectorias y minimiza `J(v, w) = alpha * d_path + beta * d_goal + gamma * c_obs`.

## 6. TF (PDF §8)

Cadena estandar NAV2 + Clearpath:

```
map -> odom -> base_link -> velodyne / imu_link / wheels
```

- `map -> odom`: AMCL.
- `odom -> base_link`: EKF (`robot_localization`).
- `base_link -> sensores`: estatico, publicado por `robot_state_publisher` desde el URDF generado por Clearpath.
- Una transformacion rigida valida cumple `R^T R = I` y `det(R) = 1`. Si una intermedia esta mal, AMCL no converge y costmap se descalibra.

Diagnostico:

```bash
ros2 run tf2_tools view_frames   # genera frames.pdf
```

## 7. Comandos clave

Mapeo (SLAM):

```bash
ros2 launch wind_tower_bringup simulation.launch.py
ros2 launch wind_tower_bringup slam.launch.py
# Conducir manualmente con ps5_teleop o teleop_twist_keyboard.
ros2 run nav2_map_server map_saver_cli -f ros2_ws/maps/wind_tower
```

Navegacion con mapa existente:

```bash
ros2 launch wind_tower_bringup simulation.launch.py
ros2 launch wind_tower_bringup navigation_amcl.launch.py
# En RViz: "2D Pose Estimate" -> "Nav2 Goal".
```

Mision con waypoints:

```bash
ros2 run wind_tower_bringup mission_navigator
```

## 8. Pruebas en Gazebo para la practica (PDF §9)

Pasos minimos:

1. Lanzar Husky en `wind_tower_world.sdf`.
2. Generar mapa con SLAM o usar el pregenerado en `ros2_ws/maps/`.
3. Activar navegacion AMCL.
4. Enviar metas desde RViz y/o ejecutar `mission_navigator`.
5. Evaluar estabilidad y precision cuantitativamente.

Metricas a recoger:

- Error medio de posicion (RMSE entre AMCL pose y ground truth de Gazebo).
- Tiempo hasta convergencia de AMCL tras inicializacion.
- Desviacion en seguimiento de trayectoria.
- Numero de replanificaciones con obstaculo dinamico.

## 9. Entregables (PDF §10)

1. **Paquete ROS 2 completo**: rama `practica-2` (contiene todo el proyecto del TFM; solo lo de NAV2 es el scope evaluable).
2. **Archivos launch**: `simulation.launch.py`, `slam.launch.py`, `navigation.launch.py`, `navigation_amcl.launch.py`.
3. **Configuracion YAML**: `ekf_map.yaml`, `amcl_params.yaml`, `nav2_params.yaml`, `slam_params.yaml`, `pointcloud_to_laserscan.yaml`, `waypoints.yaml`.
4. **Mapas**: `ros2_ws/maps/wind_tower.*`.
5. **Video demostrativo**: detalle en `docs/practica2/video_plan.md`.
6. **Informe tecnico**: detalle en `docs/practica2/memoria.tex`.

## 10. Troubleshooting (PDF §11)

| Sintoma | Diagnostico | Solucion habitual |
| --- | --- | --- |
| `No transform from base_link to map` | `ros2 run tf2_tools view_frames` | AMCL y EKF activos; revisar `frame_id` en YAML |
| AMCL no converge | `ros2 topic echo /particle_cloud` | Subir `max_particles`, ajustar `laser_sigma_hit`, revisar mapa |
| Costmap local sin obstaculos | `ros2 topic hz /scan` | Revisar `observation_sources` y QoS; pointcloud_to_laserscan operativo |
| EKF inestable | `ros2 topic echo /odometry/filtered` | Ajustar `process_noise_covariance`, `two_d_mode: true` |
| Robot no responde en Gazebo | `ros2 topic echo /cmd_vel` | `use_sim_time=true` y `/clock` publicado |
| DDS slots agotados | `ros2 doctor --report` | `CYCLONEDDS_URI` con `file://tools/cyclonedds.xml` (sube limite de 64 a 240) |

## 11. Estructura final de la entrega de Practica 2

- Codigo: rama `practica-2` del repositorio.
- Documentos: en `docs/practica2/`.
  - `memoria.tex` -> PDF de 4-6 paginas con tuning del controller como punto fuerte.
  - `video_plan.md` -> guion del video demo (3 personas).
  - `README.md` -> indice y plan de trabajo de la rama.
- Video: enlace en la memoria.
