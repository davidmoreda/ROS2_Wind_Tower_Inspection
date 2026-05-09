# PROJECT PLAN — Wind Tower Inspection

## Objetivo

Desarrollar un sistema ROS 2 para inspección del interior de un tramo de torre eólica horizontal (cilindro Ø8m × 30m), diferenciando claramente:

| Nivel | Alcance |
|---|---|
| Simulación actual | Gazebo + Husky A200 + UR5e + Velodyne VLP-16 + virador funcional |
| MVP software | Malla cilíndrica `(x, θ)`, estabilidad por IMU, detección geométrica básica e informes |
| Prototipo real | Sensores industriales, iluminación, seguridad, calibración y validación metrológica |

La inspección se basa en **calles axiales discretas con indexado angular del tubo**. La cobertura se considera válida solo si queda registrada en una malla cilíndrica `(x, θ)` con evidencias y estados de celda. No se asume cobertura 100% garantizada por una pasada helicoidal continua.

## Concepto de operación

```
INSPECCIÓN POR CALLES AXIALES CON INDEXADO ANGULAR
─────────────────────────────────────────────────────
1. Tubo parado en θ_tubo = θ_i.
2. Robot asentado en la generatriz inferior estable.
3. AXIAL_SCAN: el robot avanza en x e inspecciona una calle.
4. Detección de obstáculos/bushings/anomalías con LiDAR 3D y cámara.
5. Si hay obstáculo: parada, inspección local, decisión de esquiva.
6. BYPASS_OBSTACLE: permitido desviarse por pared, con α_robot monitorizado por IMU.
7. RETURN_TO_BOTTOM_LANE: retorno obligatorio antes de reanudar escaneo normal.
8. Fin de calle: INDEX_TUBE gira Δθ_real con solape.
9. VERIFY_INDEXED_POSITION: se valida de nuevo bottom_lane_locked.
10. Repetir hasta 360° + margen de solape.
```

Variables operativas:

| Variable | Definición | Comentario |
|---|---|---|
| `x` | Posición axial del robot | Estimada principalmente por wheel odometry y fusionada con IMU/LiDAR |
| `θ_tubo` | Ángulo del tubo | Fuente principal: encoder del virador |
| `α_robot` | Desviación angular respecto a la generatriz inferior | Estimada con IMU y geometría |
| `bottom_lane_locked` | Robot asentado en generatriz inferior estable | Requisito para escaneo normal |
| `safe_to_scan` | Estable, alineado y dentro de inclinación umbral | Requisito de `AXIAL_SCAN` |
| `safe_to_index_tube` | Postura segura para girar el tubo | Requisito de `INDEX_TUBE` |

## Criterios de estabilidad

La orientación del robot respecto a la gravedad es crítica. La IMU es obligatoria para estimar roll/pitch, detectar desviación por pared y validar `α_robot`. Para considerar que el robot está en la generatriz inferior, el eje Z del robot debe estar alineado con la aceleración de la gravedad dentro de umbral.

Parámetros iniciales propuestos:

```yaml
bottom_lane:
  max_roll_deg: 5.0
  max_pitch_deg: 5.0
  max_lateral_angle_deg: 3.0
  max_slip_ratio: 0.25
```

Reglas por estado:

| Estado | Criterio |
|---|---|
| `AXIAL_SCAN` | roll/pitch dentro de umbral, heading alineado con eje axial, `bottom_lane_locked == true`, `safe_to_scan == true` |
| `BYPASS_OBSTACLE` | `α_robot` puede ser distinto de cero, velocidad reducida, cobertura marcada como modo especial |
| `RETURN_TO_BOTTOM_LANE` | obligatorio tras esquiva antes de volver a escaneo normal |
| `INDEX_TUBE` | `safe_to_index_tube == true`; después se verifica `bottom_lane_locked` |

## Cobertura verificable en malla (x, θ)

La cobertura se acumula en una malla cilíndrica. El encoder del virador proporciona `θ_tubo`, evitando usar ICP para inferir rotación en una geometría cilíndrica simétrica.

Estados mínimos de celda:

| Estado celda | Cuenta como inspección normal | Uso |
|---|---:|---|
| `unseen` | No | Sin datos |
| `observed_lidar` | Parcial | Geometría detectada |
| `inspected_rgb` | Sí | Imagen válida con iluminación controlada |
| `blocked` | No | Obstáculo impide inspección |
| `bypass_mode` | No | Robot estaba esquivando |
| `needs_review` | No | Revisión local/manual necesaria |

Cálculo de paso angular:

```text
Δθ = ancho_útil_superficie / R
Δθ_real = Δθ · (1 - overlap)
N_calles ≈ 360° / Δθ_real
```

“Una sola vuelta” significa rotación acumulada total del tubo de `360° + margen de solape`, no giro continuo durante toda la inspección.

## Arquitectura de software

### Paquetes existentes

```
wind_tower_description
  meshes/TRAMO_TORRE.STL       ← geometría del tubo

wind_tower_simulation
  worlds/wind_tower_world.sdf  ← nave + tubo dinámico con joint revolute

wind_tower_bringup
  simulation.launch.py         ← Gazebo + robot + bridges + virador + IMU/EKF
  rtabmap.launch.py            ← mapeo RTAB-Map secundario
  slam.launch.py               ← SLAM Toolbox, no usado actualmente
  dualsense_joy.py             ← driver mando PS5 por evdev
  ps5_teleop.py                ← teleop UR5e + virador
  teleop_twist_joy_node        ← teleop Husky vía Clearpath
  turner_node.py               ← publica θ_tubo en /turner/angle y /turner/angle_deg
  scan_gate_node.py            ← filtro sectorial LaserScan
  cylinder_localizer_node.py   ← prototipo localización por ajuste de cilindro LiDAR
  map_accumulator_node.py      ← prototipo acumulador PointCloud2
```

### Paquete pendiente

```
wind_tower_inspection_behaviour
  inspection_state_machine_node.py
  stability_monitor_node.py
  bottom_lane_controller.py
  tube_indexing_controller.py
  cylindrical_map_node.py
  bushing_detector_node.py
  local_inspection_controller.py
  anomaly_detector_node.py
  report_generator_node.py
  inspection.launch.py
  inspection_params.yaml
```

El anterior `coverage_controller.py` helicoidal queda retirado como controlador de velocidad continua. Si se conserva el concepto, debe renombrarse a `coverage_manager` y limitarse a evaluar cobertura, huecos y calidad de evidencias en la malla `(x, θ)`.

## Nodos propuestos de inspección

| Nodo | Responsabilidad | Entradas | Salidas | Estado |
|---|---|---|---|---|
| `inspection_state_machine_node.py` | Ejecutar la máquina de estados | `/inspection/stability`, `/inspection/obstacles`, cobertura, odom, `θ_tubo` | `/inspection/state`, comandos de alto nivel | PENDIENTE |
| `stability_monitor_node.py` | Validar gravedad, inclinación, movimiento y geometría local | `/robot/sensors/imu_0/data`, `/robot/platform/odom/filtered`, `/cylinder_fit/stats`, `/turner/angle` | `/inspection/stability`, `bottom_lane_locked`, `safe_to_scan`, `safe_to_index_tube` | MVP validado |
| `bottom_lane_controller.py` | Alinear robot con generatriz inferior | Estabilidad, LiDAR, odom | Comando base, estado alineación | PENDIENTE |
| `tube_indexing_controller.py` | Girar el tubo por incrementos `Δθ_real` | `/turner/angle`, petición `INDEX_TUBE` | `/turner/cmd_vel`, `/inspection/index_status` | PENDIENTE |
| `cylindrical_map_node.py` | Construir malla `(x, θ)` con estados de celda | LiDAR, cámara, `x`, `θ_tubo`, modo operación | `/inspection/cylindrical_map`, métricas cobertura | PENDIENTE |
| `bushing_detector_node.py` | Detectar bushings, salientes y obstáculos | `/velodyne_points`, malla geométrica | `/inspection/obstacles`, markers RViz | PENDIENTE |
| `local_inspection_controller.py` | Gestionar parada, adquisición local y esquiva | Obstáculos, cámara, LiDAR, estado robot | Comandos base/brazo, evidencias | PENDIENTE |
| `anomaly_detector_node.py` | Detectar anomalías geométricas/visuales | LiDAR, cámara RGB, iluminación | `/inspection/anomalies` | PENDIENTE |
| `report_generator_node.py` | Consolidar informe | Cobertura, anomalías, evidencias, poses | `/inspection/report`, JSON/PDF | PENDIENTE |

## Máquina de estados propuesta

| Estado | Descripción | Condición de salida |
|---|---|---|
| `IDLE` | Sistema parado, esperando misión | Misión iniciada |
| `ALIGN_TO_BOTTOM_LANE` | Alinear robot con generatriz inferior | Error angular dentro de umbral |
| `VERIFY_BOTTOM_LOCK` | Confirmar estabilidad y gravedad | `bottom_lane_locked == true` |
| `AXIAL_SCAN` | Calle axial normal | Fin de calle u obstáculo |
| `OBSTACLE_DETECTED` | Parada segura ante detección | Clasificación local |
| `LOCAL_INSPECTION` | Captura LiDAR/cámara de detalle | Decisión: continuar, esquivar o revisar |
| `BYPASS_OBSTACLE` | Maniobra de esquiva monitorizada | Obstáculo superado |
| `RETURN_TO_BOTTOM_LANE` | Recuperar generatriz inferior | `bottom_lane_locked == true` |
| `INDEX_TUBE` | Giro discreto del tubo | `θ_tubo` alcanza objetivo |
| `VERIFY_INDEXED_POSITION` | Verificar asentamiento tras giro | `safe_to_scan == true` |
| `FINISH` | Inspección cerrada | Informe generado |

## Sensores necesarios

| Sensor | Obligatorio | Estado | Comentario |
|---|---:|---|---|
| Encoder virador | Sí | Funcional en simulación vía joint state | Fuente principal de `θ_tubo` |
| Wheel odometry | Sí | Disponible en Husky/Clearpath | Fuente principal de `x`; requiere validación de topic exacto |
| IMU | Sí | URDF/bridge/EKF configurados; validación pendiente | Estabilidad, gravedad, roll/pitch, `α_robot` |
| LiDAR 3D | Sí | `/velodyne_points` verificado | Geometría, obstáculos, bushings, cobertura y seguridad |
| Cámara RGB industrial | Sí para inspección superficial | Pendiente | Debe ir con iluminación controlada |
| Iluminación controlada | Sí para visión fiable | Pendiente | Frontal/difusa y rasante lateral |
| Joint states UR5e | Sí si cámara va en brazo | Controlador disponible | Pose de cámara y trazabilidad |
| Distancia corta end-effector | Recomendado | Futuro | Seguridad y control de distancia |
| Bumper/contacto | Recomendado | Futuro | Detección de colisión |
| E-stop | Obligatorio en real | Futuro | Seguridad industrial |

## Stack tecnológico

| Capa | Tecnología / Estado |
|------|-----------|
| Simulación | Gazebo Harmonic + ROS 2 Jazzy |
| Robot | Husky A200 + UR5e (Clearpath) |
| LiDAR | Velodyne VLP-16, `/velodyne_points`, 20Hz verificado |
| Virador | Gazebo JointController + ros_gz_bridge + `turner_node.py` |
| Localización angular | Encoder virador / joint state, no ICP |
| Localización axial | Wheel odometry + IMU + LiDAR en EKF, en validación |
| Mapeo | Malla cilíndrica `(x, θ)` pendiente |
| Inspección visual | Cámara RGB industrial + iluminación, pendiente |
| Seguridad | Monitor de estabilidad por IMU/LiDAR, pendiente |

## Fases

### Completadas

| Fase | Descripción |
|------|-------------|
| 0 | Auditoría entorno: ROS 2 Jazzy + Gazebo Harmonic + WSL2 |
| 1 | Estructura repo + paquetes base |
| 2 | Robot compuesto Husky + UR5e vía Clearpath generator |
| 3 | Visualización RViz + TF operativo |
| 4 | Simulación Gazebo con nave industrial + tubo STL |
| 5 | Velodyne VLP-16 integrado en `/velodyne_points` |
| 6 | Control base móvil + brazo UR5e + virador por mando DualSense |
| 7 | Tubo dinámico con joint revolute y JointController Gazebo |
| 8 | Virador funcional: `/turner/cmd_vel`, `/turner/angle`, `/turner/angle_deg` |

### En desarrollo / validación

| Fase | Descripción | Prioridad |
|------|-------------|-----------|
| 9A | Validar IMU simulada y odometría Clearpath `/robot/platform/odom/filtered` | Alta |
| 9B | Validar `cylinder_localizer_node.py` y `map_accumulator_node.py` como prototipos LiDAR | Alta |
| 9C | Crear paquete `wind_tower_inspection_behaviour` | Completada |
| 9D | Implementar y validar `stability_monitor_node.py` con `bottom_lane_locked`, `safe_to_scan`, `safe_to_index_tube` | Completada MVP |
| 9E | Implementar `inspection_state_machine_node.py` con estados discretos | Alta |
| 9F | Implementar `tube_indexing_controller.py` para indexado angular | Alta |
| 9G | Implementar `cylindrical_map_node.py` como malla `(x, θ)` | Alta |
| 9H | Implementar `bushing_detector_node.py` con LiDAR | Media |
| 9I | Añadir cámara RGB industrial al URDF/bridge | Media |
| 9J | Implementar `local_inspection_controller.py` y política de esquiva | Media |
| 9K | Implementar `anomaly_detector_node.py` básico | Media |
| 9L | Implementar `report_generator_node.py` JSON + RViz markers | Media |

### Futuro — CV/ML avanzado

| Fase | Descripción |
|------|-------------|
| 10A | Detector visual de soldaduras y grietas |
| 10B | Segmentación de corrosión |
| 10C | Medición geométrica precisa de bushings |
| 10D | Validación con muestras reales de defectos |

### Futuro — Integración real

| Fase | Descripción |
|------|-------------|
| 11A | Calibración cámara-LiDAR-extrínseca brazo |
| 11B | Adaptación a dinámica real del virador |
| 11C | Dashboard operador |
| 11D | Informe normalizado y trazabilidad industrial |
| 11E | Bumper, sensor corto, E-stop y análisis de riesgos |

## Topics del sistema

| Topic | Tipo | Estado | Descripción |
|-------|------|--------|-------------|
| `/velodyne_points` | `sensor_msgs/PointCloud2` | Verificado | Nube LiDAR 3D |
| `/turner/cmd_vel` | `std_msgs/Float64` | Verificado | Velocidad angular virador |
| `/turner/angle` | `std_msgs/Float64` | Verificado | `θ_tubo` acumulado en rad |
| `/turner/angle_deg` | `std_msgs/Float64` | Verificado | `θ_tubo` en grados módulo 360 |
| `/turner/joint_state` | `sensor_msgs/JointState` | Verificado | Estado real del joint del virador desde Gazebo |
| `/robot/joy_teleop/joy` | `sensor_msgs/Joy` | Verificado | Mando PS5 DualSense |
| `/robot/platform/odom` | `nav_msgs/Odometry` | Configurado | Odometría ruedas usada por EKF |
| `/robot/sensors/imu_0/data` | `sensor_msgs/Imu` | Configurado, validar | IMU simulada |
| `/robot/platform/odom/filtered` | `nav_msgs/Odometry` | Verificado | Odometría filtrada Clearpath |
| `/robot_in_tube` | `geometry_msgs/PoseStamped` | Prototipo | Localización por ajuste de cilindro |
| `/cylinder_fit/wall_points` | `sensor_msgs/PointCloud2` | Prototipo | Puntos usados para fit de pared |
| `/cylinder_fit/stats` | `std_msgs/String` | Prototipo | Diagnóstico del fit |
| `/map_cloud` | `sensor_msgs/PointCloud2` | Prototipo | Mapa acumulado de puntos |
| `/map_cloud/stats` | `std_msgs/String` | Prototipo | Estadísticas de acumulación |
| `/inspection/stability` | `std_msgs/String` JSON | Verificado MVP | Estado de estabilidad |
| `/inspection/bottom_lane_locked` | `std_msgs/Bool` | Verificado MVP | Robot asentado en generatriz inferior |
| `/inspection/safe_to_scan` | `std_msgs/Bool` | Verificado MVP | Condición para escaneo axial |
| `/inspection/safe_to_index_tube` | `std_msgs/Bool` | Verificado MVP | Condición para indexar tubo |
| `/inspection/cylindrical_map` | Por definir | PENDIENTE | Malla `(x, θ)` |
| `/inspection/obstacles` | Por definir | PENDIENTE | Obstáculos/bushings |
| `/inspection/anomalies` | Por definir | PENDIENTE | Anomalías detectadas |
| `/inspection/report` | JSON/String o fichero | PENDIENTE | Informe final |

## Parámetros clave propuestos (`inspection_params.yaml`)

```yaml
tube:
  radius: 4.0
  length: 30.0

coverage:
  useful_surface_width: 0.30
  overlap: 0.15
  min_valid_observations_per_cell: 2
  mark_bypass_as_normal_coverage: false

bottom_lane:
  max_roll_deg: 5.0
  max_pitch_deg: 5.0
  max_lateral_angle_deg: 3.0
  max_slip_ratio: 0.25

axial_scan:
  axial_speed: 0.05
  max_heading_error_deg: 3.0

bypass:
  axial_speed: 0.02
  max_alpha_robot_deg: 20.0
  require_return_to_bottom_lane: true

tube_indexing:
  angular_speed: 0.05
  settle_time_s: 2.0
  angle_tolerance_deg: 0.5

surface_inspection:
  camera_target_distance: 0.25
  require_controlled_lighting: true

anomaly:
  geometry_threshold_m: 0.05
```
