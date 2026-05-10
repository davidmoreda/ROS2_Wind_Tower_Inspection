# PROJECT PLAN — Wind Tower Inspection

## Objetivo

Desarrollar un sistema ROS 2 para inspeccionar el interior de un tramo de torre eólica horizontal, diferenciando claramente tres niveles:

| Nivel | Alcance |
|---|---|
| Simulación actual | Gazebo Harmonic + ROS 2 Jazzy + Husky A200 + UR5e + Velodyne VLP-16 + virador |
| MVP software | Calles axiales discretas, estabilidad por IMU, malla cilíndrica `(x, θ)`, captura trazable e informe inicial |
| Prototipo real | Cámara industrial, iluminación física, seguridad, calibración, ensayos y validación metrológica |

La metodología de referencia es:

**Inspección por calles axiales discretas con indexado angular del tubo.**

No se usa como base una pasada helicoidal continua. No se documenta “100% garantizado” sin métrica. La formulación correcta es:

**Cobertura verificable sobre malla cilíndrica `(x, θ)`, generada mediante calles axiales discretas con indexado angular del tubo y métricas de confianza.**

## Concepto de Operación

1. El tubo permanece parado en un ángulo `θ_tube = θ_i`.
2. El robot se coloca en la generatriz inferior estable del tubo.
3. El robot realiza una pasada axial por esa calle.
4. Durante la pasada detecta obstáculos, bushings, elementos soldados, salientes o anomalías.
5. Si detecta algo, se detiene, inspecciona localmente y decide si puede esquivar.
6. Durante `BYPASS_OBSTACLE` puede desviarse parcialmente por la pared, pero no marca cobertura nominal.
7. Tras el bypass debe volver obligatoriamente a la generatriz inferior.
8. Al terminar la calle, el virador gira el tubo `Δθ` con solape.
9. El robot verifica estabilidad y alineación.
10. Se repite hasta cubrir `360° + margen de solape`.

Regla de operación:

```text
tubo parado + robot avanza en x = AXIAL_SCAN de una calle
fin de calle + robot seguro = INDEX_TUBE
indexado completado + bottom lane verificado = siguiente calle
```

## Variables Clave

| Variable | Definición | Fuente / comentario |
|---|---|---|
| `x_robot` | Posición axial del robot | Odometría de ruedas, fusionable con IMU en `robot_localization` |
| `θ_tube` | Ángulo material del tubo | Encoder del virador; en simulación `/turner/angle` y `/turner/angle_deg` |
| `θ_surface` | Ángulo/celda de superficie inspeccionada | Derivado de `θ_tube`, lane actual y geometría de cobertura |
| `α_robot` | Desviación angular del robot respecto a la generatriz inferior | IMU y/o geometría LiDAR; proxy inicial por roll/lateral angle |
| `roll_deg` / `pitch_deg` | Orientación respecto a gravedad | IMU obligatoria |
| `bottom_lane_locked` | Robot correctamente asentado en generatriz inferior | Publicado por monitor de estabilidad |
| `safe_to_scan` | Se puede marcar cobertura nominal | Solo true con estabilidad, alineación y umbrales correctos |
| `safe_to_index_tube` | El tubo puede girar sin riesgo para el robot | Solo true con postura segura y robot prácticamente parado |

No confundir `θ_tube` con `α_robot`: `θ_tube` describe el tubo y viene del virador; `α_robot` describe cuánto se ha desviado el robot respecto a la zona inferior estable.

## Sensores

| Sensor | Obligatorio | Estado actual | Uso |
|---|---:|---|---|
| Encoder virador | Sí | Funcional en simulación vía `/turner/joint_state` | Fuente autoritativa de `θ_tube` |
| Odometría ruedas | Sí | `/robot/platform/odom` y `/robot/platform/odom/filtered` disponibles | Estimación principal de `x_robot` |
| IMU | Sí | `/robot/sensors/imu_0/data` validado | Roll, pitch, gravedad, estabilidad, `α_robot` aproximado |
| LiDAR 3D | Sí | `/velodyne_points` verificado | Geometría de cilindro, obstáculos, bushings, pared, cobertura y seguridad |
| Cámara RGB industrial | Sí para inspección superficial | Simulada en TCP del UR5e; real futura Basler/industrial | Evidencia visual trazable |
| Iluminación controlada | Sí para visión fiable | Simulada con luces rasantes; física futura | Frontal/difusa, rasante izquierda/derecha, multi-iluminación |
| Joint states UR5e | Sí si cámara va en brazo | Controlador UR5e disponible | Pose de cámara/luz |
| Sensor corto en end-effector | Recomendado | Futuro | Distancia cámara-superficie y seguridad |
| Bumper/contacto | Recomendado | Futuro | Detección de contacto |
| E-stop físico | Obligatorio en real | Futuro | Seguridad industrial |

## Decisiones Técnicas

| Área | Decisión |
|---|---|
| Nav2 | No usar Nav2 completo como núcleo del MVP. Puede aportar piezas después. |
| Nav2 Collision Monitor | Candidato futuro como capa de seguridad. |
| Regulated Pure Pursuit / MPPI | Candidatos posteriores si el controlador propio queda corto. |
| TEB | No priorizar. |
| RL | No implementar en MVP. No aporta trazabilidad ni simplicidad industrial inicial. |
| SLAM 3D global | No depender de loop closures para el MVP. La geometría cilíndrica y `θ_tube` hacen preferible una malla específica. |
| MoveIt 2 / OMPL | Sí para inspección local futura con base parada y UR5e moviendo cámara/luz hacia ROI. |
| Surface tracing continuo base+brazo | No abordar al principio. Primero andar lento para cubrir y parar para diagnosticar. |

## Arquitectura ROS 2 Actual

### Paquetes

| Paquete | Estado | Responsabilidad |
|---|---|---|
| `wind_tower_description` | Funcional | STL del tubo y URDF extra del cabezal cámara/iluminación simulado |
| `wind_tower_simulation` | Funcional | Mundo Gazebo: nave, tubo rotante, virador |
| `wind_tower_bringup` | Funcional / prototipos | Launch principal, bridges, teleop, virador, utilidades LiDAR |
| `wind_tower_inspection_behaviour` | MVP inicial | Lógica de misión; contiene `stability_monitor_node.py` |
| `gz_ros2_control` | Fork local | Fix WSL2/null-pointer para estabilidad de Gazebo |

### Nodos existentes relevantes

| Nodo / ejecutable | Paquete | Estado | Función |
|---|---|---|---|
| `dualsense_joy` | `wind_tower_bringup` | Funcional | Publica mando DualSense vía evdev |
| `ps5_teleop` | `wind_tower_bringup` | Funcional | Control UR5e y virador |
| `turner_node` | `wind_tower_bringup` | Funcional | Publica `/turner/angle` y `/turner/angle_deg` |
| `tf_static_relay` | `wind_tower_bringup` | Funcional | Relay de TF estático |
| `cylinder_localizer` | `wind_tower_bringup` | Prototipo | Fit local de cilindro desde LiDAR |
| `map_accumulator` | `wind_tower_bringup` | Prototipo | Acumulación simple de nube |
| `scan_gate` | `wind_tower_bringup` | Disponible | Filtro sectorial LaserScan |
| `stability_monitor` | `wind_tower_inspection_behaviour` | MVP validado | Publica flags de estabilidad |

### Launchers existentes

| Launcher | Estado | Uso |
|---|---|---|
| `simulation.launch.py` | Funcional | Gazebo, robot Clearpath, bridges, virador, IMU, EKF, cámara simulada y límites teleop |
| `rtabmap.launch.py` | Secundario | Pruebas RTAB-Map; no base del MVP |
| `slam.launch.py` | No usado actualmente | Pruebas SLAM Toolbox; no base del MVP |

## Arquitectura Propuesta del MVP

El MVP debe ser una arquitectura específica de misión, no una adaptación directa de navegación libre 2D/3D:

```text
state_machine_node
  ├── stability_monitor_node
  ├── bottom_lane_controller
  ├── cylindrical_map_node
  ├── tube_indexing_controller
  ├── obstacle_manager_node / bypass_manager_node
  ├── image_capture_manager_node
  ├── local_inspection_controller
  └── report_generator_node
```

El antiguo `coverage_controller` helicoidal queda retirado. Si se conserva el concepto, debe ser `coverage_manager` o parte de `cylindrical_map_node`, dedicado a cobertura y métricas, nunca a controlar una hélice continua.

## Nodos Propuestos

| Nodo | Responsabilidad | Entradas principales | Salidas principales | Estado |
|---|---|---|---|---|
| `state_machine_node.py` | Orquestar misión MVP, avance axial, giro tangencial, indexado y realineación | `/inspection/bottom_lane_locked`, `/inspection/safe_to_scan`, `/inspection/safe_to_index_tube`, `/robot/platform/odom/filtered`, `/turner/angle` | `/robot/platform/cmd_vel`, `/turner/cmd_vel`, `/inspection/state`, `/inspection/current_lane`, `/inspection/mission_status` | EN DESARROLLO / validación: sin bypass aún, pero con `ALIGN_TO_BOTTOM_LANE` y PI básico |
| `stability_monitor_node.py` | Estimar estabilidad física, roll/pitch, `α_robot`, flags de seguridad | `/robot/sensors/imu_0/data`, `/robot/platform/odom/filtered`, `/cylinder_fit/stats`, `/turner/angle` | `/inspection/stability`, `/inspection/bottom_lane_locked`, `/inspection/safe_to_scan`, `/inspection/safe_to_index_tube` | MVP validado, evolución pendiente a mensajes custom |
| `bottom_lane_controller.py` | Avanzar axialmente manteniendo generatriz inferior | bottom lane state, cylindrical pose, current lane, obstacles | `/cmd_vel` o `/inspection/cmd_vel_raw` | PENDIENTE |
| `cylindrical_map_node.py` | Mantener malla desplegada `(x, θ)` y capas de cobertura | odom, `/turner/angle`, flags de estabilidad, estado misión | `/inspection/cylindrical_pose`, `/inspection/coverage_status`, `/inspection/cylindrical_map_stats` | MVP implementado y probado manualmente |
| `tube_indexing_controller.py` | Girar virador por `Δθ` con verificación antes/después | `/turner/angle_deg`, bottom lane state, `/inspection/index_request` | `/turner/cmd_vel`, `/inspection/index_done`, `/inspection/index_status` | PENDIENTE |
| `obstacle_manager_node.py` | Detectar/clasificar obstáculos y recomendar acción | LiDAR, cylindrical map, cylindrical pose, bottom lane state | `/inspection/obstacles`, `/inspection/blocked_segments` | PENDIENTE |
| `bypass_manager_node.py` | Gestionar bypass conservador y semántica de cobertura | obstáculos, estado, estabilidad | `/inspection/bypass_request`, estado de bypass | PENDIENTE |
| `image_capture_manager_node.py` | Capturar imágenes por distancia y guardar metadatos | cámara, pose cilíndrica, state, bottom lane, odom, iluminación | `/inspection/captured_image_event`, `/inspection/image_metadata`, ficheros | PENDIENTE |
| `local_inspection_controller.py` | Inspección local con base parada y UR5e/cámara/luz | ROI, joint states, TF, cámara | comandos brazo, `/inspection/local_inspection_result`, evidencias | FUTURO MVP tardío |
| `report_generator_node.py` | Generar informe JSON/CSV/Markdown | eventos, mapa, anomalías, cobertura, metadatos | `/inspection/report`, ficheros | PENDIENTE |

## Máquina de Estados Final

| Estado | Propósito | Entradas | Salidas/comandos | Transiciones | Sensores | Cobertura nominal |
|---|---|---|---|---|---|---:|
| `IDLE` | Esperar misión o reset | operador, configuración | estado idle | start -> `ALIGN_TO_BOTTOM_LANE` | ninguno crítico | No |
| `ALIGN_TO_BOTTOM_LANE` | Colocar robot en generatriz inferior | IMU, odom, LiDAR | comandos base lentos | alineado -> `VERIFY_BOTTOM_LOCK`; fallo -> `ERROR` | IMU, LiDAR, odom | No |
| `VERIFY_BOTTOM_LOCK` | Confirmar estabilidad | bottom lane state | habilitar scan | locked -> `AXIAL_SCAN`; no locked -> `ALIGN_TO_BOTTOM_LANE` | IMU, odom, LiDAR | No |
| `AXIAL_SCAN` | Avanzar por calle y registrar cobertura | pose, estabilidad, obstáculos, cámara | cmd base, eventos captura, cobertura | obstáculo -> `OBSTACLE_DETECTED`; fin calle -> `INDEX_TUBE`; unsafe -> `RETURN_TO_BOTTOM_LANE` | IMU, odom, LiDAR, cámara | Sí, solo si `safe_to_scan` |
| `OBSTACLE_DETECTED` | Parada segura y clasificación inicial | obstáculos, LiDAR, estado | cmd_vel cero, evento | local -> `LOCAL_INSPECTION`; blocked -> `ERROR/RECOVERY` | LiDAR, IMU | No |
| `LOCAL_INSPECTION` | Diagnóstico con base parada | ROI, cámara, iluminación, UR5e | capturas locales, resultado | bypass -> `BYPASS_OBSTACLE`; continuar -> `RETURN_TO_BOTTOM_LANE`; blocked -> `ERROR/RECOVERY` | cámara, iluminación, UR5e, LiDAR | No |
| `BYPASS_OBSTACLE` | Esquiva conservadora | bypass request, estabilidad | cmd base reducido | superado -> `RETURN_TO_BOTTOM_LANE`; unsafe -> `ERROR/RECOVERY` | IMU, LiDAR, odom | No |
| `RETURN_TO_BOTTOM_LANE` | Recuperar generatriz inferior | bottom lane state | cmd base lento | locked -> `VERIFY_BOTTOM_LOCK` | IMU, LiDAR, odom | No |
| `INDEX_TUBE` | Girar tubo `Δθ` | `safe_to_index_tube`, angle target | `/turner/cmd_vel` | target -> `VERIFY_INDEXED_POSITION`; unsafe -> `ERROR/RECOVERY` | encoder virador, IMU | No |
| `VERIFY_INDEXED_POSITION` | Esperar asentamiento tras giro | bottom lane state, angle | stop virador, nueva lane | ok -> `AXIAL_SCAN`; completado -> `FINISH` | encoder, IMU, LiDAR | No |
| `FINISH` | Cerrar misión | coverage status, eventos | informe | reset -> `IDLE` | todos para resumen | No |
| `ERROR/RECOVERY` | Parar y recuperar o pedir intervención | diagnósticos | cmd cero, evento error | recuperado -> estado seguro; no recuperado -> intervención | IMU, LiDAR, odom | No |

## Cobertura Verificable

La malla cilíndrica desplegada debe separar observación, inspección nominal, confianza, obstáculos y anomalías.

Capas recomendadas:

| Capa | Uso |
|---|---|
| `observed_map` | Celdas vistas por LiDAR/cámara, aunque no sean inspección nominal |
| `nominal_scan_map` | Celdas inspeccionadas durante `AXIAL_SCAN` seguro |
| `confidence_map` | Calidad/confianza de observación por celda |
| `mode_map` | Modo de adquisición: axial, bypass, local, manual |
| `obstacle_map` | Obstáculos/bushings/salientes |
| `anomaly_map` | Defectos candidatos o anomalías confirmadas |

Solo se marca cobertura nominal si:

- `state == AXIAL_SCAN`.
- `bottom_lane_locked == true`.
- `safe_to_scan == true`.
- roll/pitch dentro de umbral.
- no hay bypass activo.

Cálculo de paso angular:

```text
Δθ = ancho_útil_superficie / R
Δθ_real = Δθ · (1 - overlap)
N_calles ≈ 360° / Δθ_real
```

## Estrategia de Captura de Imágenes

La evidencia visual no debe basarse solo en vídeo continuo. El vídeo puede servir para debug/operador, pero la evidencia principal debe ser imagen sincronizada y trazable.

Estrategia recomendada:

**Andar lento para cubrir, parar para diagnosticar.**

### Captura nominal durante `AXIAL_SCAN`

- El robot avanza lentamente por la calle axial.
- La cámara captura por trigger espacial, no solo por FPS.
- Trigger inicial recomendado: cada `Δx = 0.05 m` o `0.10 m`.
- Cada imagen se guarda con metadatos:
  - timestamp
  - `lane_id`
  - `x_robot`
  - `θ_tube`
  - `θ_surface`
  - `α_robot`
  - roll/pitch
  - `bottom_lane_locked`
  - `safe_to_scan`
  - modo de iluminación
  - estado de misión
- Una imagen nominal solo cuenta para cobertura si `safe_to_scan == true` y no hay bypass activo.

### Captura de inspección local

- Ante obstáculo, bushing o anomalía candidata, la base se detiene.
- Se ejecuta `LOCAL_INSPECTION`.
- El UR5e/cámara/luz capturan evidencias de mayor calidad.
- Secuencia recomendada:
  - `diffuse`
  - `grazing_left`
  - `grazing_right`
  - `all`
- Estas imágenes alimentan diagnóstico e informe; no son cobertura nominal de pasada axial.

## Mensajes Custom Propuestos

### `BottomLaneState.msg`

```text
std_msgs/Header header
float32 alpha_robot_deg
float32 alpha_rate_deg_s
float32 roll_deg
float32 pitch_deg
bool bottom_lane_locked
bool safe_to_scan
bool safe_to_index_tube
bool slip_detected
string status
```

### `CylindricalPose.msg`

```text
std_msgs/Header header
float32 x_m
float32 theta_tube_deg
float32 theta_surface_deg
float32 alpha_robot_deg
float32 heading_error_deg
string frame_id
```

### `InspectionState.msg`

```text
std_msgs/Header header
string state
int32 lane_id
float32 lane_theta_deg
float32 x_target_m
bool nominal_coverage_enabled
```

### `Obstacle.msg`

```text
std_msgs/Header header
int32 id
float32 x_m
float32 theta_deg
float32 size_x_m
float32 size_theta_deg
string type
string action_recommendation
```

### `CoverageStatus.msg`

```text
std_msgs/Header header
float32 nominal_coverage_percent
float32 observed_coverage_percent
int32 uncovered_cells
int32 low_confidence_cells
int32 blocked_segments
```

### `CapturedImageEvent.msg`

```text
std_msgs/Header header
string image_path
string metadata_path
int32 lane_id
float32 x_m
float32 theta_tube_deg
float32 theta_surface_deg
float32 alpha_robot_deg
float32 roll_deg
float32 pitch_deg
bool bottom_lane_locked
bool safe_to_scan
string mission_state
string illumination_mode
bool nominal_coverage_image
bool local_inspection_image
```

### `ImageMetadata.msg`

```text
std_msgs/Header header
string image_id
string image_path
int32 lane_id
float32 x_m
float32 theta_tube_deg
float32 theta_surface_deg
float32 alpha_robot_deg
string capture_mode
string illumination_mode
string associated_event_id
```

## Pseudocódigo Control Axial

```python
def axial_scan_step(state, sensors, params):
    e_heading = wrap_to_pi(state.yaw_axis_error)
    e_alpha = state.alpha_robot_deg
    roll = state.roll_deg
    pitch = state.pitch_deg
    d_front = sensors.front_obstacle_distance
    locked = state.bottom_lane_locked

    if abs(roll) > params.max_roll_deg or abs(pitch) > params.max_pitch_deg:
        publish_cmd_vel(0.0, 0.0)
        return "ERROR_RECOVERY"

    if d_front < params.stop_distance:
        publish_cmd_vel(0.0, 0.0)
        return "OBSTACLE_DETECTED"

    if not locked:
        publish_cmd_vel(0.0, 0.0)
        return "RETURN_TO_BOTTOM_LANE"

    v_cmd = params.v_nominal
    v_cmd *= stability_scale(abs(e_alpha), abs(roll), abs(pitch))
    v_cmd *= obstacle_scale(d_front)

    omega_cmd = (
        params.k_heading * e_heading +
        params.k_alpha * e_alpha +
        params.k_dalpha * state.alpha_rate_deg_s
    )

    v_cmd = clamp(v_cmd, 0.0, params.v_max)
    omega_cmd = clamp(omega_cmd, -params.w_max, params.w_max)

    publish_cmd_vel(v_cmd, omega_cmd)

    if state.safe_to_scan:
        mark_nominal_coverage(state.x_m, state.theta_surface_deg)

    mark_observed_coverage(state.x_m, state.theta_surface_deg, mode="AXIAL_SCAN")

    return "AXIAL_SCAN"
```

## Pseudocódigo Captura de Imágenes

```python
def image_capture_step(state, odom, last_capture_x, params):
    if state.mission_state == "AXIAL_SCAN":
        if not state.safe_to_scan:
            return "NO_CAPTURE_UNSAFE"

        dx = abs(odom.x_m - last_capture_x)

        if dx >= params.capture_delta_x_m:
            metadata = {
                "timestamp": now(),
                "lane_id": state.lane_id,
                "x_m": state.x_m,
                "theta_tube_deg": state.theta_tube_deg,
                "theta_surface_deg": state.theta_surface_deg,
                "alpha_robot_deg": state.alpha_robot_deg,
                "roll_deg": state.roll_deg,
                "pitch_deg": state.pitch_deg,
                "bottom_lane_locked": state.bottom_lane_locked,
                "safe_to_scan": state.safe_to_scan,
                "mission_state": state.mission_state,
                "illumination_mode": current_light_mode(),
                "nominal_coverage_image": True,
            }

            save_image_with_metadata(metadata)
            publish_captured_image_event(metadata)
            last_capture_x = odom.x_m

        return "AXIAL_CAPTURE_CHECKED"

    if state.mission_state == "LOCAL_INSPECTION":
        stop_base()
        for light_mode in ["diffuse", "grazing_left", "grazing_right", "all"]:
            set_light_mode(light_mode)
            wait_light_settle()
            metadata = build_local_inspection_metadata(state, light_mode)
            save_image_with_metadata(metadata)
            publish_captured_image_event(metadata)

        return "LOCAL_INSPECTION_CAPTURED"

    return "NO_CAPTURE_STATE"
```

## Parámetros Recomendados (`inspection_params.yaml`)

```yaml
tube:
  radius_m: 4.0
  length_m: 30.0

lanes:
  useful_surface_width_m: 0.50
  overlap: 0.20
  delta_theta_deg: null
  use_lawnmower_pattern: true

bottom_lane:
  max_roll_deg: 5.0
  max_pitch_deg: 5.0
  max_alpha_deg: 5.0
  warning_alpha_deg: 10.0
  recovery_alpha_deg: 15.0
  max_slip_ratio: 0.25

axial_scan:
  v_nominal: 0.05
  v_max: 0.10
  w_max: 0.30
  k_heading: 1.0
  k_alpha: 0.5
  k_dalpha: 0.1
  stop_distance_m: 0.75
  slow_distance_m: 1.5

indexing:
  turner_speed_rad_s: 0.02
  settle_time_s: 2.0
  angle_tolerance_deg: 0.5

coverage:
  min_confidence: 0.85
  nominal_only_when_safe_to_scan: true
  bypass_counts_as_nominal: false

image_capture:
  capture_delta_x_m: 0.10
  allow_video_debug: true
  use_spatial_trigger: true
  save_metadata_json: true
  nominal_capture_requires_safe_to_scan: true
  local_inspection_requires_base_stopped: true
  illumination_sequence:
    - diffuse
    - grazing_left
    - grazing_right
    - all
```

## Experimento LiDAR Pendiente

Comparar dos montajes del LiDAR antes de consolidar `cylindrical_map_node.py`:

| Opción | Descripción | Hipótesis |
|---|---|---|
| A | LiDAR actual de pie | Mejor visión general frontal/local, ya funcional |
| B | LiDAR tumbado con eje de giro paralelo al eje axial del tubo | Cada scan se parece más a una sección circular del cilindro |

La opción B puede facilitar:

- Ajuste de circunferencia/cilindro.
- Estimación de radio.
- Detección de salientes/bushings.
- Validación geométrica de bottom lane.
- Generación de malla `(x, θ)`.

No cambiar URDF todavía. Primero hacer prueba controlada y comparar métricas: puntos de pared, RMS de fit, estabilidad de radio, sensibilidad a obstáculos y coste computacional.

## Roadmap Nuevo

### Fases completadas o base funcional

| Fase | Estado | Descripción |
|---|---|---|
| 0 | Completada | Entorno ROS 2 Jazzy + Gazebo Harmonic + WSL2 |
| 1 | Completada | Estructura repo + paquetes base |
| 2 | Completada | Husky A200 + UR5e vía Clearpath |
| 3 | Completada | RViz + TF operativo |
| 4 | Completada | Mundo Gazebo con nave y tubo STL |
| 5 | Completada | Velodyne VLP-16 en `/velodyne_points` |
| 6 | Completada | Teleop base, brazo y virador con DualSense |
| 7 | Completada | Tubo dinámico con joint revolute |
| 8 | Completada | Virador con `/turner/cmd_vel`, `/turner/angle`, `/turner/angle_deg` |

### Fase 9 — MVP de inspección por calles

| Fase | Estado | Objetivo |
|---|---|---|
| 9A | Validada en simulación | Activar IMU + `robot_localization`; usar `/robot/platform/odom/filtered`; no SLAM complejo |
| 9B | Completada estructura | Crear paquete `wind_tower_inspection_behaviour`, params y launch base futuro |
| 9C | MVP validado | `stability_monitor_node`: roll/pitch, flags y debug |
| 9D | MVP probado manualmente | `cylindrical_map_node` mínimo: malla `(x, θ)`, `observed_map`, `nominal_scan_map` |
| 9E | En validación | `state_machine_node`: `IDLE`, `VERIFY_BOTTOM_LOCK`, `AXIAL_SCAN`, `ROTATE_TO_TANGENTIAL`, `INDEX_TUBE`, `ROTATE_TO_AXIAL`, `ALIGN_TO_BOTTOM_LANE`, `VERIFY_INDEXED_POSITION`, `FINISH`, parada segura |
| 9F | Pendiente | `bottom_lane_controller` simple con PID heading/`α_robot` y gates de estabilidad |
| 9G | Pendiente | Obstacle stop con LiDAR frontal/local y evento `OBSTACLE_DETECTED` |
| 9H | Pendiente | `tube_indexing_controller`: giro `Δθ`, verificación antes/después |
| 9I | Pendiente | `image_capture_manager_node`: captura por `Δx`, imagen + metadatos |
| 9J | Pendiente | Bypass básico conservador, sin cobertura nominal, retorno obligatorio |
| 9K | En desarrollo sensórico | Cámara + iluminación + registro multi-iluminación |
| 9L | Futuro | `local_inspection_controller` con UR5e; MoveIt 2/OMPL como soporte |
| 9M | Pendiente | `report_generator_node`: JSON inicial |

### No implementar todavía

- RL para navegación.
- MPPI custom.
- TEB.
- Nav2 completo.
- SLAM 3D global con loop closure como dependencia.
- IA para defectos como primer paso.
- Surface tracing continuo avanzado con base y brazo simultáneos.
- Garantía de 100% cobertura sin métrica.
- Vídeo continuo como evidencia principal de inspección fina.

## Topics Principales

| Topic | Tipo actual/propuesto | Estado | Uso |
|---|---|---|---|
| `/velodyne_points` | `sensor_msgs/PointCloud2` | Verificado | Nube LiDAR 3D |
| `/turner/cmd_vel` | `std_msgs/Float64` | Verificado | Comando de velocidad virador |
| `/turner/angle` | `std_msgs/Float64` | Verificado | `θ_tube` acumulado en rad |
| `/turner/angle_deg` | `std_msgs/Float64` | Verificado | `θ_tube` en grados |
| `/turner/joint_state` | `sensor_msgs/JointState` | Verificado | Encoder simulado |
| `/robot/platform/odom` | `nav_msgs/Odometry` | Configurado | Odometría ruedas |
| `/robot/platform/odom/filtered` | `nav_msgs/Odometry` | Verificado | Odometría filtrada Clearpath |
| `/robot/sensors/imu_0/data` | `sensor_msgs/Imu` | Verificado | IMU |
| `/robot/sensors/inspection_camera/image` | `sensor_msgs/Image` | Verificado | Imagen simulada directa |
| `/inspection/camera/image_raw` | `sensor_msgs/Image` | En desarrollo | Alias objetivo para inspección |
| `/inspection/camera/camera_info` | `sensor_msgs/CameraInfo` | En desarrollo | Intrínsecos simulados |
| `/cylinder_fit/stats` | `std_msgs/String` | Prototipo | Diagnóstico fit cilindro |
| `/inspection/stability` | `std_msgs/String` JSON | Verificado MVP | Debug estabilidad |
| `/inspection/bottom_lane_locked` | `std_msgs/Bool` | Verificado MVP | Asentamiento inferior |
| `/inspection/safe_to_scan` | `std_msgs/Bool` | Verificado MVP | Gate cobertura nominal |
| `/inspection/safe_to_index_tube` | `std_msgs/Bool` | Verificado MVP | Gate indexado |
| `/inspection/bottom_lane_state` | `BottomLaneState` | PROPUESTO | Estado de estabilidad tipado |
| `/inspection/cylindrical_pose` | `std_msgs/String` JSON, futuro `CylindricalPose` | EN DESARROLLO | Pose en coordenadas cilíndricas |
| `/inspection/state` | `InspectionState` | PROPUESTO | Estado de misión |
| `/inspection/obstacles` | `Obstacle[]` o msg contenedor | PROPUESTO | Obstáculos y bushings |
| `/inspection/coverage_status` | `std_msgs/String` JSON, futuro `CoverageStatus` | EN DESARROLLO | Métricas de cobertura |
| `/inspection/captured_image_event` | `CapturedImageEvent` | PROPUESTO | Evento de imagen guardada |
| `/inspection/image_metadata` | `ImageMetadata` | PROPUESTO | Metadatos de imagen |
| `/inspection/report` | String/JSON/fichero | PROPUESTO | Informe |
