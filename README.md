# ROS2 Wind Tower Inspection

Simulación de inspección del interior de un tramo de torre eólica (cilindro horizontal Ø8m × 30m) usando ROS 2 Jazzy, Gazebo Harmonic, un Husky A200 con brazo UR5e, virador de tubo y LiDAR Velodyne VLP-16.

El proyecto está en fase de simulación/MVP. Ya existe una base funcional de Gazebo + ROS 2, teleoperación, virador y sensores simulados. La lógica de inspección autónoma todavía está en desarrollo y debe implementarse en un paquete específico.

## Concepto de inspección

La metodología base es **inspección por calles axiales con indexado angular**.

No se asume una pasada helicoidal continua ni se documenta una cobertura 100% garantizada por giro constante. La cobertura debe ser **verificable mediante una malla cilíndrica (x, θ)** generada por calles axiales discretas:

1. El tubo permanece parado en un ángulo `θ_tube = θ_i`.
2. El robot se alinea con la generatriz inferior estable del tubo.
3. El robot realiza una pasada axial inspeccionando una calle.
4. Si detecta obstáculo, bushing, soldadura, saliente o anomalía, se detiene y ejecuta inspección local.
5. Si puede esquivar, lo hace a baja velocidad monitorizando inclinación con IMU.
6. Tras esquivar, debe volver a la generatriz inferior estable antes de continuar.
7. Al terminar la calle, el virador indexa el tubo un incremento `Δθ_real` con solape.
8. El robot verifica de nuevo estabilidad y asentamiento antes de iniciar la siguiente calle.
9. El ciclo se repite hasta cubrir una rotación acumulada de `360° + margen de solape`.

```
θ_tube fijo + robot avanza en x → calle axial inspeccionada
indexado Δθ_real del virador → siguiente calle axial
calles acumuladas → malla cilíndrica verificable (x, θ)
```

Durante maniobras de esquiva, la cobertura no debe marcarse como inspección normal. Esa zona debe registrarse como modo especial y requerir retorno/verificación antes de reanudar.

## Variables de navegación e inspección

| Variable / estado | Significado | Fuente principal |
|---|---|---|
| `x_robot` | Posición axial del robot dentro del tubo | Odometría de ruedas fusionada con IMU y/o LiDAR en EKF |
| `θ_tube` | Ángulo material/acumulado del tubo | Encoder del virador, simulado actualmente por `/turner/joint_state` y publicado en `/turner/angle` |
| `θ_surface` | Ángulo/celda de superficie inspeccionada | Derivado de `θ_tube`, lane actual y malla de cobertura |
| `α_robot` | Desviación angular del robot respecto a la generatriz inferior estable | IMU + geometría LiDAR |
| `bottom_lane_locked` | Booleano: robot correctamente asentado en la generatriz inferior | `stability_monitor_node.py` MVP validado |
| `safe_to_scan` | True solo si el robot está estable, alineado y dentro de umbrales | Máquina de estados + monitor de estabilidad |
| `safe_to_index_tube` | True solo si el robot está en postura segura para girar el tubo | Máquina de estados + monitor de estabilidad |

La orientación del robot respecto a la gravedad es crítica. Para considerar que está en la generatriz inferior, el eje Z del robot debe estar alineado con la aceleración de la gravedad dentro de umbral. La IMU es obligatoria, no opcional. No hay que confundir `θ_tube` con `α_robot`: el primero describe el tubo; el segundo describe la desviación del robot.

En simulación, `stability_monitor` puede calibrar automáticamente la postura inicial como referencia de `bottom lane`. Durante las primeras muestras de IMU al arrancar se calculan offsets de roll, pitch y ángulo lateral; después los flags se evalúan respecto a esa referencia, no respecto a un cero ideal del sensor.

## Sensores base

| Sensor | Estado actual | Uso técnico |
|---|---|---|
| Encoder virador | Simulado vía `/turner/joint_state`; `/turner/angle` y `/turner/angle_deg` funcionando | Fuente principal de `θ_tube`; evita depender de ICP para rotación |
| Wheel odometry | Disponible desde Husky/Clearpath; EKF configurado | Estimación principal de avance axial `x` |
| IMU | Declarada en `robot.yaml`, bridge, EKF y `stability_monitor` validados en simulación | Roll/pitch respecto a gravedad, estabilidad, `α_robot`, detección de desviación por pared |
| LiDAR 3D Velodyne VLP-16 | `/velodyne_points` funcionando | Geometría del cilindro, obstáculos, bushings, pared, cobertura y seguridad |
| Cámara RGB industrial | Prototipo simulado en `arm_0_tool0`; imagen directa verificada en `/robot/sensors/inspection_camera/image`; alias `/inspection/camera/image_raw` EN DESARROLLO | Inspección superficial con iluminación controlada |
| Iluminación controlada | Prototipo simulado con dos luces rasantes en el TCP | Frontal/difusa, rasante izquierda/derecha, opcional multi-iluminación |
| Joint states UR5e | Controlador disponible | Pose de cámara y trazabilidad de inspección |
| Distancia corta, bumper, E-stop | Recomendado para prototipo real | Seguridad de end-effector, contacto y parada de emergencia |

## Arquitectura actual del repositorio

```
ROS2_wind_tower_inspection/
├── README.md
├── PROJECT_PLAN.md      ← fases, arquitectura propuesta y roadmap
├── PROJECT_STATE.md     ← estado actual verificado
├── LAUNCH_GUIDE.md      ← instrucciones de arranque y comprobación
└── ros2_ws/src/
    ├── gz_ros2_control/                 ← fork con fix WSL2 null-ptr
    ├── wind_tower_description/          ← mesh STL del tubo
    ├── wind_tower_simulation/           ← mundo Gazebo (nave + tubo rotante)
    ├── wind_tower_bringup/              ← launchers, bridges, teleop, virador, utilidades LiDAR
    └── wind_tower_inspection_behaviour/ ← lógica autónoma de inspección (MVP inicial)
```

## Arquitectura de inspección propuesta

Paquete creado: `wind_tower_inspection_behaviour`.

| Nodo propuesto | Responsabilidad | Entradas esperadas | Salidas esperadas | Estado |
|---|---|---|---|---|
| `state_machine_node.py` | Orquestar estados de operación MVP | Flags de estabilidad, odom, encoder virador | `/robot/platform/cmd_vel`, `/turner/cmd_vel`, `/inspection/state` | Implementado y en validación: `ALIGN_TO_BOTTOM_LANE`, `AXIAL_SCAN`, `INDEX_TUBE`, `ROTATE_TO_AXIAL` |
| `stability_monitor_node.py` | Calcular `bottom_lane_locked`, `safe_to_scan`, `safe_to_index_tube` | IMU, odom Clearpath, diagnóstico LiDAR | `/inspection/stability`, flags booleanos | MVP validado |
| `bottom_lane_controller.py` | Alinear y mantener robot en generatriz inferior | IMU, LiDAR, odom | `/cmd_vel` o comando base equivalente | PENDIENTE |
| `cylindrical_map_node.py` | Mantener malla de cobertura `(x, θ)` sin mover el robot | odom, `θ_tube`, `bottom_lane_locked`, `safe_to_scan`, modo inspección | `/inspection/coverage_status`, `/inspection/cylindrical_map_stats` | EN DESARROLLO |
| `tube_indexing_controller.py` | Indexar el tubo por `Δθ_real` | `/turner/angle`, petición de indexado | `/turner/cmd_vel`, estado indexado | PENDIENTE |
| `obstacle_manager_node.py` | Detectar obstáculos/bushings/salientes y recomendar acción | LiDAR 3D, mapa cilíndrico | `/inspection/obstacles`, segmentos bloqueados | PENDIENTE |
| `bypass_manager_node.py` | Gestionar bypass conservador sin cobertura nominal | Obstáculos, estabilidad, estado | Petición/estado de bypass | PENDIENTE |
| `image_capture_manager_node.py` | Capturar imágenes por distancia y guardar metadatos | Cámara, odom, estado, pose cilíndrica | Eventos de imagen, JSON/metadatos | PENDIENTE |
| `local_inspection_controller.py` | Gestionar inspección local con base parada y UR5e | ROI, cámara, LiDAR, estado brazo | Comandos de brazo, evidencias | FUTURO |
| `report_generator_node.py` | Generar informe de inspección | Cobertura, anomalías, evidencias | JSON/PDF/markers | PENDIENTE |

El antiguo concepto `coverage_controller` helicoidal debe reformularse como `coverage_manager` o integrarse dentro de `cylindrical_map_node.py` + `state_machine_node.py`. Ya no debe controlar una hélice continua.

Implementación actual de la máquina de estados: el MVP puede avanzar una calle axial si `bottom_lane_locked=true` y `safe_to_scan=true`, detener la base al final de calle, girar el tubo por `Δθ`, pasar por `ROTATE_TO_TANGENTIAL`, recolocarse con `ALIGN_TO_BOTTOM_LANE`, verificar asentamiento y repetir en patrón tipo lawnmower. Durante `INDEX_TUBE` la base compensa el giro del tubo; no se marca cobertura nominal durante el indexado. Si se pierde estabilidad, el nodo detiene la misión y fuerza recuperación/alineación antes de volver a escanear.

Perfil actual de pruebas:

```text
lane_length_m = 1.0
lane_delta_theta_deg = 5.0
publish_rate_hz = 30.0
turner_speed_rad_s = 0.02
index_compensation_sign = 1.0
```

Para reducir terminales existe `inspection.launch.py`, que levanta los nodos de inspección principales, `dualsense_joy` y `ps5_teleop`. Por defecto deja `state_machine` en `IDLE`; pulsando Triángulo en el DualSense se publica `START_AUTO` y arranca la misión. Durante autonomía se publica `/inspection/autonomous_active=true`, `ps5_teleop` deja el virador en `0.0` y el teleop manual queda bloqueado mediante Joy neutro.

Decisión de navegación: Nav2 completo no es el núcleo del MVP. Puede aprovecharse más adelante como caja de herramientas, por ejemplo Collision Monitor, Regulated Pure Pursuit o MPPI. No se priorizan TEB, RL ni SLAM 3D global con loop closures. MoveIt 2/OMPL sí tiene sentido para inspección local futura con el UR5e y la base parada.

## Estados de operación propuestos

```
IDLE
ALIGN_TO_BOTTOM_LANE
VERIFY_BOTTOM_LOCK
AXIAL_SCAN
OBSTACLE_DETECTED
LOCAL_INSPECTION
BYPASS_OBSTACLE
RETURN_TO_BOTTOM_LANE
INDEX_TUBE
VERIFY_INDEXED_POSITION
FINISH
ERROR / RECOVERY
```

Durante `AXIAL_SCAN`, `bottom_lane_locked` y `safe_to_scan` deben ser true. Durante `BYPASS_OBSTACLE`, se permite `α_robot != 0`, se reduce velocidad y no se marca cobertura como inspección normal. Durante `INDEX_TUBE`, `safe_to_index_tube` debe ser true y después se exige `VERIFY_INDEXED_POSITION`.

## Cobertura por malla cilíndrica

La cobertura se evalúa en celdas `(x, θ)`, no por promesa geométrica de una hélice ideal. Cada celda debe distinguir al menos:

| Estado celda | Significado |
|---|---|
| `unseen` | Sin observación válida |
| `observed_lidar` | Geometría observada por LiDAR |
| `inspected_rgb` | Inspección visual válida con cámara/iluminación |
| `blocked` | Zona bloqueada por obstáculo |
| `bypass_mode` | Zona atravesada durante esquiva, no cuenta como inspección normal |
| `needs_review` | Requiere revisión local o manual |

Cálculo inicial del paso angular:

```text
Δθ = ancho_útil_superficie / R
Δθ_real = Δθ · (1 - overlap)
N_calles ≈ 360° / Δθ_real
```

“Una sola vuelta” significa rotación acumulada total del tubo de `360° + margen de solape`, no giro continuo durante toda la inspección.

## Captura de imágenes

La inspección visual tiene dos modos:

| Modo | Cuándo | Regla |
|---|---|---|
| Captura nominal | Durante `AXIAL_SCAN` | Capturar por distancia recorrida (`Δx`, por ejemplo 5-10 cm), no solo por FPS |
| Captura local | En `LOCAL_INSPECTION` | Base parada, varias iluminaciones y evidencias de diagnóstico |

Cada imagen nominal debe guardarse con timestamp, `lane_id`, `x_robot`, `θ_tube`, `θ_surface`, `α_robot`, roll/pitch, estado de estabilidad, modo de iluminación y estado de misión. Durante `BYPASS_OBSTACLE` no se marca cobertura nominal aunque la cámara vea superficie.

## Arranque rápido

Ver [LAUNCH_GUIDE.md](LAUNCH_GUIDE.md) para el procedimiento completo.

```bash
# Terminal 1 — Simulación completa
cd ~/ROS2_wind_tower_inspection/ros2_ws && ai-on
ros2 launch wind_tower_bringup simulation.launch.py

# Terminal 2 — Mando DualSense
source install/setup.bash && ros2 run wind_tower_bringup dualsense_joy

# Terminal 3 — Teleop brazo + virador
source install/setup.bash && ros2 run wind_tower_bringup ps5_teleop

# Alternativa recomendada para inspección: launch agrupado
source install/setup.bash && ros2 launch wind_tower_inspection_behaviour inspection.launch.py
```

## Controles mando DualSense

| Control | Acción |
|---|---|
| L2 + stick izquierdo | Mover Husky (adelante/atrás/giro) |
| L2 + stick derecho | Mover brazo UR5e (pan + tilt) |
| X (Cruz) | Girar virador + |
| Círculo | Solicitar `STOP` para la máquina de estados |
| Cuadrado | Girar virador − |
| Triángulo | Solicitar `START_AUTO` para la máquina de estados |

Durante autonomía, `/inspection/autonomous_active=true` bloquea el teleop manual publicando Joy neutro; Círculo queda disponible como parada de misión. El virador no debe girar durante `AXIAL_SCAN`; solo debe indexar en `INDEX_TUBE` cuando `safe_to_index_tube == true`.

El final de calle se decide por distancia axial recorrida desde el inicio de esa calle (`lane_progress_m >= lane_length_m`), no por una coordenada absoluta fija. `lane_length_m` representa distancia mínima inspeccionada, no una meta reducida por tolerancia. Los parámetros principales están en `state_machine.yaml`: `mission.axial_axis`, `mission.lane_length_m`, `mission.lane_delta_theta_deg`, `control.axial_speed_mps`, `control.turner_speed_rad_s` y `control.publish_rate_hz`. Antes de indexar existe un estado `WAIT_SAFE_TO_INDEX` para esperar a que la base esté parada y `safe_to_index_tube=true`; después del giro pasan por `ROTATE_TO_AXIAL` y `ALIGN_TO_BOTTOM_LANE` antes de reanudar la calle.

## Compilar

```bash
cd ~/ROS2_wind_tower_inspection/ros2_ws
colcon build --packages-select wind_tower_bringup wind_tower_simulation
source install/setup.bash
```
