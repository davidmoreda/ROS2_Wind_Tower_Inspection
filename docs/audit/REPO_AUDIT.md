# REPO_AUDIT — ROS2 WindTower

Fase PLAN, fecha 2026-05-12. Audit-only; no se modifica código ni docs principales.
Solo se anotan hallazgos con evidencia (ruta + línea cuando aplica).

---

## 1. Paquetes ROS 2 detectados

| Paquete | Tipo build | Estado | Evidencia |
|---|---|---|---|
| `wind_tower_description` | `ament_cmake` | Activo (solo meshes + xacro) | `ros2_ws/src/wind_tower_description/CMakeLists.txt`, `package.xml` |
| `wind_tower_simulation` | `ament_cmake` | Activo (mundo SDF) | `ros2_ws/src/wind_tower_simulation/CMakeLists.txt`, `worlds/wind_tower_world.sdf` |
| `wind_tower_bringup` | `ament_python` | Activo (launchers + nodos soporte) | `ros2_ws/src/wind_tower_bringup/setup.py` |
| `wind_tower_inspection_behaviour` | `ament_python` | Activo (lógica misión MVP) | `ros2_ws/src/wind_tower_inspection_behaviour/setup.py` |
| `gz_ros2_control` (y `_demos`, `_tests`) | fork upstream | Fork local con fix WSL2 | `ros2_ws/src/gz_ros2_control/gz_ros2_control/src/gz_ros2_control_plugin.cpp` |

`gz_ros2_control_demos` y `gz_ros2_control_tests` son **upstream demos** sin uso en este proyecto, pero forman parte del fork (no tocar).

---

## 2. Estructura del repo (raíz)

```
ROS2_wind_tower_inspection/
├── AGENTS.md
├── ARCHITECTURE.md
├── CONVERSATION_LESSONS.md
├── LAUNCH_GUIDE.md
├── PROJECT_PLAN.md
├── PROJECT_STATE.md
├── README.md
├── .agents/                    (VACÍO)
├── .codex/                     (VACÍO)
├── assets/                     (imágenes y mallas de documentación)
├── debug_runs/                 (30+ subdirectorios con capturas previas)
├── docs/
│   └── agent/
│       ├── CONTROL_DEBUG_BRIEF.md
│       ├── DEBUG_OUTPUT_POLICY.md
│       └── OPEN_CODE_WORKFLOW.md
├── tools/
│   └── debug/
│       ├── capture_inspection_debug.py
│       └── README.md
└── ros2_ws/src/
    ├── gz_ros2_control/                 (fork)
    ├── wind_tower_description/
    ├── wind_tower_simulation/
    ├── wind_tower_bringup/
    └── wind_tower_inspection_behaviour/
```

Observaciones:

- `.agents/` y `.codex/` están vacíos. `NO VERIFICADO` su propósito.
- `debug_runs/` contiene 30+ carpetas con logs JSONL antiguos del script `tools/debug/capture_inspection_debug.py`. Útil como histórico, pero no documentado en README/AGENTS.
- `assets/` contiene capturas e imágenes, no se referencia desde docs.

---

## 3. Inventario de launch files (proyecto, excluyendo `gz_ros2_control_demos`)

| Tipo | Ruta | Descripción | Evidencia |
|---|---|---|---|
| `.launch.py` | `ros2_ws/src/wind_tower_bringup/launch/simulation.launch.py` | Launcher central: Gazebo + spawn Clearpath + bridges + IMU + EKF + virador | L48-304 |
| `.launch.py` | `ros2_ws/src/wind_tower_bringup/launch/slam.launch.py` | SLAM Toolbox lifecycle (depende de `/scan`, no publicado por el pipeline actual) | L20-26 |
| `.launch.py` | `ros2_ws/src/wind_tower_bringup/launch/rtabmap.launch.py` | RTAB-Map 3D con `/velodyne_points`, loop closure off | L16-94 |
| `.launch.py` | `ros2_ws/src/wind_tower_inspection_behaviour/launch/inspection.launch.py` | Launcher central de misión: cylinder_localizer + dualsense + ps5_teleop + stability_monitor + cylindrical_map + state_machine | L139-592 |

Los launchers dentro de `gz_ros2_control_demos/launch/*.launch.py` son demos del fork; no se invocan.

---

## 4. Nodos Python encontrados (paquetes propios)

`wind_tower_bringup/wind_tower_bringup/`:

| Archivo | Nodo (entry_point en setup.py) | Lanzado por |
|---|---|---|
| `dualsense_joy.py` | `dualsense_joy` | `inspection.launch.py` (use_ps5=true) |
| `ps5_teleop.py` | `ps5_teleop` | `inspection.launch.py` (use_ps5=true) |
| `turner_node.py` | `turner_node` | `simulation.launch.py` |
| `tf_static_relay.py` | `tf_static_relay` | `simulation.launch.py` |
| `cylinder_localizer_node.py` | `cylinder_localizer` | `inspection.launch.py` (use_cylinder_localizer=true) |
| `scan_gate_node.py` | `scan_gate` | **No lanzado por ningún launcher** |
| `map_accumulator_node.py` | `map_accumulator` | **No lanzado por ningún launcher** |

`wind_tower_inspection_behaviour/wind_tower_inspection_behaviour/`:

| Archivo | Nodo | Lanzado por |
|---|---|---|
| `state_machine_node.py` | `state_machine` | `inspection.launch.py` |
| `stability_monitor_node.py` | `stability_monitor` | `inspection.launch.py` |
| `cylindrical_map_node.py` | `cylindrical_map` | `inspection.launch.py` |

Entry points verificados en:
- `ros2_ws/src/wind_tower_bringup/setup.py:37-43`
- `ros2_ws/src/wind_tower_inspection_behaviour/setup.py:34-38`

---

## 5. Archivos de configuración

| Tipo | Ruta | Cargado por | Observación |
|---|---|---|---|
| YAML | `wind_tower_bringup/config/ekf.yaml` | `simulation.launch.py:244` | EKF propio. Posible **redundante** con `/robot/platform/odom/filtered` que ya publica Clearpath (ver PROJECT_STATE.md:73) |
| YAML | `wind_tower_bringup/config/slam_toolbox.yaml` | `slam.launch.py:15-18` | Solo usado si se lanza SLAM Toolbox (no se lanza en flujo activo) |
| YAML | `wind_tower_bringup/config/robot.yaml` | externo (symlink en `~/clearpath/`) | Config Clearpath; instalado como data_file pero **no cargado por ningún launcher local** |
| YAML | `wind_tower_inspection_behaviour/config/inspection_params.yaml` | `inspection.launch.py:12-16` | Parámetros `cylindrical_map` |
| YAML | `wind_tower_inspection_behaviour/config/stability_monitor.yaml` | `inspection.launch.py:17-21` | Parámetros `stability_monitor` |
| YAML | `wind_tower_inspection_behaviour/config/state_machine.yaml` | `inspection.launch.py:22-26` | Parámetros máquina de estados |
| URDF/Xacro | `wind_tower_description/urdf/inspection_camera_lighting.urdf.xacro` | externo (Clearpath generator) | Cabezal cámara + luces para TCP UR5e |
| SDF | `wind_tower_simulation/worlds/wind_tower_world.sdf` | `simulation.launch.py:55` | Nave + tubo rotante; plugins JointController + JointStatePublisher |
| SDF | `wind_tower_simulation/models/wind_tower_tube/model.sdf` | usado por mundo | Modelo individual del tubo |

No hay archivos `.rviz`, ni params de Nav2, ni costmaps, ni configs de planner/controller Nav2 en el repo.

---

## 6. Documentación Markdown encontrada

| Documento | Líneas | Tema principal |
|---|---|---|
| `README.md` | 204 | Vista general, concepto, sensores, arquitectura, arranque rápido |
| `PROJECT_PLAN.md` | 579 | Roadmap MVP, nodos propuestos, máquina de estados, mensajes custom |
| `PROJECT_STATE.md` | 252 | Estado verificado de paquetes/nodos/topics, decisiones técnicas |
| `ARCHITECTURE.md` | 414 | Diagramas, topics, controladores, flujos de información |
| `LAUNCH_GUIDE.md` | 556 | Procedimiento operativo de arranque y depuración |
| `CONVERSATION_LESSONS.md` | 62 | Lo que funcionó / no funcionó en sesiones pasadas |
| `AGENTS.md` | 78 | Instrucciones para agentes IA en este repo |
| `docs/agent/CONTROL_DEBUG_BRIEF.md` | ~80 | Brief de debugging de controladores PI |
| `docs/agent/DEBUG_OUTPUT_POLICY.md` | ~40 | Política de output corto para agentes |
| `docs/agent/OPEN_CODE_WORKFLOW.md` | ~40 | Workflow para OpenCode agent |
| `tools/debug/README.md` | ~30 | Uso de `capture_inspection_debug.py` |
| `ros2_ws/src/gz_ros2_control/.../*.md` | varios | docs upstream del fork (no propias) |

No hay `docs/legacy/`, `docs/audit/` previa (este archivo crea la primera), ni un índice maestro de docs.

---

## 7. Principales inconsistencias detectadas (resumen — detalle en `DOCS_CONSISTENCY.md`)

1. **`slam.launch.py` referencia `/scan`** (LaserScan), pero el pipeline real publica `/velodyne_points` (PointCloud2 3D). El topic `/scan` **no existe** en el flujo actual. Además su comentario interno (L4) menciona `scan_gate` "en simulation.launch.py", pero `scan_gate` NO se lanza ahí. **CONTRADICTORIO**.
2. **`scan_gate` y `map_accumulator`** están como `entry_points` en `setup.py` y se documentan en README/STATE/PLAN/ARCH como "disponible/prototipo", pero **ningún launcher los arranca** y no se invocan manualmente en `LAUNCH_GUIDE.md`. → DOCUMENTADO PERO NO LANZADO.
3. **EKF propio (`ekf_node`)** se lanza en `simulation.launch.py:237-249` pero `PROJECT_STATE.md:73` y `ARCHITECTURE.md:277` indican que la odometría consumida realmente es `/robot/platform/odom/filtered` (la del stack Clearpath). Posible duplicación de filtro de odometría que no se aclara en la doc.
4. **PROJECT_PLAN.md** y **PROJECT_STATE.md** listan nodos `bottom_lane_controller`, `tube_indexing_controller`, `obstacle_manager_node`, `bypass_manager_node`, `image_capture_manager_node`, `local_inspection_controller`, `report_generator_node` que **no existen como archivos**. Están marcados como PENDIENTE/FUTURO, pero la lista mezcla "propuesto" e "implementado" con formato similar — puede confundir.
5. **README.md:85** dice `cylindrical_map_node.py` está "EN DESARROLLO" pero el archivo existe, está implementado y se lanza por `inspection.launch.py:493`. → DEMASIADO VAGO.
6. **Decisión Nav2 documentada como "no usar"** (README:109, PROJECT_STATE.md:160, PROJECT_PLAN.md:75-83). El proyecto actual NO tiene piezas de Nav2 (ni costmaps ni planner ni controller_server). Hay coherencia, pero la nueva dirección (Nav2 sobre cilindro desplegado) **todavía no se refleja en ninguna doc**.
7. **Sin frame `cyl_map`** ni mención de mapa cilíndrico desplegado para Nav2 en ningún archivo del repo. La aproximación cilíndrica actual es solo "malla `(x, θ)`" en `cylindrical_map_node.py` para registro de cobertura, NO para Nav2.
8. **`AGENTS.md`** es corto, claro, sin contradicciones internas. `docs/agent/*.md` también. No requieren limpieza.

---

## 8. Cosas que existen y NO están documentadas suficientemente

- Carpeta `debug_runs/` con 30+ ejecuciones; solo se menciona indirectamente en `tools/debug/README.md`.
- `assets/` y su contenido no se referencia desde docs principales.
- `inspection_camera_lighting.urdf.xacro` se documenta en README/ARCH pero su mecanismo de inclusión (vía Clearpath generator externo) solo se explica en `PROJECT_STATE.md:209-213`.
- Fork `gz_ros2_control` y el fix WSL2: documentado en `PROJECT_STATE.md:201-206`, no en README.

---

## 9. Cosas documentadas y NO existentes en el código

- 7 nodos del paquete `wind_tower_inspection_behaviour` listados como PENDIENTE/FUTURO en `PROJECT_PLAN.md` y `PROJECT_STATE.md` (ver punto 4 arriba).
- Mensajes custom `BottomLaneState.msg`, `CylindricalPose.msg`, `InspectionState.msg`, `Obstacle.msg`, `CoverageStatus.msg` descritos en `PROJECT_PLAN.md:240+` — **no existe ningún paquete de mensajes** en `ros2_ws/src/`. Hoy todo se publica como `std_msgs/String` JSON.
- `coverage_controller` helicoidal: explícitamente marcado como descartado y no implementado, pero se sigue mencionando para dejar claro que NO se usa. Aceptable.

Ver matriz completa en `DOCS_CONSISTENCY.md`.
