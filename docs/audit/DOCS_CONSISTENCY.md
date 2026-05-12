# DOCS_CONSISTENCY — Documentación vs código

Fase PLAN. Solo análisis, sin reescritura de docs.

---

## 1. Estado por documento

| Documento | Estado global | Problemas principales | Acción recomendada (para BUILD) |
|---|---|---|---|
| `README.md` | **NECESITA ACTUALIZACIÓN** | (1) declara `cylindrical_map_node.py` "EN DESARROLLO" cuando ya está implementado y se lanza por defecto. (2) tabla "Arquitectura de inspección propuesta" mezcla nodos implementados con futuros sin separación visual clara. (3) cita Nav2 como descartado: hay que añadir nota sobre la futura dirección Nav2-cilindro. | Reescribir secciones "Arquitectura propuesta" y aclarar qué existe y qué no. |
| `PROJECT_PLAN.md` | **NECESITA ACTUALIZACIÓN** | Lista de 7 nodos "PENDIENTE/FUTURO" que no existen en código. Estados marcados, pero el documento es del orden de 580 líneas y resulta difícil ver qué es real. | Mantener como roadmap; añadir bloque "Implementado a fecha 2026-05-12" para reflejar diferencia con el código. |
| `PROJECT_STATE.md` | **CORRECTO** con desfase menor | Última actualización 2026-05-09; algunas líneas (84-91) listan nodos pendientes con guion suelto sin marcar `PENDIENTE` explícitamente. Línea 68: dice `slam.launch.py` "No usado actualmente" pero no marca que **además está roto vs el pipeline actual**. | Actualizar fecha y aclarar estado de launchers SLAM/RTAB-Map (rotos/secundarios). |
| `ARCHITECTURE.md` | **CORRECTO** | Diagramas grandes, coherentes con `simulation.launch.py` e `inspection.launch.py`. Menciona explícitamente `scan_gate` y `map_accumulator` como "disponible/prototipo, no en uso activo" (L85, L88). | Solo añadir mención de la nueva dirección Nav2 cilíndrico cuando exista. |
| `LAUNCH_GUIDE.md` | **NECESITA ACTUALIZACIÓN** | (1) L313-319 menciona uso manual de `map_accumulator` aunque el nodo no se documenta como soportado. (2) No menciona que `slam.launch.py` no funciona contra el pipeline real (`/scan` no se publica). | Marcar `slam.launch.py` como obsoleto y aclarar uso de `map_accumulator`. |
| `CONVERSATION_LESSONS.md` | **CORRECTO** | Notas históricas claras. | Conservar tal cual. |
| `AGENTS.md` | **CORRECTO** | Conciso, sin contradicciones internas. Lista correctamente los docs de referencia. | Conservar tal cual. |
| `docs/agent/CONTROL_DEBUG_BRIEF.md` | **CORRECTO** | Documento vivo; útil para depuración PI. | Conservar. |
| `docs/agent/DEBUG_OUTPUT_POLICY.md` | **CORRECTO** | Política de output corto. | Conservar. |
| `docs/agent/OPEN_CODE_WORKFLOW.md` | **NO VERIFICADO** | No revisado en profundidad en este audit. Su nombre sugiere instrucciones para OpenCode CLI. | Revisión rápida en BUILD; probablemente conservar. |
| `tools/debug/README.md` | **CORRECTO** | Describe `capture_inspection_debug.py`. | Conservar. |
| `docs/` upstream `gz_ros2_control/...` | N/A (fork upstream) | No tocar. | NO TOCAR. |

---

## 2. Matriz de afirmaciones doc → código

| # | Afirmación en docs | Documento (línea aprox) | ¿Existe en código? | Evidencia | Veredicto |
|---|---|---|---|---|---|
| 1 | "Existe `state_machine_node.py`" | README:82, STATE:82, PLAN:140 | Sí | `wind_tower_inspection_behaviour/wind_tower_inspection_behaviour/state_machine_node.py` | CORRECTO |
| 2 | "Existe `stability_monitor_node.py` validado" | README:83, STATE:80 | Sí | mismo paquete | CORRECTO |
| 3 | "Existe `cylindrical_map_node.py` EN DESARROLLO" | README:85 | Sí, implementado y lanzado por defecto | `inspection.launch.py:493` | INCOHERENTE (subestima estado) |
| 4 | "Existe `bottom_lane_controller.py`" | PLAN:142, README:84 (PENDIENTE) | **No** | grep negativo en `ros2_ws/src` | DOCUMENTADO PERO NO EXISTE (marcado PENDIENTE — aceptable, pero formato confunde) |
| 5 | "Existe `tube_indexing_controller.py`", "obstacle_manager", "bypass_manager", "image_capture_manager", "local_inspection_controller", "report_generator" | PLAN:144-149, STATE:85-90 | **No**, ninguno | grep negativo | DOCUMENTADO COMO PENDIENTE; OK pero conviene moverlos a sección "Roadmap futuro" separada |
| 6 | "Se lanza con `simulation.launch.py`" | README:171, LAUNCH_GUIDE:48 | Sí | archivo existe y funciona | CORRECTO |
| 7 | "Se lanza con `inspection.launch.py`" | README:180, AGENTS.md | Sí | archivo existe | CORRECTO |
| 8 | "scan_gate garantiza que /scan solo llega cuando TF está listo" | `slam.launch.py:4` (comentario) | **No**: `scan_gate` no está lanzado en `simulation.launch.py` y `/scan` no se publica | grep negativo en `simulation.launch.py` y `inspection.launch.py` | CONTRADICTORIO |
| 9 | "El sistema usa SLAM (slam_toolbox)" | LAUNCH_GUIDE menciona `slam.launch.py` | Existe el launcher, pero **no es funcional** contra el pipeline actual (sin `/scan`) | `slam.launch.py:20-26` espera `/scan` | OBSOLETO |
| 10 | "El sistema usa RTAB-Map" | STATE:68, PLAN:115, ARCH | Existe el launcher; consume `/velodyne_points`. Decisión explícita: NO usar para `θ_tube` | `rtabmap.launch.py:16` | CORRECTO pero AUXILIAR (no MVP) |
| 11 | "El sistema usa Nav2" | README:109, STATE:160, PLAN:75 dicen explícitamente NO | No hay Nav2 en `ros2_ws/src/` | grep negativo de `nav2_*` en config/launch | CORRECTO (decisión declarada) |
| 12 | "El sistema tiene pipeline de inspección autónoma" | README, STATE, ARCH | Sí: state_machine + stability_monitor + cylindrical_map operativos | `inspection.launch.py` | CORRECTO (en validación) |
| 13 | "Publica `/turner/angle` y `/turner/angle_deg`" | ARCH:58, STATE:99 | Sí | `turner_node.py` (entry point en `setup.py:41`) | CORRECTO |
| 14 | "Publica `/inspection/bottom_lane_locked`, `/safe_to_scan`, `/safe_to_index_tube`" | README:83, ARCH:102-104 | Sí | `stability_monitor_node.py` | CORRECTO |
| 15 | "El sistema usa IMU" | README:53, STATE:107, ARCH | Sí | `imu_bridge` en `simulation.launch.py:225` | CORRECTO |
| 16 | "El sistema controla el giro del tubo" | README, STATE | Sí | `turner_cmd_bridge` + `turner_node` + state_machine en `INDEX_TUBE` | CORRECTO |
| 17 | "El sistema tiene navegación cilíndrica" | README:130, PLAN:118 | Parcialmente: hay malla `(x, θ)` para **cobertura**, NO para navegación Nav2 | `cylindrical_map_node.py` no participa en control | MATIZ: cobertura sí; navegación cilíndrica NO |
| 18 | "El sistema usa mapa 2D" | LAUNCH_GUIDE/STATE referencian SLAM | No en flujo activo (slam_toolbox no se lanza realmente y no tiene `/scan`) | — | OBSOLETO en el MVP |
| 19 | "El sistema usa lidar 3D + cámara" | README, ARCH, STATE | Sí: VLP-16 + cámara TCP | bridges en `simulation.launch.py:149,165` | CORRECTO |
| 20 | "Existen mensajes custom `BottomLaneState.msg`, `CylindricalPose.msg`, etc." | PLAN:240+ | **No**: no hay paquete de msg en `ros2_ws/src` | grep negativo de `.msg` | DOCUMENTADO PERO NO EXISTE (marcado como propuesto — OK) |
| 21 | "EKF propio en `ekf.yaml`" | STATE:73 (matiz: "en pruebas usa /robot/platform/odom") | Sí, se arranca, pero **consumido por nadie en producción** | `simulation.launch.py:237` | CONTRADICTORIO/AMBIGUO — se lanza pero se ignora |
| 22 | "Botón Triángulo → START_AUTO" | README:192, STATE:117 | Sí | `ps5_teleop.py` (no revisado aquí; coherente con doc) | CORRECTO (NO VERIFICADO al detalle) |
| 23 | "auto_start=false por defecto" | STATE:83, inspection.launch.py:183 | Sí | `inspection.launch.py:183 default_value='false'` | CORRECTO |
| 24 | "Existe `coverage_controller` helicoidal" | descartado en docs (README:93, PLAN:134, STATE:92) | No existe en código | grep negativo | CORRECTO (descartado y nunca implementado) |

---

## 3. Documentos candidatos a legacy o consolidación

Ninguno se mueve en esta fase. Solo se propone:

| Documento | Propuesta | Motivo |
|---|---|---|
| `slam.launch.py` (no es doc, pero relacionado) | Mover a `docs/legacy/` o `examples/` tras confirmación | Hoy no es funcional contra el pipeline (`/scan` no existe). Útil como referencia educativa |
| `rtabmap.launch.py` | Mantener pero documentar como experimental aislado | Decisión arquitectónica explícita de no usarlo para `θ_tube` |

No se identifica documentación obsoleta que deba moverse hoy a `docs/legacy/`. Las contradicciones detectadas son corregibles editando los docs principales.

---

## 4. Riesgo de las correcciones documentales

| Acción | Riesgo |
|---|---|
| Aclarar en README estado real de `cylindrical_map_node` | **BAJO** |
| Marcar `slam.launch.py` como obsoleto en LAUNCH_GUIDE | **BAJO** |
| Separar en PROJECT_PLAN la sección "implementado" vs "roadmap" | **BAJO** |
| Añadir sección Nav2-cilindro a docs | **MEDIO** (requiere alineamiento con usuario sobre alcance) |
| Mover `slam.launch.py` físicamente | **MEDIO** — `NO TOCAR AÚN` hasta confirmación humana |
| Reescribir `AGENTS.md` o `docs/agent/*.md` | **NO TOCAR AÚN** — funciona |
