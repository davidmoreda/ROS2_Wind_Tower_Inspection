# BUILD_INPUT — ROS2 WindTower documentation and launch cleanup

Entrada accionable para la futura fase BUILD. Generada por la fase PLAN (audit-only) en `docs/audit/`.

---

## 1. Executive summary

El repositorio tiene **dos paquetes activos** que conforman el MVP:

- `wind_tower_bringup` provee la simulación, bridges, teleop y herramientas LiDAR.
- `wind_tower_inspection_behaviour` provee la lógica autónoma (state_machine + stability_monitor + cylindrical_map).

El flujo operativo real lo describen exactamente dos launchers: `simulation.launch.py` (entorno) e `inspection.launch.py` (misión). Los launchers `slam.launch.py` y `rtabmap.launch.py` están en el repo como referencia/experimentos; el primero **es obsoleto** porque depende de `/scan` que el pipeline actual no produce.

La documentación principal (README, ARCHITECTURE, PROJECT_STATE, PROJECT_PLAN, LAUNCH_GUIDE) es **mayormente coherente con el código**, pero arrastra inconsistencias menores (estado de `cylindrical_map`, EKF redundante, nodos pendientes mezclados con implementados) y **no menciona la nueva dirección Nav2 sobre cilindro desplegado**, que hoy NO existe en código.

`AGENTS.md` y `docs/agent/*.md` están limpios y útiles; no requieren intervención.

Recomendación: BUILD debe ser **documental + clarificación + mover legacy con confirmación**. NO refactor de código funcional.

---

## 2. Current active architecture

```
[Gazebo Harmonic + wind_tower_world.sdf]
        │
        ├── Husky + UR5e + VLP-16 (vía clearpath_gz/robot_spawn)
        ├── turner_joint (revolute) + JointController plugin
        │
        ▼
[ros_gz_bridge / image_bridge / topic_tools relay]
        │
        ▼
[wind_tower_bringup nodes]
   ├─ tf_static_relay
   ├─ turner_node        → /turner/angle, /turner/angle_deg
   ├─ ekf_filter_node    (lanzado pero parece no consumido en producción)
   └─ cylinder_localizer → /robot_in_tube, /cylinder_fit/stats
        │
        ▼
[wind_tower_inspection_behaviour nodes]
   ├─ stability_monitor  → /inspection/{bottom_lane_locked,safe_to_scan,safe_to_index_tube,stability}
   ├─ cylindrical_map    → /inspection/{cylindrical_pose,coverage_status,cylindrical_map_stats}
   └─ state_machine      → /robot/platform/cmd_vel, /turner/cmd_vel,
                           /inspection/{state,state_text,current_lane,mission_status,autonomous_active}

[PS5 DualSense]
   └─ dualsense_joy → ps5_teleop → /robot/joy_teleop/joy + /inspection/mission_command
```

Detalle completo en `docs/audit/LAUNCH_GRAPH.md` y `docs/audit/REPO_AUDIT.md`.

---

## 3. Main inconsistencies found

1. `slam.launch.py` se basa en `/scan` que no se publica → **obsoleto contra el pipeline actual**. Comentario interno menciona `scan_gate` en `simulation.launch.py` pero `scan_gate` NO se lanza.
2. `scan_gate_node.py` y `map_accumulator_node.py` tienen `entry_points` y se documentan como "disponibles / prototipos", pero **ningún launcher los arranca**.
3. `cylindrical_map_node.py` aparece como "EN DESARROLLO" en README pese a estar implementado y lanzado por defecto.
4. `PROJECT_PLAN.md` y `PROJECT_STATE.md` listan 7 nodos pendientes en el paquete inspection_behaviour sin separación clara respecto a los implementados.
5. `ekf_filter_node` (robot_localization) se lanza en `simulation.launch.py` pero el flujo real consume `/robot/platform/odom/filtered` del stack Clearpath. Doble EKF posiblemente redundante.
6. Mensajes custom (`BottomLaneState.msg`, `CylindricalPose.msg`, …) descritos en `PROJECT_PLAN.md` no existen en ningún paquete de mensajes.
7. Ninguna mención en docs ni código sobre Nav2 cilíndrico, frame `cyl_map`, costmaps, ni adapter `/cmd_vel` → base+turner. Es la dirección de evolución que se quiere preparar.
8. Carpetas vacías `.agents/`, `.codex/`, y subcarpetas `wind_tower_description/{rviz,src,launch}`, `wind_tower_simulation/{src,launch,include}` no documentadas.
9. `debug_runs/` con 30+ ejecuciones acumuladas no controladas en `.gitignore` visible.

---

## 4. Files to modify in BUILD

| Prioridad | Archivo | Acción | Razón | Evidencia |
|---|---|---|---|---|
| 1 | `README.md` | Reescribir tabla "Arquitectura propuesta": separar IMPLEMENTADO / EN DESARROLLO / FUTURO. Marcar `cylindrical_map_node.py` como IMPLEMENTADO. Añadir nota sobre futura dirección Nav2-cilindro. | Hoy mezcla estados. | `README.md:80-91` |
| 1 | `PROJECT_STATE.md` | Actualizar fecha; aclarar que `slam.launch.py` es obsoleto vs pipeline; aclarar estado del EKF propio. | Última fecha 2026-05-09; algunas notas desfasadas. | `PROJECT_STATE.md:1,68,73` |
| 1 | `LAUNCH_GUIDE.md` | Marcar `slam.launch.py` como obsoleto; revisar mención de `map_accumulator` (L313-319). | El launcher no funciona con `/scan` ausente. | `LAUNCH_GUIDE.md:313-319` |
| 2 | `PROJECT_PLAN.md` | Añadir sección "Implementado al 2026-05-12" arriba; mover los 7 nodos pendientes a sub-sección "Roadmap MVP completo" claramente separada. | Doc largo difícil de leer. | `PROJECT_PLAN.md:99-149` |
| 2 | `ARCHITECTURE.md` | Pequeña nota junto a `ekf.yaml` y al bloque `scan_gate`/`map_accumulator` indicando "no integrado en flujo activo". Mantener diagramas grandes. | Doc preciso pero sin etiquetar prototipos. | `ARCHITECTURE.md:85-89` |
| 2 | `docs/audit/` | Conservar este audit como referencia versionada. | Documento de PLAN. | (este archivo) |
| 3 | `ros2_ws/src/wind_tower_bringup/setup.py` | **OPCIONAL**: si se decide retirar `scan_gate` y `map_accumulator`, eliminar sus `console_scripts`. Requiere confirmación. | Hoy generan ejecutables que no se usan. | `setup.py:40,42` |
| 3 | `ros2_ws/src/wind_tower_bringup/launch/simulation.launch.py` | **OPCIONAL** y solo tras verificación de consumidores: retirar `ekf_filter_node` y `ekf.yaml`. | Probable redundancia con EKF Clearpath. | `simulation.launch.py:237-249` |
| 3 | `.gitignore` | Añadir `debug_runs/` si no está. | Histórico de logs no controlado. | `.gitignore:?` |
| 4 | Nuevo `docs/architecture/NAV2_CYLINDRICAL.md` (futuro) | Una vez aprobado el cambio de dirección, redactar arquitectura objetivo basándose en `NAV2_CYLINDRICAL_GAP_ANALYSIS.md`. | Direction nueva. | — |

---

## 5. Files not to touch

| Archivo | Razón |
|---|---|
| `ros2_ws/src/wind_tower_inspection_behaviour/wind_tower_inspection_behaviour/*.py` | Código de misión activo y bajo validación; cualquier refactor no documental rompe MVP. |
| `ros2_ws/src/wind_tower_inspection_behaviour/launch/inspection.launch.py` | Contrato operativo activo (~60 launch args usados por el operador). |
| `ros2_ws/src/wind_tower_inspection_behaviour/config/*.yaml` | Parámetros validados de misión. |
| `ros2_ws/src/wind_tower_simulation/worlds/wind_tower_world.sdf` | Mundo activo. |
| `ros2_ws/src/wind_tower_simulation/models/wind_tower_tube/model.sdf` | Modelo del tubo. |
| `ros2_ws/src/wind_tower_description/urdf/inspection_camera_lighting.urdf.xacro` | Cabezal cámara/luces simulado. |
| `ros2_ws/src/wind_tower_description/meshes/TRAMO_TORRE.STL` | Mesh del tubo. |
| `ros2_ws/src/gz_ros2_control/**` | Fork upstream con fix WSL2; no tocar. |
| `AGENTS.md`, `docs/agent/*.md` | Vigentes y útiles. |
| `CONVERSATION_LESSONS.md` | Notas históricas vigentes. |
| `tools/debug/capture_inspection_debug.py` y `tools/debug/README.md` | Tool activa. |
| `assets/` | No estorba; potencialmente útil para docs futuras. |

---

## 6. Candidate legacy files (NO mover sin confirmación humana)

| Archivo | Razón | Riesgo | Necesita confirmación humana |
|---|---|---|---|
| `ros2_ws/src/wind_tower_bringup/launch/slam.launch.py` | Obsoleto contra pipeline; depende de `/scan` inexistente. | MEDIO | **Sí** |
| `ros2_ws/src/wind_tower_bringup/config/slam_toolbox.yaml` | Solo lo usa `slam.launch.py`. | MEDIO | **Sí** |
| `ros2_ws/src/wind_tower_bringup/wind_tower_bringup/scan_gate_node.py` | Sin uso en launch ni docs operativas. | MEDIO | **Sí** (decidir: mover a `examples/` o conservar como prototipo documentado) |
| `ros2_ws/src/wind_tower_bringup/wind_tower_bringup/map_accumulator_node.py` | Sin uso en launch; potencialmente reemplazado por `cylindrical_map_node.py`. | MEDIO | **Sí** |
| `ros2_ws/src/wind_tower_bringup/launch/rtabmap.launch.py` | Experimental aislado; decisión declarada de no usar. | MEDIO | **Sí** (preferencia: conservar pero etiquetar EXPERIMENTAL) |
| Carpetas vacías `.agents/`, `.codex/`, `wind_tower_description/rviz`, `wind_tower_description/src`, `wind_tower_description/launch`, `wind_tower_simulation/src`, `wind_tower_simulation/launch`, `wind_tower_simulation/include` | Sin contenido ni propósito documentado. | BAJO | **Sí** (eliminar o documentar) |
| `debug_runs/<timestamps>/*` ejecuciones antiguas | Histórico, no debería estar trackeado en git. | BAJO | **Sí** (mover a backup local + .gitignore) |

---

## 7. Launchers to keep as source of truth

| Launcher | Razón |
|---|---|
| `wind_tower_bringup/launch/simulation.launch.py` | Único launcher que monta mundo Gazebo + robot + bridges. |
| `wind_tower_inspection_behaviour/launch/inspection.launch.py` | Único launcher que arranca la pila de inspección autónoma. |

---

## 8. Launchers/nodes to review or deprecate

| Item | Razón | Evidencia | Recomendación |
|---|---|---|---|
| `slam.launch.py` | No funcional contra el pipeline activo | `/scan` no se publica | Mover a `docs/legacy/launch/` con README, tras confirmación |
| `rtabmap.launch.py` | Experimental aislado, no MVP | `PROJECT_STATE.md:221-225` | Mantener; documentar como EXPERIMENTAL en cabecera y `LAUNCH_GUIDE` |
| `scan_gate_node.py` | Sin uso real | `grep` negativo en launchers | Decidir: documentar como prototipo o mover a `examples/` |
| `map_accumulator_node.py` | Sin uso real | Idem | Decidir igual |
| `ekf_filter_node` (en simulation.launch.py L237-249) | Probable redundancia con Clearpath EKF | `PROJECT_STATE.md:73` | Verificar consumidores; si nadie consume `/odometry/filtered` propio → eliminar |

---

## 9. Documentation rewrite plan

Orden recomendado:

1. **README.md** — fuente principal: estado realista + sección breve sobre evolución Nav2 cilíndrico.
2. **PROJECT_STATE.md** — datar y reclasificar.
3. **LAUNCH_GUIDE.md** — marcar lo obsoleto/experimental.
4. **PROJECT_PLAN.md** — separar implementado vs roadmap.
5. **ARCHITECTURE.md** — anotaciones puntuales sobre prototipos.
6. (Futuro) **docs/architecture/NAV2_CYLINDRICAL.md** — solo cuando se apruebe la nueva dirección.

Documentos a NO reescribir: `AGENTS.md`, `docs/agent/*.md`, `CONVERSATION_LESSONS.md`, `tools/debug/README.md`.

---

## 10. Nav2 cylindrical architecture gap plan

Ver detalle en `docs/audit/NAV2_CYLINDRICAL_GAP_ANALYSIS.md`. Resumen:

- **Falta todo** (frame `cyl_map`, odom cilíndrica, costmaps, params, planner, controller, adapter `/cmd_vel` → base+turner, supervisor Nav2-aware).
- **Reutilizable**: `stability_monitor`, `cylinder_localizer`, `turner_node`, `cylindrical_map` (cobertura).
- BUILD no debe implementar Nav2 todavía. BUILD solo debe **documentar** la arquitectura objetivo y propuesta de fases (F1-F7).

---

## 11. BUILD task list

### Priority 1 — Make docs truthful

- [ ] Actualizar README.md: estado real de `cylindrical_map_node.py` + nota Nav2-cilindro.
- [ ] Actualizar PROJECT_STATE.md: fecha + reclasificar SLAM/RTAB-Map + aclarar EKF.
- [ ] Actualizar LAUNCH_GUIDE.md: marcar `slam.launch.py` y `map_accumulator` como no integrados.

### Priority 2 — Clarify active launchers

- [ ] Añadir cabeceras claras en `simulation.launch.py` e `inspection.launch.py` indicando que son **launchers centrales**.
- [ ] Añadir cabecera EXPERIMENTAL en `rtabmap.launch.py`.

### Priority 3 — Mark legacy/prototype components

- [ ] Decidir destino de `slam.launch.py` + `slam_toolbox.yaml` (mover a `docs/legacy/` o `examples/`).
- [ ] Decidir destino de `scan_gate_node.py` y `map_accumulator_node.py`.
- [ ] Eliminar (o documentar) `.agents/`, `.codex/` y carpetas vacías.
- [ ] Añadir `debug_runs/` a `.gitignore` si falta.

### Priority 4 — Add Nav2 cylindrical architecture documentation

- [ ] Crear `docs/architecture/NAV2_CYLINDRICAL.md` con arquitectura objetivo y fases F1-F7.
- [ ] Referenciarla desde `README.md` y `PROJECT_PLAN.md`.

### Priority 5 — Prepare future implementation tasks

- [ ] Especificar contrato de frames TF y messages para `cylindrical_odom_node`.
- [ ] Especificar política del adapter `/cmd_vel` → base + turner.
- [ ] Definir mensajes custom (paquete `wind_tower_msgs`) si se decide formalizarlos.

---

## 12. Acceptance criteria for BUILD

BUILD se considera bien terminado cuando:

1. Cualquier afirmación en `README.md`, `PROJECT_STATE.md`, `LAUNCH_GUIDE.md` y `PROJECT_PLAN.md` sobre nodos/launchers/topics tiene **evidencia verificable** en el código (ruta y línea).
2. `LAUNCH_GUIDE.md` distingue de forma inequívoca: launchers centrales, auxiliares, experimentales, obsoletos.
3. Cada nodo del paquete `wind_tower_bringup` tiene un estado documentado entre: USADO, USADO SOLO EN TEST, PROTOTIPO NO INTEGRADO, LEGACY.
4. Los archivos identificados como obsoletos están **movidos a `docs/legacy/` o `examples/`** (no borrados) tras confirmación humana, o explícitamente conservados con justificación.
5. La nueva dirección Nav2 cilindro está documentada en `docs/architecture/NAV2_CYLINDRICAL.md` (sin implementación todavía).
6. El comando `colcon build --packages-select wind_tower_bringup wind_tower_inspection_behaviour` sigue siendo válido y `ros2 launch wind_tower_inspection_behaviour inspection.launch.py` arranca igual que antes.
7. `AGENTS.md` y `docs/agent/*.md` no han sido modificados sin necesidad.
8. No se ha tocado código de control / state_machine / stability_monitor / cylindrical_map.
