# BUILD_SUMMARY — Resumen de la fase BUILD

Fecha: 2026-05-12. Dos sesiones: **BUILD-1** (documentación) y **BUILD-2** (limpieza física).

## Sesión BUILD-2 — Limpieza ejecutada

Acciones destructivas autorizadas por David:

### Movidos con `git mv` (historia conservada)

| Origen | Destino |
|---|---|
| `ros2_ws/src/wind_tower_bringup/launch/slam.launch.py` | `docs/legacy/launch/slam.launch.py` |
| `ros2_ws/src/wind_tower_bringup/config/slam_toolbox.yaml` | `docs/legacy/launch/slam_toolbox.yaml` |
| `ros2_ws/src/wind_tower_bringup/wind_tower_bringup/scan_gate_node.py` | `docs/legacy/nodes/scan_gate_node.py` |
| `ros2_ws/src/wind_tower_bringup/wind_tower_bringup/map_accumulator_node.py` | `docs/legacy/nodes/map_accumulator_node.py` |
| `PROJECT_STATE.md` | `docs/legacy/docs/PROJECT_STATE.md` |
| `PROJECT_PLAN.md` | `docs/legacy/docs/PROJECT_PLAN.md` |
| `ARCHITECTURE.md` | `docs/legacy/docs/ARCHITECTURE.md` |
| `LAUNCH_GUIDE.md` | `docs/legacy/docs/LAUNCH_GUIDE.md` |

### Eliminados con `git rm`

- `ros2_ws/src/wind_tower_bringup/config/ekf.yaml` — config del EKF propio retirado.

### Eliminados con `rm` (carpetas vacías sin tracking)

- `.agents/`, `.codex/`
- `ros2_ws/src/wind_tower_description/{rviz,src,launch}`
- `ros2_ws/src/wind_tower_simulation/{src,launch,include}`

### Editados

| Archivo | Cambio |
|---|---|
| `ros2_ws/src/wind_tower_bringup/setup.py` | Retirados entry_points `scan_gate`, `map_accumulator`. Retirados de `data_files`: `slam.launch.py`, `slam_toolbox.yaml`, `ekf.yaml`. |
| `ros2_ws/src/wind_tower_bringup/launch/simulation.launch.py` | Retirado bloque `ekf_node` (robot_localization), `pkg_wind_bringup` ya no necesario, import `ParameterFile` retirado, salida del `LaunchDescription` sin `ekf_node`. |
| `README.md` | Tabla "Estado actual" refleja que SLAM está RETIRADO. Sección "Advertencias" reformulada. Índice de docs apunta a `docs/legacy/README.md`. |
| `docs/operation/LAUNCHERS_REFERENCE.md` | Eliminada fila `slam.launch.py`; sección "Nota EKF" reformulada; sección "slam.launch.py LEGACY" sustituida por nota de archivado. |
| `docs/operation/NODES_REFERENCE.md` | Eliminadas filas `scan_gate`, `map_accumulator`, `ekf_filter_node`; añadida sección "Retirado en BUILD". |
| `docs/architecture/CURRENT_ARCHITECTURE.md` | Diagrama, tablas y referencias actualizadas para reflejar la limpieza. Punteros a `docs/legacy/docs/*` para citas a docs viejos. |
| `docs/operation/HOW_TO_LAUNCH.md`, `docs/development/DEVELOPMENT_GUIDE.md`, `docs/architecture/NAV2_CYLINDRICAL_NAVIGATION.md` | Re-enrutados enlaces a `docs/legacy/docs/`. |
| `docs/legacy/README.md` | Índice completo de lo archivado. |

### Verificaciones

- `python3 -c "ast.parse(...)"` confirma que `setup.py` e `simulation.launch.py` siguen siendo Python válido.
- `grep` confirma que no quedan referencias activas a `scan_gate`, `map_accumulator`, `ekf.yaml` ni `slam.launch.py` fuera de `docs/legacy/` y `docs/audit/`.

### Lo que NO tocó BUILD-2

- Cambios previos del usuario en `state_machine.yaml`, `inspection.launch.py` (paquete inspection_behaviour), `stability_monitor_node.py`, `state_machine_node.py`, `wind_tower_world.sdf` y `.gitignore` — son trabajo del usuario, ajenos a esta limpieza.
- `rtabmap.launch.py` — conservado como AUXILIAR.
- Todo el código del paquete `wind_tower_inspection_behaviour`.

---

## Sesión BUILD-1 — Documentación (resumen)

---

## 1. Archivos creados

### Nueva estructura `docs/`

- `docs/architecture/CURRENT_ARCHITECTURE.md` — qué existe hoy (con evidencia)
- `docs/architecture/TARGET_ARCHITECTURE.md` — arquitectura objetivo (Nav2 cilindro), marcada como PROPUESTA
- `docs/architecture/NAV2_CYLINDRICAL_NAVIGATION.md` — detalle técnico del enfoque cilindro desplegado
- `docs/operation/HOW_TO_LAUNCH.md` — arranque mínimo del flujo MVP
- `docs/operation/LAUNCHERS_REFERENCE.md` — tabla canónica de launchers con estado
- `docs/operation/NODES_REFERENCE.md` — tabla canónica de nodos con estado
- `docs/operation/TOPICS_AND_FRAMES.md` — topics y frames TF reales
- `docs/development/DEVELOPMENT_GUIDE.md` — clonar, ramas, build, añadir nodos
- `docs/legacy/README.md` — índice (vacío; ningún archivo movido)
- `docs/audit/BUILD_DOCS_PLAN.md` — plan de cambios documentales
- `docs/audit/BUILD_SUMMARY.md` — este archivo

Conservados de la fase PLAN previa:

- `docs/audit/BUILD_INPUT.md`
- `docs/audit/REPO_AUDIT.md`
- `docs/audit/LAUNCH_GRAPH.md`
- `docs/audit/DOCS_CONSISTENCY.md`
- `docs/audit/CLEANUP_PLAN.md`
- `docs/audit/NAV2_CYLINDRICAL_GAP_ANALYSIS.md`

---

## 2. Archivos modificados (raíz)

| Archivo | Cambio |
|---|---|
| `README.md` | **Reescritura completa.** Nueva tabla "Estado actual" con marcas IMPLEMENTADO/PROPUESTO/LEGACY, paquetes, launchers principales con estado, sección "Cómo arrancar" concisa, "Concepto de inspección", "Flujo de desarrollo", índice de documentación, advertencias importantes. |
| `PROJECT_STATE.md` | Banner de estado en cabecera redirigiendo a `docs/architecture/` y `docs/operation/`. Contenido conservado intacto. |
| `PROJECT_PLAN.md` | Banner de estado redirigiendo a `docs/architecture/` y avisando que los mensajes custom NO existen. Contenido conservado. |
| `ARCHITECTURE.md` | Banner de estado redirigiendo a `docs/architecture/CURRENT_ARCHITECTURE.md` y `docs/operation/TOPICS_AND_FRAMES.md`. Contenido conservado. |
| `LAUNCH_GUIDE.md` | Banner de estado redirigiendo a `docs/operation/HOW_TO_LAUNCH.md` con advertencias sobre `slam.launch.py` (LEGACY) y `map_accumulator` (no integrado). Contenido conservado. |
| `AGENTS.md` (no trackeado en git) | Actualizada sección "Documentos de referencia" con el nuevo orden y "Reglas para agentes" con clasificación de estados (IMPLEMENTADO/PROPUESTO/LEGACY). |

---

## 3. Archivos NO modificados (y por qué)

| Archivo | Razón |
|---|---|
| Cualquier `.py`, `.launch.py`, `.yaml`, `.sdf`, `.xacro`, `.stl` bajo `ros2_ws/src/` | Esta fase BUILD es solo documental. No se ha tocado código ni configuración. |
| `CONVERSATION_LESSONS.md` | Vigente y útil tal cual. |
| `docs/agent/CONTROL_DEBUG_BRIEF.md` | Vigente; documento vivo para depuración PI. |
| `docs/agent/DEBUG_OUTPUT_POLICY.md` | Vigente. |
| `docs/agent/OPEN_CODE_WORKFLOW.md` | No verificado en profundidad; sin razón clara para modificar. |
| `tools/debug/README.md` y `tools/debug/capture_inspection_debug.py` | Vigentes. |
| `ros2_ws/src/gz_ros2_control/**` | Fork upstream con fix WSL2; no tocar. |
| `assets/`, `debug_runs/` | Sin impacto en documentación. `debug_runs/` ya está en `.gitignore`. |

---

## 4. Documentos marcados como históricos (con banner)

- `PROJECT_STATE.md`
- `PROJECT_PLAN.md`
- `ARCHITECTURE.md`
- `LAUNCH_GUIDE.md`

Estos archivos siguen donde estaban; el banner remite al lector a la nueva fuente de verdad en `docs/`. Se conserva todo su contenido por valor histórico y por el detalle que mantienen (decisiones técnicas, fixes WSL2, procedimiento `usbipd`, diagramas grandes).

---

## 5. Documentos movidos a `docs/legacy/`

**Ninguno** en esta sesión BUILD. La auditoría identificó candidatos (ver `docs/audit/BUILD_INPUT.md` §6) pero su movimiento físico requiere confirmación humana explícita.

Candidatos pendientes:

- `ros2_ws/src/wind_tower_bringup/launch/slam.launch.py`
- `ros2_ws/src/wind_tower_bringup/config/slam_toolbox.yaml`
- `ros2_ws/src/wind_tower_bringup/wind_tower_bringup/scan_gate_node.py`
- `ros2_ws/src/wind_tower_bringup/wind_tower_bringup/map_accumulator_node.py`

Listados también en `docs/legacy/README.md`.

---

## 6. Riesgos pendientes (no abordados)

1. **EKF redundante** (`ekf_filter_node` en `simulation.launch.py:237-249`): documentado pero no eliminado; requiere verificación de consumidores con `ros2 topic info -v /odometry/filtered` antes de retirar.
2. **Mensajes custom propuestos** (`wind_tower_msgs`): documentados como PROPUESTOS pero el roadmap de `PROJECT_PLAN.md` los sigue describiendo. La nueva doc en `docs/architecture/` no los promete.
3. **`slam.launch.py`**: marcado LEGACY PROBABLE en todas las nuevas tablas, pero el archivo sigue presente. Lanzarlo no rompe nada pero genera errores; documentar en `LAUNCH_GUIDE.md` ya advierte.
4. **`scan_gate` y `map_accumulator`**: marcados PROTOTIPO NO INTEGRADO. Sus entry points en `setup.py` siguen activos.
5. **`docs/agent/OPEN_CODE_WORKFLOW.md`**: NO VERIFICADO; podría tener instrucciones desactualizadas para OpenCode CLI. Pendiente de revisión humana.

---

## 7. Qué debe revisar David manualmente

1. **README.md completo**: aprobar la nueva narrativa antes de mergear. Verificar que la tabla "Estado actual" refleja su criterio.
2. **`docs/architecture/CURRENT_ARCHITECTURE.md`**: confirmar que cada componente clasificado como IMPLEMENTADO o PARCIAL refleja su validación.
3. **`docs/architecture/TARGET_ARCHITECTURE.md` y `NAV2_CYLINDRICAL_NAVIGATION.md`**: alineamiento con la dirección que quiere para Nav2 cilindro. Revisar especialmente la decisión "plano infinito vs cíclico en Y".
4. **Banners de estado** en `PROJECT_STATE.md`, `PROJECT_PLAN.md`, `ARCHITECTURE.md`, `LAUNCH_GUIDE.md`: confirmar tono y enlaces.
5. **`AGENTS.md`**: actualizado para que sea operativo con la nueva estructura (recordar: no trackeado en git).
6. **Candidatos legacy**: dar luz verde (o no) para mover `slam.launch.py` + `slam_toolbox.yaml` + `scan_gate_node.py` + `map_accumulator_node.py` a `docs/legacy/` en una próxima sesión.
7. **EKF redundante**: decidir si retirar `ekf_filter_node` de `simulation.launch.py` o mantenerlo.

---

## 8. Próximos pasos recomendados

Orden propuesto:

1. **Aceptar o ajustar este BUILD documental** y mergear a `main`.
2. **Confirmación sobre legacy** → mover archivos identificados a `docs/legacy/` con README.
3. **Decidir sobre el EKF propio** → eliminar o conservar con justificación.
4. **(Opcional) Fase BUILD-2** para limpieza ligera de código:
   - Retirar entry points no usados de `setup.py`.
   - Comentarios de estado en cabecera de `rtabmap.launch.py` y `slam.launch.py`.
5. **Empezar la dirección Nav2 cilindro** (fase F1): crear rama `feat/nav2-cyl-f1-odom` e implementar `cylindrical_odom_node` + TF según `docs/architecture/NAV2_CYLINDRICAL_NAVIGATION.md`.
6. **Considerar fase VERIFY**: confirmar en runtime que cada afirmación de `docs/operation/TOPICS_AND_FRAMES.md` y `docs/operation/NODES_REFERENCE.md` se reproduce con `ros2 topic list`, `ros2 node list`, `ros2 topic hz`.

---

## 9. Diff resumen (alto nivel)

```text
modified:
  README.md               (reescritura)
  PROJECT_STATE.md        (banner cabecera)
  PROJECT_PLAN.md         (banner cabecera)
  ARCHITECTURE.md         (banner cabecera)
  LAUNCH_GUIDE.md         (banner cabecera)

created (tracked):
  docs/architecture/CURRENT_ARCHITECTURE.md
  docs/architecture/TARGET_ARCHITECTURE.md
  docs/architecture/NAV2_CYLINDRICAL_NAVIGATION.md
  docs/operation/HOW_TO_LAUNCH.md
  docs/operation/LAUNCHERS_REFERENCE.md
  docs/operation/NODES_REFERENCE.md
  docs/operation/TOPICS_AND_FRAMES.md
  docs/development/DEVELOPMENT_GUIDE.md
  docs/legacy/README.md
  docs/audit/BUILD_DOCS_PLAN.md
  docs/audit/BUILD_SUMMARY.md

untracked (igual que antes — no en git):
  AGENTS.md
  docs/agent/*
  debug_runs/*

NOT touched:
  ros2_ws/src/**
  CONVERSATION_LESSONS.md
  tools/**
  assets/**
```

Comandos para revisar el diff:

```bash
git status
git diff -- README.md
git diff -- PROJECT_STATE.md PROJECT_PLAN.md ARCHITECTURE.md LAUNCH_GUIDE.md
git status -- docs/
ls docs/architecture/ docs/operation/ docs/development/ docs/legacy/ docs/audit/
```
