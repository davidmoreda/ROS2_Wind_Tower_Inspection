# CLEANUP_PLAN — Propuesta de limpieza

Fase PLAN. **No se borra ni mueve nada en este paso**. Solo se anota qué hacer en BUILD, con riesgo y necesidad de confirmación humana.

---

## 1. Tabla maestra

| Archivo / carpeta | Problema | Riesgo de eliminar/mover | Recomendación |
|---|---|---|---|
| `ros2_ws/src/wind_tower_bringup/launch/slam.launch.py` | No funcional contra el pipeline actual (`/scan` no existe). Comentario interno desactualizado. | **MEDIO** | Mover a `docs/legacy/launch/` o `examples/` con README breve. Conservar como referencia educativa. **Requiere confirmación humana.** |
| `ros2_ws/src/wind_tower_bringup/config/slam_toolbox.yaml` | Solo se usa desde `slam.launch.py` (legacy). | **MEDIO** | Mover junto con `slam.launch.py`. **Requiere confirmación.** |
| `ros2_ws/src/wind_tower_bringup/launch/rtabmap.launch.py` | Experimental aislado; consume `/velodyne_points`. Decisión declarada: no es MVP. | **MEDIO** | **CONSERVAR**, pero añadir comentario y nota en `LAUNCH_GUIDE.md` indicando que es experimental y no fuente de `θ_tube`. No mover todavía. |
| `wind_tower_bringup/wind_tower_bringup/scan_gate_node.py` | Entry point definido (`setup.py:40`), pero ningún launcher lo arranca. Documentado como "disponible". | **MEDIO** | Conservar el archivo. Documentar explícitamente como prototipo no integrado, o eliminar el entry_point si no se piensa usar. **Requiere confirmación.** |
| `wind_tower_bringup/wind_tower_bringup/map_accumulator_node.py` | Entry point definido (`setup.py:42`); no se lanza por ningún launcher; `LAUNCH_GUIDE.md` lo menciona como ejecución manual. | **MEDIO** | Conservar. Si se decide que `cylindrical_map_node.py` lo reemplaza definitivamente → mover a `examples/` o `sandbox/`. **Requiere confirmación.** |
| `ros2_ws/src/wind_tower_bringup/config/ekf.yaml` + `ekf_filter_node` en `simulation.launch.py:237-249` | Probable redundancia con EKF de Clearpath (`/robot/platform/odom/filtered`). PROJECT_STATE confirma que se usa la odom Clearpath, no esta. | **MEDIO** | Validar consumidores de `/odometry/filtered` propio. Si nadie lo consume → eliminar nodo + yaml. **Requiere confirmación + verificación con `ros2 topic info`.** |
| `.agents/` (vacío) | Carpeta vacía sin documentar. | **BAJO** | Eliminar o documentar propósito. **Confirmación rápida.** |
| `.codex/` (vacío) | Carpeta vacía sin documentar. | **BAJO** | Eliminar o documentar propósito. **Confirmación rápida.** |
| `debug_runs/` (30+ subcarpetas) | Logs históricos del depurador. No están en `.gitignore` revisado. | **BAJO** | Añadir `debug_runs/` a `.gitignore` y mover ejecuciones viejas a backup local. **Requiere confirmación**: hay valor histórico. |
| `assets/` (imágenes y mallas) | No referenciado desde docs principales. | **BAJO** | Conservar; añadir referencia desde README si las imágenes ilustran arquitectura. |
| `ros2_ws/src/gz_ros2_control/gz_ros2_control_demos/` | Demos upstream del fork; no usados en este proyecto. | **NO TOCAR AÚN** | Conservar tal cual: parte del fork. |
| `ros2_ws/src/wind_tower_description/rviz/`, `src/`, `launch/` | Carpetas vacías. | **BAJO** | Pueden eliminarse o quedarse como placeholders. |
| `ros2_ws/src/wind_tower_simulation/src/`, `launch/`, `include/` | Carpetas vacías. | **BAJO** | Igual: eliminar o documentar. |
| `ros2_ws/build`, `ros2_ws/install`, `ros2_ws/log` | Artefactos de colcon. | **NO TOCAR AÚN** | Verificar que `.gitignore` los excluye (parece ya excluido). |

---

## 2. Documentación a corregir (no mover)

| Documento | Acción para BUILD |
|---|---|
| `README.md` | Marcar `cylindrical_map_node.py` como implementado; separar tabla "implementado" vs "futuro"; añadir nota sobre evolución hacia Nav2-cilindro (cuando se decida) |
| `PROJECT_STATE.md` | Actualizar fecha; reclasificar `slam.launch.py` como "obsoleto contra pipeline actual"; aclarar estado real del EKF propio |
| `PROJECT_PLAN.md` | Añadir sub-sección "Implementado a 2026-05-12" para separar roadmap de realidad |
| `LAUNCH_GUIDE.md` | Marcar `slam.launch.py` y `map_accumulator` como obsoletos/no integrados; corregir cualquier referencia que asuma `/scan` |
| `ARCHITECTURE.md` | Pequeñas notas: aclarar que `ekf_filter_node` se lanza pero no es consumido; señalar que `scan_gate`/`map_accumulator` son prototipos no integrados |

---

## 3. Nodos a documentar mejor (no son legacy, pero faltan explicaciones)

| Nodo | Falta |
|---|---|
| `cylinder_localizer` | Documentar mejor parámetros y robustez; aparece como "Prototipo" en STATE pero se lanza por defecto en `inspection.launch.py` |
| `inspection_image_relay` (en simulation.launch.py) | El alias `/inspection/camera/image_raw` está marcado "EN DESARROLLO" en STATE/README; verificar que el relay funciona y actualizar estado |
| `turner_node` | Documentar fuente de verdad del ángulo: bridge gz vs integración local |

---

## 4. Launchers a revisar (no son legacy aún)

| Launcher | Acción |
|---|---|
| `rtabmap.launch.py` | Añadir nota en cabecera del archivo que confirme "EXPERIMENTAL — no usado en MVP" (ya hay comentario; reforzar) |
| `simulation.launch.py` | Decidir si el `ekf_filter_node` debe seguir lanzándose; si no, eliminar bloque L237-249 y `ekf.yaml` |

---

## 5. Riesgos generales de la limpieza

- **Romper inferencia visual del operador**: si se mueven `slam.launch.py` / `rtabmap.launch.py`, hay que actualizar `LAUNCH_GUIDE.md` en el mismo PR.
- **Romper builds de colcon**: eliminar entry points (`scan_gate`, `map_accumulator`) implica `setup.py` + recompilación. No romper instalación.
- **Pérdida de aprendizaje**: cualquier nodo prototipo que se borre se pierde como referencia. Preferir `docs/legacy/` o `examples/`.
- **Carpetas `.agents/`, `.codex/`**: pueden ser hooks de herramientas IA específicas. Verificar antes de borrar.

---

## 6. Resumen de qué NO tocar en BUILD

- Todo el código de `wind_tower_inspection_behaviour/` (state_machine, stability_monitor, cylindrical_map).
- `simulation.launch.py` excepto bloque EKF y ajustes menores tras confirmar redundancia.
- `inspection.launch.py` (es contrato operativo activo; cualquier cambio rompe perfiles documentados).
- `gz_ros2_control/` (fork).
- `wind_tower_simulation/worlds/wind_tower_world.sdf`.
- `wind_tower_description/urdf/inspection_camera_lighting.urdf.xacro`.
- Configs `inspection_params.yaml`, `stability_monitor.yaml`, `state_machine.yaml`.
- `AGENTS.md` y `docs/agent/*.md`.
- `CONVERSATION_LESSONS.md`.
