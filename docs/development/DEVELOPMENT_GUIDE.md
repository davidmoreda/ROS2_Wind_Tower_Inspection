# DEVELOPMENT_GUIDE — Guía de desarrollo

Cómo trabajar en el repo sin romper el flujo MVP.

---

## 1. Clonar y preparar

```bash
git clone <url-del-repo> ~/ROS2_wind_tower_inspection
cd ~/ROS2_wind_tower_inspection
```

Prerrequisitos: ver [../operation/HOW_TO_LAUNCH.md](../operation/HOW_TO_LAUNCH.md) §0.
<!--  -->
El workspace ROS 2 vive en `ros2_ws/`. Los launchers, nodos y configs están en `ros2_ws/src/`.

---

## 2. Ramas

Convención mínima:

| Tipo de trabajo | Nombre de rama |
|---|---|
| Documentación | `docs/<descripcion-corta>` |
| Bug fix | `fix/<descripcion>` |
| Feature MVP | `feat/<descripcion>` |
| Experimento aislado | `exp/<descripcion>` |
| Refactor | `refactor/<descripcion>` |

Pasos:

```bash
git checkout -b feat/cylindrical-odom-node
# trabajar...
git add -p
git commit -m "feat(behaviour): primera versión de cylindrical_odom_node"
git push -u origin feat/cylindrical-odom-node
```

Antes de mergear a `main`:

1. Verificar que `colcon build --packages-select wind_tower_bringup wind_tower_inspection_behaviour` compila limpio.
2. Verificar que `simulation.launch.py` + `inspection.launch.py` arrancan sin errores.
3. Documentar cualquier nodo/launcher nuevo en `docs/operation/`.

---

## 3. Compilar

```bash
cd ~/ROS2_wind_tower_inspection/ros2_ws
ai-on
colcon build --packages-select wind_tower_bringup wind_tower_inspection_behaviour
source install/setup.bash
```

Tip: en desarrollo iterativo de Python, `colcon build --symlink-install` permite editar el código sin recompilar tras cada cambio:

```bash
colcon build --symlink-install --packages-select wind_tower_inspection_behaviour
```

---

## 4. Sourcear

Cada terminal nueva:

```bash
cd ~/ROS2_wind_tower_inspection/ros2_ws && ai-on
# (ai-on ya sourcea install/setup.bash)
```

Si por algo `ai-on` no está cargado:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ROS2_wind_tower_inspection/ros2_ws/install/setup.bash
```

---

## 5. Lanzar lo central

Ver [../operation/HOW_TO_LAUNCH.md](../operation/HOW_TO_LAUNCH.md).

Resumen mínimo:

```bash
# Terminal 1
ros2 launch wind_tower_bringup simulation.launch.py
# Terminal 2
ros2 launch wind_tower_inspection_behaviour inspection.launch.py
```

---

## 6. Añadir un nodo nuevo (paquete Python)

1. Crear archivo en `ros2_ws/src/<paquete>/<paquete>/<nuevo_nodo>.py`.
2. Definir `def main(args=None): ...` y `if __name__ == '__main__': main()`.
3. Registrar el entry point en `setup.py`:

   ```python
   entry_points={
       'console_scripts': [
           ...,
           'nuevo_nodo = <paquete>.<nuevo_nodo>:main',
       ],
   },
   ```

4. Recompilar con `colcon build --packages-select <paquete> --symlink-install`.
5. Probarlo standalone: `ros2 run <paquete> nuevo_nodo`.
6. Integrarlo en un launcher existente (preferible) o crear uno nuevo (solo si el alcance lo justifica).
7. Documentarlo en [../operation/NODES_REFERENCE.md](../operation/NODES_REFERENCE.md) con su estado (`ACTIVO`, `PROTOTIPO INTEGRADO`, etc.).

---

## 7. Añadir un launcher nuevo

1. Crearlo en `launch/<nombre>.launch.py`.
2. Añadir cabecera explicativa: qué arranca, qué configs carga, qué estado tiene (CENTRAL / AUXILIAR / EXPERIMENTAL).
3. Registrarlo en `setup.py` dentro de `data_files`:

   ```python
   ('share/' + package_name + '/launch', [
       'launch/<nombre>.launch.py',
       ...
   ]),
   ```

4. Recompilar.
5. Documentarlo en [../operation/LAUNCHERS_REFERENCE.md](../operation/LAUNCHERS_REFERENCE.md).
6. Si es CENTRAL, actualizar [../operation/HOW_TO_LAUNCH.md](../operation/HOW_TO_LAUNCH.md).

---

## 8. Convención para marcar experimental / legacy

| Caso | Cómo marcarlo |
|---|---|
| Launcher experimental | Comentario en la primera docstring del archivo `.launch.py` con la línea `# Estado: EXPERIMENTAL — no parte del MVP` |
| Launcher legacy | Misma docstring con `# Estado: LEGACY PROBABLE — ver docs/audit/BUILD_INPUT.md` |
| Nodo no integrado | Mantener entry point en `setup.py` pero documentarlo como `PROTOTIPO NO INTEGRADO` en `NODES_REFERENCE.md` |
| Documento obsoleto | Añadir banner de estado al inicio del Markdown indicando reemplazo |

NO eliminar código ni launchers sin confirmación humana explícita. Si se elimina, anotar el motivo en el mensaje de commit para que `git log --follow <ruta>` lo deje trazable.

---

## 9. Buenas prácticas para trabajo colaborativo

- **Antes de tocar `state_machine_node.py`, `stability_monitor_node.py` o `cylindrical_map_node.py`**: revisar el historial git de esos archivos (`git log -p --follow <archivo>`) para entender decisiones previas.
- **No tunear PI a ciegas**. Diagnosticar primero: target, medida, error, saturación, integrador, dt, safety gates.
- **No cambiar varios controladores a la vez** en un mismo commit.
- **Un solo publisher por topic crítico** (`/turner/cmd_vel`, `/robot/platform/cmd_vel`). Verificar con `ros2 topic info -v <topic>`.
- **Documentar `θ_tube` versus `α_robot`** correctamente en cualquier nuevo cálculo angular. No confundirlos.
- **Cobertura nominal** solo durante `AXIAL_SCAN` con `safe_to_scan=true` y sin bypass activo.
- **Mensajes custom**: hoy todo va como `std_msgs/String` JSON. Si se introduce un paquete `wind_tower_msgs`, hacerlo en un PR separado.
- **Tests**: hay scaffolding `test_copyright`, `test_flake8`, `test_pep257` en `wind_tower_bringup/test/`. No hay tests funcionales de misión. Considerar añadirlos al implementar nodos nuevos.
- **Debug**: usar `tools/debug/capture_inspection_debug.py` para grabar sesiones; producir `debug_runs/<timestamp>/agent_payload.md` para discutir con agentes IA.

---

## 10. Cómo extender hacia Nav2 cilíndrico (futuro)

Ver fases F1-F7 en [../architecture/NAV2_CYLINDRICAL_NAVIGATION.md](../architecture/NAV2_CYLINDRICAL_NAVIGATION.md). El orden propuesto es:

1. F1 — `cylindrical_odom_node` + TF `cyl_map`.
2. F2 — `cylindrical_lidar_projector`.
3. F3 — Costmaps.
4. F4 — Nav2 con goal manual.
5. F5 — Adapter `/cmd_vel` → base + turner.
6. F6 — Supervisor IMU/safety + BT conditions.
7. F7 — Migrar state_machine a BT XML.

Cada fase debería ser una rama `feat/nav2-cyl-fX-<descripcion>` y entregar un punto verificable en RViz/CLI antes de pasar a la siguiente.
