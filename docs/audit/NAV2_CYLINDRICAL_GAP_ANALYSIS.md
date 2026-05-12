# NAV2_CYLINDRICAL_GAP_ANALYSIS

Objetivo: evaluar si el repo soporta Nav2 sobre **coordenadas cilíndricas desplegadas** (X axial, Y = R·θ perímetro desarrollado), con restricciones IMU/yaw y control conjunto robot + giro de tubo. Hoy NO se usa Nav2; la decisión actual es no usarlo como núcleo del MVP (`README.md:109`, `PROJECT_STATE.md:160`).

---

## 1. Arquitectura objetivo (resumen del usuario)

- `X` = eje axial del tubo.
- `Y` = R · θ = perímetro desarrollado.
- `yaw` = orientación planar en el mapa desplegado.
- IMU mantiene restricciones de roll/pitch/gravedad y detecta desviación de la generatriz inferior.
- Nav2 (planner + controller) opera sobre ese plano desplegado, no sobre el mundo cartesiano del cilindro.
- Sistema debe poder comandar tanto la base del robot como el giro del tubo (turner).
- Robot mantiene generatriz inferior salvo bypass controlado de obstáculos.

---

## 2. Inventario: qué existe vs qué falta

| Componente necesario | ¿Existe? | Ruta / Evidencia | Falta por hacer |
|---|---|---|---|
| **Mapa axial-perimetral 2D desplegado** | **No** | `cylindrical_map_node.py` mantiene celdas `(x, θ)` solo de cobertura, no Costmap2D | Generar nav_msgs/OccupancyGrid en frame `cyl_map` |
| **Frame `cyl_map`** (o equivalente) | **No** | No hay TF custom para mapa desplegado en `tf_static_relay` ni mundo SDF | Añadir broadcaster de TF `cyl_map → base_link` derivado de odom axial + θ_tube |
| **Odometría cilíndrica** (x_axial, y_perim) | Parcial | `state_machine_node.py` calcula `lane_progress_m` y `θ_tube` desde `/turner/angle` | Componer en una odometría 2D continua proyectada al mapa desplegado |
| **Transformación axial / perimeter / yaw** | **No** | — | Nodo `cylindrical_odom_node` o equivalente que publique `Odometry` en frame `cyl_map` |
| **Uso de IMU para restricciones roll/pitch** | Parcial | `stability_monitor_node.py` publica `bottom_lane_locked`, `safe_to_scan`, `safe_to_index_tube` | Convertir a Nav2 `behavior_tree` o supervisor previo a Nav2 que bloquee plan si IMU sale de umbrales |
| **Costmap global** | **No** | No hay `nav2_costmap_2d` en `ros2_ws/src` | Configurar global_costmap sobre `cyl_map` con LiDAR proyectado al plano desplegado |
| **Costmap local** | **No** | — | local_costmap centrado en base_link con `/velodyne_points` |
| **Params Nav2** (planner_server, controller_server, behavior_server, bt_navigator) | **No** | No hay yaml de Nav2 | Crear `nav2_params.yaml` con planner (NavfnPlanner o SmacPlanner2D) y controller (RegulatedPurePursuit o MPPI según decisión MVP) |
| **Planner Nav2** | **No** | — | A elegir según naturaleza axial: probable `RegulatedPurePursuit` o `MPPIController` |
| **Controller Nav2** | **No** | — | Misma decisión |
| **Adapter `/cmd_vel` → robot/tubo** | Parcial | `state_machine_node.py` publica directamente `/robot/platform/cmd_vel` y `/turner/cmd_vel`; no hay nodo dedicado de mapeo | Crear `cylindrical_cmd_vel_adapter` que reciba `geometry_msgs/Twist` de Nav2 y reparta entre base (axial X) y turner (perímetro Y) coherentemente |
| **Supervisor de seguridad** | Parcial | `stability_monitor_node.py` + lógica safety en `state_machine_node.py` | Refactor para que el supervisor sea independiente del state_machine y publique flags consumidos por Nav2 BT (Behavior Tree) |
| **Launch principal de navegación** | **No** | — | `navigation.launch.py` futuro que arranque costmaps + planner + controller + bt + cylindrical_odom |
| **Behavior Tree de misión** | **No** | Lógica de misión hoy en `state_machine_node.py` (estados imperativos) | Migrar a BT XML si se adopta Nav2; o mantener state_machine como capa por encima de Nav2 |

---

## 3. Arquitectura implementada actualmente (evidencia)

Solo nodos que existen y se lanzan:

```
[VLP-16] ─► /velodyne_points ─► cylinder_localizer ─► /robot_in_tube
                                                  └─► /cylinder_fit/stats
[IMU]    ─► /robot/sensors/imu_0/data ─►┐
[Wheels] ─► /robot/platform/odom ─► (Clearpath EKF) ─► /robot/platform/odom/filtered ─►┐
[turner_joint Gazebo] ─► /turner/joint_state ─► turner_node ─► /turner/angle ──────────┤
                                                                                         │
                                                            ┌─► stability_monitor ◄──────┘
                                                            │     /inspection/bottom_lane_locked
                                                            │     /inspection/safe_to_scan
                                                            │     /inspection/safe_to_index_tube
                                                            │
                                                            ├─► cylindrical_map (cobertura pasiva)
                                                            │
                                                            └─► state_machine
                                                                  /robot/platform/cmd_vel
                                                                  /turner/cmd_vel
                                                                  (control PI propio)
[PS5] ─► dualsense_joy ─► ps5_teleop ─► /inspection/mission_command, /turner/cmd_vel manual
```

No hay Nav2. No hay costmaps. No hay TF `cyl_map`.

---

## 4. Arquitectura documentada actualmente

`ARCHITECTURE.md:144-267` y `README.md` documentan exactamente lo anterior. **Sin discrepancias** frente al código en cuanto a navegación: la decisión declarada es "no Nav2 todavía".

`PROJECT_PLAN.md:75-83` enumera componentes Nav2 como **opcionales futuros** (Collision Monitor, RPP, MPPI), pero **no como núcleo**. Coherente con código.

---

## 5. Arquitectura deseada / recomendada (Nav2 cilíndrico)

```
                                     ┌──────────────────────────────────┐
[/velodyne_points] ──► cyl_lidar_proj │  cylindrical_lidar_projector     │
                                     │  Proyecta nube al plano (x, R·θ) │
[turner/angle] ──► cyl_odom ────────►│                                  │
[odom/filtered] ──► cyl_odom ───────►│  cylindrical_odom_node           │
                                     │  Publica TF cyl_map → base_link  │
[IMU] ─► stability_monitor ─► flags ─┤  y nav_msgs/Odometry             │
                                     └────────────┬─────────────────────┘
                                                  │
                                                  ▼
                              ┌────────────────────────────────────────┐
                              │ Nav2 stack (sobre cyl_map):            │
                              │  - global_costmap (LiDAR proj.)        │
                              │  - local_costmap                       │
                              │  - planner_server (RPP/MPPI)           │
                              │  - controller_server                   │
                              │  - bt_navigator                        │
                              │  - behavior_server                     │
                              └─────────────┬──────────────────────────┘
                                            │  /cmd_vel (Twist)
                                            ▼
                              ┌────────────────────────────────────────┐
                              │ cylindrical_cmd_vel_adapter            │
                              │ Reparte: linear.x → base               │
                              │           linear.y → turner (R·ω)      │
                              │ Aplica restricciones IMU/yaw/safety    │
                              └─────────────┬──────────────────────────┘
                                            │
                          ┌─────────────────┴─────────────────┐
                          ▼                                   ▼
              /robot/platform/cmd_vel                 /turner/cmd_vel
                  (base Husky)                          (giro tubo)
```

Componentes `PROPUESTO / NO IMPLEMENTADO`:

- `cylindrical_odom_node`
- `cylindrical_lidar_projector`
- `cylindrical_cmd_vel_adapter`
- todo el stack Nav2 + costmaps + params
- BT principal de misión

Componentes reutilizables:

- `stability_monitor_node.py` (flags IMU)
- `cylinder_localizer_node.py` (estimación radio/eje del tubo)
- `turner_node.py` (fuente autoritativa de `θ_tube`)
- `cylindrical_map_node.py` (cobertura, se puede mantener en paralelo)

---

## 6. Gaps críticos antes de poder probar Nav2

1. Definir contrato de frames TF (mínimo: `world` → `cyl_map` → `odom` → `base_link`).
2. Decidir si `cyl_map` es plano infinito o cíclico en Y (perímetro envuelve a 2πR).
3. Implementar proyección LiDAR cilindro → plano (consistente con frame `cyl_map`).
4. Mapeo `/cmd_vel` → comandos físicos (base + turner). Política para cuándo mover el robot, cuándo el tubo o ambos.
5. Reglas de seguridad: Nav2 NO debe planificar caminos que requieran roll/pitch fuera de umbrales; el supervisor IMU debe poder cancelar el plan.
6. Elección de planner/controller Nav2 acorde a la naturaleza axial dominante (probable `RegulatedPurePursuit`).
7. Modo IDLE seguro: si Nav2 está activo y `bottom_lane_locked=false`, el adapter debe cero-clampear comandos.

---

## 7. Propuesta de fases (alto nivel)

| Fase | Objetivo |
|---|---|
| F1 | TF `cyl_map` y `cylindrical_odom_node` publicando `Odometry` válida |
| F2 | Proyección LiDAR a plano `cyl_map`; RViz verificable |
| F3 | global_costmap y local_costmap sobre `cyl_map` |
| F4 | Nav2 planner + controller con goal manual (geometría axial pura) |
| F5 | Adapter `/cmd_vel` ↔ base + turner; bias hacia generatriz inferior |
| F6 | Supervisor IMU/safety integrado con Nav2 BT |
| F7 | Integración con misión por calles axiales (state_machine pasa a llamar Nav2) |

Esto NO es BUILD inmediato: es entrada para futuras decisiones. La fase BUILD que sigue a este audit es **únicamente documental + limpieza**.
