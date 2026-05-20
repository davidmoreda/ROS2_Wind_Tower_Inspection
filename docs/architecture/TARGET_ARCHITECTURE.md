# TARGET_ARCHITECTURE — Arquitectura objetivo (PROPUESTA)

> **Este documento describe una arquitectura objetivo. NADA aquí está implementado todavía salvo lo que se marca explícitamente como IMPLEMENTADO.** La verdad de "qué existe hoy" vive en [CURRENT_ARCHITECTURE.md](CURRENT_ARCHITECTURE.md).

Objetivo: migrar la navegación del control PI propio dentro de `state_machine_node` a **Nav2 operando sobre un espacio cilíndrico desplegado**, manteniendo las restricciones de IMU/yaw, generatriz inferior y control conjunto de robot + giro del tubo.

---

## 1. Idea central

El interior de un tubo cilíndrico se puede "desenrollar" matemáticamente en un plano 2D:

- `X = posición axial del robot dentro del tubo` (metros)
- `Y = R · θ_perim`, perímetro desarrollado (metros)
- `yaw = orientación planar del robot en ese plano desplegado`

En ese plano, Nav2 puede tratarse como una navegación 2D estándar. La curvatura desaparece de la representación; la realidad física (que el robot se mueve sobre una pared curva) se respeta vía:

- restricciones de roll/pitch obtenidas de la IMU
- una preferencia continua a permanecer cerca de la generatriz inferior (Y ≈ 0)
- un adaptador que descompone `cmd_vel` desplegado en (a) avance axial del robot y (b) rotación del tubo (turner)

---

## 2. Componentes y estado

| Componente | Estado | Notas |
|---|---|---|
| Encoder virador (`θ_tube`) | **IMPLEMENTADO** | `turner_node` publica `/turner/angle` |
| IMU calibrada para `bottom_lane_locked` | **IMPLEMENTADO** | `stability_monitor_node` |
| `cylinder_localizer` (ajuste de cilindro al LiDAR) | **IMPLEMENTADO PARCIALMENTE** | Prototipo en `wind_tower_bringup` |
| Frame TF `cyl_map → odom → base_link` | **PROPUESTO / NO IMPLEMENTADO** | Falta diseño del contrato de frames |
| `cylindrical_odom_node` (publica `nav_msgs/Odometry` en `cyl_map`) | **PROPUESTO / NO IMPLEMENTADO** | Compone `x_axial` (odom Clearpath proyectada) + `y = R·θ` (turner) |
| `cylindrical_lidar_projector` (proyecta `/velodyne_points` al plano desplegado) | **PROPUESTO / NO IMPLEMENTADO** | Necesario para alimentar Costmap2D |
| `nav2_costmap_2d` global + local | **PROPUESTO / NO IMPLEMENTADO** | Sobre frame `cyl_map` |
| `nav2_planner_server` (probable `RegulatedPurePursuit` o `MPPI`) | **PROPUESTO / NO IMPLEMENTADO** | Selección pendiente |
| `nav2_controller_server` | **PROPUESTO / NO IMPLEMENTADO** | Ídem |
| `nav2_bt_navigator` + BT XML de misión | **PROPUESTO / NO IMPLEMENTADO** | Migración futura desde `state_machine_node` |
| `cylindrical_cmd_vel_adapter` | **PROPUESTO / NO IMPLEMENTADO** | Reparte `cmd_vel` desplegado entre base + turner |
| Supervisor de seguridad Nav2-aware | **PROPUESTO / NO IMPLEMENTADO** | Reutilizable: lógica de `stability_monitor` + cancel del BT |
| Paquete `wind_tower_msgs` | **PROPUESTO / NO IMPLEMENTADO** | Para mensajes custom (`CylindricalPose`, `BottomLaneState`, etc.) |

---

## 3. Diagrama objetivo

```text
[Velodyne VLP-16] ──► /velodyne_points ──►┐
                                          │
[turner_node]    ──► /turner/angle ──────►├──► cylindrical_lidar_projector  (PROPUESTO)
                                          │       │
[Clearpath EKF]  ──► /platform/odom/filtered ─►│       ▼
                                          │       /cyl_map projected cloud
                                          ▼
                                  cylindrical_odom_node          (PROPUESTO)
                                  ├─ TF: cyl_map → odom → base_link
                                  └─ /cylindrical/odom (Odometry)
                                          │
                                          ▼
                          ┌──── Nav2 stack sobre cyl_map ────┐
                          │  global_costmap (LiDAR proj.)    │
                          │  local_costmap                   │
                          │  planner_server (RPP/MPPI)       │
                          │  controller_server               │
                          │  bt_navigator + behavior_server  │
                          └──────────────┬───────────────────┘
                                         │  /cmd_vel (Twist)
                                         ▼
                                cylindrical_cmd_vel_adapter    (PROPUESTO)
                                ├─ linear.x → base axial
                                ├─ linear.y → turner (R·ω)
                                ├─ clamp por IMU/safety flags
                                │
            ┌────────────────────┴────────────────────┐
            ▼                                         ▼
  /robot/platform/cmd_vel                       /turner/cmd_vel
       (base Husky)                               (giro tubo)

                                        ▲
                                        │
        [stability_monitor]  ─── flags ─┘   (IMPLEMENTADO; reutilizable)
        [cylinder_localizer] ─── radio R, eje  (IMPLEMENTADO PARCIALMENTE)
        [cylindrical_map]    ─── cobertura (IMPLEMENTADO; corre en paralelo a Nav2)
```

---

## 4. Diferencia con la arquitectura actual

| Aspecto | HOY (CURRENT) | OBJETIVO (TARGET) |
|---|---|---|
| Planificación | Control PI imperativo en `state_machine` | Nav2 planner + controller sobre `cyl_map` |
| Espacio de planificación | Eje X axial unidimensional + estados discretos | Plano 2D desplegado (X axial, Y perímetro) |
| Costmap | Inexistente | Costmap 2D estándar Nav2 alimentado por LiDAR proyectado |
| Goals | Implícitos por estados | `geometry_msgs/PoseStamped` en `cyl_map` |
| Misión | Estados imperativos en Python | Behavior Tree XML |
| Adapter `cmd_vel` | El state_machine publica directo a base y turner | Nodo dedicado `cylindrical_cmd_vel_adapter` |
| Safety | Embebido en state_machine + stability_monitor | Supervisor independiente que puede cancelar el BT |
| Cobertura | `cylindrical_map_node` pasivo | Igual (se mantiene en paralelo a Nav2) |

---

## 5. Restricciones físicas que deben preservarse

| Restricción | Cómo se mantiene |
|---|---|
| Robot debe estar cerca de la generatriz inferior salvo bypass | Costmap penaliza Y lejos de 0; controller con preferencia hacia Y=0 |
| Roll/pitch dentro de umbrales | Supervisor IMU cancela plan o reduce velocidad; condición del BT |
| `safe_to_scan` durante AXIAL_SCAN | Condición del BT antes de habilitar planner |
| `safe_to_index_tube` durante INDEX_TUBE | Condición del BT antes de comandar turner |
| `θ_tube` viene del encoder, no de SLAM | `cylindrical_odom_node` consume `/turner/angle` |
| Cobertura nominal solo en AXIAL_SCAN seguro | `cylindrical_map_node` se mantiene sin cambios |

---

## 6. Modos de operación propuestos

| Modo | Nav2 activo | Adapter | Notas |
|---|---|---|---|
| Manual teleop | No | No | Igual que hoy; `dualsense_joy` + `ps5_teleop` |
| Inspección autónoma (HOY) | No | No | State_machine PI directo; método actual |
| Inspección autónoma con Nav2 (OBJETIVO) | Sí | Sí | Goals enviados por BT; planner sobre cyl_map |
| Safety hard-stop | N/A | Clamp a cero | Supervisor IMU/safety; prioridad máxima |

---

## 7. Roadmap mínimo

Detalle técnico en [NAV2_CYLINDRICAL_NAVIGATION.md](NAV2_CYLINDRICAL_NAVIGATION.md). Resumen:

| Fase | Entregable | Estado |
|---|---|---|
| F1 | TF `cyl_map → odom → base_link` + `cylindrical_odom_node` | PROPUESTO |
| F2 | `cylindrical_lidar_projector` con verificación en RViz | PROPUESTO |
| F3 | Global + local costmap sobre `cyl_map` | PROPUESTO |
| F4 | Nav2 planner + controller con goal manual | PROPUESTO |
| F5 | `cylindrical_cmd_vel_adapter` integrado | PROPUESTO |
| F6 | Supervisor IMU/safety + BT condition nodes | PROPUESTO |
| F7 | Migración de `state_machine` a BT que delega navegación en Nav2 | PROPUESTO |

---

## 8. Qué NO se decide aquí

- Elección final entre RegulatedPurePursuit y MPPI.
- Política exacta de reparto base ↔ turner en el adapter.
- Si `cyl_map` es plano infinito o cíclico en Y (envuelve a 2πR). Recomendación inicial: cíclico, pero requiere planner que tolere la discontinuidad o "desdoblar" virtualmente la zona alrededor del robot.
- Si el state_machine actual se mantiene como capa superior o se reemplaza completamente por BT XML.
