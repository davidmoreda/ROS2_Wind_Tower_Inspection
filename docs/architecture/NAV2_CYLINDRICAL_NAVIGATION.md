# NAV2_CYLINDRICAL_NAVIGATION — Detalle técnico (PROPUESTA)

> **Documento de diseño futuro. Ningún nodo Nav2 está integrado en el repo todavía.** La verdad de "qué existe hoy" vive en [CURRENT_ARCHITECTURE.md](CURRENT_ARCHITECTURE.md).

---

## 1. Por qué Nav2 puede usarse sobre un cilindro desplegado

Nav2 es un stack de navegación 2D estándar (`nav_msgs/OccupancyGrid` + costmaps + planner + controller + BT). Solo necesita:

- una transformación TF estable de `map → odom → base_link`,
- un costmap 2D consistente con esa TF,
- un `/cmd_vel` interpretable por el robot.

Si proyectamos el interior del tubo a un plano `(x_axial, y_perim)`, todo se vuelve "2D plano" desde la perspectiva de Nav2. Las particularidades físicas (gravedad, generatriz inferior, control del tubo) se manejan **fuera** de Nav2 a través de plugins/nodos custom y un supervisor de seguridad.

---

## 2. Transformación matemática

Sea:

- `R` = radio del tubo (m). En este proyecto ≈ 3.925 m (`state_machine.yaml` default; ver `inspection.launch.py:341-343`).
- `θ_robot_world` = ángulo angular del robot respecto al eje del tubo, en el frame mundo.
- `θ_tube` = ángulo material acumulado del tubo (encoder del virador, `/turner/angle`).
- `x_world` = posición del robot a lo largo del eje del tubo (m).

Definición de coordenadas desplegadas:

```
x = x_world                              (axial)
y = R · (θ_robot_world − θ_tube)        (perímetro relativo a la marca material del tubo)
yaw = ángulo del eje "forward" del robot en el plano (x, y) desplegado
```

Notas:

- Restando `θ_tube` se obtiene una `y` ligada a la superficie material del tubo, no al frame mundo. Esto significa que mover el robot perimetralmente o girar el tubo producen el mismo `dy`, lo que es exactamente lo que queremos para Nav2.
- El espacio `y` es cíclico con período `2πR`. Hay dos opciones:
  1. **Plano infinito**: dejar `y` crecer indefinidamente; planner ignora el wrap. Simple pero rompe cierres de bucle perimetrales.
  2. **Cíclico**: wrap a `[0, 2πR)`. Requiere planner que tolere o un costmap "extendido" alrededor de la posición actual del robot.

Recomendación inicial: **plano infinito por simplicidad en F1-F4**, migrar a cíclico cuando se necesite cerrar la inspección de 360°.

---

## 3. Qué representa `yaw` en este plano

`yaw` es el ángulo entre el eje "forward" del robot y el eje X axial del tubo, medido en el plano desplegado.

- `yaw = 0` → robot apunta a lo largo del eje del tubo (AXIAL_SCAN nominal)
- `yaw = ±π/2` → robot apunta tangencialmente (configuración para INDEX_TUBE)
- Restricción durante AXIAL_SCAN: `|yaw| < umbral_pequeño`

La IMU NO mide directamente este `yaw` en el plano desplegado; mide orientación respecto a la gravedad. Hay que componer:

- IMU → roll/pitch (gravedad)
- Odom (yaw filtrado por Clearpath/EKF) + correlación con `θ_tube` para obtener el `yaw` desplegado.

`cylindrical_odom_node` se encargará de esta composición.

---

## 4. Proyección de obstáculos al costmap

`/velodyne_points` está en el frame `base_link` (o el del sensor). Para alimentar Costmap2D en `cyl_map` hace falta:

1. Transformar cada punto a coordenadas cilíndricas relativas al eje del tubo `(x_axial, θ_punto, ρ_punto)`.
2. Filtrar puntos con `ρ_punto ≈ R` (puntos sobre la pared) — descartar resto.
3. Para los obstáculos (puntos con `ρ_punto < R - margen`), mapear a `(x_axial, R·θ_punto)` en `cyl_map`.
4. Publicar como `sensor_msgs/PointCloud2` o `LaserScan` 2D consumible por Costmap2D.

Este es el trabajo del nodo `cylindrical_lidar_projector` (PROPUESTO).

Nota: el robot mismo está sobre la pared interior; los puntos correspondientes a "estructura" del tubo (uniones, bushings, soldaduras) deben aparecer como obstáculos. Los puntos de la pared lisa nominal NO deben marcarse como obstáculos.

---

## 5. Restricciones de calle / generatriz inferior

Cómo modelar la preferencia por Y ≈ 0 (generatriz inferior):

Opciones (no excluyentes):

1. **Costmap layer custom** que añade coste creciente con `|y|`. Implementa la "atracción" a la generatriz inferior.
2. **Critic custom** en MPPI que penaliza desviación lateral.
3. **Constraint en el BT**: si `|y| > umbral_bypass`, abortar plan y delegar a estado de bypass.
4. **Adapter clamp**: el `cylindrical_cmd_vel_adapter` puede limitar `linear.y` cuando IMU detecta inestabilidad.

Recomendación inicial: combinación de (1) costmap layer + (4) adapter clamp + (3) BT condition. Evita modificar planners estándar.

---

## 6. Uso de IMU

| Función IMU | Cómo se integra |
|---|---|
| Orientación respecto a gravedad (roll/pitch) | `stability_monitor` ya publica `bottom_lane_locked`, `safe_to_scan`, `safe_to_index_tube` |
| Detección de desviación de la pared | `stability_monitor` ya calcula `lateral_angle_deg` |
| Filtro odométrico | EKF Clearpath ya fusiona IMU + ruedas en `/robot/platform/odom/filtered` |
| BT conditions | Nodos `IsBottomLaneLocked`, `IsSafeToScan`, `IsSafeToIndex` (PROPUESTOS) consumen los flags |
| Cancelación de Nav2 | Si `stability_monitor` baja `safe_to_scan`, el supervisor envía `cancel_goal` a Nav2 |

---

## 7. Qué es Nav2 estándar y qué requiere plugins/nodos custom

| Pieza | Estándar Nav2 | Custom |
|---|---|---|
| `nav2_costmap_2d` core | Sí | Capa custom para preferencia generatriz inferior |
| `nav2_planner_server` con `NavfnPlanner`, `SmacPlanner2D` o `ThetaStarPlanner` | Sí | — |
| `nav2_controller_server` con `RegulatedPurePursuit` o `MPPIController` | Sí | Posible critic custom para Y≈0 |
| `nav2_bt_navigator` | Sí | BT XML específico de la misión por calles axiales |
| `nav2_behavior_server` | Sí | Behaviors custom para `INDEX_TUBE` (acción de comandar el turner) |
| Costmap input (sensores) | Sí, `PointCloud2` o `LaserScan` | `cylindrical_lidar_projector` produce esa entrada |
| TF `map → odom → base_link` | Necesaria | `cylindrical_odom_node` la publica |
| `/cmd_vel` consumer | El robot consume Twist | `cylindrical_cmd_vel_adapter` lo descompone en base + turner |

---

## 8. Lo que falta implementar en este repo

Recapitulación priorizada:

1. **`cylindrical_odom_node`** — publica `nav_msgs/Odometry` y TF `cyl_map → odom`. Sin esto Nav2 no arranca.
2. **`cylindrical_lidar_projector`** — alimenta costmap.
3. **`nav2_params.yaml`** específico del proyecto (costmaps, planner, controller, BT navigator).
4. **BT XML** de misión.
5. **`cylindrical_cmd_vel_adapter`** — base + turner.
6. **Costmap layer y/o critic custom** para generatriz inferior.
7. **BT condition nodes** que lean flags de `stability_monitor`.
8. **Behavior server custom** para acciones `INDEX_TUBE` y similares.
9. **`navigation.launch.py`** que arranque todo lo anterior.
10. **Paquete `wind_tower_msgs`** si se decide formalizar mensajes custom.

---

## 9. Riesgos técnicos

| Riesgo | Mitigación |
|---|---|
| Discontinuidad en Y al cerrar 360° | Empezar con plano infinito; abordar cíclico solo cuando F1-F4 funcionen |
| `θ_tube` deriva si encoder simulado no es exacto | Cross-check con `cylinder_localizer` y/o reset periódico |
| Doble EKF (Clearpath + propio) → conflicto en TF | Decidir uno solo antes de F1; ver §6 de CURRENT_ARCHITECTURE |
| Planner intenta atravesar la pared | Costmap inflation suficiente; layer custom anti-generatriz superior |
| Controller saturar el turner | Adapter clamp a velocidad angular máxima del virador |
| BT bloqueado por `safe_to_*` oscilante | Histéresis temporal en `stability_monitor` (pendiente — evitar decidir `safe_to_index_tube` con una sola muestra) |

---

## 10. Roadmap mínimo (recordatorio)

| Fase | Entregable | Verifica |
|---|---|---|
| F1 | TF `cyl_map → odom → base_link` + `cylindrical_odom_node` | `tf2_echo cyl_map base_link` produce valores plausibles |
| F2 | `cylindrical_lidar_projector` | RViz muestra puntos proyectados en plano consistente |
| F3 | Costmaps sobre `cyl_map` | RViz muestra obstáculos en costmap |
| F4 | Nav2 con goal manual `PoseStamped` en `cyl_map` | Robot navega a una calle objetivo desde rqt o RViz |
| F5 | `cylindrical_cmd_vel_adapter` | Nav2 produce trayectorias que mezclan base + turner |
| F6 | Supervisor IMU/safety + BT | `safe_to_scan=false` cancela plan |
| F7 | Migración de `state_machine` a BT XML | Misión completa por calles axiales funciona con Nav2 |
