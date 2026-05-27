# Memoria Practica 2 RI

Estructura LaTeX para la memoria de la Practica 2 (Navegacion Autonoma con NAV2 en ROS 2 Humble).

## Indice

- [Alcance de la rama](#alcance-de-la-rama)
- [Estado del trabajo](#estado-del-trabajo)
- [Estructura](#estructura)
- [Compilar](#compilar)
- [Plan de trabajo](#plan-de-trabajo)
  - [Fase 1 - Estructura LaTeX (hecha)](#fase-1---estructura-latex-hecha)
  - [Fase 2 - Capturas y verificacion](#fase-2---capturas-y-verificacion)
  - [Fase 3 - Mapeo SLAM y guardado del mapa](#fase-3---mapeo-slam-y-guardado-del-mapa)
  - [Fase 4 - Pruebas de navegacion AMCL + metricas](#fase-4---pruebas-de-navegacion-amcl--metricas)
  - [Fase 5 - Redaccion](#fase-5---redaccion)
  - [Fase 6 - Compilar y revisar](#fase-6---compilar-y-revisar)

## Alcance de la rama

La rama `practica-2` contiene el repositorio completo del TFM. La memoria y el video solo cubren el subconjunto NAV2 (mundo + SLAM + EKF + AMCL + costmaps + planner/controller + TF + validacion). El codigo de las otras practicas (`wind_tower_arm_control`, `wind_tower_perception`, `wind_tower_inspection_behaviour`) coexiste en la rama pero no se evalua aqui.

## Estado del trabajo

- [x] Codigo NAV2 ya en `main` (configs, launchers, mapa, nodos, tuning del controller).
- [x] Fase 1: estructura LaTeX creada.
- [ ] Fase 2: capturas y verificacion de topicos.
- [ ] Fase 3: mapeo SLAM con la rama actual (o usar el pregenerado).
- [ ] Fase 4: pruebas de navegacion + recogida de metricas.
- [ ] Fase 5: redactar las 11 secciones de `memoria.tex`.
- [ ] Fase 6: compilar PDF y revisar.
- [ ] Video demo: guion en [`video_plan.md`](video_plan.md).

**Pre-requisito tecnico:** instalar LaTeX:

```bash
sudo apt install texlive-latex-recommended texlive-latex-extra \
                 texlive-fonts-recommended texlive-lang-spanish \
                 latexmk
```

## Estructura

```
docs/practica2/
|-- memoria.tex          # documento principal con 11 secciones esqueleto
|-- preamble.tex         # paquetes y comandos compartidos
|-- Makefile             # latexmk wrapper
|-- README.md            # este archivo
|-- video_plan.md        # guion del video demo
|-- figures/             # capturas (Fase 2-4)
|-- plots/               # graficas generadas (EKF, metricas, tuning)
|-- data/                # rosbags + CSV de pruebas
`-- scripts/             # utilidades (plot, replay rosbag...)
```

## Compilar

```bash
cd docs/practica2
make            # genera memoria.pdf
make watch      # recompila al guardar
make view       # abre el PDF
make clean      # limpia artefactos
```

---

## Plan de trabajo

Decisiones tomadas:

- Formato: **LaTeX**.
- Robot: **Husky A200 (Clearpath)**, igual que en Practica 1.
- Mundo: `wind_tower_simulation/worlds/wind_tower_world.sdf`.
- Mapa: ya hay uno en `ros2_ws/maps/wind_tower.*` (se puede regenerar).
- Equipo: 3 personas (P1, P2, P3) -- reparto en `video_plan.md`.
- Punto fuerte de la entrega: **tuning iterativo del controller** (Sec. del mismo nombre en `memoria.tex`).

### Reparto del equipo

Reparto sugerido (ajustar entre vosotros):

| Persona | Responsabilidad memoria              | Escenas video                   | Eje tematico              |
| ------- | ------------------------------------ | ------------------------------- | ------------------------- |
| **P1**  | Intro + Arquitectura + Launchers + Conclusiones | Intro + Arquitectura + Demo SLAM | Estructura del sistema    |
| **P2**  | SLAM + EKF + AMCL                    | EKF + AMCL                      | Estimacion y localizacion |
| **P3**  | Costmaps + Tuning controller + TF + Pruebas | Navegacion + Tuning + Resultados | Navegacion y resultados   |

Para las **Fases 3 y 4**:

- **P1**: capturar `architecture_diagram.png`, montar SLAM y conducir el robot.
- **P2**: capturar `slam_in_progress.png`, `map_final.png`, `ekf_filtered_odom.png`, `amcl_initial.png`, `amcl_converged.png`. Tomar metricas de convergencia AMCL.
- **P3**: capturar `costmaps_rviz.png`, `tf_frames.png`, `nav_full_run.png`. Recoger metricas RMSE / tiempo a goal con `mission_navigator`.

### Fase 1 - Estructura LaTeX (hecha)

Archivos creados:

- `memoria.tex` con 11 secciones (intro, arquitectura, launchers, SLAM, EKF, AMCL, costmaps, **tuning controller**, TF, pruebas, problemas, conclusiones).
- `preamble.tex` con paquetes en espanol, `amsmath` y `listings`.
- `Makefile`, `.gitignore`.

### Fase 2 - Capturas y verificacion

#### 0. Preparacion

```bash
cd ~/ROS2_Wind_Tower_Inspection/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

#### 1. Lanzar simulacion + validar topicos

```bash
ros2 launch wind_tower_bringup simulation.launch.py
```

En terminales paralelas:

```bash
ros2 topic list
ros2 topic echo --once /scan
ros2 topic echo --once /imu/data
ros2 topic echo --once /odom
ros2 run tf2_tools view_frames    # genera frames.pdf -> figures/tf_frames.png
```

Capturas (guardar en `docs/practica2/figures/`):

| Archivo | Contenido |
| --- | --- |
| `architecture_diagram.png` | Diagrama de la arquitectura (slide o draw.io) |
| `tf_frames.png`            | Arbol TF generado por `view_frames` |

### Fase 3 - Mapeo SLAM y guardado del mapa

Hay un mapa pregenerado en `ros2_ws/maps/wind_tower.*`. Si esta valido, saltar a Fase 4. Si no:

```bash
# Terminal 1 - Gazebo
ros2 launch wind_tower_bringup simulation.launch.py

# Terminal 2 - SLAM Toolbox + RViz
ros2 launch wind_tower_bringup slam.launch.py

# Terminal 3 - teleop
ros2 run wind_tower_bringup ps5_teleop
# o
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel

# Terminal 4 - guardar mapa cuando el SLAM se vea estable
ros2 run nav2_map_server map_saver_cli -f ros2_ws/maps/wind_tower
```

Capturas:

| Archivo | Momento |
| --- | --- |
| `slam_in_progress.png` | RViz durante el mapeo |
| `map_final.png`        | Mapa final (`.pgm` abierto con GIMP o RViz) |

### Fase 4 - Pruebas de navegacion AMCL + metricas

```bash
# Terminal 1 - Gazebo
ros2 launch wind_tower_bringup simulation.launch.py

# Terminal 2 - NAV2 + AMCL con mapa
ros2 launch wind_tower_bringup navigation_amcl.launch.py
```

En RViz:

1. `2D Pose Estimate` para inicializar AMCL. Capturar `amcl_initial.png`.
2. Conducir unos segundos para que las particulas converjan. Capturar `amcl_converged.png`.
3. `Nav2 Goal` para enviar metas. Capturar `costmaps_rviz.png` y `nav_full_run.png`.

Metricas a recoger:

- Tiempo desde `2D Pose Estimate` hasta que `/amcl_pose` se estabiliza.
- RMSE entre `/amcl_pose` y la pose ground truth de Gazebo.
- Tiempo medio para llegar a una meta a 5 m.
- Numero de replanificaciones con obstaculo dinamico.
- **Aborts del BT en la rampa antes vs despues del tuning del controller**.

Mision con waypoints:

```bash
ros2 run wind_tower_bringup mission_navigator
```

### Fase 5 - Redaccion

Cada seccion de `memoria.tex` tiene `% TODO:` con lo que falta. Orden sugerido:

1. Introduccion y objetivo.
2. Arquitectura (con `architecture_diagram.png`).
3. Launchers (tabla esquematica).
4. SLAM (con capturas).
5. EKF (modelos matematicos + plot `ekf_filtered_odom.png`).
6. AMCL (subfiguras initial/converged).
7. Costmaps + Planner.
8. **Tuning iterativo del controller** (tabla y narrativa anti-bandazos / anti-slip).
9. TF (con `tf_frames.png`).
10. Pruebas + metricas.
11. Problemas y conclusiones.

### Fase 6 - Compilar y revisar

1. `make` para generar `memoria.pdf`.
2. Verificar paginacion (objetivo 4-6 paginas).
3. Revisar coherencia entre capturas, tablas y narracion.
