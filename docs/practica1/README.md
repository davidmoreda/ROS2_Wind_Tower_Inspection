# Memoria Practica 1 RI

Estructura LaTeX para la memoria de la Practica 1 (Simulacion en Gazebo + ROS 2 Humble).

## Indice
- [Estado del trabajo](#estado-del-trabajo)
- [Estructura](#estructura)
- [Compilar](#compilar)
- [Plan de trabajo](#plan-de-trabajo)
  - [Fase 1 - Estructura LaTeX (hecha)](#fase-1---estructura-latex-hecha)
  - [Fase 2 - Capturas y verificacion (pendiente)](#fase-2---capturas-y-verificacion-pendiente)
  - [Fase 3 - Experimento D1 (pendiente)](#fase-3---experimento-d1-pendiente)
  - [Fase 4 - Redaccion (pendiente)](#fase-4---redaccion-pendiente)
  - [Fase 5 - Compilar y revisar (pendiente)](#fase-5---compilar-y-revisar-pendiente)

## Estado del trabajo

- [x] Fase 1: estructura LaTeX creada.
- [ ] Fase 2: capturas y verificacion de topicos.
- [ ] Fase 3: experimento D1 (3 corridas).
- [ ] Fase 4: redactar las 9 secciones de `memoria.tex`.
- [ ] Fase 5: compilar PDF y revisar.

**Pre-requisito tecnico:** `latexmk` y `pdflatex` no estan instalados todavia.
Hace falta para la Fase 5:

```bash
sudo apt install texlive-latex-recommended texlive-latex-extra \
                 texlive-fonts-recommended texlive-lang-spanish \
                 latexmk
```

## Estructura

```
docs/practica1/
├── memoria.tex          # documento principal con 9 secciones esqueleto y TODOs
├── preamble.tex         # paquetes y comandos compartidos
├── Makefile             # latexmk wrapper
├── README.md            # este archivo
├── figures/             # capturas de Gazebo, RViz, terminales (Fase 2)
├── plots/               # plots generados (matplotlib) del experimento D1 (Fase 3)
├── data/                # rosbags + CSV del experimento D1 (Fase 3)
│   ├── e1_friccion_baja/
│   ├── e2_friccion_alta/
│   └── e3_paso_grande/
└── scripts/             # publicador de cmd_vel + plot trayectorias (Fase 3)
```

## Compilar

```bash
cd docs/practica1
make            # genera memoria.pdf
make watch      # recompila al guardar
make view       # abre el PDF
make clean      # limpia artefactos
```

---

## Plan de trabajo

Decisiones tomadas:

- Formato: **LaTeX** (compilable con `latexmk`).
- Robot: **Husky A200 (Clearpath)** en lugar del Tracer AgileX del PDF. Justificacion en la introduccion de la memoria.
- Experimento D1: hay que correrlo desde cero.
- Portada: placeholders genericos (`[Nombre]`, `[NIA]`, `[Fecha]`).

### Fase 1 - Estructura LaTeX (hecha)

Archivos creados:

- `memoria.tex` con 9 secciones (intro, mundo, robot, sensores, integracion, D1, problemas, checklist, conclusiones) y comentarios `% TODO` indicando que falta en cada una.
- `preamble.tex` con paquetes en espanol y estilos.
- `Makefile` con targets `all`, `watch`, `view`, `clean`.
- `.gitignore` para artefactos de LaTeX y rosbags.

### Fase 2 - Capturas y verificacion (pendiente)

Lanzar la simulacion y capturar evidencias de que cada bloque funciona.

#### 0. Preparacion (terminal 1)

```bash
cd ~/ROS2_Wind_Tower_Inspection/ros2_ws
colcon build --symlink-install   # solo si no has compilado en esta rama
source install/setup.bash
```

#### 1. Lanzar la simulacion

```bash
ros2 launch wind_tower_bringup simulation.launch.py
```

Esto arranca Gazebo Sim + RViz y spawna el Husky. Espera ~15-20 s a que el robot
aparezca quieto en el suelo.

#### 2. Capturas a tomar

Guardar en `docs/practica1/figures/` con estos **nombres exactos** (la memoria ya los referencia):

| # | Archivo | Que capturar |
|---|---|---|
| 1 | `mundo_gazebo.png` | Gazebo Sim con el mundo completo (vista en perspectiva, obstaculos + iluminacion + suelo). |
| 2 | `robot_rviz.png` | RViz con TF, RobotModel y vista lateral del Husky. |
| 3 | `scan_rviz.png` | RViz con `LaserScan` (`/scan`) y/o `PointCloud2` (`/velodyne_points`) activos. |

#### 3. Validar topicos (terminales paralelas)

Cada terminal nueva: `source ~/ROS2_Wind_Tower_Inspection/ros2_ws/install/setup.bash`.

```bash
ros2 topic echo --once /imu/data      # terminal 2
ros2 topic echo --once /scan          # terminal 3
ros2 topic echo --once /odom          # terminal 4
ros2 topic list                       # terminal 5
```

Capturas (una por terminal con el output a la vista):

| Archivo | Contenido |
|---|---|
| `imu_echo.png` | `ros2 topic echo --once /imu/data` |
| `scan_echo.png` | `ros2 topic echo --once /scan` (cabecera + primeros ranges) |
| `odom_echo.png` | `ros2 topic echo --once /odom` |
| `topic_list.png` | `ros2 topic list` mostrando `/cmd_vel`, `/odom`, `/imu/data`, `/scan`, `/clock`... |

#### 4. Teleop por teclado (terminal 6)

```bash
source ~/ROS2_Wind_Tower_Inspection/ros2_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
```

Mover el robot durante 10 s con `i`/`,`/`j`/`l`. Capturar:

| Archivo | Contenido |
|---|---|
| `teleop_demo.png` | Husky desplazado en Gazebo + terminal de teleop visible. |

#### 5. Apuntar observaciones para el informe

Mientras se hace, anotar en un `.txt` cosas como:

- Tembleque del robot al spawnar?
- Atraviesa el suelo?
- El laser detecta obstaculos?
- Responde el teleop con retraso?

Se usaran en la seccion "Problemas encontrados" y en el checklist final de la memoria.

### Fase 3 - Experimento D1 (pendiente)

Tres corridas con la **misma trayectoria fija** publicada por un nodo:

| Corrida | Cambio | Como aplicarlo |
|---|---|---|
| **E1** Friccion baja | `<mu>` del suelo -> 0.1 | Editar `wind_tower_simulation/worlds/wind_tower_world.sdf`, plano del suelo. |
| **E2** Friccion alta | `<mu>` del suelo -> 1.5 | Idem. |
| **E3** Paso grande | `max_step_size` 0.001 -> 0.004 | Editar `<physics>` del mismo `.sdf`. |

Scripts a crear en `scripts/`:

- `publish_cmd_vel.py` - publica una secuencia fija a `/cmd_vel` (avance 2 s, giro 2 s, avance 2 s, stop). Permite reproducir la misma trayectoria en las 3 corridas.
- `plot_trajectories.py` - lee rosbags de `/odom` (uno por corrida) y genera `plots/d1_trajectories.png` con las 3 trayectorias X-Y superpuestas.

Procedimiento por corrida:

```bash
# Terminal 1 (Gazebo lanzado como en Fase 2)
ros2 launch wind_tower_bringup simulation.launch.py

# Terminal 2 - grabacion de odometria
ros2 bag record -o docs/practica1/data/e1_friccion_baja /odom

# Terminal 3 - publicar trayectoria fija
python3 docs/practica1/scripts/publish_cmd_vel.py

# (esperar a que termine la secuencia y cortar el bag con Ctrl+C)
```

Repetir para E2 (cambiar friccion antes) y E3 (cambiar `max_step_size`).

Despues:

```bash
python3 docs/practica1/scripts/plot_trajectories.py
# genera docs/practica1/plots/d1_trajectories.png
```

### Fase 4 - Redaccion (pendiente)

Cada seccion de `memoria.tex` lleva comentarios `% TODO:` que detallan que falta.
Rellenar en este orden:

1. Introduccion y objetivo.
2. Descripcion del mundo (con `mundo_gazebo.png`).
3. Descripcion del robot (con `robot_rviz.png`).
4. Sensores (con `imu_echo.png`, `scan_rviz.png`, rellenar tabla con frecuencias reales).
5. Integracion ROS-Gazebo (con `teleop_demo.png` y `topic_list.png`).
6. Actividad D1 (con `d1_trajectories.png` y observaciones de las 3 corridas).
7. Problemas encontrados (3-5 vinetas reales).
8. Checklist (marcar `[x]` en cada fila y enlazar la figura).
9. Conclusiones (1 parrafo).

### Fase 5 - Compilar y revisar (pendiente)

1. `make` para generar `memoria.pdf`.
2. Verificar que esta en 2-5 paginas.
3. Releer y revisar coherencia con el checklist.

Repartido aproximado de paginas:

| Seccion | Paginas |
|---|---|
| Portada | 0.5 |
| Introduccion | 0.5 |
| Mundo + Robot + Sensores | 1.5 |
| Integracion ROS-Gazebo | 0.5 |
| Actividad D1 | 1.0 |
| Problemas + Checklist + Conclusiones | 1.0 |
| **Total** | ~5 |
