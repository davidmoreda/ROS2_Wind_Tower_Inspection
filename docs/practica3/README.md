# Memoria Practica 3 RI

Estructura LaTeX para la memoria de la Practica 3 (Configuracion de MoveIt 2 para el brazo manipulador).

## Indice

- [Alcance de la rama](#alcance-de-la-rama)
- [Estado del trabajo](#estado-del-trabajo)
- [Estructura](#estructura)
- [Compilar](#compilar)
- [Plan de trabajo](#plan-de-trabajo)

## Alcance de la rama

La rama `practica-3` contiene el repositorio completo del TFM. La memoria y el video solo cubren el subconjunto MoveIt 2 (paquete `wind_tower_arm_control` + URDF/SRDF del Husky+UR5e + planificacion en RViz). El resto del codigo (`wind_tower_perception` con YOLO, `wind_tower_inspection_behaviour` con BT/voz/langchain, configs NAV2 de Practica 2) coexiste en la rama pero no se evalua aqui.

## Estado del trabajo

- [x] Codigo MoveIt ya en `main` (`wind_tower_arm_control`).
- [x] Fase 1: estructura LaTeX creada.
- [ ] Fase 2: capturas y verificacion de MoveIt cargando.
- [ ] Fase 3: pruebas (articular + cartesiana).
- [ ] Fase 4: redactar las secciones de `memoria.tex`.
- [ ] Fase 5: compilar PDF y revisar.
- [ ] Video demo: guion en [`video_plan.md`](video_plan.md).

**Pre-requisito tecnico:** instalar LaTeX:

```bash
sudo apt install texlive-latex-recommended texlive-latex-extra \
                 texlive-fonts-recommended texlive-lang-spanish \
                 latexmk
```

## Estructura

```
docs/practica3/
|-- memoria.tex          # documento principal con 9 secciones esqueleto
|-- preamble.tex         # paquetes y comandos compartidos
|-- Makefile             # latexmk wrapper
|-- README.md            # este archivo
|-- video_plan.md        # guion del video demo
|-- figures/             # capturas de RViz (Fase 2 y 3)
`-- scripts/             # utilidades opcionales (snippets MoveIt API, plots)
```

## Compilar

```bash
cd docs/practica3
make            # genera memoria.pdf
make watch      # recompila al guardar
make view       # abre el PDF
make clean      # limpia artefactos
```

---

## Plan de trabajo

Decisiones tomadas:

- Formato: **LaTeX**.
- Robot: **UR5e** (montado en Husky) en lugar del ABB IRB-120 del PDF -- autorizado por el profesor.
- Distro: **ROS 2 Humble** (no Jazzy) -- coherencia con el resto del TFM.
- Paquete: `wind_tower_arm_control` (equivale a `irb120_moveit_config` del PDF).
- Equipo: 3 personas (P1, P2, P3) -- reparto en `video_plan.md`.

### Reparto del equipo

| Persona | Responsabilidad memoria                                | Escenas video                       | Eje tematico                  |
| ------- | ------------------------------------------------------ | ----------------------------------- | ----------------------------- |
| **P1**  | Intro + Estructura + Discrepancias + Conclusiones      | Intro + Estructura + Cierre         | Vision global y discrepancias |
| **P2**  | Modelo del robot + SRDF + Configuracion YAML           | Carga del modelo + Configuracion    | URDF/SRDF/Config              |
| **P3**  | Controlador + Launcher + Pruebas (articular y cartesiana) | Pruebas en vivo + Resultados     | Ejecucion y pruebas           |

Capturas a tomar (Fase 2 y 3):

- **P2**: `rviz_robot_loaded.png` (robot cargado con MotionPlanning).
- **P3**: `plan_articular.png`, `plan_cartesiano.png` (las dos planificaciones).

### Fase 1 - Estructura LaTeX (hecha)

Archivos creados:

- `memoria.tex` con 9 secciones esqueleto: intro, estructura, modelo y SRDF, configuracion MoveIt, controlador, launcher, pruebas, discrepancias, problemas, conclusiones.
- `preamble.tex` con paquetes en espanol y `listings`.
- `Makefile`, `.gitignore`.

### Fase 2 - Capturas y verificacion

#### 0. Preparacion

```bash
cd ~/ROS2_Wind_Tower_Inspection/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

#### 1. Lanzar Gazebo + MoveIt

```bash
# Terminal 1 - simulacion Husky+UR5e en Gazebo
ros2 launch wind_tower_bringup simulation.launch.py

# Terminal 2 - MoveIt + RViz (cuando el robot este quieto en Gazebo)
ros2 launch wind_tower_arm_control move_group.launch.py
```

#### 2. Capturas a tomar

Guardar en `docs/practica3/figures/` con estos nombres exactos:

| Archivo | Contenido |
| --- | --- |
| `rviz_robot_loaded.png` | RViz con el plugin MotionPlanning visible, robot UR5e cargado, TF visible |

#### 3. Validar setup MoveIt

```bash
source ~/ROS2_Wind_Tower_Inspection/ros2_ws/install/setup.bash

# Comprobar que move_group esta vivo
ros2 node info /move_group | head -20

# Inspeccionar la SRDF generada por Clearpath
cat ~/clearpath/robot.srdf | head -50
```

### Fase 3 - Pruebas MoveIt

Cubrir los 7 puntos del PDF (Seccion 4: Pruebas minimas):

1. Cargar el robot en RViz (ya hecho en Fase 2).
2. Visualizar estado actual (TF coherente).
3. En el panel "MotionPlanning":
   - Seleccionar grupo `manipulator` (no `arm`, ver discrepancia).
   - Goal State: `ready` o random valid.
4. Pulsar "Plan" -> visualizar trayectoria articular.
5. Pulsar "Execute" -> robot se mueve en Gazebo.
6. Definir pose cartesiana del TCP mediante interactive marker.
7. Trayectoria cartesiana lineal:
   - Con Pilz LIN como planner.
   - O con `compute_cartesian_path` desde un script Python.

Capturas:

| Archivo | Contenido |
| --- | --- |
| `plan_articular.png`  | Trayectoria articular planificada visible en RViz |
| `plan_cartesiano.png` | Trayectoria cartesiana (linea recta del TCP) visible en RViz |

### Fase 4 - Redaccion

Cada seccion de `memoria.tex` tiene `% TODO:` con lo que falta. Orden sugerido:

1. Introduccion (justificar UR5e en lugar de IRB-120).
2. Estructura del paquete (con el arbol que ya esta puesto).
3. Modelo del robot + SRDF (con `rviz_robot_loaded.png` y tabla de joints).
4. Configuracion MoveIt 2 (rellenar funciones de cada YAML).
5. Controlador (mencionar nombre real del JointTrajectoryController).
6. Launcher (`move_group.launch.py`).
7. Pruebas (con las 2 capturas).
8. Discrepancias (tabla ya esquematica).
9. Problemas y conclusiones.

### Fase 5 - Compilar y revisar

1. `make` para generar `memoria.pdf`.
2. Verificar paginacion (4-6 paginas).
3. Revisar tabla de discrepancias y que justifica claramente UR5e vs IRB-120.
