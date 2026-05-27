# Practica 3 RI - Tareas y entregables

Resumen operativo del documento `Practica_3_RI (1).pdf` (Configuracion de MoveIt 2 para un brazo manipulador).

## Nota sobre el alcance de esta rama

Esta rama `practica-3` contiene el repositorio completo del TFM de inspeccion de torres eolicas. La memoria y el video de Practica 3 solo cubren el subconjunto que aporta a la rubrica de MoveIt 2 (paquete `wind_tower_arm_control` + URDF/SRDF del Husky+UR5e + planificacion articular/cartesiana en RViz). El resto del codigo (`wind_tower_perception` con YOLO y `wind_tower_inspection_behaviour` con BT/voz/langchain) forma parte de otras practicas del proyecto y coexiste en la rama sin formar parte del scope evaluable.

## Robot entregado

El PDF original pide configurar MoveIt 2 para el **ABB IRB-120** en ROS 2 Jazzy. Tras consulta con el profesor, se nos ha autorizado a entregar la practica con el **Universal Robots UR5e** montado sobre el Husky A200 que usamos en las Practicas 1 y 2, en lugar del IRB-120.

Justificacion:

- Continuidad con el TFM: el UR5e es el brazo real con el que se ejecuta la inspeccion de torres eolicas.
- El paquete `wind_tower_arm_control` ya contiene la configuracion MoveIt completa del UR5e.
- La SRDF del Husky+UR5e la genera Clearpath en `~/clearpath/robot.srdf` a partir de `robot.yaml`, con el grupo cinematico estandar de UR.
- ROS 2 distro: **Humble** (no Jazzy), coherente con el resto del proyecto. La discrepancia se comenta en la memoria.

## Objetivo general (PDF §1 y §2)

Construir el entorno de planificacion de movimientos con MoveIt 2 para el UR5e montado en el Husky:

- URDF/Xacro generado por Clearpath a partir de `robot.yaml`.
- SRDF con grupo cinematico `manipulator` (o equivalente declarado por Clearpath).
- YAMLs de configuracion: kinematics, joint_limits, controllers, OMPL, Pilz cartesian limits.
- Controlador JointTrajectory para las 6 articulaciones del UR5e.
- Launcher de `move_group` + RViz con plugin MoveIt 2.
- Demostracion de planificacion articular y cartesiana en RViz.

## Resultado final esperado

Lanzando el `move_group.launch.py` del paquete `wind_tower_arm_control`:

- RViz abre con el plugin "MotionPlanning" y el robot Husky+UR5e cargado.
- Se ve el estado actual del robot con TF coherente.
- Se puede seleccionar el grupo cinematico del brazo en el plugin.
- Se puede planificar y ejecutar una trayectoria articular desde la pose `home` hasta una pose objetivo.
- Se puede definir una pose cartesiana del TCP y planificar hacia ella.
- Se puede mostrar un desplazamiento lineal del TCP (trayectoria cartesiana).

## 1. Estructura del paquete `wind_tower_arm_control` (PDF §3.1)

Equivalencia con el `irb120_moveit_config` del PDF:

```
ros2_ws/src/wind_tower_arm_control/
|-- config/
|   |-- joint_limits.yaml          # Limites articulares (velocidad y aceleracion)
|   |-- kinematics.yaml            # Resolvedor IK (KDL o TRAC-IK)
|   |-- moveit_controllers.yaml    # Mapeo MoveIt -> ROS 2 control (FollowJointTrajectory)
|   |-- ompl_planning.yaml         # Planificadores OMPL (RRTConnect, RRTstar...)
|   |-- pilz_cartesian_limits.yaml # Limites para Pilz cartesian planning
|   |-- octomap_sensors.yaml       # Octomap desde la camara/LIDAR (extra del TFM)
|   |-- arm_inspection_params.yaml # Parametros del nodo de inspeccion (extra del TFM)
|   `-- moveit.rviz                # Config RViz con plugin MoveIt
|-- launch/
|   |-- move_group.launch.py       # Equivalente a demo.launch.py del PDF
|   `-- arm_control.launch.py      # Lanza el nodo de inspeccion (extra del TFM)
|-- wind_tower_arm_control/
|   `-- arm_inspection_node.py     # Nodo Python con secuencias de inspeccion (extra)
|-- package.xml
`-- setup.py
```

El URDF/Xacro del Husky+UR5e lo genera Clearpath en runtime desde `wind_tower_bringup/config/robot.yaml`. El SRDF correspondiente queda en `~/clearpath/robot.srdf` (no esta versionado en el repo).

## 2. Modelo del robot (PDF §3.2)

Comprobaciones equivalentes:

- El Husky+UR5e se visualiza en RViz cuando se lanza el launcher principal.
- Los nombres de los joints del UR5e son los oficiales: `shoulder_pan_joint`, `shoulder_lift_joint`, `elbow_joint`, `wrist_1_joint`, `wrist_2_joint`, `wrist_3_joint`.
- Los limites articulares se importan desde el URDF generado por Clearpath y se completan en `joint_limits.yaml`.
- El TCP es `tool0` (estandar UR), con el `inspection_camera_mount_link` montado encima para la inspeccion del TFM.

## 3. Configuracion SRDF (PDF §3.3)

La SRDF del Husky+UR5e se genera por Clearpath e incluye:

- Grupo cinematico **`manipulator`** con las 6 joints del UR5e (en el PDF se pedia llamarlo `arm`; se documenta la discrepancia en la memoria).
- Estados predefinidos:
  - `home` -> [0, -1.57, 1.57, -1.57, -1.57, 0] (configuracion replegada estandar UR).
  - `ready` -> postura de inspeccion sobre la torre.
- Pares de colisiones deshabilitadas calculadas por Clearpath, justificadas por contacto permanente o adyacencia.

Para inspeccionar la SRDF actual:

```bash
cat ~/clearpath/robot.srdf
```

## 4. Configuracion MoveIt 2 (PDF §3.4)

YAMLs entregados (en `wind_tower_arm_control/config/`):

| Archivo | Proposito |
| --- | --- |
| `joint_limits.yaml`         | Velocidad maxima y aceleracion maxima por joint del UR5e |
| `kinematics.yaml`           | Plugin IK (KDL plugin, OPW para UR, o TRAC-IK) |
| `moveit_controllers.yaml`   | Mapeo MoveIt -> JointTrajectoryController para FollowJointTrajectory |
| `ompl_planning.yaml`        | Configuracion de planificadores OMPL (RRTConnect, RRTstar) |
| `pilz_cartesian_limits.yaml`| Limites cartesianos para Pilz (LIN, PTP, CIRC) |
| `octomap_sensors.yaml`      | Octomap construido desde la camara (extra del TFM) |
| `arm_inspection_params.yaml`| Parametros del nodo de inspeccion (extra del TFM) |

## 5. Controladores (PDF §3.5)

`moveit_controllers.yaml` configura un `JointTrajectoryController` para las 6 joints del UR5e, compatible con la interfaz `FollowJointTrajectory` que usa MoveIt 2. El controlador real lo provee `ros2_control` y se carga via Clearpath / `gazebo_ros2_control` cuando se lanza la simulacion.

## 6. Launcher (PDF §3.6)

El equivalente a `demo.launch.py` del PDF es:

```bash
ros2 launch wind_tower_arm_control move_group.launch.py
```

Este launcher inicia:

- `robot_state_publisher` (gestionado por Clearpath en `simulation.launch.py`).
- `ros2_control_node` (gestionado por `gazebo_ros2_control` en simulacion).
- `move_group` (nodo principal de MoveIt 2 para planificacion).
- RViz 2 con el plugin MoveIt y la config `moveit.rviz` cargada.

Para una demo completa con robot simulado en Gazebo:

```bash
# Terminal 1: simulacion Husky+UR5e en Gazebo
ros2 launch wind_tower_bringup simulation.launch.py

# Terminal 2: MoveIt + RViz
ros2 launch wind_tower_arm_control move_group.launch.py
```

## 7. Pruebas minimas (PDF §4)

Demostrar en el video y en la memoria:

1. Cargar el Husky+UR5e en RViz con el plugin MoveIt.
2. Visualizar el estado actual del robot (TF coherente, brazo en pose inicial).
3. Seleccionar el grupo `manipulator` en el plugin Motion Planning.
4. Planificar una trayectoria articular desde `home` hasta `ready`.
5. Ejecutar la trayectoria (en simulacion -> el robot se mueve en Gazebo).
6. Definir una pose cartesiana del TCP (interactive marker o "Goal State -> random valid").
7. Planificar y ejecutar un desplazamiento lineal del TCP (Pilz LIN o Cartesian Path).

## 8. Entregables (PDF §5)

1. **Codigo fuente**: rama `practica-3` (contiene el repositorio completo del TFM; solo `wind_tower_arm_control` es el scope evaluable de Practica 3).
2. **Capturas de pantalla** de RViz:
   - Husky+UR5e cargado correctamente.
   - Planificacion articular ejecutada.
   - Planificacion cartesiana (desplazamiento lineal del TCP).
3. **Informe en PDF** (`docs/practica3/memoria.tex` -> `memoria.pdf`).
4. **Video demostrativo** (guion en `docs/practica3/video_plan.md`).

## 9. Mapeo con la rubrica del PDF (Seccion 6)

| Elemento evaluado (peso)                                         | Donde se cubre                              |
| ---------------------------------------------------------------- | ------------------------------------------- |
| URDF/Xacro correcto (15%)                                        | Generado por Clearpath + URDF extras propios |
| Configuracion SRDF y grupos cinematicos (15%)                    | `~/clearpath/robot.srdf` (grupo `manipulator`) |
| Configuracion de MoveIt 2 (25%)                                  | `wind_tower_arm_control/config/*.yaml`      |
| Integracion con ROS 2 Control (15%)                              | `moveit_controllers.yaml` + `gazebo_ros2_control` |
| Funcionamiento en RViz y planificacion de trayectorias (20%)     | Capturas + video + `move_group.launch.py`   |
| Calidad del informe y claridad (10%)                             | `docs/practica3/memoria.tex` + revision     |

## 10. Restricciones del PDF (Seccion 7) y comentarios

| Restriccion (Seccion 7 del PDF)                       | Cumplimiento en esta entrega |
| ----------------------------------------------------- | ---------------------------- |
| No copiar config de otro robot                        | Se usa la del UR5e del TFM, no copiada del IRB-120 |
| Joints y enlaces deben corresponder al robot          | UR5e (`shoulder_pan_joint`, etc.) |
| ROS 2 Jazzy                                           | Se usa **Humble** -- justificado en la memoria |
| Planificacion via MoveIt 2                            | Cumple |
| Robot visualizable y movible en RViz                  | Cumple |

## 11. Comandos clave

```bash
# Compilar el workspace
cd ~/ROS2_Wind_Tower_Inspection/ros2_ws
colcon build --symlink-install
source install/setup.bash

# Lanzar simulacion (Gazebo + Husky+UR5e)
ros2 launch wind_tower_bringup simulation.launch.py

# Lanzar MoveIt + RViz
ros2 launch wind_tower_arm_control move_group.launch.py

# Inspeccionar SRDF que genera Clearpath
cat ~/clearpath/robot.srdf

# Ejecutar el nodo de inspeccion (extra del TFM, opcional en demo)
ros2 launch wind_tower_arm_control arm_control.launch.py
```

## 12. Estructura final de la entrega de Practica 3

- Codigo: rama `practica-3` del repositorio.
- Documentos: en `docs/practica3/`.
  - `memoria.tex` -> PDF de 4-6 paginas.
  - `video_plan.md` -> guion del video demo (3 personas).
  - `README.md` -> indice y plan de trabajo de la rama.
- Video: enlace en la memoria.
