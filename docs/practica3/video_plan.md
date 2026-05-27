# Video demo Practica 3 - Plan de grabacion

Duracion objetivo: **5-6 minutos**. Equipo de **3 personas**. Estructura alineada con las pruebas minimas del PDF (Seccion 4): cargar robot, seleccionar grupo, planificar articular, ejecutar, planificar cartesiana.

## Alcance del video

La rama `practica-3` contiene el repositorio completo del TFM, pero **el video solo cubre el scope MoveIt 2**: paquete `wind_tower_arm_control`, SRDF del Husky+UR5e, planificacion articular y cartesiana en RViz. No hablamos del NAV2 (Practica 2), ni de la percepcion YOLO, ni del BT/voz/langchain, aunque coexistan en la rama.

## Reparto del equipo

| Persona | Escenas | Tiempo aprox. | Tema central |
| ------- | ------- | ------------- | ------------ |
| **P1**  | Intro + Justificacion UR5e + Estructura + Cierre | ~1:40 | Vision global y discrepancias |
| **P2**  | Modelo + SRDF + Carga en RViz | ~1:40 | Modelado y configuracion |
| **P3**  | Planificacion articular + Cartesiana + Resultados | ~2:00 | Ejecucion en vivo |

Las etiquetas `[P1]`, `[P2]`, `[P3]` indican quien narra cada escena.

## Indice
- [Antes de grabar (setup)](#antes-de-grabar-setup)
- [Guion escena por escena](#guion-escena-por-escena)
- [Comandos preparados](#comandos-preparados)
- [Tips de grabacion](#tips-de-grabacion)
- [Postproduccion](#postproduccion)

---

## Antes de grabar (setup)

1. **Compilar y sourcear**:
   ```bash
   cd ~/ROS2_Wind_Tower_Inspection/ros2_ws
   colcon build --symlink-install
   source install/setup.bash
   ```

2. **Preparar 3 terminales** abiertas y sourcedas, con titulo visible:
   - **T1** "gazebo" - `ros2 launch wind_tower_bringup simulation.launch.py`.
   - **T2** "moveit" - `ros2 launch wind_tower_arm_control move_group.launch.py`.
   - **T3** "inspect" - para `ros2 node info`, `cat ~/clearpath/robot.srdf`, etc.

3. **RViz** se abre desde el launcher de MoveIt con la config `moveit.rviz`. Comprobar antes de grabar:
   - Panel `MotionPlanning` visible y maximizado.
   - Grupo `manipulator` seleccionable.
   - Robot Husky+UR5e visible en el centro.

4. **Grabador de pantalla** (OBS) a 1080p, 30 fps, audio del microfono.

---

## Guion escena por escena

| Tiempo      | Escena                                  | Pantalla                            | Narracion |
| ----------- | --------------------------------------- | ----------------------------------- | --------- |
| 0:00 - 0:20 | **Intro** `[P1]`                        | Slide o terminal limpio             | "Hola, somos [Nombre1], [Nombre2] y [Nombre3]. En este video presentamos la Practica 3 de Robotica Inteligente: configuracion de MoveIt 2 sobre nuestro brazo manipulador. Una nota inicial sobre el alcance: la rama contiene el repositorio completo del TFM de inspeccion de torres eolicas, pero este video se centra solo en MoveIt 2." |
| 0:20 - 0:50 | **Justificacion UR5e** `[P1]`           | Slide / terminal con `cat ~/clearpath/robot.yaml` | "El PDF original pide ABB IRB-120, pero el profesor nos ha autorizado a usar el UR5e montado en el Husky por continuidad con el TFM. Mantenemos el UR5e porque sobre el se construyen las Practicas 1 y 2 y la mision de inspeccion. La SRDF la genera Clearpath automaticamente desde `robot.yaml`, con el grupo cinematico `manipulator` y los 6 joints estandar del UR5e: shoulder_pan, shoulder_lift, elbow, wrist 1, 2 y 3." |
| 0:50 - 1:20 | **Estructura del paquete** `[P1]`       | Editor mostrando `wind_tower_arm_control/` | "El paquete equivalente a `irb120_moveit_config` es `wind_tower_arm_control`. Contiene la carpeta `config/` con los YAMLs de MoveIt (joint_limits, kinematics, controllers, OMPL, Pilz cartesian, octomap, RViz config), la carpeta `launch/` con `move_group.launch.py` y `arm_control.launch.py`, y un nodo Python de inspeccion como extra del TFM." |
| 1:20 - 2:10 | **Modelo + SRDF** `[P2]`                | Terminal T3 + slide                 | "El URDF/Xacro del Husky+UR5e lo genera Clearpath en runtime; la SRDF resultante esta en `~/clearpath/robot.srdf`. (mostrar `cat ~/clearpath/robot.srdf | head -30`). Aqui se ve el grupo `manipulator` con los 6 joints, los estados predefinidos como `home` y `ready`, y los pares de colisiones deshabilitadas calculados automaticamente para colisiones permanentes y adyacencias." |
| 2:10 - 3:00 | **Carga en RViz** `[P2]`                | T1 + T2 + RViz                      | "Lanzamos primero la simulacion en Gazebo, esperamos a que el Husky aparezca quieto en el mundo, y luego arrancamos `move_group.launch.py` que inicia el nodo `move_group` y RViz con el plugin MotionPlanning. (esperar a que cargue). Se ve el robot con TF coherente, y en el panel de la izquierda podemos seleccionar el grupo `manipulator`." |
| 3:00 - 4:00 | **Planificacion articular** `[P3]`      | RViz panel MotionPlanning           | "Vamos a planificar una trayectoria articular. (en el panel, Goal State -> `ready`, despues Plan). (mostrar la trayectoria en RViz). OMPL con RRTConnect calcula la ruta en pocos milisegundos. Pulsamos Execute. (el robot se mueve en RViz y en Gazebo en sincronia). El JointTrajectoryController de ROS 2 control recibe la trayectoria a traves de la interfaz FollowJointTrajectory." |
| 4:00 - 5:00 | **Planificacion cartesiana** `[P3]`     | RViz + interactive marker           | "Ahora una trayectoria cartesiana. (mover el interactive marker del TCP hacia un punto a 20 cm). (Plan, ver la linea recta en RViz, Execute). El TCP sigue una linea recta gracias al planner Pilz LIN o al `compute_cartesian_path` de la API de MoveIt. (mostrar que el end-effector se desplaza linealmente)." |
| 5:00 - 5:30 | **Cierre y discrepancias** `[P1]`       | Slide                               | "Hemos cubierto las 7 pruebas minimas del PDF con el UR5e en lugar del IRB-120, el grupo `manipulator` en lugar de `arm` y Humble en lugar de Jazzy, justificado por la continuidad del TFM. El codigo y la memoria estan en la rama `practica-3` del repositorio. Gracias por su atencion." |

**Total**: ~5 min 30 s. Margen para edicion.

---

## Comandos preparados

```bash
# T1 - simulacion
ros2 launch wind_tower_bringup simulation.launch.py

# T2 - MoveIt + RViz (lanzar cuando el robot ya este quieto en Gazebo)
ros2 launch wind_tower_arm_control move_group.launch.py

# T3 - inspeccion
cat ~/clearpath/robot.srdf | head -50
ros2 node info /move_group | head -30
ros2 topic echo --once /joint_states
```

---

## Tips de grabacion

- **Esperar 15-20 s** a que Gazebo y MoveIt arranquen del todo antes de grabar nada.
- **No abrir el RViz manualmente**, dejar que lo abra el launcher (asi carga `moveit.rviz` automaticamente).
- **Aumentar el tamano de los paneles de MoveIt** (especialmente "MotionPlanning") para que se vean los botones y dropdowns.
- **El interactive marker del TCP** suele ser pequeno; activar "Marker Scale" alto en el panel de RViz.
- **Hablar despacio** cuando se selecciona Goal State o se mueve el marker.

## Capturas dentro del video

Las dos capturas que necesita la memoria se sacan pausando momentos concretos:

| Captura para memoria       | Momento del video | Comentario |
| -------------------------- | ----------------- | ---------- |
| `rviz_robot_loaded.png`    | 2:10-3:00         | Tras la carga, antes de planificar |
| `plan_articular.png`       | 3:00-4:00         | Tras "Plan", con la trayectoria visible |
| `plan_cartesiano.png`      | 4:00-5:00         | Tras "Plan" de la cartesiana, con la linea recta |

## Postproduccion

- **Editor**: Kdenlive o DaVinci Resolve.
- **Zooms**: hacer zoom-in sobre el panel MotionPlanning y sobre el efector durante la trayectoria cartesiana.
- **Subtitulos**: opcional pero recomendado para el dropdown del grupo y el nombre de los estados.
- **Exportar**: 1080p H.264, MP4. Tamano objetivo ~25-40 MB para 5:30.
