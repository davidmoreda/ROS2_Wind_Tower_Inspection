# Video demo Practica 1 - Plan de grabacion

Duracion objetivo: **5-6 minutos**. Equipo de **3 personas**. Estructura alineada con la rubrica del PDF (mundo 20%, robot 25%, sensores 20%, integracion 20%, D1 15%).

## Reparto del equipo

Cada persona cubre aproximadamente 1/3 del tiempo y 1/3 de la rubrica. Propuesta:

| Persona | Escenas | Tiempo aprox. | Rubrica cubierta |
| --- | --- | --- | --- |
| **P1** | Intro + Nota Husky vs Tracer + Mundo + Cierre/Checklist | ~2:00 | Mundo (20%) |
| **P2** | Robot + Sensores | ~2:00 | Robot (25%) + Sensores (20%) |
| **P3** | Integracion + Teleop + Experimento D1 | ~2:00 | Integracion (20%) + D1 (15%) |

Quien sea cada P1/P2/P3 lo decide el equipo. La narracion del guion de abajo lleva una etiqueta `[P#]` al principio de cada escena.

## Indice
- [Antes de grabar (setup)](#antes-de-grabar-setup)
- [Guion escena por escena](#guion-escena-por-escena)
- [Comandos preparados](#comandos-preparados)
- [Tips de grabacion](#tips-de-grabacion)
- [Postproduccion](#postproduccion)

---

## Antes de grabar (setup)

1. **Compilar y sourcear** una sola vez:
   ```bash
   cd ~/ROS2_Wind_Tower_Inspection/ros2_ws
   colcon build --symlink-install
   source install/setup.bash
   ```

2. **Preparar 4 terminales** abiertas en el escritorio, sourcedas, con titulo visible:
   - **T1** "launch" - aqui se lanza Gazebo.
   - **T2** "topics" - aqui se lanzan los `ros2 topic echo` y `ros2 topic list`.
   - **T3** "teleop" - aqui se lanza `teleop_twist_keyboard`.
   - **T4** "bag" - aqui se graban los rosbags del experimento D1.

3. **Tener RViz** ya con la configuracion adecuada (TF, RobotModel, LaserScan en `/scan`, PointCloud2 en `/velodyne_points`). Si Clearpath lo lanza con una config por defecto, mejor.

4. **Grabador de pantalla** elegido:
   - **OBS Studio** (recomendado, gratuito, audio + camara separada) o
   - **SimpleScreenRecorder** (mas ligero) o
   - **Kazam** (basico).
   Configurar: 1080p, 30 fps, audio del microfono.

5. **Tener el guion abierto en una pantalla aparte** (movil, tablet o segunda pantalla).

---

## Guion escena por escena

| Tiempo  | Escena             | Pantalla                       | Narracion (lo que decir)                                                                                                                                                                                                                                                                                                                                                                                                                |
| ------- | ------------------ | ------------------------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 0:00 - 0:15 | **Intro** `[P1]`       | Diapositiva o terminal limpio  | "Hola, somos [Nombre1], [Nombre2] y [Nombre3]. En este video presentamos la Practica 1 de Robotica Inteligente: una simulacion en Gazebo Sim integrada con ROS 2 Humble. El robot es un Husky A200 de Clearpath, con IMU y LiDAR, y vamos a demostrar mundo, sensores, integracion y un experimento de fisica."                                                                                                                                                       |
| 0:15 - 0:30 | **Nota Husky vs Tracer** `[P1]` | Terminal con `cat docs/practica1/README.md` o slide | "El PDF propone el Tracer de AgileX, pero hemos optado por el Husky porque nuestro TFM construye encima la inspeccion de torres eolicas con UR5e y vision YOLO. La plataforma es identica conceptualmente: robot diferencial con IMU y LiDAR." |
| 0:30 - 1:15 | **Mundo Gazebo (20%)** `[P1]` | Gazebo Sim                  | (Lanzar `ros2 launch wind_tower_bringup simulation.launch.py` en T1, esperar al spawn). "Aqui esta el mundo `wind_tower_world.sdf`: una nave industrial con un tramo de torre eolica, iluminacion direccional, suelo con friccion controlada y obstaculos con colisiones. Los parametros fisicos son: `max_step_size` 0.001, `real_time_factor` 1.0, gravedad estandar -9.81. Carga los plugins de fisica, sensores y user-commands." |
| 1:15 - 2:00 | **Robot (25%)** `[P2]` | Gazebo + RViz                  | "El robot es un Husky A200. La descripcion URDF se genera desde `robot.yaml` con el generador de Clearpath, que vive en `wind_tower_bringup/config/`. En RViz se ve la cadena cinematica: base_link, ruedas izquierda y derecha, top_plate y los attachments. Sin vibraciones, sin penetracion del suelo, pose inicial estable."                                                                                                       |
| 2:00 - 3:00 | **Sensores (20%)** `[P2]` | Terminal T2 + RViz           | "Los sensores obligatorios estan declarados en `robot.yaml`: IMU `phidgets_spatial` y LiDAR 2D Hokuyo. Hemos anadido tambien un Velodyne 3D para el TFM. Validamos que publican: (ejecutar `ros2 topic echo --once /imu/data`) - se ven aceleraciones y giroscopio. (cambiar a) `ros2 topic echo --once /scan` - LaserScan con 720 muestras. (mostrar RViz) - los rayos pintan los obstaculos en tiempo real."                              |
| 3:00 - 3:45 | **Integracion (20%)** `[P3]` | Terminal T2                 | "El puente ROS-Gazebo lo gestiona Clearpath internamente. Listamos los topicos: (ejecutar `ros2 topic list`). Aparecen `/cmd_vel` para enviar velocidades, `/odom` para odometria, `/clock` para reloj de simulacion, mas los sensores. La comunicacion es bidireccional sin necesidad de un `ros_gz_bridge.yaml` propio."                                                                                                              |
| 3:45 - 4:15 | **Teleop por teclado** `[P3]` | Terminal T3 + Gazebo       | (Lanzar `ros2 run teleop_twist_keyboard teleop_twist_keyboard`). "Con el teleop por teclado publicamos en `/cmd_vel` y el robot responde inmediatamente. Pulso `i` para avanzar, `j` y `l` para girar, espacio para frenar. (mover 10 segundos). El plugin diff-drive de Clearpath traduce el Twist en velocidades de rueda."                                                                                                              |
| 4:15 - 5:30 | **Experimento D1 (15%)** `[P3]` | Plots + clip rapido       | "Para la actividad D1 hemos hecho tres corridas con la misma trayectoria, cambiando un parametro cada vez. **E1**: friccion baja, el robot patina en los giros. **E2**: friccion alta, gira casi sobre su eje. **E3**: paso de simulacion grande, aparecen vibraciones y los sensores pierden resolucion temporal. (mostrar `plots/d1_trajectories.png`). Las trayectorias X-Y muestran claramente las tres dinamicas distintas."          |
| 5:30 - 5:50 | **Checklist y cierre** `[P1]` | memoria.pdf en pantalla     | "El checklist del PDF queda verde: mundo, robot estable, IMU, LiDAR, `/cmd_vel`, `/odom`, teleop y experimento D1 documentado. Todo el codigo esta en la rama `practica-1` del repositorio. Gracias por su atencion."                                                                                                                                                                                                                                  |

**Total**: ~5 min 50 s. Margen para edicion.

---

## Comandos preparados

Tener listos en historial (`Ctrl+R`) o en un `cheatsheet.txt` aparte:

```bash
# T1 - Launch
ros2 launch wind_tower_bringup simulation.launch.py

# T2 - Validacion (ejecutar uno a uno mientras narras)
ros2 topic echo --once /imu/data
ros2 topic echo --once /scan
ros2 topic echo --once /odom
ros2 topic list

# T3 - Teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel

# T4 - Rosbag (solo si grabas el D1 en directo; si no, ya tienes los plots)
ros2 bag record -o docs/practica1/data/demo_run /odom
```

---

## Tips de grabacion

- **Hablar despacio** y vocalizar. Mas vale 6 minutos claros que 4 atropellados.
- **Mover el raton lento** y senalar con el cursor antes de hablar de algo.
- **Aumentar el tamano del terminal** (`Ctrl+`) hasta que la fuente sea legible al 720p.
- **Tema oscuro o claro consistente** en todas las ventanas, no mezclar.
- **Una sola toma por escena** si puedes; si te trabas, repite la escena entera, luego cortas.
- **Pausa de 1 segundo** entre escena y escena (ayuda a la edicion).
- **Si fallas un comando**, no lo borres - simplemente di "vamos a corregir" y corrige; demuestra conocimiento.

---

## Postproduccion

- **Editor sugerido**: Kdenlive (gratuito, en apt) o DaVinci Resolve (mas potente).
- **Cortes**: quitar silencios largos > 2 s entre comandos.
- **Zoom puntual**: hacer zoom-in en los outputs criticos (`ros2 topic echo /imu/data`, plot D1).
- **Subtitulos**: opcional pero ayuda al profesor a evaluar; auto con Whisper si te interesa.
- **Titulares en pantalla**: una linea de texto al empezar cada escena (ej. "Sensores - IMU y LiDAR (20%)") refuerza la rubrica.
- **Exportar**: 1080p H.264, MP4, ~10-30 MB para 6 minutos.

## Plan de capturas dentro del video

A medida que vayas grabando el video, ya estaras capturando lo que necesita la memoria. Si el grabador permite extraer fotogramas (Kdenlive si), ahorras tiempo:

| Captura para memoria  | Momento del video | Que hacer                  |
| --------------------- | ----------------- | -------------------------- |
| `mundo_gazebo.png`    | escena 0:30-1:15  | pause + screenshot         |
| `robot_rviz.png`      | escena 1:15-2:00  | pause + screenshot         |
| `imu_echo.png`        | escena 2:00-3:00  | pause justo tras el echo   |
| `scan_rviz.png`       | escena 2:00-3:00  | pause con RViz activo      |
| `topic_list.png`      | escena 3:00-3:45  | pause tras `ros2 topic list` |
| `teleop_demo.png`     | escena 3:45-4:15  | pause con Husky en marcha  |

Asi grabas video y memoria a la vez.
