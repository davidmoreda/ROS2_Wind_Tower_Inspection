# Video demo Practica 2 - Plan de grabacion

Duracion objetivo: **6-8 minutos**. Equipo de **3 personas**. Estructura alineada con la rubrica del PDF (launchers + LIDAR/IMU + AMCL + costmaps + planner/controller + TF + pruebas).

## Alcance del video

La rama `practica-2` contiene el repositorio completo del TFM, pero **el video solo cubre el scope NAV2**: SLAM, EKF, AMCL, costmaps, planner/controller (incluyendo el tuning iterativo) y la mision de waypoints. No hablamos del brazo UR5e, ni de la percepcion YOLO, ni del BT/voz/langchain, aunque coexistan en la rama.

## Reparto del equipo

| Persona | Escenas | Tiempo aprox. | Tema central |
| ------- | ------- | ------------- | ------------ |
| **P1**  | Intro + Arquitectura + Launchers + Demo SLAM + Cierre | ~2:30 | Estructura del sistema |
| **P2**  | EKF + AMCL | ~2:00 | Estimacion y localizacion |
| **P3**  | Costmaps + Tuning controller + Mision con waypoints + Resultados | ~3:00 | Navegacion y resultados |

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

2. **Preparar 5 terminales**:
   - **T1** "gazebo" - `simulation.launch.py`.
   - **T2** "slam-or-nav" - `slam.launch.py` o `navigation_amcl.launch.py` segun escena.
   - **T3** "topics" - `ros2 topic echo`, `ros2 topic list`, `ros2 lifecycle list`.
   - **T4** "teleop" - `ps5_teleop` o `teleop_twist_keyboard`.
   - **T5** "mission" - `mission_navigator`.

3. **RViz preconfigurado**:
   - SLAM: `config/slam.rviz`.
   - Nav2: `config/navigation.rviz` (TF, /map, costmaps, /plan, /local_plan, /amcl_pose, /particle_cloud).

4. **Grabador** (OBS) a 1080p, 30 fps, audio del microfono.

5. **Mapa pregenerado** disponible en `ros2_ws/maps/wind_tower.*` para no depender de un SLAM en vivo.

---

## Guion escena por escena

| Tiempo      | Escena                                | Pantalla                             | Narracion |
| ----------- | ------------------------------------- | ------------------------------------ | --------- |
| 0:00 - 0:25 | **Intro** `[P1]`                      | Slide o terminal limpio              | "Hola, somos [Nombre1], [Nombre2] y [Nombre3]. En este video presentamos la Practica 2: navegacion autonoma con NAV2 sobre ROS 2 Humble. El robot es el Husky A200 de Clearpath, como en la Practica 1. Aclaramos primero el alcance: este proyecto es un TFM de inspeccion de torres eolicas, asi que en la rama veis tambien codigo de otras practicas (brazo, percepcion, comportamiento). En este video solo hablamos del scope NAV2." |
| 0:25 - 1:00 | **Arquitectura** `[P1]`               | Slide con `architecture_diagram.png` | "El stack se organiza en cuatro capas: Gazebo publica los sensores; `robot_localization` fusiona IMU y odometria con un EKF en 2D; AMCL localiza globalmente sobre un mapa generado por SLAM Toolbox; y NAV2 ejecuta planner, controller y arbol de comportamiento. Todo se orquesta con un `lifecycle_manager`." |
| 1:00 - 2:00 | **Launchers y SLAM rapido** `[P1]`    | T1 + T2 + RViz `slam.rviz`           | "Lanzamos primero la simulacion y luego SLAM Toolbox. (mostrar terminales y RViz). El robot recorre el entorno y el mapa se construye en tiempo real. (mostrar grafica de ocupacion). El mapa final se guarda con `map_saver_cli` en `ros2_ws/maps/wind_tower.pgm`." |
| 2:00 - 3:00 | **Fusion EKF** `[P2]`                 | T3 con `ros2 topic echo` + plot      | "Para fusionar odometria de las ruedas y la IMU usamos `robot_localization` con un EKF en `two_d_mode`. El vector de estado es \[x, y, theta, vx, omega\]. (mostrar `/odometry/filtered`). La salida es continua y suaviza el ruido de la odometria diferencial. (mostrar plot `ekf_filtered_odom.png`)." |
| 3:00 - 4:00 | **AMCL** `[P2]`                       | RViz `navigation.rviz`               | "Con el mapa ya generado, AMCL se encarga de la localizacion global. Pulsamos `2D Pose Estimate` y la nube de particulas se dispersa. (mostrar). Tras unos pequenos movimientos del robot, las particulas convergen y el sistema publica `map -> odom`. Hemos configurado `min_particles=500` y `max_particles=2000` con KLD-sampling." |
| 4:00 - 4:30 | **Costmaps** `[P3]`                   | RViz                                 | "Sobre el mapa estatico se montan dos costmaps: global (en `map`) y local (rolling window en `odom`). El obstacle layer incorpora obstaculos detectados por el LIDAR Velodyne convertido a 2D con `pointcloud_to_laserscan`, y la inflation layer mantiene distancia de seguridad." |
| 4:30 - 5:45 | **Tuning del controller** `[P3]`      | Slide con tabla + Gazebo en rampa    | "Aqui esta una de las contribuciones principales de esta entrega. La rampa de 13 grados del mundo nos obligo a iterar el controller. Nuestra primera hipotesis fue subir velocidad y aceleracion para vencer la pendiente. Resultado: el robot patinaba, AMCL perdia coherencia y la odometria daba saltos. Segunda hipotesis: el problema era traccion, no potencia. Bajamos las aceleraciones del smoother a un tercio, alargamos el lookahead del Pure Pursuit y permitimos aproximacion lenta. (mostrar tabla antes/despues). Ahora el Husky sube la rampa sin slip y la trayectoria es suave." |
| 5:45 - 6:30 | **Mision con waypoints** `[P3]`       | T5 + Gazebo + RViz                   | "Para validar el stack ejecutamos `mission_navigator`, que lee `waypoints.yaml` y envia metas en secuencia. (lanzar). El robot recorre los waypoints. El controller evita obstaculos en el camino y replanifica si encuentra uno nuevo." |
| 6:30 - 7:00 | **Resultados y metricas** `[P3]`      | Slide con tabla de metricas          | "Hemos medido tiempo de convergencia de AMCL, RMSE de la trayectoria, tiempo medio a meta y, lo mas significativo, aborts del BT en la rampa antes y despues del tuning. (mostrar tabla). Los valores estan en la memoria." |
| 7:00 - 7:30 | **Problemas resueltos + Cierre** `[P1]` | Slide                              | "Resumimos tres problemas resueltos: agotamiento de slots DDS por el numero de nodos (`CYCLONEDDS_URI` con `tools/cyclonedds.xml`); QoS de `/scan` (`scan_qos_bridge`); y el tuning del controller que ya hemos contado. Con esto cerramos. El codigo y la memoria estan en la rama `practica-2`. Gracias." |

**Total**: ~7 min 30 s.

---

## Comandos preparados

```bash
# T1 - simulacion
ros2 launch wind_tower_bringup simulation.launch.py

# T2 - SLAM (escena 1:00-2:00)
ros2 launch wind_tower_bringup slam.launch.py
ros2 run nav2_map_server map_saver_cli -f ros2_ws/maps/wind_tower

# T2 - Navegacion con AMCL (escenas 3:00+)
ros2 launch wind_tower_bringup navigation_amcl.launch.py

# T3 - Validacion
ros2 topic list
ros2 topic echo --once /scan
ros2 topic echo --once /imu/data
ros2 topic echo --once /odometry/filtered
ros2 topic echo --once /amcl_pose
ros2 run tf2_tools view_frames
ros2 lifecycle list

# T4 - Teleop
ros2 run wind_tower_bringup ps5_teleop

# T5 - Mision con waypoints
ros2 run wind_tower_bringup mission_navigator
```

---

## Tips de grabacion

- **Hablar despacio**. Un video de 7:30 con narracion clara vale mas que uno de 5:00 atropellado.
- **Pausa de 1 s** entre escena y escena -> facilita la edicion.
- **Cursor lento** y senalar antes de hablar.
- **Zoom alto** en terminales y RViz (que se vea desde lejos).
- **Subtitulos en pantalla** para cada escena ("Tuning del controller", "AMCL") refuerzan la rubrica.
- Si `navigation_amcl.launch.py` lanza nodos extras (mission_controller, voice_command_node) que son del TFM y no de Practica 2, ignorarlos en la narracion; estan en la rama porque conviven con el resto del proyecto.
- Si algun launcher falla, tener rosbag de respaldo para esa escena.

## Capturas dentro del video

Mientras se graba el video, ya se cubren las capturas que necesita la memoria. Pausar y screenshot:

| Captura para memoria       | Momento del video | Comentario |
| -------------------------- | ----------------- | ---------- |
| `architecture_diagram.png` | 0:25-1:00         | Usar el slide directamente |
| `slam_in_progress.png`     | 1:00-2:00         | Pausar con mapa parcial y RViz visible |
| `map_final.png`            | 1:00-2:00         | Tras `map_saver_cli`, abrir `.pgm` |
| `ekf_filtered_odom.png`    | 2:00-3:00         | Plot generado por scripts |
| `amcl_initial.png`         | 3:00-4:00         | Tras "2D Pose Estimate" |
| `amcl_converged.png`       | 3:00-4:00         | Tras conducir unos segundos |
| `costmaps_rviz.png`        | 4:00-4:30         | RViz con global y local costmap |
| `nav_full_run.png`         | 5:45-6:30         | Gazebo + RViz lado a lado durante mision |
| `tf_frames.png`            | preparacion       | `ros2 run tf2_tools view_frames` |

## Postproduccion

- **Editor**: Kdenlive (apt) o DaVinci Resolve.
- **Cortes**: silencios > 2 s.
- **Zooms**: en `/amcl_pose`, en la tabla del tuning, en el plot del EKF.
- **Subtitulos automaticos** con Whisper si se quiere accesibilidad.
- **Exportar**: 1080p H.264, MP4. Tamano objetivo 30-50 MB para 7-8 minutos.
