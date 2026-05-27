# Practica 1 RI - Tareas y entregables

Resumen operativo del documento `Practica1_RI.pdf`.

## Nota sobre el robot entregado

El PDF propone como base el robot diferencial Tracer de AgileX y un paquete tipo `diff_robot_urdf`. En esta entrega se utiliza el **Husky A200 de Clearpath Robotics** como plataforma diferencial, manteniendo todos los requisitos de la practica (mundo SDF, sensores IMU + LiDAR obligatorios, bridge ROS-Gazebo, teleop por velocidad).

Razon: el resto del Trabajo Fin de Master (inspeccion de torres eolicas con brazo UR5e + camara + YOLO) se construye sobre el Husky, por lo que se entrega la Practica 1 con la misma plataforma para evitar mantener dos descripciones de robot en paralelo.

Mapeo de componentes en el repo:

- Mundo SDF: `ros2_ws/src/wind_tower_simulation/worlds/wind_tower_world.sdf`
- Robot diferencial: `ros2_ws/src/wind_tower_bringup/config/robot.yaml` (Husky A200, generado por Clearpath)
- URDF extras: `ros2_ws/src/wind_tower_description/urdf/inspection_camera_lighting.urdf.xacro`
- IMU obligatoria: declarada en `robot.yaml` como `phidgets_spatial` sobre `base_link`
- LiDAR obligatorio: `hokuyo_ust` (2D) + `velodyne_lidar` (3D), declarados en `robot.yaml`
- Launch principal: `ros2_ws/src/wind_tower_bringup/launch/simulation.launch.py`
- Bridge ROS-Gazebo: lo gestiona internamente Clearpath al spawnear el robot, no se necesita un `ros_gz_bridge.yaml` propio

El launcher copia `wind_tower_bringup/config/robot.yaml` a `~/clearpath/robot.yaml` antes de arrancar Gazebo, por lo que la configuracion del robot vive versionada dentro del repo.

## Objetivo general

Construir una simulacion en Gazebo integrada con ROS 2 Humble:

- Mundo SDF con obstaculos, iluminacion y parametros fisicos configurables.
- Robot diferencial personalizado, preferiblemente en URDF/Xacro.
- Sensores simulados: IMU y LiDAR obligatorios; camara opcional.
- Integracion Gazebo <-> ROS 2 mediante `ros_gz_bridge`.
- Control del robot desde teclado usando `teleop_twist_keyboard`.

## Resultado final esperado

Al terminar, debe poder lanzarse una simulacion donde:

- Gazebo carga el mundo de la practica.
- El robot Tracer/diferencial aparece correctamente en el mundo.
- El robot se mueve con comandos ROS 2 publicados en `/cmd_vel`.
- Gazebo recibe esos comandos en `/model/tracer/cmd_vel`.
- ROS 2 recibe informacion de simulacion, al menos `/clock` y `/odom`.
- Los sensores IMU y LiDAR funcionan y publican sus topicos.

## 1. Preparar workspace y paquete ROS 2

Crear o reutilizar el workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

Crear el paquete Python de ROS 2:

```bash
cd ~/ros2_ws/src
ros2 pkg create diff_robot_urdf --build-type ament_python --dependencies \
  rclpy xacro robot_state_publisher joint_state_publisher tf2_ros \
  geometry_msgs sensor_msgs
```

Crear carpetas internas:

```bash
cd ~/ros2_ws/src/diff_robot_urdf
mkdir launch worlds urdf meshes models config
```

Estructura minima esperada:

```text
diff_robot_urdf/
  launch/
  worlds/
  urdf/
  meshes/
  models/
  config/
  resource/
  test/
  package.xml
  setup.py
  setup.cfg
  diff_robot_urdf/
    __init__.py
```

## 2. Parte A - Mundo Gazebo en SDF/world

Hay que crear o adaptar un mundo tipo `lab_world.sdf`, `tracer_classroom.sdf` o el `.world` proporcionado por el profesor.

Debe incluir:

- Suelo con friccion controlada.
- 2 o 3 obstaculos con colisiones.
- Iluminacion.
- Camara inicial o vista util de simulacion.
- Parametros fisicos visibles y modificables:
  - `max_step_size`
  - `real_time_factor`
  - `real_time_update_rate`
  - gravedad

Tambien hay que anadir los plugins necesarios de Gazebo, por ejemplo:

```xml
<plugin filename="gz-sim-physics-system"
        name="gz::sim::systems::Physics"/>
<plugin filename="gz-sim-user-commands-system"
        name="gz::sim::systems::UserCommands"/>
<plugin filename="gz-sim-scene-broadcaster-system"
        name="gz::sim::systems::SceneBroadcaster"/>
<plugin filename="gz-sim-sensors-system"
        name="gz::sim::systems::Sensors">
  <render_engine>ogre2</render_engine>
</plugin>
```

Actividad A1:

- Cambiar `max_step_size`, por ejemplo de `0.001` a `0.004`.
- Observar estabilidad del robot y ruido en sensores.
- Documentar que mejora o empeora: aliasing, penetraciones, consumo de CPU, estabilidad.
- Adaptar el fichero `.world` dado por el profesor al montaje final.

### Diferencias entre `.world` y `.sdf`

El PDF dedica una seccion completa a esta distincion. Resumen operativo:

- Ambos formatos comparten la misma sintaxis XML basada en SDF (Simulation Description Format).
- `.world` es una convencion heredada de Gazebo Classic (Gazebo 7-11), normalmente con SDF 1.4-1.7.
- `.sdf` es el estandar actual en Gazebo Sim/Ignition (Fortress, Garden, Harmonic), soporta SDF 1.8-1.10.
- Para esta practica, el profesor puede entregar el mundo como `.world` (compatible) o como `.sdf` (recomendado).

Etiquetas principales que aparecen en el mundo:

- `<world name="...">` - contenedor principal del entorno.
- `<physics>` con atributos `type` (`ode`, `bullet`, `dart`), `max_step_size`, `real_time_factor`, `real_time_update_rate`.
- `<gravity>` - vector 3D, por defecto `0 0 -9.81`.
- `<include>` con `<uri>` - reutilizar modelos de Gazebo Fuel o locales (suelo, sol, robots).
- `<model>` con `<static>`, `<pose>`, `<link>` - definir objetos.
- `<link>` contiene `<collision>` (interaccion fisica) y `<visual>` (representacion grafica).

Ejemplo de inclusion de un modelo en el mundo:

```xml
<include>
  <uri>model://my_diffbot</uri>
  <pose>0 0 0.1 0 0 0</pose>
</include>
```

## 3. Parte B - Robot diferencial personalizado

Crear/adaptar un robot diferencial usando URDF/Xacro o SDF. El documento recomienda URDF/Xacro.

Debe incluir:

- Chasis/base.
- Dos ruedas motrices.
- Rueda loca opcional.
- Colisiones coherentes.
- Masas e inercias razonables; evitar inercias nulas.
- Marcos principales:
  - `base_link`
  - rueda izquierda
  - rueda derecha
  - `imu_link`
  - `lidar_link`

Actividad B1:

- Modelar el robot minimo.
- Usar masas realistas, por ejemplo chasis entre 5 y 15 kg.
- Usar ruedas con dimensiones coherentes, por ejemplo diametro cercano a 0.2 m.
- Ajustar inercias para que el solver fisico no sea inestable.

Actividad B2:

- Insertar el robot en el mundo.
- Validar que no cae ni tiembla de forma extrema.
- Validar que las colisiones no atraviesan el suelo.
- Comprobar que la pose inicial es correcta.

## 4. Configurar `setup.py` y `package.xml`

En `setup.py`, instalar tambien recursos del paquete:

- `launch/*.launch.py`
- `worlds/*`
- `urdf/*`
- `meshes/*`
- `config/*`

En `package.xml`, comprobar estas dependencias:

```xml
<depend>rclpy</depend>
<depend>geometry_msgs</depend>
<depend>sensor_msgs</depend>
<depend>robot_state_publisher</depend>
<depend>xacro</depend>
<depend>tf2_ros</depend>
<depend>ros_gz_sim</depend>
<depend>ros_gz_bridge</depend>
```

## 5. Crear lanzador principal `sim.launch.py`

Crear `launch/sim.launch.py`.

Debe encargarse de:

- Localizar el paquete `diff_robot_urdf`.
- Localizar el mundo, por ejemplo `worlds/tracer_classroom.sdf`.
- Convertir el Xacro principal a URDF temporal.
- Lanzar Gazebo mediante `ros_gz_sim`.
- Lanzar `robot_state_publisher`.
- Spawnear el robot en Gazebo.
- Cargar el bridge desde `config/ros_gz_bridge.yaml`.

Argumentos del launch que deben declararse con `DeclareLaunchArgument`:

- `use_sim_time` con valor por defecto `true`, para que los nodos ROS usen el reloj de Gazebo.
- `world` con valor por defecto la ruta al `.sdf` o `.world` del paquete.

Variable importante que debe aparecer:

```python
bridge_config = os.path.join(pkg_share, 'config', 'ros_gz_bridge.yaml')
```

El nodo bridge debe usar:

```python
Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    name='ros_gz_bridge',
    output='screen',
    parameters=[{'config_file': bridge_config}]
)
```

## 6. Ajustar archivos Xacro

En `tracer_v1.xacro`:

- Sustituir referencias a `tracer_description` por `diff_robot_urdf`.
- Eliminar la inclusion final de `tracer.gazebo` si aparece como:

```xml
<xacro:include filename="$(find diff_robot_urdf)/urdf/tracer.gazebo" />
```

Validar el modelo:

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
xacro ~/ros2_ws/src/diff_robot_urdf/urdf/tracer_v1.xacro -o /tmp/tracer_v1.urdf
```

Alternativa equivalente usando el ejecutable de ROS 2 (la que aparece en el PDF):

```bash
ros2 run xacro xacro ~/ros2_ws/src/diff_robot_urdf/urdf/tracer_v1.xacro > test.urdf
```

Configurar recursos de Gazebo:

```bash
echo 'export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/ros2_ws/src' >> ~/.bashrc
source ~/.bashrc
```

## 7. Control diferencial y bridge

Anadir plugin `diff_drive` en el Xacro principal. Los nombres de juntas deben coincidir exactamente con el URDF/Xacro real.

Ejemplo adaptado:

```xml
<gazebo>
  <plugin filename="ignition-gazebo-diff-drive-system"
          name="ignition::gazebo::systems::DiffDrive">
    <left_joint>left_wheel</left_joint>
    <right_joint>right_wheel</right_joint>
    <wheel_separation>0.3791</wheel_separation>
    <wheel_radius>0.16</wheel_radius>
    <topic>/model/tracer/cmd_vel</topic>
    <odom_topic>/model/tracer/odometry</odom_topic>
    <tf_topic>/model/tracer/tf</tf_topic>
  </plugin>
</gazebo>
```

Crear `config/ros_gz_bridge.yaml` con, al menos:

```yaml
- ros_topic_name: "/cmd_vel"
  gz_topic_name: "/model/tracer/cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"
  direction: ROS_TO_GZ

- ros_topic_name: "/clock"
  gz_topic_name: "/clock"
  ros_type_name: "rosgraph_msgs/msg/Clock"
  gz_type_name: "gz.msgs.Clock"
  direction: GZ_TO_ROS

- ros_topic_name: "/odom"
  gz_topic_name: "/model/tracer/odometry"
  ros_type_name: "nav_msgs/msg/Odometry"
  gz_type_name: "gz.msgs.Odometry"
  direction: GZ_TO_ROS
```

Comandos finales:

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 launch diff_robot_urdf sim.launch.py
```

En otra terminal:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 8. Parte C - Sensores

IMU obligatoria:

- Sensor IMU en `imu_link`.
- Frecuencia recomendada: 100 Hz.
- Validar que las aceleraciones cambian al mover el robot.

LiDAR 2D obligatorio:

- Sensor LiDAR en `lidar_link`.
- Campo de vision recomendado: 270 o 360 grados.
- Rango recomendado: 0.12 m a 12 m.
- Resolucion recomendada: 0.5 grados.

Camara opcional:

- Publicar imagen.
- Publicar `camera_info`.
- Ajustar FOV y resolucion con criterio.

Para descubrir topicos:

```bash
gz topic -l
gz topic -i -t <TOPIC>
ros2 topic list
```

## 9. Parte D - Fisica y realismo

Checklist de realismo:

- Masas razonables.
- Inercias no nulas y coherentes.
- Colisiones sin solapamientos importantes.
- Friccion del suelo y ruedas ajustada.
- Revisar deslizamiento lateral.
- Ajustar `max_step_size` segun estabilidad y CPU.

Actividad D1:

Hacer 3 corridas controladas:

| Corrida | Cambio | Que observar |
| --- | --- | --- |
| E1 | Friccion baja | El robot patina en giros |
| E2 | Friccion alta | El robot gira mas sobre su eje |
| E3 | Paso de simulacion grande | Degradacion de estabilidad/sensores |

Registrar en el informe:

- Trayectoria aproximada, con captura o plot.
- Estabilidad visual.
- Coherencia de sensores.
- Conclusiones sobre los parametros fisicos.

## 10. Parte E - Integracion Gazebo <-> ROS 2

Debe quedar funcionando:

- ROS 2 -> Gazebo: comandos de velocidad.
- Gazebo -> ROS 2: sensores, reloj y odometria.
- Bridge configurado mediante `ros_gz_bridge`.
- Teleoperacion con teclado.

Validaciones recomendadas:

```bash
ros2 topic list
ros2 topic echo /odom
ros2 topic echo /clock
ros2 topic echo /scan
ros2 topic echo /imu/data
gz topic -l
```

## 11. Problemas tipicos y diagnostico rapido

Sintomas frecuentes y por donde mirar primero:

- El robot vibra o sale disparado al cargar el mundo.
  - Causa habitual: inercias mal definidas, masas demasiado pequenas o colisiones que se solapan con el suelo.
  - Accion: revisar `<inertia>` en el Xacro, evitar valores nulos, reducir `max_step_size`.
- No aparecen topicos esperados en ROS 2 (`/scan`, `/imu/data`, `/odom`).
  - Causa habitual: el bridge no esta cargando el YAML, el nombre del topico de Gazebo no coincide o el par de tipos no es valido.
  - Accion: comprobar con `gz topic -l` el nombre real en Gazebo, validar el par `ros_type_name`/`gz_type_name` en `ros_gz_bridge.yaml`.
- Gazebo y ROS no se "hablan" aunque el bridge este corriendo.
  - Causa habitual: combinaciones incompatibles de versiones de Gazebo y ROS 2.
  - Accion: revisar la guia oficial *Installing Gazebo with ROS* y la matriz de compatibilidad.
- El robot no responde al teclado.
  - Causa habitual: nombres de juntas del plugin `diff_drive` no coinciden con el URDF, o el bridge no esta puenteando `/cmd_vel`.
  - Accion: comprobar `ros2 topic echo /cmd_vel` mientras se mueve el teclado, y luego `gz topic -e -t /model/tracer/cmd_vel` en Gazebo.
- El URDF no se genera al lanzar.
  - Causa habitual: referencias a `tracer_description` o a `tracer.gazebo` que no se han sustituido o eliminado.
  - Accion: revisar todos los `xacro:include` del Xacro principal.

## 12. Recursos entregados por el profesor

Ademas del PDF, el material de la practica puede incluir:

- Archivo `tracer_classroom.sdf` ya preparado con paredes, obstaculos y plugins de Gazebo Sim (aparece como Apendice A del PDF). Sirve como mundo de referencia listo para usar.
- Modelos del robot Tracer de AgileX (mallas `.stl`/`.dae` y archivos Xacro) que hay que adaptar al paquete `diff_robot_urdf` segun se indica en la seccion 6 de este documento.

Si el profesor entrega un `.world` en lugar de un `.sdf`, no es necesario convertirlo: ambos formatos son compatibles con Gazebo Sim (ver seccion 2 sobre diferencias).

## Entregables

1. Codigo del paquete ROS 2

   El PDF menciona como entregable un paquete `my_sim_bringup`, pero el desarrollo de la practica usa `diff_robot_urdf`. Lo importante es entregar un paquete coherente con:

   - Mundo en `worlds/`.
   - Modelo/robot en `urdf/`, `meshes/` y/o `models/`.
   - Lanzadores en `launch/`.
   - Configuracion del bridge en `config/ros_gz_bridge.yaml`.
   - `setup.py` y `package.xml` correctamente configurados.

2. Informe breve

   Extension: 2 a 5 paginas.

   Debe incluir:

   - Descripcion del mundo y del robot.
   - Capturas clave de Gazebo/RViz si aplica.
   - Resultados de la Actividad D1.
   - Comparacion entre friccion baja, friccion alta y paso de simulacion grande.
   - Problemas encontrados y soluciones aplicadas.

3. Checklist de verificacion

   Tabla final indicando si cada bloque funciona:

   | Bloque | Estado | Evidencia |
   | --- | --- | --- |
   | Mundo carga en Gazebo | Pendiente | Captura o comando |
   | Obstaculos tienen colision | Pendiente | Prueba en simulacion |
   | Robot aparece en pose correcta | Pendiente | Captura |
   | Robot no vibra ni cae | Pendiente | Observacion |
   | DiffDrive responde | Pendiente | Teleop |
   | `/cmd_vel` puenteado | Pendiente | `ros2 topic echo` / movimiento |
   | `/odom` publicado | Pendiente | `ros2 topic echo /odom` |
   | IMU funcionando | Pendiente | `ros2 topic echo /imu/data` |
   | LiDAR funcionando | Pendiente | `ros2 topic echo /scan` |
   | Experimento D1 documentado | Pendiente | Informe |

## Rubrica indicada en el PDF

| Criterio | Peso |
| --- | ---: |
| Mundo SDF correcto: colisiones, obstaculos, organizacion | 20 % |
| Robot personalizado: inercias, joints, estabilidad | 25 % |
| Sensores funcionando: IMU + LiDAR; camara opcional | 20 % |
| Fisica: experimento D1 documentado y conclusiones | 15 % |
| Integracion ROS 2: bridge, teleop y visualizacion | 20 % |

## Extensiones opcionales para nota extra

- Anadir camara y pipeline de visualizacion/deteccion.
- Anadir sensor de contacto y eventos de colision.
- Exportar odometria y comparar trayectorias con distintos parametros fisicos.
- Documentar diferencias practicas entre Gazebo Classic y Gazebo Sim.
