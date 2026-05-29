# Estructura del workspace y comandos de lanzamiento

> Referencia rápida del proyecto **ROS 2 Wind Tower Inspection**: cómo está
> organizado `src/` y cómo arrancar la simulación + SLAM de mapeo + Foxglove.

---

## 1. Estructura de carpetas de `src/`

### Los 6 paquetes propios + 1 ajeno

```
ros2_ws/src/
├── wind_tower_bringup/              ← orquestación: launch + config + nodos de pegamento
├── wind_tower_description/          ← el robot/mundo en URDF (modelo geométrico)
├── wind_tower_simulation/           ← lo que Gazebo carga (mundos SDF + modelos 3D)
├── wind_tower_inspection_behaviour/ ← "el cerebro": misión, voz, comportamiento
├── wind_tower_perception/           ← visión: detección de defectos YOLO + datasets
├── wind_tower_arm_control/          ← brazo UR5e con MoveIt
└── gz_ros2_control/                 ← DEPENDENCIA de terceros (COLCON_IGNORE → ignórala)
```

Cada paquete sigue el **mismo esqueleto** de paquete ROS 2, por eso se repiten
las mismas carpetas.

### El esqueleto común (aparece en casi todos)

| Carpeta/archivo        | Para qué sirve |
|------------------------|----------------|
| `package.xml`          | Manifiesto: nombre, dependencias, mantenedor. Lo lee colcon. |
| `setup.py` / `setup.cfg` | Build de paquetes **Python** (define los ejecutables → `entry_points`). |
| `CMakeLists.txt`       | Build de paquetes **C++/instalación de ficheros** (los que solo instalan datos). |
| `resource/`            | Marcador del índice ament (archivo vacío con el nombre del paquete). No tocar. |
| `launch/`              | Los `.launch.py` (cómo arrancar los nodos). |
| `config/`              | Los `.yaml` de parámetros (cómo se comportan los nodos). |
| `<nombre_paquete>/`    | El **código Python** del paquete (los nodos). |
| `test/`                | Tests de estilo (copyright, flake8, pep257). |

**Pista clave:** los paquetes con **`setup.py`** son Python (`bringup`,
`behaviour`, `perception`, `arm_control`); los que tienen **`CMakeLists.txt`**
son `ament_cmake` y solo instalan ficheros de datos (`description`,
`simulation`). Eso te dice de un vistazo si un paquete contiene lógica o solo
assets.

---

### Paquete por paquete

#### `wind_tower_bringup/` — el director de orquesta
El paquete más "lleno" porque coordina todo lo demás.
- **`launch/`** → los 4 launchers de base+navegación (`simulation`, `slam`,
  `navigation`, `navigation_amcl`).
- **`config/`** → todos los `.yaml` de tuning: `nav2_params.yaml`,
  `amcl_params.yaml`, `slam_params.yaml`, `ekf_map.yaml`,
  `pointcloud_to_laserscan.yaml`, `robot.yaml` (config Clearpath),
  `waypoints.yaml`, y los `.rviz` (vistas de RViz para SLAM y navegación).
- **`behavior_trees/`** → los XML de Behavior Tree que usa `bt_navigator` de
  Nav2 (navegar a un punto / por waypoints, con replanning y recovery).
- **`wind_tower_bringup/`** (código) → **nodos "pegamento"**, pequeños y de
  infraestructura:
  - `scan_qos_bridge.py` (BEST_EFFORT→RELIABLE del scan)
  - `tf_static_relay.py` (relay de TF estático con QoS correcto)
  - `obstacle_cloud_filter.py` (filtra el suelo de la nube de puntos)
  - `turner_node.py` (control del virador)
  - `dualsense_joy.py` + `ps5_teleop.py` (mando PS5)
  - `mission_navigator.py` (cliente CLI para mandar al robot a una estación)

#### `wind_tower_description/` — el modelo del robot/escena (URDF)
Paquete **CMake**, solo datos. Casi vacío a propósito porque el robot completo
(Husky+UR5e) lo genera Clearpath en `~/clearpath`; aquí solo van **añadidos
propios**:
- `meshes/` → `TRAMO_TORRE.STL` (la geometría 3D del tramo de torre eólica).
- `urdf/` → `inspection_camera_lighting.urdf.xacro` (la cámara de inspección +
  luz que cuelga del brazo).

#### `wind_tower_simulation/` — lo que Gazebo carga
Paquete **CMake**, solo assets de simulación.
- **`worlds/`** → los mundos `.sdf`:
  - `wind_tower_world.sdf` (el principal),
  - `wind_tower_world_defects_actors.sdf` (con defectos + personas que caminan),
  - `wind_tower_world_synthetic.sdf` (para generar dataset),
  - `defects_ground_truth.yaml` (la "verdad" de dónde están los defectos).
- **`models/`** → modelos 3D que el mundo referencia con `model://`:
  - `wind_tower_tube/` (el tubo con la junta del virador),
  - `actor_walking/` (la malla animada `walk.dae` de personas caminando),
  - `person_worker/` (un operario estático con texturas .obj/.png).

#### `wind_tower_inspection_behaviour/` — el cerebro de la misión
Paquete **Python**. No tiene `launch/` ni `config/` propios (lo lanzan desde
`bringup`).
- **`wind_tower_inspection_behaviour/`** (código):
  - `mission_controller.py` (orquesta la misión + sirve la UI web puerto 5000),
  - `voice_command_node.py` + `natural_language_commands.py` (comandos por voz),
  - `inspection_bt.py` (árbol de comportamiento de inspección),
  - `people_collision_sync.py` (mueve cápsulas de colisión sobre los actores),
  - `random_walk_people.py` (versión vieja, sustituida por la anterior).
- **`static/`** → imágenes (`battery.png`, `tool.png`) que sirve la UI web.

#### `wind_tower_perception/` — la visión
Paquete **Python**.
- **`launch/`** → `perception.launch.py`.
- **`config/`** → `perception_params.yaml`, `dataset.yaml`, `synthetic_dataset.yaml`.
- **`wind_tower_perception/`** (código):
  - `detector_node.py` (YOLO/Hough), `image_capture_node.py`,
    `defect_mapper_node.py` (proyección cilíndrica), `synthetic_capture_node.py`,
    `auto_dataset_node.py`.
  - **`scripts/`** → utilidades que **no son nodos** (se ejecutan a mano):
    `train_yolo.py`, `generate_synthetic_world.py`, `generate_inspection_report.py`.

#### `wind_tower_arm_control/` — el brazo UR5e
Paquete **Python**, pero con mucho **`config/`** porque MoveIt se configura con
YAMLs:
- **`launch/`** → `move_group.launch.py` + `arm_control.launch.py`.
- **`config/`** → toda la config MoveIt: `kinematics.yaml`, `joint_limits.yaml`,
  `moveit_controllers.yaml`, `ompl_planning.yaml`, `pilz_cartesian_limits.yaml`,
  `octomap_sensors.yaml`, `moveit.rviz`, y `arm_inspection_params.yaml`.
- **`wind_tower_arm_control/`** (código) → `arm_inspection_node.py` (el
  comportamiento del brazo).

---

### La regla mental para no perderte

Dentro de **cualquier** paquete, la separación es siempre la misma idea de tres
patas:

- **`launch/`** = *cómo se arranca* (qué nodos, en qué orden, con qué delays).
- **`config/`** = *cómo se comporta* (parámetros, sin tocar código).
- **`<paquete>/`** = *qué hace* (el código de los nodos).

Y entre paquetes, la separación es **por subsistema del robot**: orquestación
(`bringup`), modelo (`description`), mundo (`simulation`), cerebro
(`behaviour`), ojos (`perception`), brazo (`arm_control`).

---

## 2. Comandos: simulación + SLAM de mapeo (sin mapa previo) + Foxglove

> `foxglove_bridge` está instalado pero no en ningún launcher del proyecto, así
> que se arranca aparte. Este es el flujo de **mapeo desde cero**, una terminal
> por bloque.

En **cada terminal** primero hay que sourcear el workspace:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ROS2_Wind_Tower_Inspection/ros2_ws/install/setup.bash
```

### Terminal 1 — Simulación (Gazebo + robot + bridges)
```bash
ros2 launch wind_tower_bringup simulation.launch.py rviz:=false
```
`rviz:=false` porque vas a visualizar en Foxglove, no en RViz. Espera ~15-20 s a
que Gazebo y el `/clock` se estabilicen antes de seguir.

### Terminal 2 — SLAM de mapeo + teleop mando PS5
```bash
ros2 launch wind_tower_bringup slam.launch.py rviz:=false
```
Arranca `slam_toolbox` (online_async), el `pointcloud_to_laserscan` y el mando
para que muevas el robot y vaya construyendo el mapa. (Si quieres también la
RViz propia de SLAM, quita el `rviz:=false`.)

### Terminal 3 — Foxglove Bridge
```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml use_sim_time:=true
```
Luego abre **Foxglove Studio** → *Open connection* → `ws://localhost:8765`. El
`use_sim_time:=true` es importante para que los timestamps casen con el reloj de
Gazebo.

Si `foxglove_bridge_launch.xml` diera error de nombre, alternativa equivalente:
```bash
ros2 run foxglove_bridge foxglove_bridge --ros-args -p use_sim_time:=true
```

### Terminal 4 — Guardar el mapa (cuando hayas mapeado todo)
```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/wind_tower
```
Genera `~/maps/wind_tower.yaml` + `.pgm`. A partir de ahí ya usarías
`navigation_amcl.launch.py map:=~/maps/wind_tower.yaml`.

---

### Orden y notas

1. Arranca la **1**, espera a que cargue Gazebo.
2. Luego la **2** (SLAM tiene un delay interno de 15 s a propósito para no
   romperse con el `/clock` inestable de WSL2).
3. La **3** cuando quieras, en cualquier momento.
4. Mueve el robot despacio con el mando para que el scan matching sea preciso;
   cuando el mapa esté completo, lanza la **4**.

Para ver el mapa creciendo en Foxglove, añade un panel y suscríbete a `/map`,
`/scan` y los TF.
