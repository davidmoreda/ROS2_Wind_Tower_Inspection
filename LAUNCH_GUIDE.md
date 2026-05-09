# Guía de arranque — Wind Tower Inspection

Esta guía arranca la simulación, las herramientas manuales actuales y el primer monitor MVP de inspección. La operación autónoma completa por calles axiales con indexado angular todavía está pendiente.

Regla metodológica vigente: durante una futura inspección autónoma, el virador no debe girar continuamente mientras el robot avanza. El giro del tubo debe hacerse solo en el estado `INDEX_TUBE`, cuando `safe_to_index_tube == true`.

## Requisitos previos (una vez por sesión)

### 1. Conectar mando PS5 DualSense por USB

En **PowerShell de Windows como administrador**:

```powershell
usbipd list
# Anota el BUSID del DualSense (aparece como "Sony..." o "DualSense Wireless...")
usbipd bind --busid 1-3         # solo la primera vez
usbipd attach --wsl --busid 1-3 # repetir en cada sesión
```

Verificar en WSL2:

```bash
ls /dev/input/event*   # debe aparecer al menos un event*
```

### 2. Activar entorno GPU + ROS 2

```bash
ai-on   # alias que activa GPU, ROS2 Jazzy y el workspace
```

El alias `ai-on` exporta:

```bash
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
export GALLIUM_DRIVER=d3d12
export PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH
source /opt/ros/jazzy/setup.bash
source ~/ROS2_wind_tower_inspection/ros2_ws/install/setup.bash
```

## Arranque de la simulación

### Terminal 1 — Simulación Gazebo + RViz

```bash
cd ~/ROS2_wind_tower_inspection/ros2_ws && ai-on
ros2 launch wind_tower_bringup simulation.launch.py
```

Espera a que aparezcan los mensajes de controladores activos:

```text
[INFO] platform_velocity_controller: Configured and activated
[INFO] arm_0_joint_trajectory_controller: Configured and activated
```

El robot aparece dentro del tubo. La GPU RTX 4070 se usa automáticamente si `ai-on` está bien configurado.

Opciones disponibles:

```bash
# Sin RViz (más ligero)
ros2 launch wind_tower_bringup simulation.launch.py rviz:=false

# Posición de spawn personalizada
ros2 launch wind_tower_bringup simulation.launch.py x:=0.0 y:=-5.0 z:=0.3
```

### Terminal 2 — Mando DualSense

```bash
cd ~/ROS2_wind_tower_inspection/ros2_ws && ai-on
ros2 run wind_tower_bringup dualsense_joy
```

### Terminal 3 — Teleop brazo + virador

```bash
cd ~/ROS2_wind_tower_inspection/ros2_ws && ai-on
ros2 run wind_tower_bringup ps5_teleop
```

## Controles del mando DualSense

| Control | Acción |
|---------|--------|
| **L2** + stick izquierdo | Mover Husky (adelante/atrás/giro) |
| **L2** + stick derecho X | Rotar base brazo UR5e (joint 0) |
| **L2** + stick derecho Y | Subir/bajar hombro UR5e (joint 1) |
| **X (Cruz)** | Girar virador + |
| **Cuadrado** | Girar virador − |

Velocidades:

- Virador: 0.15 rad/s ≈ 8.6°/s (ajustable en `ps5_teleop.py` → `TURNER_VEL`).
- Brazo: 0.05 rad/tick a 10Hz → ~0.5 rad/s máx (ajustable en `MAX_JOINT_VEL`).
- Husky: `simulation.launch.py` limita automáticamente `teleop_twist_joy_node` a 0.15 m/s y 0.08 rad/s en modo normal para pruebas de inspección.

Uso correcto en pruebas de la nueva metodología:

- Para simular una calle axial, deja el tubo parado y mueve el Husky en dirección axial.
- Para simular indexado, gira el virador en pequeños incrementos y vuelve a detenerlo.
- No mezcles giro continuo del tubo con avance axial si estás generando datos de cobertura para la estrategia nueva.

## Verificar el virador

```bash
# Ver ángulo en tiempo real
ros2 topic echo /turner/angle_deg

# Verificar frecuencia del bridge Gazebo
ros2 topic hz /turner/joint_state

# Enviar comando manual de prueba
ros2 topic pub /turner/cmd_vel std_msgs/msg/Float64 "{data: 0.1}" --once
```

Señales relevantes:

| Topic | Uso |
|---|---|
| `/turner/joint_state` | Encoder simulado del virador |
| `/turner/angle` | `θ_tubo` acumulado en radianes |
| `/turner/angle_deg` | Diagnóstico en grados |
| `/turner/cmd_vel` | Comando de velocidad angular |

En el sistema autónomo futuro, `tube_indexing_controller.py` deberá cerrar el lazo con `/turner/angle` y no depender de tiempo abierto.

## Verificar el LiDAR

```bash
# Comprobar que llegan puntos
ros2 topic hz /velodyne_points

# Ver en RViz: añadir display PointCloud2 → topic /velodyne_points
```

El LiDAR 3D se usa para geometría del cilindro, detección de obstáculos/bushings, pared, cobertura y seguridad. No sustituye al encoder del virador para `θ_tubo`.

## Verificar IMU y EKF

`simulation.launch.py` lanza `imu_bridge` y `ekf_node`. Falta validar formalmente su salida en la documentación de estado.

```bash
# IMU simulada
ros2 topic hz /robot/sensors/imu_0/data
ros2 topic echo /robot/sensors/imu_0/data --once

# EKF
ros2 topic hz /robot/platform/odom/filtered
ros2 topic echo /robot/platform/odom/filtered --once
```

La IMU es obligatoria para la nueva metodología. Debe permitir estimar roll/pitch respecto a gravedad, `α_robot` y si el eje Z del robot está alineado con la aceleración de la gravedad.

## Monitor de estabilidad MVP

Con `cylinder_localizer` corriendo para publicar `/cylinder_fit/stats`, lanza:

```bash
cd ~/ROS2_wind_tower_inspection/ros2_ws && ai-on
ros2 run wind_tower_inspection_behaviour stability_monitor
```

Verifica:

```bash
ros2 topic echo /inspection/stability
ros2 topic echo /inspection/bottom_lane_locked
ros2 topic echo /inspection/safe_to_scan
ros2 topic echo /inspection/safe_to_index_tube
```

Comportamiento validado:

- Robot parado y estable: `bottom_lane_locked=true`, `safe_to_scan=true`, `safe_to_index_tube=true`.
- Avance axial lento: `bottom_lane_locked=true`, `safe_to_scan=true`, `safe_to_index_tube=false`.

## Prototipos LiDAR disponibles

Estos nodos existen en `wind_tower_bringup`, pero no sustituyen todavía a la arquitectura final de inspección:

```bash
ros2 run wind_tower_bringup cylinder_localizer
ros2 run wind_tower_bringup map_accumulator
```

| Nodo | Estado | Salidas |
|---|---|---|
| `cylinder_localizer` | Prototipo | `/robot_in_tube`, `/cylinder_fit/wall_points`, `/cylinder_fit/stats` |
| `map_accumulator` | Prototipo | `/map_cloud`, `/map_cloud/stats` |

Estos prototipos ayudan a aprender y validar geometría LiDAR, pero el nodo final debe ser `cylindrical_map_node.py` en `wind_tower_inspection_behaviour`, con malla `(x, θ)` y estados de cobertura.

## Compilar tras cambios

```bash
cd ~/ROS2_wind_tower_inspection/ros2_ws && ai-on

# Compilar todo
colcon build && source install/setup.bash

# Compilar paquetes concretos (más rápido)
colcon build --packages-select wind_tower_bringup wind_tower_simulation
source install/setup.bash
```

Para recompilar también el paquete de comportamiento de inspección:

```bash
colcon build --packages-select wind_tower_bringup wind_tower_simulation wind_tower_inspection_behaviour
source install/setup.bash
```

## Solución de problemas

**Mando no detectado**

```bash
ls /dev/input/event*   # si no aparece, reconectar USB
# En PowerShell admin:
usbipd attach --wsl --busid X-X
```

**Gazebo usa llvmpipe en vez de GPU**

```bash
glxinfo | grep renderer   # debe mostrar "D3D12 (NVIDIA...)"
# Si no: verificar que ai-on está activo
```

**Error `No module named 'apt'`**

```bash
export PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH
```

**El robot no aparece en Gazebo**

- Esperar más tiempo; el spawn tarda ~15-20s.
- Verificar que `wind_tower_bringup` y `wind_tower_simulation` están compilados.

**El tubo no gira sobre su eje (gira sobre vértice)**

- Verificar que `wind_tower_simulation` está compilado con la corrección del joint pose `(4.0, 4.0, 0)`.
- Ejecutar `colcon build --packages-select wind_tower_simulation`.

**El botón X no mueve el virador**

- Verificar mapeo de botones: `ros2 topic echo /robot/joy_teleop/joy`.
- X (Cruz) debe activar `buttons[1]`, no `buttons[0]`.
- Verificar que `ps5_teleop` está corriendo: `ros2 node list | grep teleop`.

**RTAB-Map: "Did not receive data since 5 seconds"**

```bash
# Verificar que llegan puntos LiDAR
ros2 topic hz /velodyne_points
# Si no llegan, verificar que lidar3d_bridge está activo
ros2 node list | grep bridge
```

**La cobertura parece incoherente**

- Comprueba que no estás moviendo el virador de forma continua durante una calle axial.
- Verifica `/turner/angle` y el modo de operación usado para marcar celdas.
- Durante esquiva, la cobertura debe marcarse como `bypass_mode`, no como inspección normal.
