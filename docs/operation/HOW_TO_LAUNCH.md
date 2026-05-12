# HOW_TO_LAUNCH — Arranque mínimo

> Fuente vigente para el flujo MVP.

---

## 0. Prerrequisitos

- WSL2 con Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic instalados.
- Clearpath generator ejecutado y `~/clearpath/robot.yaml` presente.
- DualSense PS5 conectado al WSL2 mediante `usbipd attach --wsl --busid <id>` desde PowerShell admin.
- Alias `ai-on` configurado (exporta `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA`, `GALLIUM_DRIVER=d3d12`, sourcea ROS y el workspace).

Verificación rápida:

```bash
ls /dev/input/event*           # debe aparecer al menos uno
echo $ROS_DISTRO               # jazzy
```

---

## 1. Compilar

```bash
cd ~/ROS2_wind_tower_inspection/ros2_ws
ai-on
colcon build --packages-select wind_tower_simulation wind_tower_description wind_tower_bringup wind_tower_inspection_behaviour
source install/setup.bash
```

Si `gz_ros2_control` no se ha compilado antes:

```bash
colcon build --packages-up-to wind_tower_bringup
```

---

## 2. Lanzar — flujo MVP (dos terminales)

### Terminal 1: simulación + bridges

```bash
cd ~/ROS2_wind_tower_inspection/ros2_ws && ai-on
ros2 launch wind_tower_bringup simulation.launch.py
```

Esperar a ver:

```
[INFO] platform_velocity_controller: Configured and activated
[INFO] arm_0_joint_trajectory_controller: Configured and activated
```

### Terminal 2: misión de inspección

```bash
cd ~/ROS2_wind_tower_inspection/ros2_ws && ai-on
ros2 launch wind_tower_inspection_behaviour inspection.launch.py
```

Por defecto `state_machine_auto_start:=false` — la misión queda en `IDLE`.

### Arrancar la misión

- Botón **Triángulo** del DualSense → `START_AUTO`.
- O publicar manualmente: `ros2 topic pub --once /inspection/mission_command std_msgs/String "data: START_AUTO"`.

### Parar la misión

- Botón **Círculo** del DualSense → `STOP`.
- O: `ros2 topic pub --once /inspection/mission_command std_msgs/String "data: STOP"`.

---

## 3. Perfil corto de pruebas (depuración)

```bash
ros2 launch wind_tower_inspection_behaviour inspection.launch.py \
    state_machine_auto_start:=true \
    state_machine_lane_length_m:=1.0 \
    state_machine_lane_delta_theta_deg:=5.0 \
    state_machine_axial_speed_mps:=0.05 \
    state_machine_turner_speed_rad_s:=0.02 \
    state_machine_publish_rate_hz:=30.0
```

---

## 4. Variantes y opciones

| Caso | Opción |
|---|---|
| Sin RViz (más ligero) | `ros2 launch wind_tower_bringup simulation.launch.py rviz:=false` |
| Posición de spawn personalizada | `ros2 launch wind_tower_bringup simulation.launch.py x:=0.0 y:=-5.0 z:=0.3 yaw:=1.5708` |
| Sin cylinder_localizer | `inspection.launch.py use_cylinder_localizer:=false` |
| Sin teleop PS5 | `inspection.launch.py use_ps5:=false` |
| Sin state_machine (solo monitorización) | `inspection.launch.py use_state_machine:=false` |

---

## 5. Captura de debug "caja negra"

```bash
python3 tools/debug/capture_inspection_debug.py --duration 90
```

Genera `debug_runs/<timestamp>/` con `summary.md` listo para compartir con un agente. Más detalles en `tools/debug/README.md`.

---

## 6. Lo que NO se debe lanzar en el flujo MVP

| Launcher | Por qué |
|---|---|
| `slam.launch.py` | LEGACY PROBABLE. Espera `/scan` (LaserScan), que el pipeline activo no produce. Ver `LAUNCHERS_REFERENCE.md`. |
| `rtabmap.launch.py` | AUXILIAR. Útil para experimentos LiDAR puros, pero `θ_tube` debe venir del encoder del virador, no de ICP. |

---

## 7. Diagnóstico rápido

```bash
ros2 topic list | grep inspection                 # ver topics de la misión
ros2 topic echo /inspection/state_text --once     # estado actual del state_machine
ros2 topic echo /inspection/stability --once      # JSON con flags y ángulos
ros2 topic hz /turner/angle                       # debe ser ~20 Hz
ros2 topic info -v /turner/cmd_vel                # IMPORTANTE: comprobar que no hay 2 publishers
ros2 topic echo /inspection/autonomous_active     # true durante misión autónoma
```
