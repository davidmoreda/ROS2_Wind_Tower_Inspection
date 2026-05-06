# Guía de arranque — Wind Tower Inspection

## Requisitos previos (una sola vez)

### Conectar mando PS5 (DualSense) por USB
En **PowerShell de Windows como administrador**:
```powershell
usbipd list
# Anota el BUSID del DualSense (aparece como "Sony..." o "DualSense")
usbipd bind --busid 1-3      # cambia 1-3 por tu BUSID
usbipd attach --wsl --busid 1-3
```
Verificar en WSL2:
```bash
ls /dev/input/event*   # debe aparecer event0 o similar
```

---

## Arranque de la simulación

### Terminal 1 — Simulación Gazebo + RViz
```bash
cd ~/ROS2_wind_tower_inspection/ros2_ws
ai-on
export PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH
ros2 launch wind_tower_bringup simulation.launch.py
```
Espera a que Gazebo cargue completamente (verás los mensajes de controladores activos).

### Terminal 2 — Mando DualSense
```bash
cd ~/ROS2_wind_tower_inspection/ros2_ws
ai-on
source install/setup.bash
ros2 run wind_tower_bringup dualsense_joy
```

### Terminal 3 — Teleop del brazo UR5e
```bash
cd ~/ROS2_wind_tower_inspection/ros2_ws
ai-on
source install/setup.bash
ros2 run wind_tower_bringup ps5_teleop
```

---

## Controles del mando

| Botón / Stick | Acción |
|---|---|
| **L1** (mantener) + stick izquierdo | Mover el Husky (adelante/atrás/giro) |
| **L2** (mantener) + stick derecho X | Rotar base del brazo UR5e |
| **L2** (mantener) + stick derecho Y | Subir/bajar hombro del brazo UR5e |

---

## Compilar tras cambios en el código

Desde `~/ROS2_wind_tower_inspection/ros2_ws` con `ai-on` activo:
```bash
actualizar
```
El alias `actualizar` ejecuta `colcon build && source install/setup.bash`.

Para compilar solo un paquete (más rápido):
```bash
colcon build --packages-select wind_tower_bringup
source install/setup.bash
```

---

## Solución de problemas

**Gazebo no arranca / error de recursos**
```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/ros2_ws/src
```

**Error `No module named 'apt'`**
```bash
export PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH
```

**Mando no detectado (`/dev/input/event*` no existe)**
- Reconectar USB y repetir `usbipd attach --wsl --busid X-X` en PowerShell admin

**El brazo va demasiado rápido o lento**
- Editar `MAX_JOINT_VEL` en `ros2_ws/src/wind_tower_bringup/wind_tower_bringup/ps5_teleop.py`
- Valor actual: `0.05` rad/tick a 10 Hz → ~0.5 rad/s máximo
