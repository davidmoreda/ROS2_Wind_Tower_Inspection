# Guía de arranque — Wind Tower Inspection

## Requisitos previos (una sola vez por sesión)

### Conectar mando PS5 (DualSense) por USB
En **PowerShell de Windows como administrador**:
```powershell
usbipd list
# Anota el BUSID del DualSense (aparece como "Sony..." o "DualSense")
usbipd bind --busid 1-3      # solo la primera vez
usbipd attach --wsl --busid 1-3   # repetir en cada sesión
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
ros2 launch wind_tower_bringup simulation.launch.py
```
Espera a que Gazebo cargue completamente (verás los mensajes de controladores activos).
El robot aparece dentro del tubo. La GPU se usa automáticamente (RTX 4070 via Mesa D3D12).

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
| **L2** (mantener) + stick izquierdo | Mover el Husky (adelante/atrás/giro) |
| **L2** (mantener) + stick derecho X | Rotar base del brazo UR5e |
| **L2** (mantener) + stick derecho Y | Subir/bajar hombro del brazo UR5e |

---

## Compilar tras cambios en el código

Desde `~/ROS2_wind_tower_inspection/ros2_ws` con `ai-on` activo:
```bash
actualizar
```
El alias `actualizar` ejecuta `colcon build && source install/setup.bash`.

Para compilar solo paquetes concretos (más rápido):
```bash
colcon build --packages-select wind_tower_bringup wind_tower_simulation wind_tower_description
source install/setup.bash
```

---

## Solución de problemas

**Mando no detectado (`/dev/input/event*` no existe)**
- Reconectar USB y repetir `usbipd attach --wsl --busid X-X` en PowerShell admin

**Gazebo renderiza lento / llvmpipe en vez de GPU**
- Verificar que `ai-on` está activo (exporta GALLIUM_DRIVER=d3d12)
- Comprobar: `glxinfo | grep renderer` → debe mostrar `D3D12 (NVIDIA...)`

**Error `No module named 'apt'`**
```bash
export PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH
```

**El tubo no aparece en Gazebo**
- Verificar que compilaste `wind_tower_description` y `wind_tower_simulation`
- Comprobar que el STL está en `ros2_ws/src/wind_tower_description/meshes/TRAMO_TORRE.STL`

**El brazo va demasiado rápido o lento**
- Editar `MAX_JOINT_VEL` en `ros2_ws/src/wind_tower_bringup/wind_tower_bringup/ps5_teleop.py`
- Valor actual: `0.05` rad/tick a 10 Hz → ~0.5 rad/s máximo
