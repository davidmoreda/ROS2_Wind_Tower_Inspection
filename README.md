# ROS2 Wind Tower Mobile Inspection

Simulación de inspección visual automatizada de un tramo de torre eólica
tumbado sobre viradores, usando un robot móvil Husky con brazo UR5e.

## Estado actual
Fase 7 completada — mundo Gazebo personalizado con nave industrial y tramo de torre eólica real (STL). Robot spawna dentro del tubo.
Ver [LAUNCH_GUIDE.md](LAUNCH_GUIDE.md) para instrucciones de arranque.

## Entorno requerido
- Ubuntu 24.04 LTS (WSL2)
- ROS 2 Jazzy
- Gazebo Harmonic
- NVIDIA GPU con driver WSL2 (Mesa D3D12 para rendering GPU)

## Arquitectura
```
Husky (base móvil diff drive)
└── UR5e (brazo robótico 6DOF)
    └── Cámara RealSense D435 (en TCP/flange)
LiDAR Hokuyo (en base Husky)
Escena: nave industrial + tramo de torre eólica (STL, Ø8m × 30m)
```

## Estructura del repositorio
```
ROS2_wind_tower_inspection/
├── README.md
├── PROJECT_PLAN.md
├── PROJECT_STATE.md
├── LAUNCH_GUIDE.md                   ← instrucciones de arranque
├── docs/
├── assets/
│   ├── meshes/
│   └── images/
└── ros2_ws/
    └── src/
        ├── gz_ros2_control/          ← fork con fix WSL2
        ├── wind_tower_description/   ← meshes STL del entorno
        │   └── meshes/
        │       └── TRAMO_TORRE.STL
        ├── wind_tower_bringup/       ← launcher + teleop DualSense
        ├── wind_tower_simulation/    ← mundo Gazebo personalizado
        │   ├── worlds/
        │   │   └── wind_tower_world.sdf
        │   └── models/
        ├── wind_tower_control/
        ├── wind_tower_perception/
        └── wind_tower_inspection_behaviour/
```

## Arranque rápido
Ver [LAUNCH_GUIDE.md](LAUNCH_GUIDE.md) para el procedimiento completo.

```bash
# Terminal 1 — Simulación
cd ~/ROS2_wind_tower_inspection/ros2_ws && ai-on
ros2 launch wind_tower_bringup simulation.launch.py

# Terminal 2 — Mando
source install/setup.bash && ros2 run wind_tower_bringup dualsense_joy

# Terminal 3 — Teleop brazo
source install/setup.bash && ros2 run wind_tower_bringup ps5_teleop
```

## Controles mando DualSense
| Control | Acción |
|---|---|
| L2 + stick izquierdo | Mover Husky |
| L2 + stick derecho | Mover brazo UR5e |

## Compilar
```bash
cd ~/ROS2_wind_tower_inspection/ros2_ws
actualizar   # alias: colcon build && source install/setup.bash
```

## Fases
Ver [PROJECT_PLAN.md](PROJECT_PLAN.md) para el detalle completo.

| Fase | Descripción | Estado |
|------|-------------|--------|
| 0 | Auditoría del entorno | ✓ |
| 1 | Repo + estructura de paquetes | ✓ |
| 2 | Robot compuesto Husky + UR5e (URDF) | ✓ |
| 3 | Visualización en RViz | ✓ |
| 4 | Simulación básica en Gazebo | ✓ |
| 5 | Cámara y LiDAR simulados | ✓ |
| 6 | Control base móvil y brazo | ✓ |
| 7 | Tramo de torre eólica + nave industrial | ✓ |
| 8 | Rotación del tramo mediante viradores | pendiente |
| 9 | Misión de inspección | pendiente |
| 10 | Captura de imágenes desde cámara | pendiente |
| 11 | Percepción/IA para defectos | pendiente |
| 12 | Integración final y demo | pendiente |
