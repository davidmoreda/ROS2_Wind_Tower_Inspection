# ROS2 Wind Tower Mobile Inspection

Simulación de inspección visual automatizada de un tramo de torre eólica
tumbado sobre viradores, usando un robot móvil Husky con brazo UR5e.

## Estado actual
Fase 0 completada — auditoría del entorno.
Fase 1 en curso — estructura del proyecto.

## Entorno requerido
- Ubuntu 24.04 LTS
- ROS 2 Jazzy
- Gazebo Harmonic

## Arquitectura
```
Husky (base móvil diff drive)
└── UR5e (brazo robótico 6DOF)
    └── Cámara (en TCP/flange)
LiDAR (en base Husky)
Escena: tramo de torre eólica sobre viradores
```

## Estructura del repositorio
```
ROS2_wind_tower_inspection/
├── README.md
├── PROJECT_PLAN.md
├── PROJECT_STATE.md
├── docs/
├── assets/
│   ├── meshes/
│   └── images/
└── ros2_ws/
    └── src/
        ├── wind_tower_description/
        ├── wind_tower_bringup/
        ├── wind_tower_simulation/
        ├── wind_tower_control/
        ├── wind_tower_perception/
        └── wind_tower_inspection_behaviour/
```

## Comandos principales
```bash
# Activar entorno
ai-on

# Compilar
cd ~/ROS2_wind_tower_inspection/ros2_ws
colcon build

# Source
source install/setup.bash
```

## Fases
Ver PROJECT_PLAN.md para el detalle completo de fases.
