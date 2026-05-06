# PROJECT STATE

## Fase actual: 1 - Estructura de paquetes ROS 2 (COMPLETADA)

## Entorno
- OS: Ubuntu 24.04 LTS (WSL2)
- ROS 2: Jazzy
- Gazebo: Harmonic (gz sim)
- Workspace del proyecto: ~/ROS2_wind_tower_inspection/ros2_ws

## Paquetes del sistema instalados
- robot_state_publisher ✓
- joint_state_publisher + gui ✓
- ros2_control + ros2_controllers ✓
- diff_drive_controller ✓
- gz_ros2_control ✓
- ur_description ✓
- husky_description: pendiente (desde source, fase 2)

## Paquetes ROS 2 del proyecto
- wind_tower_description (ament_cmake) ✓
- wind_tower_bringup (ament_python) ✓
- wind_tower_simulation (ament_cmake) ✓
- wind_tower_control: pendiente
- wind_tower_perception: pendiente
- wind_tower_inspection_behaviour: pendiente

## Último paso completado
- 3 paquetes ROS 2 creados con licencia Apache-2.0
- Estructura de carpetas (urdf, meshes, launch, rviz, worlds, config) creada
- Bug GZ_SIM_RESOURCE_PATH corregido en .bashrc
- 2 commits en rama main

## Próximo paso
- Fase 2: Crear URDF/Xacro del robot compuesto Husky + UR5e
