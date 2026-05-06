# PROJECT STATE

## Fase actual: 4 - Simulación básica en Gazebo (COMPLETADA)

## Entorno
- OS: Ubuntu 24.04 LTS (WSL2)
- ROS 2: Jazzy
- Gazebo: Harmonic (gz sim v8.10.0)
- Workspace del proyecto: ~/ROS2_wind_tower_inspection/ros2_ws

## Paquetes del sistema instalados
- robot_state_publisher ✓
- joint_state_publisher + gui ✓
- ros2_control + ros2_controllers ✓
- diff_drive_controller ✓
- gz_ros2_control ✓ (desde source, con fix WSL2)
- ur_description ✓
- clearpath_gz ✓
- clearpath_config ✓
- clearpath_config_live ✓
- clearpath_viz ✓
- ros-jazzy-clearpath-* (plataforma a200 / Husky) ✓

## Paquetes ROS 2 del proyecto
- wind_tower_description (ament_cmake) ✓
- wind_tower_bringup (ament_python) ✓
- wind_tower_simulation (ament_cmake) ✓
- wind_tower_control: pendiente
- wind_tower_perception: pendiente
- wind_tower_inspection_behaviour: pendiente

## gz_ros2_control — fix WSL2
- Clonado desde source: jazzy branch
- Ruta: ros2_ws/src/gz_ros2_control/
- Bug: null-pointer dereference en ECM JointType/JointAxis components
- Fix: null-checks con RCLCPP_WARN y continue en gz_system.cpp ~línea 296
- Compilado con: colcon build --packages-select gz_ros2_control

## Simulación funcionando
- Lanzar: ros2 launch clearpath_gz simulation.launch.py world:=warehouse rviz:=true
- Entorno: ai-on + export PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH
- Controladores activos: platform_velocity_controller, joint_state_broadcaster, arm_0_joint_trajectory_controller
- Joints cargados: 4 ruedas Husky + 6 joints UR5e
- Sensores publicando: cámara (inspection_camera) + LiDAR (hokuyo)

## Configuración del robot
- Archivo: ros2_ws/src/wind_tower_bringup/config/robot.yaml
- Symlink: ~/clearpath/robot.yaml → robot.yaml del proyecto
- Robot: Husky a200 + UR5e + Hokuyo LiDAR + Intel RealSense D435

## Último paso completado
- Fix gz_ros2_control WSL2 (segfault por nullptr en ECM components)
- Simulación completa Husky + UR5e en Gazebo Harmonic ✓
- Robot visible en RViz + Gazebo con todos los joints y controladores activos

## Próximo paso
- Fase 7: Añadir tramo de torre eólica (STL) + viradores al mundo Gazebo
- Crear launch file propio en wind_tower_bringup
- Añadir PYTHONPATH fix permanente al entorno
