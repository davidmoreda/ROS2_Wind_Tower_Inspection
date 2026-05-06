# PROJECT STATE

## Fase actual: 6 - Control base móvil y brazo (COMPLETADA)

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
- python3-evdev ✓ (lectura DualSense sin módulo joydev)

## Paquetes ROS 2 del proyecto
- wind_tower_description (ament_cmake) ✓
- wind_tower_bringup (ament_python) ✓
  - simulation.launch.py — launcher principal
  - dualsense_joy.py — driver mando PS5 vía evdev
  - ps5_teleop.py — teleop brazo UR5e a 10 Hz
- wind_tower_simulation (ament_cmake) ✓
- wind_tower_control: pendiente
- wind_tower_perception: pendiente
- wind_tower_inspection_behaviour: pendiente

## gz_ros2_control — fix WSL2
- Clonado desde source: jazzy branch
- Ruta: ros2_ws/src/gz_ros2_control/
- Bug: null-pointer dereference en ECM JointType/JointAxis components
- Fix: null-checks con RCLCPP_WARN y continue en gz_system.cpp ~línea 296

## Simulación funcionando
- Lanzar: ver LAUNCH_GUIDE.md
- Controladores activos: platform_velocity_controller, joint_state_broadcaster, arm_0_joint_trajectory_controller
- Joints cargados: 4 ruedas Husky + 6 joints UR5e

## Control con mando DualSense
- Mando conectado vía usbipd (USB passthrough WSL2)
- Lectura directa con evdev (sin módulo joydev, no disponible en kernel WSL2)
- L1 + stick izq → Husky (gestionado por teleop_twist_joy_node de Clearpath)
- L2 + stick der → brazo UR5e (gestionado por ps5_teleop nuestro, 10 Hz)
- Topic joy: /robot/joy_teleop/joy

## Configuración del robot
- Archivo: ros2_ws/src/wind_tower_bringup/config/robot.yaml
- Symlink: ~/clearpath/robot.yaml → robot.yaml del proyecto
- Robot: Husky a200 + UR5e + Hokuyo LiDAR + Intel RealSense D435

## Último paso completado
- Driver DualSense propio (evdev) para WSL2 sin joydev
- Teleop brazo UR5e con timer 10 Hz (velocidad controlada)
- Husky controlado por nodo Clearpath existente (L1 deadman)
- Control completo Husky + brazo UR5e desde mando PS5

## Próximo paso
- Fase 7: Añadir tramo de torre eólica (STL cilíndrico) + viradores al mundo Gazebo
- Crear world SDF personalizado en wind_tower_simulation
