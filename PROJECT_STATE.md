# PROJECT STATE

## Fase actual: 7 - Tramo de torre eólica + nave industrial (COMPLETADA)

## Entorno
- OS: Ubuntu 24.04 LTS (WSL2)
- ROS 2: Jazzy
- Gazebo: Harmonic (gz sim v8.10.0)
- Workspace del proyecto: ~/ROS2_wind_tower_inspection/ros2_ws
- GPU rendering: RTX 4070 Laptop vía Mesa D3D12 (GALLIUM_DRIVER=d3d12)

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
- mesa-vulkan-drivers ✓ (rendering GPU via D3D12 en WSL2)

## Paquetes ROS 2 del proyecto
- wind_tower_description (ament_cmake) ✓
  - meshes/TRAMO_TORRE.STL — malla del tramo de torre (Ø8m × 30m, en mm, eje Z axial)
- wind_tower_bringup (ament_python) ✓
  - simulation.launch.py — launcher principal (usa gz_sim + robot_spawn directamente)
  - dualsense_joy.py — driver mando PS5 vía evdev
  - ps5_teleop.py — teleop brazo UR5e a 10 Hz
- wind_tower_simulation (ament_cmake) ✓
  - worlds/wind_tower_world.sdf — mundo Gazebo: nave industrial + tramo de torre
  - models/wind_tower_tube/ — modelo primitivo cilíndrico (reserva)
- wind_tower_control: pendiente
- wind_tower_perception: pendiente
- wind_tower_inspection_behaviour: pendiente

## gz_ros2_control — fix WSL2
- Clonado desde source: jazzy branch
- Ruta: ros2_ws/src/gz_ros2_control/
- Bug: null-pointer dereference en ECM JointType/JointAxis components
- Fix: null-checks con RCLCPP_WARN y continue en gz_system.cpp ~línea 296

## Mundo Gazebo — wind_tower_world
- Nave industrial: 30m × 60m × 18m, sin techo (para visibilidad en editor)
- Tramo de torre: STL TRAMO_TORRE.STL, escala 0.001 (mm→m)
  - Diámetro: 8 m, Longitud: 30 m, eje axial original: Z
  - Pose en mundo: -4 15 0 1.5708 0 0 → tumbado con eje en Y mundial
  - Centro mundial: (0, 0, 4) — base del tubo en z=0 (suelo)
  - Tubo va de y=-15 a y=+15
- Suelo: plano infinito con fricción
- Lanzador: bypasea clearpath/simulation.launch.py (choices restriction)
  → usa ros_gz_sim/gz_sim.launch.py con ruta absoluta al world
  → usa clearpath_gz/robot_spawn.launch.py para el robot

## Robot spawn por defecto
- Posición: x=0, y=-10, z=0.3 (dentro del tubo, extremo sur)
- Orientación: yaw=1.5708 (mirando hacia el centro del tubo, +Y)

## GPU rendering en WSL2
- Driver: Mesa 25.2 con backend D3D12
- Variables en ai-on: MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA, GALLIUM_DRIVER=d3d12
- Verificar: glxinfo | grep renderer → "D3D12 (NVIDIA GeForce RTX 4070 Laptop GPU)"
- CUDA disponible en /usr/lib/wsl/lib/libcuda.so (para fases IA)

## Controladores activos
- platform_velocity_controller
- joint_state_broadcaster
- arm_0_joint_trajectory_controller
- Joints: 4 ruedas Husky + 6 joints UR5e

## Control con mando DualSense
- Mando conectado vía usbipd (USB passthrough WSL2) — repetir attach en cada sesión
- Lectura directa con evdev (sin módulo joydev, no disponible en kernel WSL2)
- L2 + stick izq → Husky (teleop_twist_joy_node de Clearpath, deadman L2)
- L2 + stick der → brazo UR5e (ps5_teleop nuestro, 10 Hz)
- Topic joy: /robot/joy_teleop/joy

## Configuración del robot
- Archivo: ros2_ws/src/wind_tower_bringup/config/robot.yaml
- Symlink: ~/clearpath/robot.yaml → robot.yaml del proyecto
- Robot: Husky a200 + UR5e + Hokuyo LiDAR + Intel RealSense D435

## Próximo paso
- Fase 10: Captura de imágenes desde cámara RealSense D435
  - Verificar topics de imagen publicados por la cámara simulada
  - Visualizar stream en RViz o rqt_image_view
  - Preparar para Fase 11 (percepción/IA)
