# PROJECT PLAN — Wind Tower Mobile Inspection

## Objetivo
Simular una inspección visual automatizada de un tramo de torre eólica
tumbado sobre viradores, usando un robot móvil Husky con brazo UR5e.

## Arquitectura prevista
- Base móvil: Husky (diff drive)
- Brazo: UR5e montado sobre la base
- Cámara: en TCP/flange del UR5e
- LiDAR: en la base móvil
- Simulación: Gazebo Harmonic + RViz2
- Control: ros2_control + gz_ros2_control
- Escena: tramo de torre eólica (STL) sobre dos viradores rotantes

## Fases

| Fase | Descripción | Estado |
|------|-------------|--------|
| 0 | Auditoría del entorno | COMPLETADA |
| 1 | Repo Git + documentación base + estructura de paquetes | EN CURSO |
| 2 | Robot compuesto Husky + UR5e (URDF/Xacro) | PENDIENTE |
| 3 | Visualización en RViz con TF correcto | PENDIENTE |
| 4 | Simulación básica en Gazebo | PENDIENTE |
| 5 | Cámara y LiDAR simulados | PENDIENTE |
| 6 | Control base móvil y brazo | PENDIENTE |
| 7 | Tramo de torre eólica + viradores | PENDIENTE |
| 8 | Rotación del tramo mediante viradores | PENDIENTE |
| 9 | Misión de inspección | PENDIENTE |
| 10 | Captura de imágenes desde cámara | PENDIENTE |
| 11 | Percepción/IA para defectos | PENDIENTE |
| 12 | Integración final y demo | PENDIENTE |

## Paquetes ROS 2 previstos
- `wind_tower_description` — URDFs/Xacro del robot compuesto y escena
- `wind_tower_bringup` — launch files principales
- `wind_tower_simulation` — mundos Gazebo, configuración de simulación
- `wind_tower_control` — configuración ros2_control, controladores
- `wind_tower_perception` — nodos de cámara, visión, IA
- `wind_tower_inspection_behaviour` — planificación y lógica de misión

## Decisiones técnicas
- ROS 2 Jazzy + Gazebo Harmonic (gz sim)
- Husky desde source (no disponible en apt para Jazzy)
- ur_description desde apt (ros-jazzy-ur-description)
- gz_ros2_control desde apt
- Rama principal: main
