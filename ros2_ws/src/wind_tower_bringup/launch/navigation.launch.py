"""
Nav2 + SLAM simultáneo (sin mapa previo).

Arranca sobre una simulación ya activa (simulation.launch.py).
SLAM Toolbox construye el mapa en tiempo real y publica TF map→odom.
Nav2 usa ese mapa creciente para planificación global.

NOTA IMPORTANTE (Jazzy nav2_bringup 1.3.x):
  navigation_launch.py de nav2_bringup incluye en su lifecycle manager nodos
  extras (route_server, collision_monitor, docking_server) que no están en
  nuestros parámetros. Si alguno falla al activar, bloquea toda la cadena y
  bt_navigator nunca llega a Active → el action server navigate_to_pose no
  se registra → RViz reporta "action server not available".
  Solución: lanzamos los nodos Nav2 directamente con nuestro propio
  lifecycle manager que solo incluye los nodos que usamos.

Uso:
  # Terminal 1 — simulación
  ros2 launch wind_tower_bringup simulation.launch.py

  # Terminal 2 — Nav2 + SLAM + RViz
  ros2 launch wind_tower_bringup navigation.launch.py

  # Guardar mapa cuando esté listo:
  ros2 run nav2_map_server map_saver_cli -f ~/maps/wind_tower

  # Sin RViz (headless):
  ros2 launch wind_tower_bringup navigation.launch.py rviz:=false

Arquitectura TF:
  map → odom        (publicado por SLAM Toolbox)
  odom → base_link  (publicado por Clearpath EKF1, ya activo)

Arbitraje mando vs Nav2 (twist_mux de Clearpath):
  El twist_mux interno de Clearpath (/robot/twist_mux) decide quién mueve
  el Husky. Las prioridades son (mayor número = mayor prioridad):

    slot joy      → /robot/joy_teleop/cmd_vel   prioridad 10  (mando PS5, vía Clearpath teleop_twist_joy)
    slot external → /robot/cmd_vel              prioridad  1  (Nav2 velocity_smoother)

  Regla: mientras el mando envíe mensajes al slot joy, el Husky obedece al
  mando. En cuanto el stick se suelta, el slot joy expira (timeout 0.5 s) y
  twist_mux pasa el control al slot external (Nav2).
  NO es necesario matar procesos ni silenciar el joystick — el arbitraje
  es automático y reversible.

  El mando PS5 (dualsense_joy + ps5_teleop) sigue activo durante la
  navegación para:
    - Controlar el brazo UR5e (stick derecho + L2)
    - Controlar el virador (botones X / Cuadrado)
    - Emitir START_AUTO / STOP de misión (Triángulo / Círculo)
    - Tomar el control del Husky en cualquier momento (pulsando R1 + stick izq)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory('wind_tower_bringup')
    pkg_slam    = get_package_share_directory('slam_toolbox')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz     = LaunchConfiguration('rviz')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Usar reloj de simulación',
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Lanzar RViz con configuración de navegación Nav2',
    )

    nav2_params   = os.path.join(pkg_bringup, 'config', 'nav2_params.yaml')
    slam_params   = os.path.join(pkg_bringup, 'config', 'slam_params.yaml')
    nav2_rviz_cfg = os.path.join(pkg_bringup, 'config', 'navigation.rviz')

    remappings_tf = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # ── Mando PS5 — disponible durante navegación autónoma ───────────────────
    # El arbitraje Nav2 ↔ mando lo gestiona el twist_mux de Clearpath:
    #   - Clearpath teleop_twist_joy → /robot/joy_teleop/cmd_vel  (prioridad 10)
    #   - Nav2 velocity_smoother     → /robot/cmd_vel             (prioridad 1, slot external)
    # Mientras el stick del Husky esté en reposo, el slot joy expira en 0.5 s
    # y twist_mux cede el control a Nav2. Al mover el stick, el mando recupera
    # el control inmediatamente sin necesidad de matar ningún proceso.
    # ps5_teleop gestiona brazo, virador y comandos de misión; NO interfiere
    # con el flujo cmd_vel del Husky (en modo autónomo su scaler retorna sin
    # publicar gracias a la flag _autonomous_active).
    dualsense_joy = Node(
        package='wind_tower_bringup',
        executable='dualsense_joy',
        name='dualsense_joy',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    ps5_teleop = Node(
        package='wind_tower_bringup',
        executable='ps5_teleop',
        name='ps5_teleop',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── pointcloud_to_laserscan + scan_bridge + SLAM ─────────────────────────
    # Delay de 15s para que el reloj de Gazebo esté estable antes de arrancar
    # SLAM (evita el bug "jump back in time" que deja SLAM zombie).
    perception_and_slam = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='pointcloud_to_laserscan',
                executable='pointcloud_to_laserscan_node',
                name='pointcloud_to_laserscan_node',
                output='screen',
                parameters=[
                    os.path.join(pkg_bringup, 'config', 'pointcloud_to_laserscan.yaml'),
                    {'use_sim_time': use_sim_time},
                ],
                remappings=[
                    ('cloud_in', '/velodyne_points'),
                    ('scan',     '/scan_raw'),
                ],
            ),
            Node(
                package='wind_tower_bringup',
                executable='scan_qos_bridge',
                name='scan_qos_bridge',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_slam, 'launch', 'online_async_launch.py')
                ),
                launch_arguments={
                    'slam_params_file': slam_params,
                    'use_sim_time':     use_sim_time,
                }.items(),
            ),
        ],
    )

    # ── Nav2 navigation — nodos individuales con lifecycle manager propio ──────
    # Delay de 20s: 5s adicionales tras SLAM para que map→odom esté estable.
    # Lanzamos solo los nodos que tenemos configurados en nav2_params.yaml para
    # evitar que el lifecycle manager quede bloqueado esperando nodos extras
    # (route_server, collision_monitor, docking_server) presentes en
    # navigation_launch.py de nav2_bringup pero ausentes en nuestra config.
    navigation = TimerAction(
        period=20.0,
        actions=[
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                parameters=[nav2_params, {'use_sim_time': use_sim_time}],
                # Cadena cmd_vel estándar Nav2:
                #   controller_server → cmd_vel (topic por defecto del nodo)
                #   velocity_smoother suscribe cmd_vel, publica cmd_vel_smoothed
                #   remap cmd_vel_smoothed → /robot/cmd_vel en velocity_smoother
                # NO se remapea cmd_vel aquí — el smoother suscribe a cmd_vel directamente.
                remappings=remappings_tf,
            ),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                parameters=[nav2_params, {'use_sim_time': use_sim_time}],
                remappings=remappings_tf,
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[nav2_params, {'use_sim_time': use_sim_time}],
                remappings=remappings_tf,
            ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=[nav2_params, {'use_sim_time': use_sim_time}],
                # behavior_server también publica cmd_vel (spin, backup, drive_on_heading).
                # Debe apuntar a /robot/cmd_vel para que los recoveries muevan el robot.
                remappings=remappings_tf + [('cmd_vel', '/robot/cmd_vel')],
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[nav2_params, {'use_sim_time': use_sim_time}],
                remappings=remappings_tf,
            ),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                parameters=[nav2_params, {'use_sim_time': use_sim_time}],
                remappings=remappings_tf,
            ),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                parameters=[nav2_params, {'use_sim_time': use_sim_time}],
                # velocity_smoother: SUB cmd_vel (salida de controller_server)
                #                    PUB cmd_vel_smoothed → remapeado a /robot/cmd_vel
                # /robot/cmd_vel es el slot external del twist_mux de Clearpath (TwistStamped).
                remappings=remappings_tf + [('cmd_vel_smoothed', '/robot/cmd_vel')],
            ),
            # ── Lifecycle manager — solo nodos que tenemos configurados ────────
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart':    True,
                    'node_names': [
                        'controller_server',
                        'smoother_server',
                        'planner_server',
                        'behavior_server',
                        'velocity_smoother',
                        'bt_navigator',
                        'waypoint_follower',
                    ],
                }],
            ),
        ],
    )

    # ── RViz2 con configuración de Nav2 ───────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', nav2_rviz_cfg],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_rviz,
        dualsense_joy,
        ps5_teleop,
        perception_and_slam,
        navigation,
        rviz,
    ])
