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
                # Cadena cmd_vel: controller → cmd_vel_smoothed → velocity_smoother → /robot/cmd_vel
                # Clearpath twist_mux (use_stamped: True) consume /robot/cmd_vel como TwistStamped.
                # CRÍTICO: remap cmd_vel → cmd_vel_smoothed para conectar con velocity_smoother.
                # Sin este remap, el controller publica en /cmd_vel y el smoother escucha
                # cmd_vel_smoothed → los dos nodos nunca se comunican → robot inmóvil.
                remappings=remappings_tf + [('cmd_vel', 'cmd_vel_smoothed')],
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
                remappings=remappings_tf,
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
                remappings=remappings_tf,
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
        perception_and_slam,
        navigation,
        rviz,
    ])
