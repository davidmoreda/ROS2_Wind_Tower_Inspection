"""
Fase 2 — Navegación autónoma con Nav2 (AMCL + EKF2 + RPP).

Arranca sobre una simulación ya activa (simulation.launch.py).
Requiere un mapa guardado previamente con slam.launch.py.

Uso:
  # Terminal 1 — simulación
  ros2 launch wind_tower_bringup simulation.launch.py

  # Terminal 2 — Nav2 completo
  ros2 launch wind_tower_bringup navigation.launch.py map:=$HOME/maps/wind_tower.yaml

  # Enviar goal desde RViz (herramienta "Nav2 Goal") o desde código:
  ros2 run nav2_simple_commander demo_waypoints

Arquitectura TF:
  map → odom        (publicado por EKF2 = robot_localization)
  odom → base_link  (publicado por Clearpath EKF1, ya activo)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup   = get_package_share_directory('wind_tower_bringup')
    pkg_nav2      = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml     = LaunchConfiguration('map')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Usar reloj de simulación',
    )
    declare_map = DeclareLaunchArgument(
        'map',
        description='Path al fichero .yaml del mapa (generado por map_saver_cli)',
    )

    nav2_params = os.path.join(pkg_bringup, 'config', 'nav2_params.yaml')

    # ── pointcloud_to_laserscan ───────────────────────────────────────────────
    pc2scan = Node(
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
            ('scan',     '/scan'),
        ],
    )

    # ── EKF2 — fusión map → odom ──────────────────────────────────────────────
    # Funde /robot/platform/odom/filtered + /amcl_pose → TF map→odom
    # AMCL tiene tf_broadcast:false para que no haya conflicto
    ekf_map = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_map_node',
        output='screen',
        parameters=[
            os.path.join(pkg_bringup, 'config', 'ekf_map.yaml'),
            {'use_sim_time': use_sim_time},
        ],
    )

    # ── Nav2 localization (map_server + AMCL) ────────────────────────────────
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map':          map_yaml,
            'use_sim_time': use_sim_time,
            'params_file':  nav2_params,
            'autostart':    'true',
        }.items(),
    )

    # ── Nav2 navigation (planner + controller + behaviors + bt) ──────────────
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file':  nav2_params,
            'autostart':    'true',
        }.items(),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map,
        pc2scan,
        ekf_map,
        localization,
        navigation,
    ])
