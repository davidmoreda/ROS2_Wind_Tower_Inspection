"""
Fase 1 — Construcción del mapa con slam_toolbox.

Arranca sobre una simulación ya activa (simulation.launch.py).
Incluye el teleop PS5 para mapear con el mando.

Uso:
  # Terminal 1 — simulación
  ros2 launch wind_tower_bringup simulation.launch.py

  # Terminal 2 — SLAM + teleop mando
  ros2 launch wind_tower_bringup slam.launch.py

  # Terminal 3 — guardar mapa cuando estés satisfecho
  ros2 run nav2_map_server map_saver_cli -f ~/maps/wind_tower

Argumentos opcionales:
  ros2 launch wind_tower_bringup slam.launch.py use_sim_time:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup     = get_package_share_directory('wind_tower_bringup')
    pkg_slam        = get_package_share_directory('slam_toolbox')

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Usar reloj de simulación',
    )

    # ── pointcloud_to_laserscan ───────────────────────────────────────────────
    # Convierte /velodyne_points (PointCloud2) → /scan (LaserScan 2D)
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

    # ── slam_toolbox online_async ─────────────────────────────────────────────
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': os.path.join(
                pkg_bringup, 'config', 'slam_params.yaml'
            ),
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # ── Teleop mando PS5 (DualSense) ─────────────────────────────────────────
    # Usa el nodo dualsense_joy para publicar /joy, y ps5_teleop para
    # convertir botones/ejes → /robot/cmd_vel.
    # El robot se mueve exactamente igual que en modo libre — solo hay que
    # mapearlo despacio para que el scan matching sea preciso.
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

    return LaunchDescription([
        declare_use_sim_time,
        pc2scan,
        slam,
        dualsense_joy,
        ps5_teleop,
    ])
