"""
Fase 1 — Construcción del mapa con slam_toolbox.

Arranca sobre una simulación ya activa (simulation.launch.py).
Incluye el teleop PS5 para mapear con el mando.

Uso:
  # Terminal 1 — simulación (sin RViz de Clearpath, usamos el propio)
  ros2 launch wind_tower_bringup simulation.launch.py rviz:=false

  # Terminal 2 — SLAM + teleop mando + RViz
  ros2 launch wind_tower_bringup slam.launch.py

  # Terminal 3 — guardar mapa cuando estés satisfecho
  ros2 run nav2_map_server map_saver_cli -f ~/maps/wind_tower

Argumentos opcionales:
  ros2 launch wind_tower_bringup slam.launch.py use_sim_time:=true rviz:=false
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
    pkg_bringup     = get_package_share_directory('wind_tower_bringup')
    pkg_slam        = get_package_share_directory('slam_toolbox')

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_enabled = LaunchConfiguration('rviz')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Usar reloj de simulación',
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        choices=['true', 'false'],
        description='Lanzar RViz con config de SLAM',
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
            ('scan',     '/scan_raw'),  # BEST_EFFORT → bridge lo convierte a RELIABLE
        ],
    )

    scan_bridge = Node(
        package='wind_tower_bringup',
        executable='scan_qos_bridge',
        name='scan_qos_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
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

    # Relay: cmd_vel escalado por ps5_teleop → plataforma Husky
    # ps5_teleop intercepta /robot/cmd_vel, aplica factor, publica /robot/cmd_vel_scaled
    # Este relay lo lleva a /robot/platform/cmd_vel (input directo de la plataforma)
    cmd_vel_relay = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_scale_relay',
        arguments=['/robot/cmd_vel_scaled', '/robot/platform/cmd_vel'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_bringup, 'config', 'slam.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz_enabled),
    )

    # Espera 15 s a que el reloj de Gazebo se estabilice antes de arrancar
    # pc2scan y slam_toolbox — evita el "jump back in time" que los deja KO
    delayed_slam = TimerAction(period=15.0, actions=[pc2scan, scan_bridge, slam])

    return LaunchDescription([
        declare_use_sim_time,
        declare_rviz,
        dualsense_joy,
        ps5_teleop,
        cmd_vel_relay,
        rviz_node,
        delayed_slam,
    ])
