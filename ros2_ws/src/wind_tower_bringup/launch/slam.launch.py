"""
SLAM Toolbox — mapeo online del interior del tubo.
Lanzar DESPUÉS de simulation.launch.py.
scan_gate (en simulation.launch.py) garantiza que /scan solo llega cuando TF está listo.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('wind_tower_bringup'),
        'config', 'slam_toolbox.yaml'
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}],
    )

    configure = TimerAction(period=5.0, actions=[
        ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'configure'],
            output='screen',
        )
    ])
    activate = TimerAction(period=9.0, actions=[
        ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'activate'],
            output='screen',
        )
    ])

    return LaunchDescription([slam_node, configure, activate])
