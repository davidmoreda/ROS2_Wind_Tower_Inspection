"""
Wind Tower Inspection — main simulation launcher.

Usage:
  ros2 launch wind_tower_bringup simulation.launch.py
  ros2 launch wind_tower_bringup simulation.launch.py world:=warehouse rviz:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution


ARGUMENTS = [
    DeclareLaunchArgument(
        'world',
        default_value='warehouse',
        description='Gazebo world to load',
    ),
    DeclareLaunchArgument(
        'rviz',
        default_value='true',
        choices=['true', 'false'],
        description='Launch RViz alongside Gazebo',
    ),
    DeclareLaunchArgument(
        'setup_path',
        default_value=[EnvironmentVariable('HOME'), '/clearpath/'],
        description='Path to clearpath robot.yaml directory',
    ),
    DeclareLaunchArgument('x', default_value='0.0', description='Robot spawn X'),
    DeclareLaunchArgument('y', default_value='0.0', description='Robot spawn Y'),
    DeclareLaunchArgument('z', default_value='0.3', description='Robot spawn Z'),
    DeclareLaunchArgument('yaw', default_value='0.0', description='Robot spawn yaw'),
]


def generate_launch_description():
    pkg_clearpath_gz = get_package_share_directory('clearpath_gz')

    # Make apt Python module visible (needed by Clearpath config generators)
    set_pythonpath = SetEnvironmentVariable(
        name='PYTHONPATH',
        value='/usr/lib/python3/dist-packages:' + os.environ.get('PYTHONPATH', ''),
    )

    clearpath_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_clearpath_gz, 'launch', 'simulation.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'rviz': LaunchConfiguration('rviz'),
            'setup_path': LaunchConfiguration('setup_path'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'yaw': LaunchConfiguration('yaw'),
            'use_sim_time': 'true',
        }.items(),
    )

    return LaunchDescription(ARGUMENTS + [set_pythonpath, clearpath_sim])
