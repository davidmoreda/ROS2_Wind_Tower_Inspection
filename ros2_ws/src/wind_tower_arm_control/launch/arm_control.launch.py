"""Full UR5e inspection-arm bring-up.

Launches MoveIt (``move_group`` + optional RViz, via ``move_group.launch.py``)
and the ``arm_inspection`` behaviour node together. This is the entry point to
use during an autonomous mission; ``move_group.launch.py`` on its own is for
MoveIt-only / RViz planning tests.

Usage::

    # simulation must already be running
    ros2 launch wind_tower_bringup simulation.launch.py
    # then:
    ros2 launch wind_tower_arm_control arm_control.launch.py
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('wind_tower_arm_control')

    move_group_launch_path = PathJoinSubstitution([
        share, 'launch', 'move_group.launch.py'])
    arm_params = PathJoinSubstitution([
        share, 'config', 'arm_inspection_params.yaml'])

    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    setup_path = LaunchConfiguration('setup_path')
    use_arm_node = LaunchConfiguration('use_arm_node')

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(move_group_launch_path),
        launch_arguments={
            'use_rviz': use_rviz,
            'use_sim_time': use_sim_time,
            'setup_path': setup_path,
        }.items(),
    )

    arm_inspection = Node(
        package='wind_tower_arm_control',
        executable='arm_inspection',
        name='arm_inspection',
        output='screen',
        parameters=[arm_params, {'use_sim_time': use_sim_time}],
        condition=IfCondition(use_arm_node),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Launch RViz with the MoveIt MotionPlanning panel.'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the Gazebo /clock.'),
        DeclareLaunchArgument(
            'setup_path',
            default_value=os.path.join(
                os.environ.get('HOME', '~'), 'clearpath'),
            description='Clearpath setup dir with robot.urdf.xacro + robot.srdf.'),
        DeclareLaunchArgument(
            'use_arm_node', default_value='true',
            description='Also launch the arm_inspection behaviour node.'),
        move_group,
        arm_inspection,
    ])
