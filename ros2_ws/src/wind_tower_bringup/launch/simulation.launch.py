"""
Wind Tower Inspection — launcher principal.

Arranca Gazebo con el mundo personalizado (nave + tramo de torre),
spawna el robot Husky+UR5e dentro del tubo y lanza RViz.

Uso:
  ros2 launch wind_tower_bringup simulation.launch.py
  ros2 launch wind_tower_bringup simulation.launch.py rviz:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument(
        'rviz',
        default_value='true',
        choices=['true', 'false'],
        description='Lanzar RViz',
    ),
    DeclareLaunchArgument(
        'setup_path',
        default_value=[EnvironmentVariable('HOME'), '/clearpath/'],
        description='Ruta al directorio con robot.yaml de Clearpath',
    ),
    # Robot spawn dentro del tubo (cerca del extremo sur, eje Y)
    DeclareLaunchArgument('x',   default_value='0.0',   description='Spawn X del robot'),
    DeclareLaunchArgument('y',   default_value='-10.0', description='Spawn Y del robot'),
    DeclareLaunchArgument('z',   default_value='0.3',   description='Spawn Z del robot'),
    DeclareLaunchArgument('yaw', default_value='1.5708',description='Spawn yaw (rad)'),
]


def generate_launch_description():
    pkg_clearpath_gz   = get_package_share_directory('clearpath_gz')
    pkg_ros_gz_sim     = get_package_share_directory('ros_gz_sim')
    pkg_wind_sim       = get_package_share_directory('wind_tower_simulation')
    pkg_wind_desc      = get_package_share_directory('wind_tower_description')

    world_file  = os.path.join(pkg_wind_sim,  'worlds',  'wind_tower_world.sdf')
    gui_config  = os.path.join(pkg_clearpath_gz, 'config', 'gui.config')
    meshes_path = os.path.join(pkg_wind_desc, 'meshes')

    # Fix para módulo apt de Python
    set_pythonpath = SetEnvironmentVariable(
        name='PYTHONPATH',
        value='/usr/lib/python3/dist-packages:' + os.environ.get('PYTHONPATH', ''),
    )

    # GZ_SIM_RESOURCE_PATH: clearpath worlds/meshes + nuestros meshes + todos los share
    packages_paths = [
        os.path.join(p, 'share')
        for p in os.environ.get('AMENT_PREFIX_PATH', '').split(':')
        if p
    ]
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([
            os.path.join(pkg_clearpath_gz, 'worlds'),
            os.path.join(pkg_clearpath_gz, 'meshes'),
            meshes_path,   # para que Gazebo encuentre TRAMO_TORRE.STL
        ] + packages_paths),
    )

    # Gazebo con nuestro mundo (ruta absoluta, sin restricción de choices)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments=[
            ('gz_args', f'{world_file} -r -v 4 --gui-config {gui_config}'),
        ],
    )

    # Bridge de reloj ROS ↔ Gazebo
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    # Spawn del robot + controladores Clearpath
    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_clearpath_gz, 'launch', 'robot_spawn.launch.py')
        ),
        launch_arguments={
            'world':      'wind_tower_world',
            'rviz':       LaunchConfiguration('rviz'),
            'setup_path': LaunchConfiguration('setup_path'),
            'x':          LaunchConfiguration('x'),
            'y':          LaunchConfiguration('y'),
            'z':          LaunchConfiguration('z'),
            'yaw':        LaunchConfiguration('yaw'),
            'use_sim_time': 'true',
        }.items(),
    )

    return LaunchDescription(ARGUMENTS + [
        set_pythonpath,
        set_gz_resource_path,
        gz_sim,
        clock_bridge,
        robot_spawn,
    ])
