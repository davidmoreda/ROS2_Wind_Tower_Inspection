"""
Navegación autónoma con mapa ya guardado (AMCL localization).

Este launch se usa DESPUÉS de haber guardado el mapa con SLAM.
Reemplaza slam_toolbox por map_server + AMCL para localización.

Uso:
  # Terminal 1 — simulación
  ros2 launch wind_tower_bringup simulation.launch.py

  # Terminal 2 — navegación con mapa
  ros2 launch wind_tower_bringup navigation_amcl.launch.py map:=$HOME/maps/wind_tower.yaml

  # Opcional — sin RViz:
  ros2 launch wind_tower_bringup navigation_amcl.launch.py map:=$HOME/maps/wind_tower.yaml rviz:=false

  # Ir a estación de carga:
  ros2 run wind_tower_bringup mission_navigator --ros-args -p target:=charging_station

  # Ir a zona de mantenimiento:
  ros2 run wind_tower_bringup mission_navigator --ros-args -p target:=maintenance_station

Arquitectura TF:
  map → odom        (publicado directamente por AMCL con tf_broadcast:true)
  odom → base_link  (publicado por Clearpath EKF1)

  AMCL tiene tf_broadcast:true → publica map→odom directamente en cada ciclo.
  El EKF2 (ekf_map_node) ha sido eliminado: causaba que las correcciones grandes
  de pose (2D Pose Estimate desde RViz) fueran rechazadas por el filtro Kalman
  (pose0_rejection_threshold), haciendo que Nav2 planificara desde la posición
  antigua y el robot navegara en la dirección incorrecta.

IMPORTANTE — Pose inicial:
  Al arrancar, el robot puede estar desorientado en el mapa.
  Usa RViz → "2D Pose Estimate" para darle la posición real al inicio.
  Con tf_broadcast:true AMCL actualiza map→odom inmediatamente.
  AMCL convergerá en 2-3 movimientos.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory('wind_tower_bringup')

    # Sube el límite de participants de CycloneDDS (default ~64 → 240).
    # Sin esto los nodos Nav2 mueren con SIGABRT por slots agotados al
    # haber tantos nodos vivos (Clearpath sim + AMCL + Nav2 stack).
    set_cyclonedds_uri = SetEnvironmentVariable(
        name='CYCLONEDDS_URI',
        value='file://' + os.path.join(
            os.path.expanduser('~'),
            'ROS2_Wind_Tower_Inspection', 'tools', 'cyclonedds.xml',
        ),
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz     = LaunchConfiguration('rviz')
    map_yaml     = LaunchConfiguration('map')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Usar reloj de simulación',
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Lanzar RViz',
    )
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(os.path.expanduser('~'), 'maps', 'wind_tower.yaml'),
        description='Ruta al fichero .yaml del mapa guardado',
    )

    nav2_params  = os.path.join(pkg_bringup, 'config', 'nav2_params.yaml')
    amcl_params  = os.path.join(pkg_bringup, 'config', 'amcl_params.yaml')
    nav2_rviz    = os.path.join(pkg_bringup, 'config', 'navigation.rviz')
    nav_to_pose_bt = os.path.join(
        pkg_bringup,
        'behavior_trees',
        'navigate_to_pose_w_replanning_and_recovery.xml',
    )
    nav_through_poses_bt = os.path.join(
        pkg_bringup,
        'behavior_trees',
        'navigate_through_poses_w_replanning_and_recovery.xml',
    )

    remappings_tf = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # ── Mando PS5 ─────────────────────────────────────────────────────────────
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

    # ── Percepción: pointcloud → laserscan → bridge QoS ───────────────────────
    # Delay 15s para que el reloj de Gazebo se estabilice
    perception = TimerAction(
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
            # Filtro de suelo consciente de pendiente: /velodyne_points →
            # /obstacle_points (solo lo que sobresale del suelo de cada celda).
            # Quita la rampa (la trata como suelo) pero conserva objetos bajos y
            # paredes. Lo consume el obstacle_layer de los costmaps en 360°.
            Node(
                package='wind_tower_bringup',
                executable='obstacle_cloud_filter',
                name='obstacle_cloud_filter',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'input_topic': '/velodyne_points',
                    'output_topic': '/obstacle_points',
                    'cell_size': 0.20,
                    'ground_threshold': 0.12,
                    'min_range': 0.4,
                    'max_range': 10.0,
                    'max_obstacle_height': 2.0,
                    # Recinto del tubo (dentro de las barreras x=±5): descarta
                    # los ~80 defectos esféricos + superficie del tubo del
                    # costmap. Las barreras quedan fuera → siguen marcándose.
                    'exclude_zone_enabled': True,
                    'exclude_frame': 'map',
                    'exclude_xmin': -4.6,
                    'exclude_xmax': 4.6,
                    'exclude_ymin': -14.0,
                    'exclude_ymax': 14.0,
                }],
            ),
        ],
    )

    # ── map_server + AMCL ─────────────────────────────────────────────────────
    # Delay 17s (2s tras percepción para que /scan esté disponible)
    # AMCL publica map→odom directamente (tf_broadcast:true).
    # El EKF2 ha sido eliminado: causaba que el "2D Pose Estimate" de RViz no se
    # propagara a Nav2 porque el filtro Kalman rechazaba correcciones grandes.
    localization = TimerAction(
        period=17.0,
        actions=[
            # Sirve el mapa guardado
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'yaml_filename': map_yaml},
                ],
                remappings=remappings_tf,
            ),
            # AMCL localiza el robot en el mapa (tf_broadcast:true → publica map→odom)
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=[amcl_params, {'use_sim_time': use_sim_time}],
                remappings=remappings_tf,
            ),
            # Lifecycle manager para map_server y amcl
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': True,
                    'node_names': ['map_server', 'amcl'],
                }],
            ),
        ],
    )

    # ── Nav2 navigation — nodos primero, lifecycle_manager después ────────────
    # Delay 22s para los nodos: 5s tras AMCL para que esté convergido.
    # IMPORTANTE: el lifecycle_manager va en SU PROPIO TimerAction (30s) para
    # que los 7 nodos tengan 8s para registrar sus servicios get_state ANTES de
    # que el manager intente activarlos. Sin esta separación, el manager puede
    # quedar bloqueado en "Waiting for service smoother_server/get_state" si
    # llama antes de que el smoother_server esté listo (race condition que
    # bloquea TODA la cadena Nav2 → navigate_to_pose nunca aparece).
    navigation_nodes = TimerAction(
        period=22.0,
        actions=[
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                parameters=[nav2_params, {'use_sim_time': use_sim_time}],
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
                remappings=remappings_tf + [('cmd_vel', '/robot/cmd_vel')],
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[
                    nav2_params,
                    {
                        'use_sim_time': use_sim_time,
                        'default_nav_to_pose_bt_xml': nav_to_pose_bt,
                        'default_nav_through_poses_bt_xml': nav_through_poses_bt,
                    },
                ],
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
                remappings=remappings_tf + [('cmd_vel_smoothed', '/robot/cmd_vel')],
            ),
        ],
    )

    # Lifecycle manager 8s después de los nodos → tienen tiempo de registrar
    # sus servicios get_state antes de que el manager los busque.
    navigation_lifecycle = TimerAction(
        period=30.0,
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': True,
                    # bond_timeout 0.0 → no asume "node dead" si tarda en responder
                    # al heartbeat durante carga inicial (evita re-activación en bucle).
                    'bond_timeout': 0.0,
                    'attempt_respawn_reconnection': False,
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

    # ── RViz ──────────────────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', nav2_rviz],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_rviz),
    )


    # ── Behaviour nodes (mission controller + voice) ───────────────────────
    # Se lanzan 35 s después para que Nav2 lifecycle esté activo antes de que
    # mission_controller intente conectarse al action server de Nav2.
    behaviour_nodes = TimerAction(
        period=35.0,
        actions=[
            Node(
                package='wind_tower_inspection_behaviour',
                executable='mission_controller',
                name='mission_controller',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            ),
            Node(
                package='wind_tower_inspection_behaviour',
                executable='voice_command_node',
                name='voice_command_node',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            ),
        ],
    )

    return LaunchDescription([
        set_cyclonedds_uri,
        declare_use_sim_time,
        declare_rviz,
        declare_map,
        dualsense_joy,
        ps5_teleop,
        perception,
        localization,
        navigation_nodes,
        navigation_lifecycle,
        behaviour_nodes,
        rviz,
    ])
