"""
RTAB-Map — SLAM 3D con LiDAR Velodyne VLP-16.
Odometría: ICP sobre nubes LiDAR (mejor que ruedas en pared curva).
Loop closure DESACTIVADO: cilindro simétrico → falsos positivos inevitables.
Mapear solo axialmente (a lo largo del eje del tubo).
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # ICP odometry: nube LiDAR → estimación incremental de movimiento
    # Funciona bien para movimiento axial; NO rotar dentro del cilindro
    icp_odometry = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        output='screen',
        parameters=[{
            'use_sim_time':                  True,
            'frame_id':                      'base_link',
            'odom_frame_id':                 'odom',
            'publish_tf':                    True,
            'Icp/PM':                        'false',
            'Icp/PointToPlane':              'false',
            'Icp/Iterations':                '30',
            'Icp/MaxCorrespondenceDistance': '1.0',
            'Icp/VoxelSize':                 '0.1',
            'Icp/Epsilon':                   '0.001',
            'Icp/MaxTranslation':            '0.5',   # limita saltos bruscos
            'Icp/MaxRotation':               '0.5',   # ~28° máx entre scans
            'Odom/GuessMotion':              'true',
            'OdomF2M/MaxSize':               '30000',
        }],
        remappings=[('scan_cloud', '/velodyne_points')],
    )

    # RTAB-Map: mapeo puro sin loop closure
    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time':                   True,
            'frame_id':                       'base_link',
            'odom_frame_id':                  'odom',
            'subscribe_depth':                False,
            'subscribe_rgb':                  False,
            'subscribe_scan_cloud':           True,
            'subscribe_odom':                 True,
            # Sin loop closure (cilindro simétrico)
            'RGBD/NeighborLinkRefining':      'false',
            'RGBD/ProximityBySpace':          'false',
            'RGBD/ProximityByTime':           'false',
            'RGBD/LinearUpdate':              '0.1',
            'RGBD/AngularUpdate':             '0.05',
            'Mem/IncrementalMemory':          'true',
            'Mem/InitWMWithAllNodes':         'false',
            'Reg/Strategy':                   '1',
            'Reg/Force3DoF':                  'false',
            'Icp/PM':                         'false',
            'Icp/PointToPlane':               'false',
            'Icp/MaxCorrespondenceDistance':  '1.0',
            'Icp/VoxelSize':                  '0.1',
            'Grid/3D':                        'true',
            'Grid/RangeMax':                  '8.0',
            'RGBD/OptimizeMaxError':          '0.0',
        }],
        remappings=[
            ('scan_cloud', '/velodyne_points'),
            ('odom',       '/odom'),
        ],
    )

    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('scan_cloud', '/velodyne_points'),
            ('odom',       '/odom'),
        ],
    )

    return LaunchDescription([
        icp_odometry,
        rtabmap,
        rtabmap_viz,
    ])
