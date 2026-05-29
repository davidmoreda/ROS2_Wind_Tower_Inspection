from setuptools import find_packages, setup

package_name = 'wind_tower_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/simulation.launch.py',
            'launch/slam.launch.py',
            'launch/navigation.launch.py',
            'launch/navigation_amcl.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/robot.yaml',
            'config/pointcloud_to_laserscan.yaml',
            'config/slam_params.yaml',
            'config/ekf_map.yaml',
            'config/nav2_params.yaml',
            'config/amcl_params.yaml',
            'config/waypoints.yaml',
            'config/slam.rviz',
            'config/navigation.rviz',
        ]),
        ('share/' + package_name + '/behavior_trees', [
            'behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml',
            'behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dmore',
    maintainer_email='dmoreda29@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ps5_teleop = wind_tower_bringup.ps5_teleop:main',
            'dualsense_joy = wind_tower_bringup.dualsense_joy:main',
            'tf_static_relay = wind_tower_bringup.tf_static_relay:main',
            'turner_node = wind_tower_bringup.turner_node:main',
            'scan_qos_bridge = wind_tower_bringup.scan_qos_bridge:main',
            'obstacle_cloud_filter = wind_tower_bringup.obstacle_cloud_filter:main',
            'mission_navigator = wind_tower_bringup.mission_navigator:main',
        ],
    },
)
