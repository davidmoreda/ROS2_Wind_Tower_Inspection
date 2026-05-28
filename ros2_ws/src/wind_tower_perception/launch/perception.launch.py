"""Launcher for the wind-tower perception layer.

Brings up:

* a static ``world -> odom`` transform derived from the robot spawn pose,
  so detector and mapper can work in absolute world coordinates;
* the ``detector_node`` (HoughCircles by default, YOLO if ``yolo_model_path``
  is provided);
* the ``image_capture_node`` (writes JPEGs + JSON sidecars + detections.ndjson);
* the ``defect_mapper_node`` (projects detections to cylindrical (x, theta));
* optionally the ``synthetic_capture_node`` when ``use_synthetic_capture:=true``
  for offline dataset generation.

Typical use (runtime inspection)::

    ros2 launch wind_tower_perception perception.launch.py

To enable a trained YOLO model::

    ros2 launch wind_tower_perception perception.launch.py \
        backend:=yolo yolo_model_path:=/path/to/best.pt

To collect a synthetic dataset (assumes ``generate_synthetic_world`` was run
and the synthetic SDF is being simulated)::

    ros2 launch wind_tower_perception perception.launch.py \
        use_synthetic_capture:=true \
        ground_truth_path:=~/wind_tower_synthetic/defects_ground_truth.yaml
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('wind_tower_perception')

    perception_params = PathJoinSubstitution([
        share, 'config', 'perception_params.yaml',
    ])
    synthetic_params = PathJoinSubstitution([
        share, 'config', 'synthetic_dataset.yaml',
    ])

    backend = LaunchConfiguration('backend')
    yolo_model_path = LaunchConfiguration('yolo_model_path')
    use_detector = LaunchConfiguration('use_detector')
    use_image_capture = LaunchConfiguration('use_image_capture')
    use_defect_mapper = LaunchConfiguration('use_defect_mapper')
    use_synthetic_capture = LaunchConfiguration('use_synthetic_capture')
    publish_static_world_tf = LaunchConfiguration('publish_static_world_tf')
    ground_truth_path = LaunchConfiguration('ground_truth_path')
    dataset_output_dir = LaunchConfiguration('dataset_output_dir')
    synthetic_save_empty_frames = LaunchConfiguration(
        'synthetic_save_empty_frames')
    synthetic_write_label_files = LaunchConfiguration(
        'synthetic_write_label_files')
    synthetic_camera_frame = LaunchConfiguration('synthetic_camera_frame')
    synthetic_tf_fallback_camera_frame = LaunchConfiguration(
        'synthetic_tf_fallback_camera_frame')
    synthetic_allow_intrinsics_fallback = LaunchConfiguration(
        'synthetic_allow_intrinsics_fallback')

    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_roll = LaunchConfiguration('spawn_roll')
    spawn_pitch = LaunchConfiguration('spawn_pitch')
    spawn_yaw = LaunchConfiguration('spawn_yaw')

    declared = [
        DeclareLaunchArgument(
            'backend', default_value='yolo',
            description="Detector backend: 'hough' or 'yolo'.",
        ),
        DeclareLaunchArgument(
            'yolo_model_path',
            default_value=os.path.expanduser(
                '~/ROS2_Wind_Tower_Inspection/ros2_ws/models/best.pt'),
            description='Path to a .pt YOLO checkpoint when backend=yolo.',
        ),
        DeclareLaunchArgument(
            'use_detector', default_value='true',
            description='Run the defect detector node.',
        ),
        DeclareLaunchArgument(
            'use_image_capture', default_value='true',
            description='Run the image capture node.',
        ),
        DeclareLaunchArgument(
            'use_defect_mapper', default_value='true',
            description='Run the defect mapper node.',
        ),
        DeclareLaunchArgument(
            'use_synthetic_capture', default_value='false',
            description='Run the synthetic dataset capture node.',
        ),
        DeclareLaunchArgument(
            'publish_static_world_tf', default_value='true',
            description=(
                'Publish a static world->odom transform from the spawn pose. '
                'Required for cylindrical projection in world coordinates.'
            ),
        ),
        DeclareLaunchArgument(
            'ground_truth_path', default_value='',
            description='Path to the defects ground-truth YAML (synthetic capture).',
        ),
        DeclareLaunchArgument(
            'dataset_output_dir',
            default_value=(
                '~/ROS2_Wind_Tower_Inspection/ros2_ws/datasets/'
                'wind_tower_dataset'
            ),
            description='Output directory for the YOLO dataset.',
        ),
        DeclareLaunchArgument(
            'synthetic_save_empty_frames',
            default_value='false',
            description=(
                'When true, synthetic_capture writes frames even if no defect '
                'label is visible. Useful to debug camera and TF.'
            ),
        ),
        DeclareLaunchArgument(
            'synthetic_write_label_files',
            default_value='true',
            description=(
                'When false, synthetic_capture saves only images for manual '
                'labelling tools such as Roboflow.'
            ),
        ),
        DeclareLaunchArgument(
            'synthetic_camera_frame',
            default_value='auto',
            description=(
                'TF frame used by synthetic_capture for projection. Use auto '
                'to read the frame from the image header.'
            ),
        ),
        DeclareLaunchArgument(
            'synthetic_tf_fallback_camera_frame',
            default_value='inspection_camera_optical_frame',
            description=(
                'TF frame used when the image header contains a Gazebo sensor '
                'path instead of a ROS TF frame.'
            ),
        ),
        DeclareLaunchArgument(
            'synthetic_allow_intrinsics_fallback',
            default_value='true',
            description=(
                'Use camera intrinsics derived from image size and the URDF '
                'horizontal FOV if /inspection/camera/camera_info is missing.'
            ),
        ),
        # Defaults sincronizados con simulation.launch.py: spawn dentro de
        # la sala de carga (Y=25), mirando al sur hacia la rampa/tubo.
        DeclareLaunchArgument('spawn_x', default_value='0.0',
                              description='Robot spawn X (world). Matches simulation.launch.'),
        DeclareLaunchArgument('spawn_y', default_value='25.0',
                              description='Robot spawn Y (world). Matches simulation.launch.'),
        DeclareLaunchArgument('spawn_z', default_value='0.3',
                              description='Robot spawn Z (world). Matches simulation.launch.'),
        DeclareLaunchArgument('spawn_roll', default_value='0.0',
                              description='Robot spawn roll (rad).'),
        DeclareLaunchArgument('spawn_pitch', default_value='0.0',
                              description='Robot spawn pitch (rad).'),
        DeclareLaunchArgument('spawn_yaw', default_value='3.14159',
                              description='Robot spawn yaw (rad). π = mirando al sur hacia la rampa.'),
    ]

    world_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_odom_static_tf',
        output='screen',
        arguments=[
            '--x', spawn_x, '--y', spawn_y, '--z', spawn_z,
            '--roll', spawn_roll, '--pitch', spawn_pitch, '--yaw', spawn_yaw,
            '--frame-id', 'world',
            '--child-frame-id', 'odom',
        ],
        condition=IfCondition(publish_static_world_tf),
    )

    detector = Node(
        package='wind_tower_perception',
        executable='detector',
        name='defect_detector',
        output='screen',
        parameters=[
            perception_params,
            {
                'backend': backend,
                'yolo.model_path': yolo_model_path,
            },
        ],
        condition=IfCondition(use_detector),
    )

    image_capture = Node(
        package='wind_tower_perception',
        executable='image_capture',
        name='image_capture',
        output='screen',
        parameters=[perception_params],
        condition=IfCondition(use_image_capture),
    )

    defect_mapper = Node(
        package='wind_tower_perception',
        executable='defect_mapper',
        name='defect_mapper',
        output='screen',
        parameters=[perception_params],
        condition=IfCondition(use_defect_mapper),
    )

    synthetic_capture = Node(
        package='wind_tower_perception',
        executable='synthetic_capture',
        name='synthetic_capture',
        output='screen',
        parameters=[
            synthetic_params,
            {
                'ground_truth_path': ground_truth_path,
                'output_dir': dataset_output_dir,
                'save_empty_frames': synthetic_save_empty_frames,
                'write_label_files': synthetic_write_label_files,
                'camera_frame': synthetic_camera_frame,
                'tf_fallback_camera_frame':
                    synthetic_tf_fallback_camera_frame,
                'allow_intrinsics_fallback':
                    synthetic_allow_intrinsics_fallback,
            },
        ],
        condition=IfCondition(use_synthetic_capture),
    )

    return LaunchDescription(declared + [
        world_to_odom_tf,
        detector,
        image_capture,
        defect_mapper,
        synthetic_capture,
    ])
