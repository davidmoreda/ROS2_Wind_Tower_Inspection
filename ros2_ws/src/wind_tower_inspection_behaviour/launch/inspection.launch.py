from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    behaviour_share = FindPackageShare('wind_tower_inspection_behaviour')

    inspection_params = PathJoinSubstitution([
        behaviour_share,
        'config',
        'inspection_params.yaml',
    ])
    stability_params = PathJoinSubstitution([
        behaviour_share,
        'config',
        'stability_monitor.yaml',
    ])
    state_machine_params = PathJoinSubstitution([
        behaviour_share,
        'config',
        'state_machine.yaml',
    ])

    use_cylinder_localizer = LaunchConfiguration('use_cylinder_localizer')
    use_ps5 = LaunchConfiguration('use_ps5')
    use_stability_monitor = LaunchConfiguration('use_stability_monitor')
    imu_auto_calibrate_on_start = LaunchConfiguration('imu_auto_calibrate_on_start')
    use_cylindrical_map = LaunchConfiguration('use_cylindrical_map')
    use_state_machine = LaunchConfiguration('use_state_machine')
    state_machine_auto_start = LaunchConfiguration('state_machine_auto_start')
    state_machine_axial_axis = LaunchConfiguration('state_machine_axial_axis')
    state_machine_lane_length_m = LaunchConfiguration('state_machine_lane_length_m')
    state_machine_lane_delta_theta_deg = LaunchConfiguration(
        'state_machine_lane_delta_theta_deg')
    state_machine_axial_speed_mps = LaunchConfiguration('state_machine_axial_speed_mps')
    state_machine_publish_rate_hz = LaunchConfiguration(
        'state_machine_publish_rate_hz')
    state_machine_turner_speed_rad_s = LaunchConfiguration(
        'state_machine_turner_speed_rad_s')
    state_machine_k_axial_heading_p = LaunchConfiguration(
        'state_machine_k_axial_heading_p')
    state_machine_k_axial_heading_i = LaunchConfiguration(
        'state_machine_k_axial_heading_i')
    state_machine_k_axial_lateral_p = LaunchConfiguration(
        'state_machine_k_axial_lateral_p')
    state_machine_k_axial_lateral_i = LaunchConfiguration(
        'state_machine_k_axial_lateral_i')
    state_machine_axial_lateral_sign = LaunchConfiguration(
        'state_machine_axial_lateral_sign')
    state_machine_align_linear_speed_mps = LaunchConfiguration(
        'state_machine_align_linear_speed_mps')
    state_machine_k_align_heading_p = LaunchConfiguration(
        'state_machine_k_align_heading_p')
    state_machine_k_align_lateral_p = LaunchConfiguration(
        'state_machine_k_align_lateral_p')
    state_machine_k_align_lateral_i = LaunchConfiguration(
        'state_machine_k_align_lateral_i')
    state_machine_align_lateral_sign = LaunchConfiguration(
        'state_machine_align_lateral_sign')
    state_machine_align_lock_hold_s = LaunchConfiguration(
        'state_machine_align_lock_hold_s')
    state_machine_use_tangential_indexing = LaunchConfiguration(
        'state_machine_use_tangential_indexing')
    state_machine_rotate_angular_speed_rad_s = LaunchConfiguration(
        'state_machine_rotate_angular_speed_rad_s')
    state_machine_rotate_yaw_tolerance_deg = LaunchConfiguration(
        'state_machine_rotate_yaw_tolerance_deg')
    state_machine_k_rotate_yaw_p = LaunchConfiguration(
        'state_machine_k_rotate_yaw_p')
    state_machine_k_rotate_yaw_i = LaunchConfiguration(
        'state_machine_k_rotate_yaw_i')
    state_machine_tube_radius_m = LaunchConfiguration(
        'state_machine_tube_radius_m')
    state_machine_index_compensation_sign = LaunchConfiguration(
        'state_machine_index_compensation_sign')
    state_machine_max_index_linear_speed_mps = LaunchConfiguration(
        'state_machine_max_index_linear_speed_mps')
    state_machine_k_index_lateral_p = LaunchConfiguration(
        'state_machine_k_index_lateral_p')
    state_machine_k_index_lateral_i = LaunchConfiguration(
        'state_machine_k_index_lateral_i')
    state_machine_require_bottom_lane_lock = LaunchConfiguration(
        'state_machine_require_bottom_lane_lock')
    state_machine_require_safe_to_scan = LaunchConfiguration(
        'state_machine_require_safe_to_scan')
    state_machine_require_safe_to_index = LaunchConfiguration(
        'state_machine_require_safe_to_index')
    ps5_start_button_index = LaunchConfiguration('ps5_start_button_index')
    ps5_stop_button_index = LaunchConfiguration('ps5_stop_button_index')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_cylinder_localizer',
            default_value='true',
            description='Launch LiDAR cylinder fit debug node from wind_tower_bringup.',
        ),
        DeclareLaunchArgument(
            'use_ps5',
            default_value='true',
            description='Launch DualSense evdev driver and PS5 inspection teleop helper.',
        ),
        DeclareLaunchArgument(
            'use_stability_monitor',
            default_value='true',
            description='Launch stability monitor and bottom-lane safety flags.',
        ),
        DeclareLaunchArgument(
            'imu_auto_calibrate_on_start',
            default_value='true',
            description='Use initial IMU samples as bottom-lane reference.',
        ),
        DeclareLaunchArgument(
            'use_cylindrical_map',
            default_value='true',
            description='Launch cylindrical coverage map MVP.',
        ),
        DeclareLaunchArgument(
            'use_state_machine',
            default_value='true',
            description='Launch mission state machine. It only moves if auto_start is true.',
        ),
        DeclareLaunchArgument(
            'state_machine_auto_start',
            default_value='false',
            description='If true, the state machine starts commanding base/turner immediately.',
        ),
        DeclareLaunchArgument(
            'state_machine_axial_axis',
            default_value='x',
            description='Odometry axis used as tube axial coordinate: x or y.',
        ),
        DeclareLaunchArgument(
            'state_machine_lane_length_m',
            default_value='1.0',
            description='Relative distance to travel before indexing the tube.',
        ),
        DeclareLaunchArgument(
            'state_machine_lane_delta_theta_deg',
            default_value='5.0',
            description='Tube angular increment per lane. Small default is for fast simulation tests.',
        ),
        DeclareLaunchArgument(
            'state_machine_axial_speed_mps',
            default_value='0.05',
            description='Base linear speed during AXIAL_SCAN.',
        ),
        DeclareLaunchArgument(
            'state_machine_publish_rate_hz',
            default_value='30.0',
            description='Command/update rate for the mission state machine.',
        ),
        DeclareLaunchArgument(
            'state_machine_turner_speed_rad_s',
            default_value='0.02',
            description='Tube turner angular speed during INDEX_TUBE.',
        ),
        DeclareLaunchArgument(
            'state_machine_k_axial_heading_p',
            default_value='0.8',
            description='P gain for heading hold during AXIAL_SCAN.',
        ),
        DeclareLaunchArgument(
            'state_machine_k_axial_heading_i',
            default_value='0.0',
            description='I gain for heading hold during AXIAL_SCAN.',
        ),
        DeclareLaunchArgument(
            'state_machine_k_axial_lateral_p',
            default_value='0.0',
            description='P gain on IMU lateral angle during AXIAL_SCAN.',
        ),
        DeclareLaunchArgument(
            'state_machine_k_axial_lateral_i',
            default_value='0.0',
            description='I gain on IMU lateral angle during AXIAL_SCAN.',
        ),
        DeclareLaunchArgument(
            'state_machine_axial_lateral_sign',
            default_value='1.0',
            description='Sign applied to IMU lateral correction during AXIAL_SCAN.',
        ),
        DeclareLaunchArgument(
            'state_machine_align_linear_speed_mps',
            default_value='0.02',
            description='Low forward speed used while actively aligning to bottom lane.',
        ),
        DeclareLaunchArgument(
            'state_machine_k_align_heading_p',
            default_value='0.5',
            description='P gain for yaw hold while aligning to bottom lane.',
        ),
        DeclareLaunchArgument(
            'state_machine_k_align_lateral_p',
            default_value='0.8',
            description='P gain on IMU lateral angle while aligning to bottom lane.',
        ),
        DeclareLaunchArgument(
            'state_machine_k_align_lateral_i',
            default_value='0.0',
            description='I gain on IMU lateral angle while aligning to bottom lane.',
        ),
        DeclareLaunchArgument(
            'state_machine_align_lateral_sign',
            default_value='1.0',
            description='Sign applied to IMU lateral correction while aligning to bottom lane.',
        ),
        DeclareLaunchArgument(
            'state_machine_align_lock_hold_s',
            default_value='1.0',
            description='Time that safe_to_scan must remain true before leaving ALIGN_TO_BOTTOM_LANE.',
        ),
        DeclareLaunchArgument(
            'state_machine_use_tangential_indexing',
            default_value='true',
            description='Rotate the base to tangential during tube indexing, then to the next axial yaw.',
        ),
        DeclareLaunchArgument(
            'state_machine_rotate_angular_speed_rad_s',
            default_value='0.30',
            description='Maximum yaw rate for autonomous 90/180 degree base rotations.',
        ),
        DeclareLaunchArgument(
            'state_machine_rotate_yaw_tolerance_deg',
            default_value='5.0',
            description='Yaw tolerance for base rotation targets.',
        ),
        DeclareLaunchArgument(
            'state_machine_k_rotate_yaw_p',
            default_value='1.0',
            description='P gain for autonomous yaw rotations.',
        ),
        DeclareLaunchArgument(
            'state_machine_k_rotate_yaw_i',
            default_value='0.0',
            description='I gain for autonomous yaw rotations.',
        ),
        DeclareLaunchArgument(
            'state_machine_tube_radius_m',
            default_value='3.925',
            description='Tube radius used to convert tube angular speed into surface speed.',
        ),
        DeclareLaunchArgument(
            'state_machine_index_compensation_sign',
            default_value='1.0',
            description='Sign applied to base compensation speed during tube indexing.',
        ),
        DeclareLaunchArgument(
            'state_machine_max_index_linear_speed_mps',
            default_value='0.45',
            description='Maximum base linear speed while compensating tube indexing.',
        ),
        DeclareLaunchArgument(
            'state_machine_k_index_lateral_p',
            default_value='0.0',
            description='P gain on IMU lateral angle during tube-index compensation.',
        ),
        DeclareLaunchArgument(
            'state_machine_k_index_lateral_i',
            default_value='0.0',
            description='I gain on IMU lateral angle during tube-index compensation.',
        ),
        DeclareLaunchArgument(
            'state_machine_require_bottom_lane_lock',
            default_value='true',
            description='Require bottom_lane_locked before autonomous motion.',
        ),
        DeclareLaunchArgument(
            'state_machine_require_safe_to_scan',
            default_value='true',
            description='Require safe_to_scan during AXIAL_SCAN.',
        ),
        DeclareLaunchArgument(
            'state_machine_require_safe_to_index',
            default_value='true',
            description='Require safe_to_index_tube during INDEX_TUBE.',
        ),
        DeclareLaunchArgument(
            'ps5_start_button_index',
            default_value='3',
            description='Joy button index used to send START_AUTO.',
        ),
        DeclareLaunchArgument(
            'ps5_stop_button_index',
            default_value='2',
            description='Joy button index used to send STOP. Default matches the tested Circle button.',
        ),

        Node(
            package='wind_tower_bringup',
            executable='cylinder_localizer',
            name='cylinder_localizer',
            output='screen',
            condition=IfCondition(use_cylinder_localizer),
        ),
        Node(
            package='wind_tower_bringup',
            executable='dualsense_joy',
            name='dualsense_joy',
            output='screen',
            parameters=[{
                'autonomous_stop_button_index': ps5_stop_button_index,
            }],
            condition=IfCondition(use_ps5),
        ),
        Node(
            package='wind_tower_bringup',
            executable='ps5_teleop',
            name='ps5_teleop',
            output='screen',
            parameters=[{
                'button_start_index': ps5_start_button_index,
                'button_stop_index': ps5_stop_button_index,
            }],
            condition=IfCondition(use_ps5),
        ),
        Node(
            package='wind_tower_inspection_behaviour',
            executable='stability_monitor',
            name='stability_monitor',
            output='screen',
            parameters=[
                stability_params,
                {'imu_calibration.auto_calibrate_on_start':
                    imu_auto_calibrate_on_start},
            ],
            condition=IfCondition(use_stability_monitor),
        ),
        Node(
            package='wind_tower_inspection_behaviour',
            executable='cylindrical_map',
            name='cylindrical_map',
            output='screen',
            parameters=[inspection_params],
            condition=IfCondition(use_cylindrical_map),
        ),
        Node(
            package='wind_tower_inspection_behaviour',
            executable='state_machine',
            name='state_machine',
            output='screen',
            parameters=[
                state_machine_params,
                {
                    'auto_start': state_machine_auto_start,
                    'mission.axial_axis': state_machine_axial_axis,
                    'mission.lane_length_m': state_machine_lane_length_m,
                    'mission.lane_delta_theta_deg':
                        state_machine_lane_delta_theta_deg,
                    'mission.use_tangential_indexing':
                        state_machine_use_tangential_indexing,
                    'control.axial_speed_mps': state_machine_axial_speed_mps,
                    'control.publish_rate_hz': state_machine_publish_rate_hz,
                    'control.turner_speed_rad_s': state_machine_turner_speed_rad_s,
                    'control.k_axial_heading_p': state_machine_k_axial_heading_p,
                    'control.k_axial_heading_i': state_machine_k_axial_heading_i,
                    'control.k_axial_lateral_p': state_machine_k_axial_lateral_p,
                    'control.k_axial_lateral_i': state_machine_k_axial_lateral_i,
                    'control.axial_lateral_sign': state_machine_axial_lateral_sign,
                    'control.align_linear_speed_mps':
                        state_machine_align_linear_speed_mps,
                    'control.k_align_heading_p': state_machine_k_align_heading_p,
                    'control.k_align_lateral_p': state_machine_k_align_lateral_p,
                    'control.k_align_lateral_i': state_machine_k_align_lateral_i,
                    'control.align_lateral_sign': state_machine_align_lateral_sign,
                    'control.rotate_angular_speed_rad_s':
                        state_machine_rotate_angular_speed_rad_s,
                    'control.rotate_yaw_tolerance_deg':
                        state_machine_rotate_yaw_tolerance_deg,
                    'control.k_rotate_yaw_p': state_machine_k_rotate_yaw_p,
                    'control.k_rotate_yaw_i': state_machine_k_rotate_yaw_i,
                    'control.tube_radius_m': state_machine_tube_radius_m,
                    'control.index_compensation_sign':
                        state_machine_index_compensation_sign,
                    'control.max_index_linear_speed_mps':
                        state_machine_max_index_linear_speed_mps,
                    'control.k_index_lateral_p': state_machine_k_index_lateral_p,
                    'control.k_index_lateral_i': state_machine_k_index_lateral_i,
                    'safety.require_bottom_lane_lock':
                        state_machine_require_bottom_lane_lock,
                    'safety.require_safe_to_scan_for_axial':
                        state_machine_require_safe_to_scan,
                    'safety.require_safe_to_index_tube':
                        state_machine_require_safe_to_index,
                    'safety.align_lock_hold_s': state_machine_align_lock_hold_s,
                },
            ],
            condition=IfCondition(use_state_machine),
        ),
    ])
