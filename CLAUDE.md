# ROS 2 Wind Tower Inspection — Project Instructions

This is a ROS 2 robotics project. Apply `ros2-engineering:ros2-engineering-skills` guidelines automatically for every task in this conversation without requiring explicit invocation.

## Stack
- ROS 2 Jazzy — Ubuntu 24.04 (WSL2)
- Robot: Clearpath Husky A200 + UR5e arm + Velodyne VLP-16
- Simulator: Gazebo Sim 8
- Navigation: Nav2 (AMCL + RPP)
- Localization: robot_localization (EKF, Clearpath stack)
- Perception: pointcloud_to_laserscan, LiDAR, YOLOv8
- Workspace: `/home/dmore/ROS2_Wind_Tower_Inspection/ros2_ws`

## Key packages
- `wind_tower_bringup` — launch files, config (Nav2, AMCL, EKF, sensors)
- `wind_tower_description` — URDF/xacro
- `wind_tower_simulation` — SDF worlds (ramps, doors)
- `wind_tower_inspection_behaviour` — behaviours (en desarrollo)
- `wind_tower_perception` — detección defectos YOLOv8

## Clearpath namespace & topics
- Robot namespace: `/robot`
- Odometría EKF: `/robot/platform/odom/filtered`
- IMU: `/robot/sensors/imu_0/data`
- LiDAR pointcloud: `/robot/sensors/lidar3d_0/scan/points` → bridge → `/velodyne_points`
- Velocidad: `/robot/cmd_vel` (`geometry_msgs/TwistStamped`)
- TF relay: `/robot/tf` → `/tf` (frame IDs sin prefijo: `base_link`, `odom`, `map`)
- TF static relay: `/robot/tf_static` → `/tf_static` (TRANSIENT_LOCAL)
- Robot description relay: `/robot/robot_description` → `/robot_description`

## Working conventions
- Do not run `sudo`, `colcon build`, or commands that modify the system.
- Give direct, concise answers.
- Current active branch: `nav2-control`
