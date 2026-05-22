"""Auto-capture dataset node.

Teleports the robot to controlled positions near each defect in the ground
truth YAML and waits while ``synthetic_capture_node`` autolabels the frames.

Strategy
--------
For every defect the node computes N robot positions at different axial offsets
along the tube (Y axis), covering front/side/angled views of the same defect.
The robot is teleported using the Gazebo ``set_pose`` service; no manual driving
is needed.  ``synthetic_capture_node`` runs in parallel and saves+labels every
frame where a defect projects visibly into the camera.

Typical result with 20 defects × 5 offsets × ~3 frames/position:
    ~300 labelled images, all defect classes represented.

Usage
-----
    ros2 run wind_tower_perception auto_dataset \\
        --ros-args \\
        -p ground_truth_path:=<path/to/defects_ground_truth.yaml>
"""

import math
import os
import subprocess
import time

import rclpy
import yaml
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy._rclpy_pybind11 import RCLError


class AutoDatasetNode(Node):
    """Teleport the robot near each defect and collect labelled frames."""

    def __init__(self):
        super().__init__('auto_dataset')

        self.declare_parameter('ground_truth_path', '')
        self.declare_parameter('world_name', 'wind_tower_world')
        # Clearpath names the Gazebo model as "<namespace>/robot".
        # The project robot.yaml uses namespace "robot", so the model is
        # "robot/robot" rather than plain "robot".
        self.declare_parameter('robot_name', 'robot/robot')
        # Robot stays at the bottom of the tube: X=0, Z=0.3, yaw=π/2 (+Y)
        self.declare_parameter('robot_x', 0.0)
        self.declare_parameter('robot_z', 0.3)
        self.declare_parameter('robot_yaw', 1.5708)
        # Axial offsets (metres) applied around each defect's Y coordinate.
        # Five offsets give front/oblique/side views of each defect.
        self.declare_parameter('axial_offsets_m', [-2.0, -1.0, 0.0, 1.0, 2.0])
        # Seconds to dwell at each position (≥ min_frame_period_s in capture node)
        self.declare_parameter('dwell_s', 2.0)
        self.declare_parameter('gz_timeout_ms', 2000)
        # Keep the robot inside the usable tube length
        self.declare_parameter('tube_y_min', -14.5)
        self.declare_parameter('tube_y_max', 14.5)

        gt_path = os.path.expanduser(
            str(self.get_parameter('ground_truth_path').value))
        if not gt_path:
            raise RuntimeError("Parameter 'ground_truth_path' is required.")
        if not os.path.isfile(gt_path):
            raise RuntimeError(f'Ground truth file not found: {gt_path}')

        self._world = str(self.get_parameter('world_name').value)
        self._robot = str(self.get_parameter('robot_name').value)
        self._rx = float(self.get_parameter('robot_x').value)
        self._rz = float(self.get_parameter('robot_z').value)
        self._yaw = float(self.get_parameter('robot_yaw').value)
        self._offsets: list = list(self.get_parameter('axial_offsets_m').value)
        self._dwell = float(self.get_parameter('dwell_s').value)
        self._gz_timeout = int(self.get_parameter('gz_timeout_ms').value)
        self._y_min = float(self.get_parameter('tube_y_min').value)
        self._y_max = float(self.get_parameter('tube_y_max').value)

        with open(gt_path, 'r', encoding='utf-8') as fh:
            payload = yaml.safe_load(fh)
        self._defects = payload.get('defects', [])
        if not self._defects:
            raise RuntimeError(f'No defects found in {gt_path}')

        n_positions = len(self._defects) * len(self._offsets)
        self.get_logger().info(
            f'Auto-dataset: {len(self._defects)} defects × '
            f'{len(self._offsets)} offsets = {n_positions} positions. '
            f'Dwell {self._dwell}s each → ~{n_positions * self._dwell / 60:.1f} min total.'
        )

        self._started = False
        # Give synthetic_capture_node 2 s to start up before moving
        self.create_timer(2.0, self._run)

    # ------------------------------------------------------------------ runner

    def _run(self):
        if self._started:
            return
        self._started = True

        total = len(self._defects) * len(self._offsets)
        count = 0

        for defect in self._defects:
            d_id = defect['defect_id']
            d_class = defect['class_name']
            d_y = float(defect['world_y'])

            for offset in self._offsets:
                robot_y = max(self._y_min, min(self._y_max, d_y + offset))
                count += 1
                self.get_logger().info(
                    f'[{count}/{total}] defect {d_id} ({d_class})'
                    f'  y_defect={d_y:.2f}  offset={offset:+.1f}'
                    f'  → robot_y={robot_y:.2f}'
                )
                self._teleport(self._rx, robot_y, self._rz, self._yaw)
                time.sleep(self._dwell)

        self.get_logger().info(
            f'Auto-capture complete — {total} positions visited.'
        )
        rclpy.shutdown()

    # ---------------------------------------------------------------- teleport

    def _teleport(self, x: float, y: float, z: float, yaw: float) -> None:
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        req = (
            f'name: "{self._robot}" '
            f'position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}} '
            f'orientation: {{x: 0.0, y: 0.0, z: {qz:.6f}, w: {qw:.6f}}}'
        )
        cmd = [
            'gz', 'service',
            '-s', f'/world/{self._world}/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', str(self._gz_timeout),
            '--req', req,
        ]
        try:
            result = subprocess.run(
                cmd, capture_output=True, text=True, timeout=5.0)
            if result.returncode != 0:
                self.get_logger().warn(
                    f'set_pose failed (code {result.returncode}): '
                    f'{result.stderr.strip()}'
                )
        except subprocess.TimeoutExpired:
            self.get_logger().warn('set_pose timed out — Gazebo may be busy.')


def main(args=None):
    rclpy.init(args=args)
    node = AutoDatasetNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
