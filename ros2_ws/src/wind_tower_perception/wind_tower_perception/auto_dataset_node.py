"""Auto-capture dataset node.

For every defect in the ground-truth YAML the node:
  1. Pauses Gazebo physics.
  2. Teleports the robot to a position near the defect's axial coordinate.
  3. Resumes physics and waits for the robot to settle on the tube floor.
  4. Calls MoveIt compute_ik to aim the inspection camera at the defect.
  5. Executes the arm trajectory and waits for it to complete.
  6. Dwells at that position so ``synthetic_capture_node`` can autolabel frames.

The capture loop runs in a background thread so the ROS executor (TF, joint
states, service callbacks) keeps spinning normally in the main thread.

Usage
-----
    ros2 run wind_tower_perception auto_dataset \\
        --ros-args \\
        -p ground_truth_path:=<path/to/defects_ground_truth.yaml>
"""

import math
import os
import subprocess
import threading
import time

import numpy as np
import rclpy
import rclpy.duration
import rclpy.time
import yaml
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy._rclpy_pybind11 import RCLError
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener, TransformException
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# ─────────────────────────── helpers ────────────────────────────────────────

def _rot_to_quat(R: np.ndarray):
    """3×3 rotation matrix → (x, y, z, w) quaternion."""
    trace = float(R[0, 0] + R[1, 1] + R[2, 2])
    if trace > 0.0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return float(x), float(y), float(z), float(w)


# ─────────────────────────── node ───────────────────────────────────────────

class AutoDatasetNode(Node):
    """Teleport + arm-aim + autolabel for every defect in the ground truth."""

    # joints that belong to the arm (used to filter /joint_states)
    _ARM_JOINTS = [
        'arm_0_shoulder_pan_joint',
        'arm_0_shoulder_lift_joint',
        'arm_0_elbow_joint',
        'arm_0_wrist_1_joint',
        'arm_0_wrist_2_joint',
        'arm_0_wrist_3_joint',
    ]

    def __init__(self):
        super().__init__('auto_dataset')

        # ── parameters ──────────────────────────────────────────────────────
        self.declare_parameter('ground_truth_path', '')
        self.declare_parameter('world_name', 'wind_tower_world')
        # Clearpath robot namespace → Gazebo model is "robot/robot"
        self.declare_parameter('robot_name', 'robot/robot')
        # Robot rests at (0, y, 0.3) on the tube floor; yaw=π/2 faces +Y
        self.declare_parameter('robot_x', 0.0)
        self.declare_parameter('robot_z', 0.3)
        self.declare_parameter('robot_yaw', 1.5708)
        # Axial offsets around each defect's Y: multiple views per defect
        self.declare_parameter('axial_offsets_m', [-1.5, -0.5, 0.5, 1.5])
        # How long to wait at each position for the capture node to grab frames
        self.declare_parameter('dwell_s', 3.0)
        self.declare_parameter('gz_timeout_ms', 2000)
        # After resuming physics, wait this long for the robot to settle
        self.declare_parameter('settle_s', 0.5)
        # Usable tube length along Y
        self.declare_parameter('tube_y_min', -14.5)
        self.declare_parameter('tube_y_max', 14.5)
        # Arm: MoveIt planning group and TF frame of the TCP
        self.declare_parameter('arm_group', 'arm_0')
        self.declare_parameter('arm_tcp_frame', 'arm_0_tool0')
        self.declare_parameter('arm_base_frame', 'arm_0_base_link')
        # Camera is 0.22 m along TCP X; target the camera 0.3–2.0 m from defect
        self.declare_parameter('camera_tcp_offset_m', 0.22)
        self.declare_parameter('camera_target_dist_m', 0.60)
        # Seconds for the arm to complete its motion
        self.declare_parameter('arm_motion_s', 2.5)
        self.declare_parameter('arm_trajectory_topic',
                               '/robot/arm_0_joint_trajectory_controller/joint_trajectory')

        # ── read parameters ──────────────────────────────────────────────────
        gt_path = os.path.expanduser(
            str(self.get_parameter('ground_truth_path').value))
        if not gt_path:
            raise RuntimeError("Parameter 'ground_truth_path' is required.")
        if not os.path.isfile(gt_path):
            raise RuntimeError(f'Ground truth file not found: {gt_path}')

        self._world        = str(self.get_parameter('world_name').value)
        self._robot        = str(self.get_parameter('robot_name').value)
        self._rx           = float(self.get_parameter('robot_x').value)
        self._rz           = float(self.get_parameter('robot_z').value)
        self._yaw          = float(self.get_parameter('robot_yaw').value)
        self._offsets      = list(self.get_parameter('axial_offsets_m').value)
        self._dwell        = float(self.get_parameter('dwell_s').value)
        self._gz_timeout   = int(self.get_parameter('gz_timeout_ms').value)
        self._settle_s     = float(self.get_parameter('settle_s').value)
        self._y_min        = float(self.get_parameter('tube_y_min').value)
        self._y_max        = float(self.get_parameter('tube_y_max').value)
        self._arm_group    = str(self.get_parameter('arm_group').value)
        self._tcp_frame    = str(self.get_parameter('arm_tcp_frame').value)
        self._base_frame   = str(self.get_parameter('arm_base_frame').value)
        self._cam_offset   = float(self.get_parameter('camera_tcp_offset_m').value)
        self._cam_dist     = float(self.get_parameter('camera_target_dist_m').value)
        self._arm_motion   = float(self.get_parameter('arm_motion_s').value)

        # ── load ground truth ────────────────────────────────────────────────
        with open(gt_path, 'r', encoding='utf-8') as fh:
            payload = yaml.safe_load(fh)
        self._defects = payload.get('defects', [])
        if not self._defects:
            raise RuntimeError(f'No defects found in {gt_path}')

        n_pos = len(self._defects) * len(self._offsets)
        self.get_logger().info(
            f'Auto-dataset: {len(self._defects)} defects × '
            f'{len(self._offsets)} offsets = {n_pos} positions. '
            f'~{n_pos * (self._dwell + self._arm_motion + self._settle_s) / 60:.1f} min.'
        )

        # ── TF ──────────────────────────────────────────────────────────────
        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ── joint state (IK seed) ────────────────────────────────────────────
        self._joint_state_lock = threading.Lock()
        self._latest_js: JointState | None = None
        self.create_subscription(JointState, '/joint_states',
                                 self._js_cb, 10)

        # ── arm trajectory publisher ─────────────────────────────────────────
        traj_topic = str(self.get_parameter('arm_trajectory_topic').value)
        self._arm_pub = self.create_publisher(JointTrajectory, traj_topic, 5)

        # ── MoveIt IK service ────────────────────────────────────────────────
        self._ik_client = self.create_client(GetPositionIK, '/compute_ik')

        # ── start capture loop in background thread ──────────────────────────
        self._started = False
        # Give synthetic_capture_node and TF time to start before moving
        self.create_timer(3.0, self._start_loop)

    # ── callbacks ────────────────────────────────────────────────────────────

    def _js_cb(self, msg: JointState) -> None:
        with self._joint_state_lock:
            self._latest_js = msg

    def _start_loop(self) -> None:
        if self._started:
            return
        self._started = True
        threading.Thread(target=self._run, daemon=True).start()

    # ── main capture loop ─────────────────────────────────────────────────────

    def _run(self) -> None:
        total = len(self._defects) * len(self._offsets)
        count = 0

        for defect in self._defects:
            d_id    = defect['defect_id']
            d_class = defect['class_name']
            d_xyz   = (float(defect['world_x']),
                       float(defect['world_y']),
                       float(defect['world_z']))

            for offset in self._offsets:
                robot_y = max(self._y_min,
                              min(self._y_max, d_xyz[1] + offset))
                count += 1
                self.get_logger().info(
                    f'[{count}/{total}] defect {d_id} ({d_class})'
                    f'  offset={offset:+.1f}m  robot_y={robot_y:.2f}m'
                )

                # 1 ─ teleport (pause → pose → resume → settle)
                self._teleport(self._rx, robot_y, self._rz, self._yaw)

                # 2 ─ aim arm at defect via MoveIt IK
                aimed = self._aim_arm(d_xyz)
                if not aimed:
                    self.get_logger().warn(
                        f'  IK failed for defect {d_id} at offset {offset:+.1f}m'
                        ' — skipping position.'
                    )
                    continue

                # 3 ─ dwell: synthetic_capture_node captures frames here
                time.sleep(self._dwell)

        self.get_logger().info(
            f'Auto-capture complete — {count} positions visited.')
        rclpy.shutdown()

    # ── teleport (pause → set_pose → resume → settle) ────────────────────────

    def _teleport(self, x: float, y: float, z: float, yaw: float) -> None:
        self._physics(pause=True)
        self._set_pose(x, y, z, yaw)
        self._physics(pause=False)
        time.sleep(self._settle_s)

    def _physics(self, *, pause: bool) -> None:
        req  = 'pause: true' if pause else 'pause: false'
        cmd  = [
            'gz', 'service',
            '-s', f'/world/{self._world}/control',
            '--reqtype', 'gz.msgs.WorldControl',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', str(self._gz_timeout),
            '--req', req,
        ]
        label = 'pause' if pause else 'resume'
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=5.0)
            if r.returncode != 0:
                self.get_logger().warn(
                    f'physics {label} failed: {r.stderr.strip()}')
        except subprocess.TimeoutExpired:
            self.get_logger().warn(f'physics {label} timed out.')

    def _set_pose(self, x: float, y: float, z: float, yaw: float) -> None:
        qz  = math.sin(yaw / 2.0)
        qw  = math.cos(yaw / 2.0)
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
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=5.0)
            if r.returncode != 0:
                self.get_logger().warn(
                    f'set_pose failed (code {r.returncode}): {r.stderr.strip()}')
        except subprocess.TimeoutExpired:
            self.get_logger().warn('set_pose timed out.')

    # ── arm aiming via MoveIt IK ──────────────────────────────────────────────

    def _aim_arm(self, defect_world: tuple) -> bool:
        """Point the inspection camera at defect_world using MoveIt compute_ik."""

        # 1 ─ get arm base position in world via TF
        try:
            tf = self._tf_buffer.lookup_transform(
                'world', self._base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f'TF world→{self._base_frame} unavailable: {exc}')
            return False

        arm_base = np.array([
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z,
        ])
        defect = np.array(defect_world, dtype=float)

        # 2 ─ direction vector and target TCP position
        direction = defect - arm_base
        dist      = float(np.linalg.norm(direction))
        if dist < 0.05:
            self.get_logger().warn('Defect too close to arm base — skipping.')
            return False
        d_hat = direction / dist

        # Place TCP so the camera lens ends up self._cam_dist from the defect.
        # TCP is self._cam_offset behind the lens along d_hat.
        tcp_dist = max(0.15, dist - self._cam_dist - self._cam_offset)
        tcp_pos  = arm_base + d_hat * tcp_dist

        # 3 ─ TCP orientation: X axis = d_hat (camera forward = TCP X)
        x_ax = d_hat
        up   = np.array([0.0, 0.0, 1.0])
        y_ax = np.cross(up, x_ax)
        if np.linalg.norm(y_ax) < 1e-3:          # x_ax nearly vertical
            up   = np.array([0.0, 1.0, 0.0])
            y_ax = np.cross(up, x_ax)
        y_ax /= np.linalg.norm(y_ax)
        z_ax  = np.cross(x_ax, y_ax)

        R = np.column_stack([x_ax, y_ax, z_ax])
        qx, qy, qz, qw = _rot_to_quat(R)

        # 4 ─ build IK request
        if not self._ik_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('/compute_ik service not available.')
            return False

        ik_req = GetPositionIK.Request()
        ik_req.ik_request.group_name    = self._arm_group
        ik_req.ik_request.ik_link_name  = self._tcp_frame
        ik_req.ik_request.avoid_collisions = True
        ik_req.ik_request.timeout.sec   = 5

        ps = PoseStamped()
        ps.header.frame_id      = 'world'
        ps.header.stamp         = self.get_clock().now().to_msg()
        ps.pose.position.x      = float(tcp_pos[0])
        ps.pose.position.y      = float(tcp_pos[1])
        ps.pose.position.z      = float(tcp_pos[2])
        ps.pose.orientation.x   = qx
        ps.pose.orientation.y   = qy
        ps.pose.orientation.z   = qz
        ps.pose.orientation.w   = qw
        ik_req.ik_request.pose_stamped = ps

        with self._joint_state_lock:
            if self._latest_js is not None:
                ik_req.ik_request.robot_state.joint_state = self._latest_js

        # 5 ─ call IK service (async, signalled via threading.Event)
        done   = threading.Event()
        result = [None]

        def _cb(future):
            result[0] = future.result()
            done.set()

        self._ik_client.call_async(ik_req).add_done_callback(_cb)
        if not done.wait(timeout=8.0) or result[0] is None:
            self.get_logger().warn('IK service call timed out.')
            return False

        resp = result[0]
        if resp.error_code.val != 1:   # MoveItErrorCodes.SUCCESS = 1
            self.get_logger().warn(
                f'IK returned error code {resp.error_code.val} '
                '(no solution found for this defect/offset).'
            )
            return False

        # 6 ─ send joint trajectory
        js = resp.solution.joint_state
        arm_names = [n for n in js.name if n in self._ARM_JOINTS]
        arm_pos   = [p for n, p in zip(js.name, js.position)
                     if n in self._ARM_JOINTS]

        if not arm_names:
            self.get_logger().warn('IK solution contains no arm joints.')
            return False

        traj = JointTrajectory()
        traj.header.stamp  = self.get_clock().now().to_msg()
        traj.joint_names   = arm_names
        pt = JointTrajectoryPoint()
        pt.positions       = arm_pos
        pt.time_from_start = Duration(
            sec=int(self._arm_motion),
            nanosec=int((self._arm_motion % 1.0) * 1e9),
        )
        traj.points.append(pt)
        self._arm_pub.publish(traj)

        # wait for the arm to reach the target pose
        time.sleep(self._arm_motion + 0.3)
        return True


# ── entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = AutoDatasetNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
