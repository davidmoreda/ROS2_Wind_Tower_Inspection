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
        -p ground_truth_path:=<path/to/defects_ground_truth.yaml> \\
        -p robot_name:=robot/robot
"""

import copy
import math
import os
import subprocess
import threading
import time

import cv2
import numpy as np
import rclpy
import rclpy.duration
import rclpy.time
import yaml
from builtin_interfaces.msg import Duration
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy._rclpy_pybind11 import RCLError
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformBroadcaster, TransformListener, TransformException
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
        self.declare_parameter('robot_x', 0.0)
        self.declare_parameter('robot_z', 0.3)
        self.declare_parameter('robot_yaw', 1.5708)
        self.declare_parameter('axial_offsets_m', [-1.5, -0.5, 0.5, 1.5])
        self.declare_parameter('dwell_s', 3.0)
        self.declare_parameter('gz_timeout_ms', 2000)
        self.declare_parameter('settle_s', 0.5)
        self.declare_parameter('tube_y_min', -14.5)
        self.declare_parameter('tube_y_max', 14.5)
        # TF correction: the odom frame name (Clearpath default: 'odom')
        self.declare_parameter('odom_frame', 'odom')
        # Arm aiming: True = MoveIt IK (default), False = skip arm movement
        self.declare_parameter('aim_arm', True)
        self.declare_parameter('arm_group', 'arm_0')
        self.declare_parameter('arm_tcp_frame', 'arm_0_tool0')
        self.declare_parameter('arm_base_frame', 'arm_0_base_link')
        self.declare_parameter('camera_tcp_offset_m', 0.22)
        self.declare_parameter('camera_target_dist_m', 0.60)
        # UR5e practical reach from its base link (~850 mm max, use 0.65 m safe)
        self.declare_parameter('arm_max_reach_m', 0.65)
        self.declare_parameter('arm_motion_s', 2.5)
        self.declare_parameter(
            'arm_action_topic',
            '/robot/arm_0_joint_trajectory_controller/follow_joint_trajectory')
        # capture_images: save raw JPEGs for manual Roboflow labeling
        self.declare_parameter('capture_images', False)
        self.declare_parameter('image_topic', '/inspection/camera/image_raw')
        self.declare_parameter(
            'image_output_dir',
            '~/ROS2_Wind_Tower_Inspection/ros2_ws/datasets/roboflow_positions',
        )
        self.declare_parameter('images_per_position', 2)
        self.declare_parameter('capture_delay_s', 0.5)
        self.declare_parameter('capture_interval_s', 0.3)
        self.declare_parameter('jpeg_quality', 92)
        self.declare_parameter('val_every_n', 5)

        # ── read parameters ──────────────────────────────────────────────────
        gt_path = os.path.expanduser(
            str(self.get_parameter('ground_truth_path').value))
        if not gt_path:
            raise RuntimeError("Parameter 'ground_truth_path' is required.")
        if not os.path.isfile(gt_path):
            raise RuntimeError(f'Ground truth file not found: {gt_path}')

        self._world      = str(self.get_parameter('world_name').value)
        self._robot      = str(self.get_parameter('robot_name').value)
        self._rx         = float(self.get_parameter('robot_x').value)
        self._rz         = float(self.get_parameter('robot_z').value)
        self._yaw        = float(self.get_parameter('robot_yaw').value)
        self._offsets    = list(self.get_parameter('axial_offsets_m').value)
        self._dwell      = float(self.get_parameter('dwell_s').value)
        self._gz_timeout = int(self.get_parameter('gz_timeout_ms').value)
        self._settle_s   = float(self.get_parameter('settle_s').value)
        self._y_min      = float(self.get_parameter('tube_y_min').value)
        self._y_max      = float(self.get_parameter('tube_y_max').value)
        self._odom_frame = str(self.get_parameter('odom_frame').value)
        self._do_aim     = bool(self.get_parameter('aim_arm').value)
        self._arm_group  = str(self.get_parameter('arm_group').value)
        self._tcp_frame  = str(self.get_parameter('arm_tcp_frame').value)
        self._base_frame = str(self.get_parameter('arm_base_frame').value)
        self._cam_offset  = float(self.get_parameter('camera_tcp_offset_m').value)
        self._cam_dist    = float(self.get_parameter('camera_target_dist_m').value)
        self._arm_reach   = float(self.get_parameter('arm_max_reach_m').value)
        self._arm_motion  = float(self.get_parameter('arm_motion_s').value)

        self._capture_images    = bool(self.get_parameter('capture_images').value)
        self._image_output_dir  = os.path.expanduser(
            str(self.get_parameter('image_output_dir').value))
        self._images_per_pos    = max(1, int(self.get_parameter('images_per_position').value))
        self._capture_delay_s   = max(0.0, float(self.get_parameter('capture_delay_s').value))
        self._capture_interval_s = max(0.0, float(self.get_parameter('capture_interval_s').value))
        self._jpeg_quality      = int(self.get_parameter('jpeg_quality').value)
        self._val_every_n       = max(2, int(self.get_parameter('val_every_n').value))

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
        if self._do_aim:
            self.get_logger().info(
                'Arm aiming via MoveIt IK enabled '
                f'(group={self._arm_group}, tcp={self._tcp_frame}).'
            )
        if self._capture_images:
            self.get_logger().info(
                f'Roboflow capture enabled: {self._image_output_dir} '
                f'({self._images_per_pos} images/position).'
            )

        # ── TF ──────────────────────────────────────────────────────────────
        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ── joint state (IK seed) ────────────────────────────────────────────
        self._joint_state_lock = threading.Lock()
        self._latest_js: JointState | None = None
        # Clearpath publishes all joints (platform + arm) on this topic
        self.create_subscription(JointState, '/robot/platform/joint_states',
                                 self._js_cb, 10)

        # ── TF correction: publish world→odom to compensate for teleport ────────
        # After a Gazebo teleport, `odom→base_link` is NOT updated (odometry-based,
        # not ground-truth). We correct this by publishing `world→odom` with a
        # translation offset = teleport_pos − odom_base_link_pos. Publishing at
        # 20 Hz overrides the static world→odom from the bringup launch.
        self._tf_broadcaster    = TransformBroadcaster(self)
        self._world_odom_lock   = threading.Lock()
        self._world_odom_cached: TransformStamped | None = None
        self.create_timer(0.05, self._republish_world_odom)   # 20 Hz

        # ── arm action client (FollowJointTrajectory) ────────────────────────
        action_topic = str(self.get_parameter('arm_action_topic').value)
        self._arm_action = ActionClient(self, FollowJointTrajectory, action_topic)

        # ── MoveIt IK service ────────────────────────────────────────────────
        self._ik_client = self.create_client(GetPositionIK, '/compute_ik')

        # ── image capture (Roboflow mode) ────────────────────────────────────
        self._bridge         = CvBridge()
        self._image_lock     = threading.Lock()
        self._latest_frame   = None
        self._image_counter  = 0
        self._saved_train    = 0
        self._saved_val      = 0
        if self._capture_images:
            self._train_dir = os.path.join(self._image_output_dir, 'images', 'train')
            self._val_dir   = os.path.join(self._image_output_dir, 'images', 'val')
            os.makedirs(self._train_dir, exist_ok=True)
            os.makedirs(self._val_dir, exist_ok=True)
            self.create_subscription(
                Image,
                str(self.get_parameter('image_topic').value),
                self._image_cb,
                5,
            )

        # ── start capture loop in background thread ──────────────────────────
        self._started = False
        self.create_timer(3.0, self._start_loop)

    # ── callbacks ────────────────────────────────────────────────────────────

    def _js_cb(self, msg: JointState) -> None:
        with self._joint_state_lock:
            self._latest_js = msg

    def _image_cb(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'Could not decode image: {exc}')
            return
        with self._image_lock:
            self._latest_frame = frame.copy()

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
                robot_y = max(self._y_min, min(self._y_max, d_xyz[1] + offset))
                count += 1
                self.get_logger().info(
                    f'[{count}/{total}] defect {d_id} ({d_class})'
                    f'  offset={offset:+.1f}m  robot_y={robot_y:.2f}m'
                )

                # 1 ─ teleport (pause → pose → resume → settle)
                self._teleport(self._rx, robot_y, self._rz, self._yaw)

                # 2 ─ aim arm at defect via MoveIt IK
                if self._do_aim:
                    aimed = self._aim_arm(d_xyz)
                    if not aimed:
                        self.get_logger().warn(
                            f'  IK failed for defect {d_id} at offset {offset:+.1f}m'
                            ' — skipping position.'
                        )
                        continue

                # 3 ─ optional: save raw images for Roboflow manual labeling
                if self._capture_images:
                    time.sleep(self._capture_delay_s)
                    self._capture_images_for_position(count, d_id, d_class)

                # 4 ─ dwell: synthetic_capture_node captures autolabeled frames
                time.sleep(self._dwell)

        self.get_logger().info(
            f'Auto-capture complete — {count} positions visited.')
        if self._capture_images:
            self.get_logger().info(
                f'Roboflow images saved: train={self._saved_train} val={self._saved_val}')
        rclpy.shutdown()

    # ── teleport (pause → set_pose → resume → settle) ────────────────────────

    def _teleport(self, x: float, y: float, z: float, yaw: float) -> None:
        self._physics(pause=True)
        self._set_pose(x, y, z, yaw)
        self._physics(pause=False)
        time.sleep(self._settle_s)
        self._update_world_odom(x, y, z, yaw)

    def _republish_world_odom(self) -> None:
        """20 Hz timer: keep broadcasting the current world→odom correction."""
        with self._world_odom_lock:
            cached = self._world_odom_cached
        if cached is None:
            return
        msg = copy.copy(cached)
        msg.header.stamp = self.get_clock().now().to_msg()
        self._tf_broadcaster.sendTransform(msg)

    def _update_world_odom(self, x: float, y: float, z: float, yaw: float) -> None:
        """After a teleport, compute and broadcast world→odom so that the full
        chain world→odom→base_link reflects the actual teleported position.

        The odometry node keeps publishing odom→base_link at the spawn position
        (it doesn't know about Gazebo teleports). By adjusting world→odom we
        correct the chain without touching base_link's parent frame, which avoids
        the dual-parent conflict that would arise from publishing world→base_link
        directly while odom is already doing so."""
        try:
            odom_base = self._tf_buffer.lookup_transform(
                self._odom_frame, 'base_link',
                self.get_clock().now(),
                timeout=rclpy.duration.Duration(seconds=2.0),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f'Cannot lookup {self._odom_frame}→base_link for TF correction: {exc}')
            return

        t = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id  = self._odom_frame
        # world→odom translation = teleport_world_pos − odom→base_link translation
        # (valid when world and odom share the same orientation, which is always
        # true in a standard Clearpath sim that starts with identity world→odom).
        t.transform.translation.x = x - odom_base.transform.translation.x
        t.transform.translation.y = y - odom_base.transform.translation.y
        t.transform.translation.z = z - odom_base.transform.translation.z
        t.transform.rotation.w    = 1.0   # no rotation correction needed

        with self._world_odom_lock:
            self._world_odom_cached = t

        # Publish immediately so the TF buffer is updated before _aim_arm runs.
        self._tf_broadcaster.sendTransform(t)
        time.sleep(0.05)   # give the executor one cycle to deliver to tf buffer

        self.get_logger().debug(
            f'TF correction world→{self._odom_frame}: '
            f'Δy={t.transform.translation.y:+.3f}m'
        )

    def _physics(self, *, pause: bool) -> None:
        req   = 'pause: true' if pause else 'pause: false'
        label = 'pause' if pause else 'resume'
        cmd   = [
            'gz', 'service',
            '-s', f'/world/{self._world}/control',
            '--reqtype', 'gz.msgs.WorldControl',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', str(self._gz_timeout),
            '--req', req,
        ]
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
                self.get_clock().now(),
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

        # 2 ─ direction vector and target TCP position (in arm_base frame)
        direction = defect - arm_base
        dist      = float(np.linalg.norm(direction))
        if dist < 0.05:
            self.get_logger().warn('Defect too close to arm base — skipping.')
            return False
        d_hat = direction / dist
        # Tube wall is ~4 m away — cap TCP within the arm's physical reach.
        # The camera orientation still points toward the defect; the FOV covers it.
        tcp_dist = min(self._arm_reach, max(0.15, dist - self._cam_dist - self._cam_offset))
        # Express TCP position in arm_base_link frame (eliminate MoveIt TF lookup)
        tcp_world = arm_base + d_hat * tcp_dist
        tcp_local = tcp_world - arm_base   # relative to arm_0_base_link origin

        # 3 ─ TCP orientation: X axis = d_hat (camera optical axis = TCP X)
        # Expressed in arm_0_base_link frame (arm_base has no rotation in world here)
        x_ax = d_hat
        up   = np.array([0.0, 0.0, 1.0])
        y_ax = np.cross(up, x_ax)
        if np.linalg.norm(y_ax) < 1e-3:
            up   = np.array([0.0, 1.0, 0.0])
            y_ax = np.cross(up, x_ax)
        y_ax /= np.linalg.norm(y_ax)
        z_ax  = np.cross(x_ax, y_ax)

        R = np.column_stack([x_ax, y_ax, z_ax])
        qx, qy, qz, qw = _rot_to_quat(R)

        self.get_logger().debug(
            f'IK target: base={arm_base.round(3)} defect={np.array(defect_world).round(3)} '
            f'dist={dist:.2f}m tcp_dist={tcp_dist:.2f}m tcp_local={tcp_local.round(3)}'
        )

        # 4 ─ build IK request in arm_0_base_link frame (KDL's native frame)
        if not self._ik_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('/compute_ik service not available.')
            return False

        ik_req = GetPositionIK.Request()
        ik_req.ik_request.group_name       = self._arm_group
        ik_req.ik_request.ik_link_name     = self._tcp_frame
        ik_req.ik_request.avoid_collisions = False   # faster; walls are static
        ik_req.ik_request.timeout.sec      = 5

        ps = PoseStamped()
        ps.header.frame_id    = self._base_frame   # arm_0_base_link — KDL native
        ps.header.stamp       = self.get_clock().now().to_msg()
        ps.pose.position.x    = float(tcp_local[0])
        ps.pose.position.y    = float(tcp_local[1])
        ps.pose.position.z    = float(tcp_local[2])
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        ik_req.ik_request.pose_stamped = ps

        with self._joint_state_lock:
            if self._latest_js is not None:
                ik_req.ik_request.robot_state.joint_state = self._latest_js

        # 5 ─ call IK service async, signal via threading.Event
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

        # 6 ─ send joint trajectory via FollowJointTrajectory action
        js        = resp.solution.joint_state
        arm_names = [n for n in js.name if n in self._ARM_JOINTS]
        arm_pos   = [p for n, p in zip(js.name, js.position)
                     if n in self._ARM_JOINTS]

        if not arm_names:
            self.get_logger().warn(
                f'IK solution has no arm joints. Names in solution: {list(js.name)[:6]}')
            return False

        self.get_logger().info(
            f'IK OK — sending trajectory: '
            + ', '.join(f'{n}={p:.3f}' for n, p in zip(arm_names, arm_pos))
        )

        traj = JointTrajectory()
        traj.joint_names = arm_names
        pt = JointTrajectoryPoint()
        pt.positions       = arm_pos
        pt.time_from_start = Duration(
            sec=int(self._arm_motion),
            nanosec=int((self._arm_motion % 1.0) * 1e9),
        )
        traj.points.append(pt)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        if not self._arm_action.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn(
                'FollowJointTrajectory action server not available. '
                'Is the arm controller running?')
            return False

        # Wait only for goal ACCEPTANCE (not for execution to finish).
        # The result callback has threading issues in background threads; instead
        # we sleep arm_motion seconds (wall clock) which is independent of sim time.
        accept_done = threading.Event()
        accepted    = [False]

        def _goal_cb(future):
            handle = future.result()
            if handle.accepted:
                self.get_logger().info('FollowJointTrajectory accepted — arm executing.')
                accepted[0] = True
            else:
                self.get_logger().warn('FollowJointTrajectory goal REJECTED by controller.')
            accept_done.set()

        self._arm_action.send_goal_async(goal).add_done_callback(_goal_cb)

        if not accept_done.wait(timeout=6.0):
            self.get_logger().warn('Goal acceptance timed out (action server unresponsive).')
            return False

        if not accepted[0]:
            return False

        # Sleep arm_motion seconds (wall clock) for the arm to execute the trajectory.
        time.sleep(self._arm_motion)
        return True

    # ── Roboflow image capture ────────────────────────────────────────────────

    def _capture_images_for_position(
            self, position_idx: int, defect_id: int, defect_class: str) -> None:
        for sample_idx in range(self._images_per_pos):
            self._save_latest_image(position_idx, defect_id, defect_class, sample_idx)
            time.sleep(self._capture_interval_s)

    def _save_latest_image(
            self, position_idx: int, defect_id: int,
            defect_class: str, sample_idx: int) -> bool:
        with self._image_lock:
            frame = None if self._latest_frame is None else self._latest_frame.copy()
        if frame is None:
            self.get_logger().warn('No camera frame available; skipping Roboflow image.')
            return False

        self._image_counter += 1
        is_val  = (self._image_counter % self._val_every_n) == 0
        img_dir = self._val_dir if is_val else self._train_dir
        safe_cls = ''.join(
            ch if ch.isalnum() or ch in ('-', '_') else '_' for ch in defect_class)
        stem  = f'defect_{int(defect_id):04d}_{safe_cls}_pos_{position_idx:04d}_{sample_idx:02d}'
        path  = os.path.join(img_dir, f'{stem}.jpg')
        ok    = cv2.imwrite(path, frame, [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality])
        if not ok:
            self.get_logger().warn(f'Failed to write image: {path}')
            return False

        if is_val:
            self._saved_val += 1
        else:
            self._saved_train += 1
        self.get_logger().info(f'Saved Roboflow image {self._image_counter}: {path}')
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
