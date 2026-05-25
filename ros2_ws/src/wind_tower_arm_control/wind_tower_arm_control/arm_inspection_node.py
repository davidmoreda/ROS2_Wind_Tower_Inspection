"""Arm inspection behaviour node.

Drives the UR5e through a small behaviour state machine during an autonomous
inspection mission by sending goals to the running MoveIt ``move_group``.

Behaviours
----------
* ``home``        — arm tucked at the home pose; default when idle or while the
                    base performs alignment/recovery manoeuvres.
* ``hold_lookat`` — arm holds the inspection "base pose" with the camera aimed
                    at the tube wall. Used while the base drives (AXIAL_SCAN)
                    or the tube rotates (INDEX_TUBE).
* ``sweep``       — arm raster-scans the wall: a tilt x pan grid of the two
                    wrist joints is generated around the base pose, so the
                    camera scans a 2-D patch of the tube wall. Runs as
                    home -> raster -> home when ``sweep_via_home`` is true.
* ``inspect_defect`` — placeholder; implemented in F4 (close-up on a detection).

Capturing the base pose
-----------------------
The base pose (``hold_lookat``) is the one thing that needs eyes on the
simulation. Instead of hand-editing joint numbers, jog the arm in RViz until
the camera frames the wall, then call::

    ros2 service call /arm/capture_hold_lookat std_srvs/srv/Trigger

The node snapshots the current arm joints as the new base pose, regenerates
the raster around it, and persists it to ``hold_lookat_pose_file`` so it
survives restarts. No rebuild needed.

Coordination (topics already in docs/development/TEAM_WORKFLOW.md §3)
--------------------------------------------------------------------
Subscribes:
  /inspection/state_text         std_msgs/String   — mission state (David)
  /inspection/autonomous_active  std_msgs/Bool     — autonomous flag (David)
  /robot/platform/joint_states   sensor_msgs/JointState — live arm pose

Publishes:
  /arm/inspection_ready  std_msgs/Bool   — TRUE when the arm is settled and the
                                           mission may proceed.
  /arm/state             std_msgs/String — JSON status, for debugging.

Provides:
  /arm/capture_hold_lookat  std_srvs/Trigger — snapshot the current pose.

Motion is executed by sending ``moveit_msgs/action/MoveGroup`` goals to
``/move_action`` (the move_group started by move_group.launch.py).
"""

import json
import os
from typing import List, Optional

import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy._rclpy_pybind11 import RCLError
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


# Behaviour keys.
HOME = 'home'
HOLD_LOOKAT = 'hold_lookat'
SWEEP = 'sweep'
INSPECT_DEFECT = 'inspect_defect'
PASSIVE = 'passive'   # autonomous not active — node hands the arm to teleop


class ArmInspectionNode(Node):
    """Behaviour state machine for the UR5e inspection arm."""

    def __init__(self):
        super().__init__('arm_inspection')

        self._declare_parameters()
        self._load_parameters()

        # Runtime state.
        self._mission_state: str = 'UNKNOWN'
        self._autonomous_active: bool = False
        self._current_arm_joints: Optional[List[float]] = None
        self._commanded: Optional[str] = None
        self._busy: bool = False
        self._behaviour_reached: bool = False
        self._last_attempt_s: float = 0.0
        self._sweep_index: int = 0
        self._sweep_active: bool = False
        self._sweep_done: bool = False
        self._server_warned: bool = False
        self._hold_lookat_source: str = 'parameter'
        # Joy override: when set, takes precedence over mission_state and the
        # autonomous_active gate (manual pad control).
        self._joy_override: Optional[str] = None
        self._last_buttons: Optional[List[int]] = None

        # If a base pose was captured in a previous run, use it.
        loaded = self._load_hold_lookat_file()
        if loaded is not None:
            self._hold_lookat_pose = loaded
            self._hold_lookat_source = 'captured file'
        self._regenerate_sweep()

        self._action_client = ActionClient(
            self, MoveGroup, self._move_group_action)

        self._pub_ready = self.create_publisher(
            Bool, '/arm/inspection_ready', 10)
        self._pub_state = self.create_publisher(String, '/arm/state', 10)

        self.create_subscription(
            String, self._state_text_topic, self._state_text_cb, 10)
        self.create_subscription(
            Bool, self._autonomous_topic, self._autonomous_cb, 10)
        self.create_subscription(
            JointState, self._joint_states_topic, self._joint_states_cb, 10)
        if self._enable_joy:
            self.create_subscription(
                Joy, self._joy_topic, self._joy_cb, 10)

        self.create_service(
            Trigger, '/arm/capture_hold_lookat', self._capture_cb)

        self.create_timer(1.0 / self._publish_rate_hz, self._tick)

        self.get_logger().info(
            f'Arm inspection node ready (group={self._planning_group}, '
            f'hold_lookat from {self._hold_lookat_source}, '
            f'{len(self._sweep_waypoints)} sweep waypoints, '
            f'{len(self._sweep_sequence)} total poses, '
            f'joy={"on" if self._enable_joy else "off"} '
            f'[btn{self._joy_button_home}=home, btn{self._joy_button_sweep}=sweep]).'
        )

    # ------------------------------------------------------------------ params
    def _declare_parameters(self):
        self.declare_parameter('state_text_topic', '/inspection/state_text')
        self.declare_parameter(
            'autonomous_active_topic', '/inspection/autonomous_active')
        self.declare_parameter(
            'joint_states_topic', '/robot/platform/joint_states')
        self.declare_parameter('move_group_action', '/move_action')
        self.declare_parameter('planning_group', 'arm_0')

        self.declare_parameter('joint_names', [
            'arm_0_shoulder_pan_joint',
            'arm_0_shoulder_lift_joint',
            'arm_0_elbow_joint',
            'arm_0_wrist_1_joint',
            'arm_0_wrist_2_joint',
            'arm_0_wrist_3_joint',
        ])

        self.declare_parameter('require_autonomous_active', True)
        self.declare_parameter('publish_rate_hz', 5.0)

        # PS5 DualSense bindings (via the dualsense_joy driver in
        # wind_tower_bringup). When the user presses one of these buttons,
        # the joy override drives the arm DIRECTLY, bypassing the mission-
        # state mapping and the require_autonomous_active gate. Note that
        # dualsense_joy masks every button except STOP when
        # /inspection/autonomous_active is true, so manual joy bindings only
        # work while autonomous is OFF (which is the natural state for joy
        # control). Defaults match the indices Dani observed: R1=2, Δ=3.
        self.declare_parameter('enable_joy_control', True)
        self.declare_parameter('joy_topic', '/robot/joy_teleop/joy')
        self.declare_parameter('joy_button_home', 2)    # R1
        self.declare_parameter('joy_button_sweep', 3)   # Triangle

        self.declare_parameter('velocity_scaling', 0.15)
        self.declare_parameter('acceleration_scaling', 0.15)
        self.declare_parameter('planning_time_s', 5.0)
        self.declare_parameter('planning_attempts', 10)
        self.declare_parameter('joint_goal_tolerance_rad', 0.01)
        self.declare_parameter('retry_cooldown_s', 3.0)

        self.declare_parameter('mission_states', [
            'AXIAL_SCAN', 'INDEX_TUBE', 'DETAIL_SCAN', 'IDLE'])
        self.declare_parameter('mission_behaviours', [
            'hold_lookat', 'hold_lookat', 'sweep', 'home'])
        self.declare_parameter('default_behaviour', 'home')

        # Default = the "camera faces forward" pose Dani picked:
        # [0, -56, 50, -181, 0, 34] degrees in joint_names order.
        self.declare_parameter(
            'home_joint_positions',
            [0.0000, -0.9774, 0.8727, -3.1591, 0.0000, 0.5934])
        self.declare_parameter(
            'hold_lookat_joint_positions',
            [0.0000, -0.9774, 0.8727, -3.1591, 0.0000, 0.5934])
        self.declare_parameter(
            'hold_lookat_pose_file', '~/.wind_tower_arm/hold_lookat.json')

        # Sweep: explicit list of joint-space waypoints. Six values per
        # waypoint (in joint_names order), all concatenated end-to-end.
        # Executed in the order given. Capture poses by jogging the arm in
        # RViz (MotionPlanning panel -> Joints tab) and reading the sliders.
        self.declare_parameter('sweep_via_home', True)
        self.declare_parameter(
            'sweep_joint_positions_flat',
            # 7 waypoints rotating shoulder_pan through 360° in 45° steps.
            # lift/elbow/wrists stay at the home values, so only the base
            # rotates the camera around. With sweep_via_home=true the full
            # run is home (pan=0°) -> 45° -> 90° -> ... -> 315° -> home.
            [
                0.7854, -0.9774, 0.8727, -3.1591, 0.0000, 0.5934,
                1.5708, -0.9774, 0.8727, -3.1591, 0.0000, 0.5934,
                2.3562, -0.9774, 0.8727, -3.1591, 0.0000, 0.5934,
                3.1416, -0.9774, 0.8727, -3.1591, 0.0000, 0.5934,
                3.9270, -0.9774, 0.8727, -3.1591, 0.0000, 0.5934,
                4.7124, -0.9774, 0.8727, -3.1591, 0.0000, 0.5934,
                5.4978, -0.9774, 0.8727, -3.1591, 0.0000, 0.5934,
            ])

    def _load_parameters(self):
        self._state_text_topic = str(
            self.get_parameter('state_text_topic').value)
        self._autonomous_topic = str(
            self.get_parameter('autonomous_active_topic').value)
        self._joint_states_topic = str(
            self.get_parameter('joint_states_topic').value)
        self._move_group_action = str(
            self.get_parameter('move_group_action').value)
        self._planning_group = str(self.get_parameter('planning_group').value)
        self._joint_names = list(self.get_parameter('joint_names').value)

        self._require_autonomous = bool(
            self.get_parameter('require_autonomous_active').value)
        self._publish_rate_hz = max(
            1.0, float(self.get_parameter('publish_rate_hz').value))

        self._enable_joy = bool(
            self.get_parameter('enable_joy_control').value)
        self._joy_topic = str(self.get_parameter('joy_topic').value)
        self._joy_button_home = int(
            self.get_parameter('joy_button_home').value)
        self._joy_button_sweep = int(
            self.get_parameter('joy_button_sweep').value)

        self._vel_scaling = float(self.get_parameter('velocity_scaling').value)
        self._acc_scaling = float(
            self.get_parameter('acceleration_scaling').value)
        self._planning_time = float(
            self.get_parameter('planning_time_s').value)
        self._planning_attempts = int(
            self.get_parameter('planning_attempts').value)
        self._joint_tol = float(
            self.get_parameter('joint_goal_tolerance_rad').value)
        self._retry_cooldown = float(
            self.get_parameter('retry_cooldown_s').value)

        states = list(self.get_parameter('mission_states').value)
        behaviours = list(self.get_parameter('mission_behaviours').value)
        if len(states) != len(behaviours):
            raise ValueError(
                'mission_states and mission_behaviours must have equal length'
            )
        self._state_behaviour = dict(zip(states, behaviours))
        self._default_behaviour = str(
            self.get_parameter('default_behaviour').value)

        n = len(self._joint_names)
        self._home_pose = self._validate_pose('home_joint_positions', n)
        self._hold_lookat_pose = self._validate_pose(
            'hold_lookat_joint_positions', n)
        self._pose_file = str(
            self.get_parameter('hold_lookat_pose_file').value)

        self._sweep_via_home = bool(
            self.get_parameter('sweep_via_home').value)
        self._sweep_waypoints = self._validate_sweep(n)

    def _validate_pose(self, param_name: str, n: int) -> List[float]:
        pose = [float(v) for v in self.get_parameter(param_name).value]
        if len(pose) != n:
            raise ValueError(
                f'{param_name} must have {n} values, got {len(pose)}')
        return pose

    # --------------------------------------------------------- sweep build
    def _validate_sweep(self, n: int) -> List[List[float]]:
        flat = [float(v) for v in
                self.get_parameter('sweep_joint_positions_flat').value]
        if not flat or len(flat) % n != 0:
            raise ValueError(
                'sweep_joint_positions_flat length must be a non-zero '
                f'multiple of the joint count ({n}); got {len(flat)}'
            )
        return [flat[i:i + n] for i in range(0, len(flat), n)]

    def _regenerate_sweep(self):
        if self._sweep_via_home:
            self._sweep_sequence = (
                [list(self._home_pose)]
                + [list(wp) for wp in self._sweep_waypoints]
                + [list(self._home_pose)])
        else:
            self._sweep_sequence = [list(wp) for wp in self._sweep_waypoints]

    # ------------------------------------------------------ pose persistence
    def _load_hold_lookat_file(self) -> Optional[List[float]]:
        path = os.path.expanduser(self._pose_file)
        if not os.path.isfile(path):
            return None
        try:
            with open(path, 'r', encoding='utf-8') as fh:
                data = json.load(fh)
            pose = data.get('hold_lookat_joint_positions')
            if pose and len(pose) == len(self._joint_names):
                return [float(v) for v in pose]
        except (OSError, ValueError) as exc:
            self.get_logger().warn(f'Could not read {path}: {exc}')
        return None

    def _save_hold_lookat_file(self):
        path = os.path.expanduser(self._pose_file)
        try:
            os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
            with open(path, 'w', encoding='utf-8') as fh:
                json.dump(
                    {'hold_lookat_joint_positions': self._hold_lookat_pose,
                     'joint_names': self._joint_names},
                    fh, indent=2)
        except OSError as exc:
            self.get_logger().warn(f'Could not save base pose to {path}: {exc}')

    # ------------------------------------------------------------- callbacks
    def _state_text_cb(self, msg: String):
        self._mission_state = str(msg.data).strip() or 'UNKNOWN'

    def _autonomous_cb(self, msg: Bool):
        new_state = bool(msg.data)
        if new_state and not self._autonomous_active and self._joy_override is not None:
            # Mission takes over → drop any manual joy override so the
            # mission_state mapping drives the arm again.
            self.get_logger().info(
                'autonomous_active enabled; clearing joy override.')
            self._joy_override = None
            self._commanded = None
        self._autonomous_active = new_state

    def _joy_cb(self, msg: Joy):
        buttons = list(msg.buttons)
        if self._last_buttons is None:
            self._last_buttons = buttons
            return

        def rising(idx: int) -> bool:
            return (
                0 <= idx < len(buttons)
                and 0 <= idx < len(self._last_buttons)
                and buttons[idx] == 1
                and self._last_buttons[idx] == 0
            )

        if rising(self._joy_button_home):
            self._trigger_joy_override(
                HOME, f'home button (btn{self._joy_button_home})')
        if rising(self._joy_button_sweep):
            self._trigger_joy_override(
                SWEEP, f'sweep button (btn{self._joy_button_sweep})')

        self._last_buttons = buttons

    def _trigger_joy_override(self, behaviour: str, label: str):
        self.get_logger().info(f'Joy {label} -> {behaviour}')
        self._joy_override = behaviour
        # Force the tick to re-issue (also lets the SWEEP one-shot re-fire on
        # every R1 press).
        self._commanded = None
        self._behaviour_reached = False
        self._sweep_done = False

    def _joint_states_cb(self, msg: JointState):
        name_to_pos = dict(zip(msg.name, msg.position))
        if all(j in name_to_pos for j in self._joint_names):
            self._current_arm_joints = [
                float(name_to_pos[j]) for j in self._joint_names]

    def _capture_cb(self, request, response):
        if self._current_arm_joints is None:
            response.success = False
            response.message = (
                f'No joint state on {self._joint_states_topic} yet.')
            return response
        if self._busy or self._sweep_active:
            response.success = False
            response.message = 'Arm is moving; capture again once it is idle.'
            return response

        self._hold_lookat_pose = list(self._current_arm_joints)
        self._hold_lookat_source = 'captured (this run)'
        self._save_hold_lookat_file()
        # Force the next tick to re-issue the behaviour with the new pose.
        self._commanded = None
        self._behaviour_reached = False

        pose_str = ', '.join(f'{v:.4f}' for v in self._hold_lookat_pose)
        response.success = True
        response.message = (
            f'Captured hold_lookat = [{pose_str}]; saved to '
            f'{os.path.expanduser(self._pose_file)}.')
        self.get_logger().info(response.message)
        return response

    # --------------------------------------------------------------- control
    def _tick(self):
        desired = self._compute_desired_behaviour()
        self._publish_status(desired)

        if self._busy:
            return

        if desired == PASSIVE:
            self._commanded = PASSIVE
            return

        if desired == SWEEP:
            if self._commanded != SWEEP:
                self._commanded = SWEEP
                self._sweep_done = False
                self._start_sweep()
            return

        retry_due = (
            not self._behaviour_reached
            and (self._now_s() - self._last_attempt_s) >= self._retry_cooldown
        )
        if self._commanded != desired or retry_due:
            self._commanded = desired
            self._behaviour_reached = False
            self._last_attempt_s = self._now_s()
            pose = self._pose_for(desired)
            if pose is not None:
                self.get_logger().info(f'Commanding arm -> {desired}')
                self._send_joint_goal(
                    pose, lambda ok: self._on_pose_done(desired, ok))

    def _compute_desired_behaviour(self) -> str:
        # Manual joy override wins over everything else: lets the user run
        # L1=home or R1=sweep without enabling autonomous mode.
        if self._joy_override is not None:
            return self._joy_override
        if self._require_autonomous and not self._autonomous_active:
            return PASSIVE
        return self._state_behaviour.get(
            self._mission_state, self._default_behaviour)

    def _pose_for(self, behaviour: str) -> Optional[List[float]]:
        if behaviour == HOME:
            return self._home_pose
        if behaviour == HOLD_LOOKAT:
            return self._hold_lookat_pose
        return None

    def _on_pose_done(self, behaviour: str, ok: bool):
        if ok:
            self._behaviour_reached = True
            self.get_logger().info(f'Arm reached {behaviour}.')
        else:
            self._behaviour_reached = False
            self.get_logger().warn(
                f'Arm failed to reach {behaviour}; will retry.')

    # ----------------------------------------------------------------- sweep
    def _start_sweep(self):
        self._sweep_index = 0
        self._sweep_active = True
        self.get_logger().info(
            f'Starting raster sweep: {len(self._sweep_sequence)} poses.')
        self._send_joint_goal(
            self._sweep_sequence[0], self._on_sweep_waypoint_done)

    def _on_sweep_waypoint_done(self, ok: bool):
        if not ok:
            # A single unreachable raster cell must not kill the whole scan.
            self.get_logger().warn(
                f'Sweep pose {self._sweep_index + 1} failed; skipping it.')
        if self._compute_desired_behaviour() != SWEEP:
            self.get_logger().info('Sweep interrupted: mission state changed.')
            self._sweep_active = False
            return
        self._sweep_index += 1
        if self._sweep_index >= len(self._sweep_sequence):
            self.get_logger().info(
                'Sweep complete'
                + (' (arm back at home).' if self._sweep_via_home else '.'))
            self._sweep_active = False
            self._sweep_done = True
            self._behaviour_reached = True
            return
        self.get_logger().info(
            f'Sweep pose {self._sweep_index + 1}/{len(self._sweep_sequence)}.')
        self._send_joint_goal(
            self._sweep_sequence[self._sweep_index],
            self._on_sweep_waypoint_done,
        )

    # -------------------------------------------------------- MoveGroup goal
    def _send_joint_goal(self, positions: List[float], on_done):
        if not self._action_client.server_is_ready():
            if not self._server_warned:
                self.get_logger().warn(
                    f'MoveGroup action server {self._move_group_action} not '
                    'available yet; is move_group running?'
                )
                self._server_warned = True
            on_done(False)
            return
        self._server_warned = False

        goal = self._build_joint_goal(positions)
        self._busy = True
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(
            lambda f: self._goal_response_cb(f, on_done))

    def _build_joint_goal(self, positions: List[float]) -> MoveGroup.Goal:
        goal = MoveGroup.Goal()
        goal.request.group_name = self._planning_group
        goal.request.num_planning_attempts = self._planning_attempts
        goal.request.allowed_planning_time = self._planning_time
        goal.request.max_velocity_scaling_factor = self._vel_scaling
        goal.request.max_acceleration_scaling_factor = self._acc_scaling
        goal.request.start_state.is_diff = True

        constraints = Constraints()
        for name, position in zip(self._joint_names, positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(position)
            jc.tolerance_above = self._joint_tol
            jc.tolerance_below = self._joint_tol
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        goal.request.goal_constraints.append(constraints)

        goal.planning_options.plan_only = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        return goal

    def _goal_response_cb(self, future, on_done):
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'MoveGroup goal send failed: {exc}')
            self._busy = False
            on_done(False)
            return
        if not handle.accepted:
            self.get_logger().warn('MoveGroup goal was rejected.')
            self._busy = False
            on_done(False)
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self._result_cb(f, on_done))

    def _result_cb(self, future, on_done):
        self._busy = False
        try:
            result = future.result().result
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'MoveGroup result error: {exc}')
            on_done(False)
            return
        ok = result.error_code.val == MoveItErrorCodes.SUCCESS
        if not ok:
            self.get_logger().warn(
                f'MoveGroup motion failed (error code {result.error_code.val}).'
            )
        on_done(ok)

    # ---------------------------------------------------------------- status
    def _inspection_ready(self, desired: str) -> bool:
        if desired == PASSIVE:
            return True
        if self._busy:
            return False
        if self._commanded == SWEEP and not self._sweep_done:
            return False
        return True

    def _publish_status(self, desired: str):
        ready = self._inspection_ready(desired)
        self._pub_ready.publish(Bool(data=ready))

        status = {
            'behaviour': self._commanded or 'NONE',
            'desired': desired,
            'busy': self._busy,
            'inspection_ready': ready,
            'mission_state': self._mission_state,
            'autonomous_active': self._autonomous_active,
            'hold_lookat_source': self._hold_lookat_source,
            'joy_override': self._joy_override,
            'sweep_index': (
                self._sweep_index if self._commanded == SWEEP else None),
            'sweep_total': len(self._sweep_sequence),
        }
        self._pub_state.publish(String(data=json.dumps(status)))

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = ArmInspectionNode()
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
