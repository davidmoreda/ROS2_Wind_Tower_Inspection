"""Mission state machine for axial-lane tube inspection.

This MVP commands a simple autonomous sequence:
axial scan -> rotate base to tangential -> index tube while compensating the
base -> rotate base to the next axial direction -> scan next lane.
"""

import json
import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy._rclpy_pybind11 import RCLError
from std_msgs.msg import Bool, Float64, String


IDLE = 'IDLE'
ALIGN_TO_BOTTOM_LANE = 'ALIGN_TO_BOTTOM_LANE'
VERIFY_BOTTOM_LOCK = 'VERIFY_BOTTOM_LOCK'
AXIAL_SCAN = 'AXIAL_SCAN'
RETURN_TO_BOTTOM_LANE = 'RETURN_TO_BOTTOM_LANE'
WAIT_SAFE_TO_INDEX = 'WAIT_SAFE_TO_INDEX'
ROTATE_TO_TANGENTIAL = 'ROTATE_TO_TANGENTIAL'
INDEX_TUBE = 'INDEX_TUBE'
ROTATE_TO_AXIAL = 'ROTATE_TO_AXIAL'
VERIFY_INDEXED_POSITION = 'VERIFY_INDEXED_POSITION'
FINISH = 'FINISH'
ERROR_RECOVERY = 'ERROR_RECOVERY'


@dataclass
class OdomState:
    stamp_s: float
    x_m: float
    y_m: float
    yaw_rad: float


@dataclass
class TurnerState:
    stamp_s: float
    theta_rad: float


@dataclass
class FlagState:
    stamp_s: float
    value: bool


@dataclass
class StabilityTelemetry:
    stamp_s: float
    lateral_angle_deg: float
    roll_deg: float
    pitch_deg: float


def _quat_to_yaw(q) -> float:
    x, y, z, w = q.x, q.y, q.z, q.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class InspectionStateMachineNode(Node):
    """Autonomous axial-lane mission prototype."""

    def __init__(self):
        super().__init__('state_machine')

        self._declare_parameters()
        self._load_parameters()

        self._state = IDLE
        self._lane_id = 0
        self._lane_direction = 1.0
        self._lane_start_position_m: Optional[float] = None
        self._lane_target_position_m: Optional[float] = None
        self._desired_yaw_rad: Optional[float] = None
        self._axial_yaw_target_rad: Optional[float] = None
        self._tangential_yaw_target_rad: Optional[float] = None
        self._index_start_theta_rad: Optional[float] = None
        self._index_target_theta_rad: Optional[float] = None
        self._settle_start_s: Optional[float] = None
        self._rotation_done_deg = 0.0
        self._last_transition_reason = 'startup'
        self._last_input_failure = {}
        self._start_requested = self._auto_start
        self._last_index_error_deg = 0.0
        self._last_index_progress_deg = 0.0
        self._last_index_overshoot_deg = 0.0
        self._index_error_integral = 0.0
        self._last_index_base_cmd_mps = 0.0
        self._last_turner_cmd_rad_s = 0.0
        self._index_motion_committed = False
        self._yaw_error_integral = 0.0
        self._last_yaw_error_deg = 0.0
        self._last_yaw_cmd_rad_s = 0.0
        self._axial_heading_error_integral = 0.0
        self._axial_lateral_error_integral = 0.0
        self._last_axial_heading_error_deg = 0.0
        self._last_axial_lateral_error_deg = 0.0
        self._last_axial_cmd_rad_s = 0.0
        self._align_lateral_error_integral = 0.0
        self._align_lock_start_s: Optional[float] = None
        self._last_align_heading_error_deg = 0.0
        self._last_align_lateral_error_deg = 0.0
        self._last_align_linear_cmd_mps = 0.0
        self._last_align_cmd_rad_s = 0.0

        self._odom: Optional[OdomState] = None
        self._turner: Optional[TurnerState] = None
        self._bottom_lane_locked: Optional[FlagState] = None
        self._safe_to_scan: Optional[FlagState] = None
        self._safe_to_index_tube: Optional[FlagState] = None
        self._stability: Optional[StabilityTelemetry] = None

        self._pub_cmd_vel = self.create_publisher(
            TwistStamped, self._cmd_vel_topic, 10)
        self._pub_turner_cmd = self.create_publisher(
            Float64, self._turner_cmd_vel_topic, 10)
        self._pub_state = self.create_publisher(
            String, '/inspection/state', 10)
        self._pub_state_text = self.create_publisher(
            String, '/inspection/state_text', 10)
        self._pub_current_lane = self.create_publisher(
            String, '/inspection/current_lane', 10)
        self._pub_status = self.create_publisher(
            String, '/inspection/mission_status', 10)
        self._pub_autonomous_active = self.create_publisher(
            Bool, '/inspection/autonomous_active', 10)

        self.create_subscription(Odometry, self._odom_topic, self._odom_cb, 10)
        self.create_subscription(
            String, self._stability_topic, self._stability_cb, 10)
        self.create_subscription(
            String, self._mission_command_topic, self._mission_command_cb, 10)
        self.create_subscription(
            Float64, self._turner_angle_topic, self._turner_cb, 10)
        self.create_subscription(
            Bool, self._bottom_lane_locked_topic, self._bottom_lane_cb, 10)
        self.create_subscription(
            Bool, self._safe_to_scan_topic, self._safe_to_scan_cb, 10)
        self.create_subscription(
            Bool, self._safe_to_index_tube_topic, self._safe_to_index_cb, 10)

        self.create_timer(1.0 / self._publish_rate_hz, self._tick)

        self.get_logger().info(
            'State machine lista: '
            f'auto_start={self._auto_start}, lane_delta={self._lane_delta_theta_deg}deg, '
            f'x=[{self._x_min_m}, {self._x_max_m}]m, axial_speed={self._axial_speed_mps}m/s'
        )

    def _declare_parameters(self):
        self.declare_parameter('auto_start', False)
        self.declare_parameter('cmd_vel_topic', '/robot/platform/cmd_vel')
        self.declare_parameter('turner_cmd_vel_topic', '/turner/cmd_vel')
        self.declare_parameter('odom_topic', '/robot/platform/odom/filtered')
        self.declare_parameter('stability_topic', '/inspection/stability')
        self.declare_parameter('turner_angle_topic', '/turner/angle')
        self.declare_parameter('mission_command_topic', '/inspection/mission_command')
        self.declare_parameter(
            'bottom_lane_locked_topic', '/inspection/bottom_lane_locked')
        self.declare_parameter('safe_to_scan_topic', '/inspection/safe_to_scan')
        self.declare_parameter(
            'safe_to_index_tube_topic', '/inspection/safe_to_index_tube')

        self.declare_parameter('mission.x_min_m', -15.0)
        self.declare_parameter('mission.x_max_m', 15.0)
        self.declare_parameter('mission.axial_axis', 'x')
        self.declare_parameter('mission.lane_length_m', 30.0)
        self.declare_parameter('mission.lane_end_tolerance_m', 0.25)
        self.declare_parameter('mission.target_total_rotation_deg', 360.0)
        self.declare_parameter('mission.lane_delta_theta_deg', 90.0)
        self.declare_parameter('mission.turner_angle_tolerance_deg', 0.5)
        self.declare_parameter('mission.index_overshoot_guard_deg', 2.0)
        self.declare_parameter('mission.settle_time_s', 2.0)
        self.declare_parameter('mission.use_lawnmower_pattern', True)
        self.declare_parameter('mission.use_tangential_indexing', True)
        self.declare_parameter('mission.tangential_yaw_offset_deg', 90.0)

        self.declare_parameter('control.axial_speed_mps', 0.05)
        self.declare_parameter('control.max_angular_z_rad_s', 0.20)
        self.declare_parameter('control.k_heading', 0.8)
        self.declare_parameter('control.k_axial_heading_p', 0.8)
        self.declare_parameter('control.k_axial_heading_i', 0.0)
        self.declare_parameter('control.max_axial_heading_integral_deg_s', 30.0)
        self.declare_parameter('control.k_axial_lateral_p', 0.0)
        self.declare_parameter('control.k_axial_lateral_i', 0.0)
        self.declare_parameter('control.axial_lateral_sign', 1.0)
        self.declare_parameter('control.max_axial_lateral_integral_deg_s', 30.0)
        self.declare_parameter('control.align_linear_speed_mps', 0.02)
        self.declare_parameter('control.align_max_angular_z_rad_s', 0.15)
        self.declare_parameter('control.k_align_heading_p', 0.5)
        self.declare_parameter('control.k_align_lateral_p', 0.8)
        self.declare_parameter('control.k_align_lateral_i', 0.0)
        self.declare_parameter('control.align_lateral_sign', 1.0)
        self.declare_parameter('control.max_align_lateral_integral_deg_s', 30.0)
        self.declare_parameter('control.turner_speed_rad_s', 0.02)
        self.declare_parameter('control.publish_rate_hz', 10.0)
        self.declare_parameter('control.rotate_angular_speed_rad_s', 0.30)
        self.declare_parameter('control.rotate_yaw_tolerance_deg', 5.0)
        self.declare_parameter('control.k_rotate_yaw_p', 1.0)
        self.declare_parameter('control.k_rotate_yaw_i', 0.0)
        self.declare_parameter('control.max_rotate_yaw_integral_deg_s', 30.0)
        self.declare_parameter('control.tube_radius_m', 3.925)
        self.declare_parameter('control.index_compensation_sign', 1.0)
        self.declare_parameter('control.max_index_linear_speed_mps', 0.45)
        self.declare_parameter('control.k_index_lateral_p', 0.0)
        self.declare_parameter('control.k_index_lateral_i', 0.0)

        self.declare_parameter('safety.max_odom_age_s', 0.5)
        self.declare_parameter('safety.max_turner_age_s', 1.0)
        self.declare_parameter('safety.max_flags_age_s', 0.5)
        self.declare_parameter('safety.max_stability_age_s', 2.0)
        self.declare_parameter('safety.require_bottom_lane_lock', True)
        self.declare_parameter('safety.require_safe_to_scan_for_axial', True)
        self.declare_parameter('safety.require_safe_to_index_tube', True)
        self.declare_parameter('safety.align_lock_hold_s', 1.0)

    def _load_parameters(self):
        self._auto_start = bool(self.get_parameter('auto_start').value)
        self._cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self._turner_cmd_vel_topic = self.get_parameter('turner_cmd_vel_topic').value
        self._odom_topic = self.get_parameter('odom_topic').value
        self._stability_topic = self.get_parameter('stability_topic').value
        self._turner_angle_topic = self.get_parameter('turner_angle_topic').value
        self._mission_command_topic = self.get_parameter('mission_command_topic').value
        self._bottom_lane_locked_topic = self.get_parameter(
            'bottom_lane_locked_topic').value
        self._safe_to_scan_topic = self.get_parameter('safe_to_scan_topic').value
        self._safe_to_index_tube_topic = self.get_parameter(
            'safe_to_index_tube_topic').value

        self._x_min_m = float(self.get_parameter('mission.x_min_m').value)
        self._x_max_m = float(self.get_parameter('mission.x_max_m').value)
        self._axial_axis = str(self.get_parameter('mission.axial_axis').value).lower()
        if self._axial_axis not in ('x', 'y'):
            self.get_logger().warn(
                f'Valor mission.axial_axis={self._axial_axis} no válido; usando x')
            self._axial_axis = 'x'
        self._lane_length_m = max(
            0.1, float(self.get_parameter('mission.lane_length_m').value))
        self._lane_end_tolerance_m = float(
            self.get_parameter('mission.lane_end_tolerance_m').value)
        self._target_total_rotation_deg = float(
            self.get_parameter('mission.target_total_rotation_deg').value)
        self._lane_delta_theta_deg = float(
            self.get_parameter('mission.lane_delta_theta_deg').value)
        self._turner_angle_tolerance_rad = math.radians(float(
            self.get_parameter('mission.turner_angle_tolerance_deg').value))
        self._index_overshoot_guard_rad = math.radians(abs(float(
            self.get_parameter('mission.index_overshoot_guard_deg').value)))
        self._settle_time_s = float(self.get_parameter('mission.settle_time_s').value)
        self._use_lawnmower_pattern = bool(
            self.get_parameter('mission.use_lawnmower_pattern').value)
        self._use_tangential_indexing = bool(
            self.get_parameter('mission.use_tangential_indexing').value)
        self._tangential_yaw_offset_rad = math.radians(float(
            self.get_parameter('mission.tangential_yaw_offset_deg').value))

        self._axial_speed_mps = float(
            self.get_parameter('control.axial_speed_mps').value)
        self._max_angular_z_rad_s = float(
            self.get_parameter('control.max_angular_z_rad_s').value)
        self._k_heading = float(self.get_parameter('control.k_heading').value)
        self._k_axial_heading_p = float(
            self.get_parameter('control.k_axial_heading_p').value)
        self._k_axial_heading_i = float(
            self.get_parameter('control.k_axial_heading_i').value)
        self._max_axial_heading_integral_rad_s = math.radians(abs(float(
            self.get_parameter('control.max_axial_heading_integral_deg_s').value)))
        self._k_axial_lateral_p = float(
            self.get_parameter('control.k_axial_lateral_p').value)
        self._k_axial_lateral_i = float(
            self.get_parameter('control.k_axial_lateral_i').value)
        self._axial_lateral_sign = float(
            self.get_parameter('control.axial_lateral_sign').value)
        self._max_axial_lateral_integral_rad_s = math.radians(abs(float(
            self.get_parameter('control.max_axial_lateral_integral_deg_s').value)))
        self._align_linear_speed_mps = abs(float(
            self.get_parameter('control.align_linear_speed_mps').value))
        self._align_max_angular_z_rad_s = abs(float(
            self.get_parameter('control.align_max_angular_z_rad_s').value))
        self._k_align_heading_p = float(
            self.get_parameter('control.k_align_heading_p').value)
        self._k_align_lateral_p = float(
            self.get_parameter('control.k_align_lateral_p').value)
        self._k_align_lateral_i = float(
            self.get_parameter('control.k_align_lateral_i').value)
        self._align_lateral_sign = float(
            self.get_parameter('control.align_lateral_sign').value)
        self._max_align_lateral_integral_rad_s = math.radians(abs(float(
            self.get_parameter('control.max_align_lateral_integral_deg_s').value)))
        self._turner_speed_rad_s = float(
            self.get_parameter('control.turner_speed_rad_s').value)
        self._publish_rate_hz = max(
            1.0, float(self.get_parameter('control.publish_rate_hz').value))
        self._rotate_angular_speed_rad_s = abs(float(
            self.get_parameter('control.rotate_angular_speed_rad_s').value))
        self._rotate_yaw_tolerance_rad = math.radians(float(
            self.get_parameter('control.rotate_yaw_tolerance_deg').value))
        self._k_rotate_yaw_p = float(
            self.get_parameter('control.k_rotate_yaw_p').value)
        self._k_rotate_yaw_i = float(
            self.get_parameter('control.k_rotate_yaw_i').value)
        self._max_rotate_yaw_integral_rad_s = math.radians(abs(float(
            self.get_parameter('control.max_rotate_yaw_integral_deg_s').value)))
        self._tube_radius_m = float(self.get_parameter('control.tube_radius_m').value)
        self._index_compensation_sign = float(
            self.get_parameter('control.index_compensation_sign').value)
        self._max_index_linear_speed_mps = abs(float(
            self.get_parameter('control.max_index_linear_speed_mps').value))
        self._k_index_lateral_p = float(
            self.get_parameter('control.k_index_lateral_p').value)
        self._k_index_lateral_i = float(
            self.get_parameter('control.k_index_lateral_i').value)

        self._max_odom_age_s = float(
            self.get_parameter('safety.max_odom_age_s').value)
        self._max_turner_age_s = float(
            self.get_parameter('safety.max_turner_age_s').value)
        self._max_flags_age_s = float(
            self.get_parameter('safety.max_flags_age_s').value)
        self._max_stability_age_s = float(
            self.get_parameter('safety.max_stability_age_s').value)
        self._require_bottom_lane_lock = bool(
            self.get_parameter('safety.require_bottom_lane_lock').value)
        self._require_safe_to_scan = bool(
            self.get_parameter('safety.require_safe_to_scan_for_axial').value)
        self._require_safe_to_index = bool(
            self.get_parameter('safety.require_safe_to_index_tube').value)
        self._align_lock_hold_s = float(
            self.get_parameter('safety.align_lock_hold_s').value)

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _fresh(self, state, max_age_s: float) -> bool:
        return state is not None and (self._now_s() - state.stamp_s) <= max_age_s

    def _odom_cb(self, msg: Odometry):
        self._odom = OdomState(
            stamp_s=self._now_s(),
            x_m=float(msg.pose.pose.position.x),
            y_m=float(msg.pose.pose.position.y),
            yaw_rad=_quat_to_yaw(msg.pose.pose.orientation),
        )

    def _turner_cb(self, msg: Float64):
        self._turner = TurnerState(
            stamp_s=self._now_s(),
            theta_rad=float(msg.data),
        )

    def _stability_cb(self, msg: String):
        try:
            payload = json.loads(msg.data)
            imu = payload.get('imu') or {}
            self._stability = StabilityTelemetry(
                stamp_s=self._now_s(),
                lateral_angle_deg=float(imu.get('lateral_angle_deg', 0.0)),
                roll_deg=float(imu.get('roll_deg', 0.0)),
                pitch_deg=float(imu.get('pitch_deg', 0.0)),
            )
        except (TypeError, ValueError, json.JSONDecodeError):
            return

    def _mission_command_cb(self, msg: String):
        command = msg.data.strip().upper()
        if command in ('START', 'START_AUTO', 'AUTO_START'):
            self._start_requested = True
            if self._state in (IDLE, FINISH, ERROR_RECOVERY):
                self._reset_mission_runtime()
                self._transition(VERIFY_BOTTOM_LOCK, 'mission_command_start')
        elif command in ('STOP', 'ABORT', 'IDLE'):
            self._start_requested = False
            self._stop_base()
            self._stop_turner()
            self._transition(IDLE, f'mission_command_{command.lower()}')

    def _reset_mission_runtime(self):
        self._state = IDLE
        self._lane_id = 0
        self._lane_direction = 1.0
        self._lane_start_position_m = None
        self._lane_target_position_m = None
        self._desired_yaw_rad = None
        self._axial_yaw_target_rad = None
        self._tangential_yaw_target_rad = None
        self._index_start_theta_rad = None
        self._index_target_theta_rad = None
        self._settle_start_s = None
        self._rotation_done_deg = 0.0
        self._last_index_error_deg = 0.0
        self._last_index_progress_deg = 0.0
        self._last_index_overshoot_deg = 0.0
        self._index_error_integral = 0.0
        self._last_index_base_cmd_mps = 0.0
        self._last_turner_cmd_rad_s = 0.0
        self._reset_yaw_control()
        self._reset_axial_control()
        self._reset_align_control()

    def _bottom_lane_cb(self, msg: Bool):
        self._bottom_lane_locked = FlagState(self._now_s(), bool(msg.data))

    def _safe_to_scan_cb(self, msg: Bool):
        self._safe_to_scan = FlagState(self._now_s(), bool(msg.data))

    def _safe_to_index_cb(self, msg: Bool):
        self._safe_to_index_tube = FlagState(self._now_s(), bool(msg.data))

    def _transition(self, new_state: str, reason: str):
        if self._state == new_state:
            return
        self.get_logger().info(f'{self._state} -> {new_state}: {reason}')
        self._state = new_state
        self._last_transition_reason = reason
        if new_state == VERIFY_INDEXED_POSITION:
            self._settle_start_s = self._now_s()
        if new_state in (ROTATE_TO_TANGENTIAL, ROTATE_TO_AXIAL):
            self._reset_yaw_control()
        if new_state == AXIAL_SCAN:
            self._reset_axial_control()
        if new_state == ALIGN_TO_BOTTOM_LANE:
            self._reset_align_control()

    def _signals_ready(self) -> bool:
        self._last_input_failure = self._input_freshness()
        return all(self._last_input_failure.values())

    def _input_freshness(self):
        return (
            {
                'odom': self._fresh(self._odom, self._max_odom_age_s),
                'turner': self._fresh(self._turner, self._max_turner_age_s),
                'bottom_lane_locked': self._fresh(
                    self._bottom_lane_locked, self._max_flags_age_s),
                'safe_to_scan': self._fresh(
                    self._safe_to_scan, self._max_flags_age_s),
                'safe_to_index_tube': self._fresh(
                    self._safe_to_index_tube, self._max_flags_age_s),
            }
        )

    def _bottom_locked(self) -> bool:
        if not self._require_bottom_lane_lock:
            return True
        return (
            self._fresh(self._bottom_lane_locked, self._max_flags_age_s) and
            self._bottom_lane_locked.value
        )

    def _scan_allowed(self) -> bool:
        if not self._bottom_locked():
            return False
        if not self._require_safe_to_scan:
            return True
        return (
            self._fresh(self._safe_to_scan, self._max_flags_age_s) and
            self._safe_to_scan.value
        )

    def _index_allowed(self) -> bool:
        if not self._bottom_locked():
            return False
        if not self._require_safe_to_index:
            return True
        return (
            self._fresh(self._safe_to_index_tube, self._max_flags_age_s) and
            self._safe_to_index_tube.value
        )

    def _prepare_lane(self):
        if self._lane_target_position_m is not None:
            return
        if self._use_lawnmower_pattern:
            self._lane_direction = 1.0 if self._lane_id % 2 == 0 else -1.0
        else:
            self._lane_direction = 1.0
        self._lane_start_position_m = self._axial_position_m()
        self._lane_target_position_m = (
            self._lane_start_position_m + self._lane_direction * self._lane_length_m
        )
        self._desired_yaw_rad = self._odom.yaw_rad
        self._axial_yaw_target_rad = self._desired_yaw_rad
        self._reset_axial_control()

    def _axial_position_m(self) -> float:
        if self._odom is None:
            return 0.0
        return self._odom.y_m if self._axial_axis == 'y' else self._odom.x_m

    def _lane_progress_m(self) -> float:
        if self._lane_start_position_m is None:
            return 0.0
        return abs(self._axial_position_m() - self._lane_start_position_m)

    def _lane_finished(self) -> bool:
        if self._lane_target_position_m is None or self._odom is None:
            return False
        return self._lane_progress_m() >= self._lane_length_m

    def _prepare_index(self):
        self._index_start_theta_rad = self._turner.theta_rad
        delta_rad = math.radians(self._lane_delta_theta_deg)
        self._index_target_theta_rad = self._index_start_theta_rad + delta_rad
        self._last_index_error_deg = self._lane_delta_theta_deg
        self._last_index_progress_deg = 0.0
        self._last_index_overshoot_deg = 0.0
        self._index_error_integral = 0.0
        self._last_index_base_cmd_mps = 0.0
        self._last_turner_cmd_rad_s = 0.0
        self._index_motion_committed = False

    def _prepare_tangential_rotation(self):
        if self._odom is None:
            return
        self._axial_yaw_target_rad = self._desired_yaw_rad or self._odom.yaw_rad
        # Tangential offset is always in the same world direction regardless of
        # lane direction. The lawnmower reverses linear_x, not the heading.
        self._tangential_yaw_target_rad = _wrap_pi(
            self._axial_yaw_target_rad + self._tangential_yaw_offset_rad
        )

    def _prepare_next_axial_rotation(self):
        # Axial heading stays constant across lanes. lane_direction controls
        # the sign of linear_x in AXIAL_SCAN so the robot reverses without
        # turning around. Both tangential rotations are therefore always in
        # the same angular direction.
        pass

    def _reset_yaw_control(self):
        self._yaw_error_integral = 0.0
        self._last_yaw_error_deg = 0.0
        self._last_yaw_cmd_rad_s = 0.0

    def _reset_axial_control(self):
        self._axial_heading_error_integral = 0.0
        self._axial_lateral_error_integral = 0.0
        self._last_axial_heading_error_deg = 0.0
        self._last_axial_lateral_error_deg = 0.0
        self._last_axial_cmd_rad_s = 0.0

    def _reset_align_control(self):
        self._align_lateral_error_integral = 0.0
        self._align_lock_start_s = None
        self._last_align_heading_error_deg = 0.0
        self._last_align_lateral_error_deg = 0.0
        self._last_align_linear_cmd_mps = 0.0
        self._last_align_cmd_rad_s = 0.0

    def _alignment_locked_for_hold_time(self) -> bool:
        if self._scan_allowed():
            if self._align_lock_start_s is None:
                self._align_lock_start_s = self._now_s()
            return self._now_s() - self._align_lock_start_s >= self._align_lock_hold_s
        self._align_lock_start_s = None
        return False

    def _publish_align_to_bottom_lane_cmd(self):
        if self._odom is None:
            self._stop_base()
            return
        dt = 1.0 / self._publish_rate_hz
        target_yaw = (
            self._axial_yaw_target_rad
            if self._axial_yaw_target_rad is not None else self._odom.yaw_rad
        )
        heading_error = _wrap_pi(target_yaw - self._odom.yaw_rad)

        lateral_error = 0.0
        if self._fresh(self._stability, self._max_stability_age_s):
            lateral_error = math.radians(self._stability.lateral_angle_deg)
        self._align_lateral_error_integral += lateral_error * dt
        self._align_lateral_error_integral = max(
            -self._max_align_lateral_integral_rad_s,
            min(
                self._max_align_lateral_integral_rad_s,
                self._align_lateral_error_integral,
            ),
        )

        angular_z = (
            self._k_align_heading_p * heading_error +
            self._align_lateral_sign * (
                self._k_align_lateral_p * lateral_error +
                self._k_align_lateral_i * self._align_lateral_error_integral
            )
        )
        angular_z = max(
            -self._align_max_angular_z_rad_s,
            min(self._align_max_angular_z_rad_s, angular_z),
        )
        self._last_align_heading_error_deg = math.degrees(heading_error)
        self._last_align_lateral_error_deg = math.degrees(lateral_error)
        self._last_align_linear_cmd_mps = self._align_linear_speed_mps
        self._last_align_cmd_rad_s = angular_z
        self._publish_cmd_vel(self._align_linear_speed_mps, angular_z)

    def _axial_scan_angular_command(self, target_yaw_rad: Optional[float]) -> float:
        if target_yaw_rad is None or self._odom is None:
            return 0.0
        dt = 1.0 / self._publish_rate_hz

        heading_error = _wrap_pi(target_yaw_rad - self._odom.yaw_rad)
        self._axial_heading_error_integral += heading_error * dt
        self._axial_heading_error_integral = max(
            -self._max_axial_heading_integral_rad_s,
            min(
                self._max_axial_heading_integral_rad_s,
                self._axial_heading_error_integral,
            ),
        )

        lateral_error = 0.0
        if self._fresh(self._stability, self._max_stability_age_s):
            lateral_error = math.radians(self._stability.lateral_angle_deg)
        self._axial_lateral_error_integral += lateral_error * dt
        self._axial_lateral_error_integral = max(
            -self._max_axial_lateral_integral_rad_s,
            min(
                self._max_axial_lateral_integral_rad_s,
                self._axial_lateral_error_integral,
            ),
        )

        angular_z = (
            self._k_axial_heading_p * heading_error +
            self._k_axial_heading_i * self._axial_heading_error_integral +
            self._axial_lateral_sign * (
                self._k_axial_lateral_p * lateral_error +
                self._k_axial_lateral_i * self._axial_lateral_error_integral
            )
        )
        angular_z = max(
            -self._max_angular_z_rad_s,
            min(self._max_angular_z_rad_s, angular_z),
        )
        self._last_axial_heading_error_deg = math.degrees(heading_error)
        self._last_axial_lateral_error_deg = math.degrees(lateral_error)
        self._last_axial_cmd_rad_s = angular_z
        return angular_z

    def _yaw_pi_command(self, target_yaw_rad: Optional[float]) -> Optional[float]:
        if target_yaw_rad is None or self._odom is None:
            return None
        error = _wrap_pi(target_yaw_rad - self._odom.yaw_rad)
        self._last_yaw_error_deg = math.degrees(error)
        dt = 1.0 / self._publish_rate_hz
        self._yaw_error_integral += error * dt
        self._yaw_error_integral = max(
            -self._max_rotate_yaw_integral_rad_s,
            min(self._max_rotate_yaw_integral_rad_s, self._yaw_error_integral),
        )
        angular_z = (
            self._k_rotate_yaw_p * error +
            self._k_rotate_yaw_i * self._yaw_error_integral
        )
        angular_z = max(
            -self._rotate_angular_speed_rad_s,
            min(self._rotate_angular_speed_rad_s, angular_z),
        )
        self._last_yaw_cmd_rad_s = angular_z
        return angular_z

    def _rotate_to_yaw(self, target_yaw_rad: Optional[float]) -> bool:
        if target_yaw_rad is None or self._odom is None:
            self._stop_base()
            return False
        error = _wrap_pi(target_yaw_rad - self._odom.yaw_rad)
        self._last_yaw_error_deg = math.degrees(error)
        if abs(error) <= self._rotate_yaw_tolerance_rad:
            self._stop_base()
            self._reset_yaw_control()
            return True
        angular_z = self._yaw_pi_command(target_yaw_rad)
        if angular_z is None:
            self._stop_base()
            return False
        self._publish_cmd_vel(0.0, angular_z)
        return False

    def _publish_index_compensation(self, turner_velocity_rad_s: float):
        lateral_error_deg = 0.0
        if self._fresh(self._stability, self._max_stability_age_s):
            lateral_error_deg = self._stability.lateral_angle_deg
        dt = 1.0 / self._publish_rate_hz
        self._index_error_integral += lateral_error_deg * dt
        surface_speed_mps = self._tube_radius_m * turner_velocity_rad_s
        correction = (
            self._k_index_lateral_p * lateral_error_deg +
            self._k_index_lateral_i * self._index_error_integral
        )
        linear_x = self._index_compensation_sign * surface_speed_mps + correction
        linear_x = max(
            -self._max_index_linear_speed_mps,
            min(self._max_index_linear_speed_mps, linear_x),
        )
        yaw_target = self._tangential_yaw_target_rad
        angular_z = self._yaw_pi_command(yaw_target)
        if angular_z is None:
            angular_z = 0.0
        self._last_index_base_cmd_mps = linear_x
        self._last_turner_cmd_rad_s = turner_velocity_rad_s
        self._publish_cmd_vel(linear_x, angular_z)

    def _index_error_rad(self) -> float:
        if (
            self._index_start_theta_rad is None or
            self._index_target_theta_rad is None or
            self._turner is None
        ):
            self._last_index_error_deg = 0.0
            self._last_index_progress_deg = 0.0
            self._last_index_overshoot_deg = 0.0
            return 0.0
        desired_delta = self._index_target_theta_rad - self._index_start_theta_rad
        measured_delta = _wrap_pi(self._turner.theta_rad - self._index_start_theta_rad)
        error = desired_delta - measured_delta
        self._last_index_error_deg = math.degrees(error)
        self._last_index_progress_deg = math.degrees(measured_delta)
        self._last_index_overshoot_deg = math.degrees(max(0.0, -error))
        return error

    def _index_target_reached_or_overshot(self, error_rad: float) -> bool:
        if abs(error_rad) <= self._turner_angle_tolerance_rad:
            return True
        return error_rad < -self._index_overshoot_guard_rad

    def _publish_cmd_vel(self, linear_x: float, angular_z: float):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'state_machine'
        msg.twist.linear.x = float(linear_x)
        msg.twist.angular.z = float(angular_z)
        self._pub_cmd_vel.publish(msg)

    def _stop_base(self):
        self._publish_cmd_vel(0.0, 0.0)

    def _publish_turner(self, velocity_rad_s: float):
        self._pub_turner_cmd.publish(Float64(data=float(velocity_rad_s)))

    def _stop_turner(self):
        self._publish_turner(0.0)

    def _tick(self):
        if self._state == IDLE and self._start_requested:
            self._transition(VERIFY_BOTTOM_LOCK, 'auto_start')

        if self._state in (IDLE, FINISH, ERROR_RECOVERY):
            self._stop_base()
            self._stop_turner()
            self._publish_state()
            return

        if not self._signals_ready():
            self._stop_base()
            self._stop_turner()
            if self._state in (
                ALIGN_TO_BOTTOM_LANE,
                VERIFY_BOTTOM_LOCK,
                AXIAL_SCAN,
                RETURN_TO_BOTTOM_LANE,
                WAIT_SAFE_TO_INDEX,
                ROTATE_TO_TANGENTIAL,
                INDEX_TUBE,
                ROTATE_TO_AXIAL,
                VERIFY_INDEXED_POSITION,
            ):
                self._last_transition_reason = 'waiting_for_fresh_inputs'
                self._publish_state()
                return
            self._transition(ERROR_RECOVERY, 'stale_or_missing_inputs')
            self._publish_state()
            return

        if self._state == VERIFY_BOTTOM_LOCK:
            self._stop_base()
            self._stop_turner()
            if self._scan_allowed():
                self._prepare_lane()
                self._transition(AXIAL_SCAN, 'bottom_lane_verified')
            else:
                self._transition(ALIGN_TO_BOTTOM_LANE, 'bottom_lane_not_locked')

        elif self._state == ALIGN_TO_BOTTOM_LANE:
            self._stop_turner()
            if self._alignment_locked_for_hold_time():
                self._stop_base()
                self._transition(VERIFY_BOTTOM_LOCK, 'bottom_lane_aligned')
            else:
                self._publish_align_to_bottom_lane_cmd()

        elif self._state == AXIAL_SCAN:
            self._stop_turner()
            if self._lane_finished():
                self._stop_base()
                if self._rotation_done_deg >= self._target_total_rotation_deg:
                    self._transition(FINISH, 'target_rotation_completed')
                elif self._index_allowed():
                    self._prepare_index()
                    if self._use_tangential_indexing:
                        self._prepare_tangential_rotation()
                        self._transition(ROTATE_TO_TANGENTIAL, 'lane_finished')
                    else:
                        self._transition(INDEX_TUBE, 'lane_finished')
                else:
                    self._transition(WAIT_SAFE_TO_INDEX, 'waiting_safe_to_index')
            elif not self._scan_allowed():
                self._stop_base()
                self._transition(RETURN_TO_BOTTOM_LANE, 'scan_not_safe')
            else:
                angular_z = self._axial_scan_angular_command(self._desired_yaw_rad)
                linear_x = self._axial_speed_mps * self._lane_direction
                self._publish_cmd_vel(linear_x, angular_z)

        elif self._state == RETURN_TO_BOTTOM_LANE:
            self._stop_base()
            self._stop_turner()
            if self._scan_allowed():
                self._transition(VERIFY_BOTTOM_LOCK, 'bottom_lane_recovered')
            else:
                self._transition(ALIGN_TO_BOTTOM_LANE, 'return_requires_active_alignment')

        elif self._state == WAIT_SAFE_TO_INDEX:
            self._stop_base()
            self._stop_turner()
            if self._index_allowed():
                self._prepare_index()
                if self._use_tangential_indexing:
                    self._prepare_tangential_rotation()
                    self._transition(ROTATE_TO_TANGENTIAL, 'safe_to_index_ready')
                else:
                    self._transition(INDEX_TUBE, 'safe_to_index_ready')

        elif self._state == ROTATE_TO_TANGENTIAL:
            self._stop_turner()
            if self._rotate_to_yaw(self._tangential_yaw_target_rad):
                self._transition(INDEX_TUBE, 'tangential_yaw_reached')

        elif self._state == INDEX_TUBE:
            # Pre-motion gate: only before first command; after that only bottom lock matters
            if self._index_motion_committed and not self._bottom_locked():
                self._stop_base()
                self._stop_turner()
                self._index_motion_committed = False
                self._transition(ALIGN_TO_BOTTOM_LANE, 'index_bottom_lost')
            elif not self._index_motion_committed and not self._index_allowed():
                self._stop_base()
                self._stop_turner()
                self._transition(WAIT_SAFE_TO_INDEX, 'index_not_safe')
            else:
                if self._index_target_theta_rad is None:
                    self._prepare_index()
                    self._last_transition_reason = 'index_target_recreated'
                error = self._index_error_rad()
                if self._index_target_reached_or_overshot(error):
                    self._stop_base()
                    self._stop_turner()
                    self._index_motion_committed = False
                    self._rotation_done_deg += abs(self._lane_delta_theta_deg)
                    if self._use_tangential_indexing:
                        self._prepare_next_axial_rotation()
                    self._lane_id += 1
                    self._lane_start_position_m = None
                    self._lane_target_position_m = None
                    if not self._use_tangential_indexing:
                        self._desired_yaw_rad = None
                    reason = (
                        'index_target_reached'
                        if abs(error) <= self._turner_angle_tolerance_rad
                        else 'index_target_overshoot_guard'
                    )
                    if self._use_tangential_indexing:
                        self._transition(ROTATE_TO_AXIAL, reason)
                    else:
                        self._transition(VERIFY_INDEXED_POSITION, reason)
                else:
                    direction = 1.0 if error > 0.0 else -1.0
                    turner_cmd = direction * self._turner_speed_rad_s
                    self._publish_turner(turner_cmd)
                    self._index_motion_committed = True
                    if self._use_tangential_indexing:
                        self._publish_index_compensation(turner_cmd)
                    else:
                        self._stop_base()

        elif self._state == ROTATE_TO_AXIAL:
            self._stop_turner()
            if self._rotate_to_yaw(self._axial_yaw_target_rad):
                self._transition(ALIGN_TO_BOTTOM_LANE, 'axial_yaw_reached')

        elif self._state == VERIFY_INDEXED_POSITION:
            self._stop_base()
            self._stop_turner()
            settled = (
                self._settle_start_s is not None and
                self._now_s() - self._settle_start_s >= self._settle_time_s
            )
            if settled and self._scan_allowed():
                if self._rotation_done_deg >= self._target_total_rotation_deg:
                    self._transition(FINISH, 'target_rotation_completed')
                else:
                    self._prepare_lane()
                    self._transition(AXIAL_SCAN, 'indexed_position_verified')
            elif settled:
                self._transition(ALIGN_TO_BOTTOM_LANE, 'indexed_position_not_aligned')

        self._publish_state()

    def _publish_state(self):
        nominal_coverage_enabled = self._state == AXIAL_SCAN and self._scan_allowed()
        lane_theta_deg = self._lane_id * self._lane_delta_theta_deg
        status = {
            'state': self._state,
            'lane_id': self._lane_id,
            'lane_theta_deg': lane_theta_deg,
            'lane_direction': self._lane_direction,
            'lane_axis': self._axial_axis,
            'lane_start_position_m': self._lane_start_position_m,
            'lane_target_position_m': self._lane_target_position_m,
            'lane_length_m': self._lane_length_m,
            'lane_progress_m': self._lane_progress_m(),
            'rotation_done_deg': self._rotation_done_deg,
            'target_total_rotation_deg': self._target_total_rotation_deg,
            'nominal_coverage_enabled': nominal_coverage_enabled,
            'last_transition_reason': self._last_transition_reason,
            'yaw_targets': {
                'axial_yaw_deg': (
                    math.degrees(self._axial_yaw_target_rad)
                    if self._axial_yaw_target_rad is not None else None
                ),
                'tangential_yaw_deg': (
                    math.degrees(self._tangential_yaw_target_rad)
                    if self._tangential_yaw_target_rad is not None else None
                ),
            },
            'yaw_controller': {
                'last_error_deg': self._last_yaw_error_deg,
                'last_cmd_rad_s': self._last_yaw_cmd_rad_s,
                'k_p': self._k_rotate_yaw_p,
                'k_i': self._k_rotate_yaw_i,
                'tolerance_deg': math.degrees(self._rotate_yaw_tolerance_rad),
            },
            'axial_controller': {
                'heading_error_deg': self._last_axial_heading_error_deg,
                'lateral_error_deg': self._last_axial_lateral_error_deg,
                'last_cmd_rad_s': self._last_axial_cmd_rad_s,
                'k_heading_p': self._k_axial_heading_p,
                'k_heading_i': self._k_axial_heading_i,
                'k_lateral_p': self._k_axial_lateral_p,
                'k_lateral_i': self._k_axial_lateral_i,
                'lateral_sign': self._axial_lateral_sign,
            },
            'align_controller': {
                'heading_error_deg': self._last_align_heading_error_deg,
                'lateral_error_deg': self._last_align_lateral_error_deg,
                'last_linear_cmd_mps': self._last_align_linear_cmd_mps,
                'last_cmd_rad_s': self._last_align_cmd_rad_s,
                'k_heading_p': self._k_align_heading_p,
                'k_lateral_p': self._k_align_lateral_p,
                'k_lateral_i': self._k_align_lateral_i,
                'lateral_sign': self._align_lateral_sign,
                'lock_hold_s': self._align_lock_hold_s,
                'locked_hold_elapsed_s': (
                    self._now_s() - self._align_lock_start_s
                    if self._align_lock_start_s is not None else 0.0
                ),
            },
            'index_compensation': {
                'enabled': self._use_tangential_indexing,
                'tube_radius_m': self._tube_radius_m,
                'compensation_sign': self._index_compensation_sign,
                'last_base_cmd_mps': self._last_index_base_cmd_mps,
                'last_turner_cmd_rad_s': self._last_turner_cmd_rad_s,
                'lateral_angle_deg': (
                    self._stability.lateral_angle_deg
                    if self._stability is not None else None
                ),
            },
            'index_target': {
                'start_theta_deg': (
                    math.degrees(self._index_start_theta_rad)
                    if self._index_start_theta_rad is not None else None
                ),
                'target_theta_deg': (
                    math.degrees(self._index_target_theta_rad)
                    if self._index_target_theta_rad is not None else None
                ),
                'current_theta_deg': (
                    math.degrees(self._turner.theta_rad)
                    if self._turner is not None else None
                ),
                'error_deg': self._last_index_error_deg,
                'progress_delta_deg': self._last_index_progress_deg,
                'overshoot_deg': self._last_index_overshoot_deg,
                'tolerance_deg': math.degrees(self._turner_angle_tolerance_rad),
                'overshoot_guard_deg': math.degrees(self._index_overshoot_guard_rad),
                'delta_theta_deg': self._lane_delta_theta_deg,
            },
            'safety_config': {
                'require_bottom_lane_lock': self._require_bottom_lane_lock,
                'require_safe_to_scan': self._require_safe_to_scan,
                'require_safe_to_index': self._require_safe_to_index,
            },
            'inputs': {
                'signals_ready': self._signals_ready(),
                'fresh': self._last_input_failure,
                'bottom_lane_locked': (
                    self._bottom_lane_locked.value
                    if self._bottom_lane_locked is not None else None
                ),
                'safe_to_scan': (
                    self._safe_to_scan.value
                    if self._safe_to_scan is not None else None
                ),
                'safe_to_index_tube': (
                    self._safe_to_index_tube.value
                    if self._safe_to_index_tube is not None else None
                ),
                'x_m': self._odom.x_m if self._odom is not None else None,
                'y_m': self._odom.y_m if self._odom is not None else None,
                'axial_position_m': (
                    self._axial_position_m() if self._odom is not None else None
                ),
                'theta_tube_deg': (
                    math.degrees(self._turner.theta_rad)
                    if self._turner is not None else None
                ),
            },
        }
        payload = json.dumps(status)
        self._pub_state.publish(String(data=payload))
        self._pub_state_text.publish(String(data=self._state))
        self._pub_current_lane.publish(String(data=json.dumps({
            'lane_id': self._lane_id,
            'lane_theta_deg': lane_theta_deg,
            'direction': self._lane_direction,
            'axis': self._axial_axis,
            'start_position_m': self._lane_start_position_m,
            'target_position_m': self._lane_target_position_m,
            'progress_m': self._lane_progress_m(),
        })))
        self._pub_status.publish(String(data=payload))
        self._pub_autonomous_active.publish(Bool(data=self._autonomous_active()))

    def _autonomous_active(self) -> bool:
        return self._state not in (IDLE, FINISH, ERROR_RECOVERY)


def main(args=None):
    rclpy.init(args=args)
    node = InspectionStateMachineNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        node._stop_base()
        node._stop_turner()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
