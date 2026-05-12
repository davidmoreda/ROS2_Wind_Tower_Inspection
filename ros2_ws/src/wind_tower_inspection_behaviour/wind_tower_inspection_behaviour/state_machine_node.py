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
RECOVER_BOTTOM_AFTER_INDEX = 'RECOVER_BOTTOM_AFTER_INDEX'
DESCEND_TO_BOTTOM_LANE = 'DESCEND_TO_BOTTOM_LANE'
REALIGN_AXIAL_YAW = 'REALIGN_AXIAL_YAW'
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
    linear_speed_mps: float
    yaw_rate_rad_s: float


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
        self._last_index_surface_speed_mps = 0.0
        self._last_index_lateral_correction_mps = 0.0
        self._index_motion_committed = False
        self._index_bottom_loss_start_s: Optional[float] = None
        self._yaw_error_integral = 0.0
        self._last_yaw_error_deg = 0.0
        self._last_yaw_cmd_rad_s = 0.0
        self._axial_heading_error_integral = 0.0
        self._axial_lateral_error_integral = 0.0
        self._last_axial_heading_error_deg = 0.0
        self._last_axial_lateral_error_deg = 0.0
        self._last_axial_cmd_rad_s = 0.0
        self._last_axial_mode = 'idle'
        self._align_lateral_error_integral = 0.0
        self._align_lock_start_s: Optional[float] = None
        self._last_align_heading_error_deg = 0.0
        self._last_align_lateral_error_deg = 0.0
        self._last_align_ready = False
        self._last_align_ready_reason = 'not_evaluated'
        self._last_align_mode = 'idle'
        self._last_align_linear_cmd_mps = 0.0
        self._last_align_cmd_rad_s = 0.0
        self._align_force_yaw_phase = False
        self._align_post_axial_phase: Optional[str] = None
        self._align_target_yaw_override_rad: Optional[float] = None
        self._recover_bottom_lock_start_s: Optional[float] = None
        self._last_recover_bottom_ready = False
        self._last_recover_bottom_reason = 'not_evaluated'
        self._descend_lock_start_s: Optional[float] = None
        self._last_descend_ready = False
        self._last_descend_reason = 'not_evaluated'

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
        self._pub_control_debug = self.create_publisher(
            String, '/inspection/control_debug', 10)

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

        # Debug cache for /inspection/control_debug. This is observability only and
        # does not affect control outputs.
        self._last_control_debug = None

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
        self.declare_parameter('control.k_axial_lateral_p', 1.0)
        self.declare_parameter('control.k_axial_lateral_i', 0.15)
        self.declare_parameter('control.axial_lateral_sign', 1.0)
        self.declare_parameter('control.max_axial_lateral_integral_deg_s', 30.0)
        self.declare_parameter('control.axial_yaw_priority_deg', 2.0)
        self.declare_parameter('control.align_linear_speed_mps', 0.01)
        self.declare_parameter('control.align_max_angular_z_rad_s', 0.05)
        self.declare_parameter('control.k_align_heading_p', 0.35)
        self.declare_parameter('control.k_align_lateral_p', 2.0)
        self.declare_parameter('control.k_align_lateral_i', 0.0)
        self.declare_parameter('control.align_lateral_sign', 1.0)
        self.declare_parameter('control.max_align_lateral_integral_deg_s', 30.0)
        self.declare_parameter('control.turner_speed_rad_s', 0.02)
        self.declare_parameter('control.publish_rate_hz', 10.0)
        self.declare_parameter('control.rotate_angular_speed_rad_s', 0.30)
        self.declare_parameter('control.rotate_yaw_tolerance_deg', 2.0)
        self.declare_parameter('control.k_rotate_yaw_p', 1.0)
        self.declare_parameter('control.k_rotate_yaw_i', 0.0)
        self.declare_parameter('control.max_rotate_yaw_integral_deg_s', 30.0)
        self.declare_parameter('control.k_rotate_lateral_p', 0.5)
        self.declare_parameter('control.tube_radius_m', 3.925)
        self.declare_parameter('control.index_compensation_gain', 1.0)
        self.declare_parameter('control.index_compensation_sign', 1.0)
        self.declare_parameter('control.index_compensation_alternate_by_lane', True)
        self.declare_parameter('control.max_index_linear_speed_mps', 0.45)
        self.declare_parameter('control.k_index_lateral_p', 0.0)
        self.declare_parameter('control.k_index_lateral_i', 0.0)
        self.declare_parameter('control.index_lateral_sign', 1.0)

        self.declare_parameter('safety.max_odom_age_s', 0.5)
        self.declare_parameter('safety.max_turner_age_s', 1.0)
        self.declare_parameter('safety.max_flags_age_s', 0.5)
        self.declare_parameter('safety.max_stability_age_s', 2.0)
        self.declare_parameter('safety.require_bottom_lane_lock', True)
        self.declare_parameter('safety.require_safe_to_scan_for_axial', True)
        self.declare_parameter('safety.require_safe_to_index_tube', True)
        self.declare_parameter('safety.abort_index_on_bottom_loss', False)
        self.declare_parameter('safety.index_bottom_loss_grace_s', 1.0)
        self.declare_parameter('safety.recover_bottom_hold_s', 1.0)
        self.declare_parameter('safety.recover_bottom_require_pitch', False)
        self.declare_parameter('safety.recover_bottom_max_roll_deg', 0.7)
        self.declare_parameter('safety.recover_bottom_max_pitch_deg', 1.0)
        self.declare_parameter('safety.recover_bottom_max_lateral_angle_deg', 0.7)
        self.declare_parameter('safety.recover_bottom_max_linear_speed_mps', 0.01)
        self.declare_parameter('safety.recover_bottom_max_yaw_rate_rad_s', 0.02)
        self.declare_parameter('safety.descend_bottom_hold_s', 1.0)
        self.declare_parameter('safety.descend_bottom_max_lateral_angle_deg', 0.7)
        self.declare_parameter('safety.align_lock_hold_s', 1.5)
        self.declare_parameter('safety.align_max_lateral_angle_deg', 0.7)
        self.declare_parameter('safety.align_yaw_tolerance_deg', 1.5)
        self.declare_parameter('safety.align_yaw_lateral_capture_deg', 5.0)
        self.declare_parameter('safety.align_yaw_reacquire_deg', 15.0)

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
        self._axial_yaw_priority_deg = float(
            self.get_parameter('control.axial_yaw_priority_deg').value)
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
        self._k_rotate_lateral_p = float(
            self.get_parameter('control.k_rotate_lateral_p').value)
        self._tube_radius_m = float(self.get_parameter('control.tube_radius_m').value)
        self._index_compensation_gain = float(
            self.get_parameter('control.index_compensation_gain').value)
        self._index_compensation_sign = float(
            self.get_parameter('control.index_compensation_sign').value)
        self._index_compensation_alternate_by_lane = bool(
            self.get_parameter('control.index_compensation_alternate_by_lane').value)
        self._max_index_linear_speed_mps = abs(float(
            self.get_parameter('control.max_index_linear_speed_mps').value))
        self._k_index_lateral_p = float(
            self.get_parameter('control.k_index_lateral_p').value)
        self._k_index_lateral_i = float(
            self.get_parameter('control.k_index_lateral_i').value)
        self._index_lateral_sign = float(
            self.get_parameter('control.index_lateral_sign').value)

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
        self._abort_index_on_bottom_loss = bool(
            self.get_parameter('safety.abort_index_on_bottom_loss').value)
        self._index_bottom_loss_grace_s = float(
            self.get_parameter('safety.index_bottom_loss_grace_s').value)
        self._recover_bottom_hold_s = float(
            self.get_parameter('safety.recover_bottom_hold_s').value)
        self._recover_bottom_require_pitch = bool(
            self.get_parameter('safety.recover_bottom_require_pitch').value)
        self._recover_bottom_max_roll_deg = float(
            self.get_parameter('safety.recover_bottom_max_roll_deg').value)
        self._recover_bottom_max_pitch_deg = float(
            self.get_parameter('safety.recover_bottom_max_pitch_deg').value)
        self._recover_bottom_max_lateral_angle_deg = float(
            self.get_parameter('safety.recover_bottom_max_lateral_angle_deg').value)
        self._recover_bottom_max_linear_speed_mps = float(
            self.get_parameter('safety.recover_bottom_max_linear_speed_mps').value)
        self._recover_bottom_max_yaw_rate_rad_s = float(
            self.get_parameter('safety.recover_bottom_max_yaw_rate_rad_s').value)
        self._descend_bottom_hold_s = float(
            self.get_parameter('safety.descend_bottom_hold_s').value)
        self._descend_bottom_max_lateral_angle_deg = float(
            self.get_parameter('safety.descend_bottom_max_lateral_angle_deg').value)
        self._align_lock_hold_s = float(
            self.get_parameter('safety.align_lock_hold_s').value)
        self._align_max_lateral_angle_deg = float(
            self.get_parameter('safety.align_max_lateral_angle_deg').value)
        self._align_yaw_tolerance_deg = float(
            self.get_parameter('safety.align_yaw_tolerance_deg').value)
        self._align_yaw_lateral_capture_deg = float(
            self.get_parameter('safety.align_yaw_lateral_capture_deg').value)
        self._align_yaw_reacquire_deg = float(
            self.get_parameter('safety.align_yaw_reacquire_deg').value)

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _publish_control_debug(self, controllers: list[dict], *, cmd_linear_x: float, cmd_angular_z: float):
        # Keep payload compact: one JSON per tick with a controllers list.
        yaw_deg = math.degrees(self._odom.yaw_rad) if self._odom is not None else None
        target_yaw_rad = None
        try:
            if self._odom is not None:
                target_yaw_rad = self._current_align_target_yaw()
        except Exception:
            target_yaw_rad = None
        payload = {
            'state': self._state,
            'controllers': controllers,
            'dt': 1.0 / self._publish_rate_hz,
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
            'cmd_linear_x': float(cmd_linear_x),
            'cmd_angular_z': float(cmd_angular_z),
            'recover_bottom_ready': self._last_recover_bottom_ready,
            'recover_bottom_reason': self._last_recover_bottom_reason,
            'angles_deg': {
                'yaw': yaw_deg,
                'roll': (
                    float(self._stability.roll_deg)
                    if self._stability is not None else None
                ),
                'pitch': (
                    float(self._stability.pitch_deg)
                    if self._stability is not None else None
                ),
                'lateral_angle': (
                    float(self._stability.lateral_angle_deg)
                    if self._stability is not None else None
                ),
            },
            'targets_deg': {
                'align_yaw': (
                    math.degrees(target_yaw_rad)
                    if target_yaw_rad is not None else None
                ),
                'axial_yaw': (
                    math.degrees(self._axial_yaw_target_rad)
                    if self._axial_yaw_target_rad is not None else None
                ),
                'tangential_yaw': (
                    math.degrees(self._tangential_yaw_target_rad)
                    if self._tangential_yaw_target_rad is not None else None
                ),
            },
        }
        # Avoid publishing identical payloads repeatedly if a state is holding.
        compact = json.dumps(payload, separators=(',', ':'), ensure_ascii=True)
        if compact == self._last_control_debug:
            return
        self._last_control_debug = compact
        self._pub_control_debug.publish(String(data=compact))

    def _fresh(self, state, max_age_s: float) -> bool:
        return state is not None and (self._now_s() - state.stamp_s) <= max_age_s

    def _odom_cb(self, msg: Odometry):
        vx = float(msg.twist.twist.linear.x)
        vy = float(msg.twist.twist.linear.y)
        self._odom = OdomState(
            stamp_s=self._now_s(),
            x_m=float(msg.pose.pose.position.x),
            y_m=float(msg.pose.pose.position.y),
            yaw_rad=_quat_to_yaw(msg.pose.pose.orientation),
            linear_speed_mps=math.sqrt(vx * vx + vy * vy),
            yaw_rate_rad_s=abs(float(msg.twist.twist.angular.z)),
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
        self._align_force_yaw_phase = False
        self._align_post_axial_phase = None
        self._align_target_yaw_override_rad = None
        self._recover_bottom_lock_start_s = None
        self._last_recover_bottom_ready = False
        self._last_recover_bottom_reason = 'not_evaluated'
        self._descend_lock_start_s = None
        self._last_descend_ready = False
        self._last_descend_reason = 'not_evaluated'
        self._last_control_debug = None

    def _bottom_lane_cb(self, msg: Bool):
        self._bottom_lane_locked = FlagState(self._now_s(), bool(msg.data))

    def _safe_to_scan_cb(self, msg: Bool):
        self._safe_to_scan = FlagState(self._now_s(), bool(msg.data))

    def _safe_to_index_cb(self, msg: Bool):
        self._safe_to_index_tube = FlagState(self._now_s(), bool(msg.data))

    def _transition(self, new_state: str, reason: str):
        if self._state == new_state:
            return
        old_state = self._state
        self.get_logger().info(f'{self._state} -> {new_state}: {reason}')
        self._state = new_state
        self._last_transition_reason = reason
        if new_state == VERIFY_INDEXED_POSITION:
            self._settle_start_s = self._now_s()
        if new_state in (ROTATE_TO_TANGENTIAL, ROTATE_TO_AXIAL):
            self._reset_yaw_control()
        if new_state == AXIAL_SCAN:
            self._reset_axial_control()
        if new_state in (ALIGN_TO_BOTTOM_LANE, RECOVER_BOTTOM_AFTER_INDEX):
            self._reset_align_control()
            self._align_target_yaw_override_rad = None
            # After ROTATE_TO_AXIAL, use an explicit sequence:
            # 1) settle yaw, 2) recover lateral alignment, 3) hold ready.
            if new_state == ALIGN_TO_BOTTOM_LANE and old_state == ROTATE_TO_AXIAL:
                self._align_force_yaw_phase = True
                self._align_post_axial_phase = 'yaw'
            elif new_state == RECOVER_BOTTOM_AFTER_INDEX:
                self._align_force_yaw_phase = False
                self._align_post_axial_phase = 'lateral'
                self._align_target_yaw_override_rad = self._tangential_yaw_target_rad
            else:
                self._align_force_yaw_phase = False
                self._align_post_axial_phase = None
        if new_state == DESCEND_TO_BOTTOM_LANE:
            self._reset_align_control()
            self._descend_lock_start_s = None
            self._last_descend_ready = False
            self._last_descend_reason = 'not_evaluated'
            self._align_target_yaw_override_rad = self._current_realign_target_yaw()
        elif old_state == ALIGN_TO_BOTTOM_LANE:
            self._align_force_yaw_phase = False
            self._align_post_axial_phase = None
            self._align_target_yaw_override_rad = None
        elif old_state == RECOVER_BOTTOM_AFTER_INDEX:
            self._align_force_yaw_phase = False
            self._align_post_axial_phase = None
            self._align_target_yaw_override_rad = None
            self._recover_bottom_lock_start_s = None
        elif old_state == DESCEND_TO_BOTTOM_LANE:
            self._descend_lock_start_s = None

    def _current_align_target_yaw(self) -> float:
        if self._align_target_yaw_override_rad is not None:
            return self._align_target_yaw_override_rad
        return (
            self._axial_yaw_target_rad
            if self._axial_yaw_target_rad is not None else self._odom.yaw_rad
        )

    def _current_realign_target_yaw(self) -> float:
        if self._axial_yaw_target_rad is not None:
            return self._axial_yaw_target_rad
        if self._desired_yaw_rad is not None:
            return self._desired_yaw_rad
        return self._odom.yaw_rad if self._odom is not None else 0.0

    def _signed_bottom_error_rad(self) -> float:
        if not self._fresh(self._stability, self._max_stability_age_s):
            return 0.0
        return math.radians(self._stability.roll_deg)

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
        if self._use_tangential_indexing:
            # In tangential indexing the robot physically turns 180 deg between
            # lanes, so it should always drive forward in its own base frame.
            self._lane_direction = 1.0
        elif self._use_lawnmower_pattern:
            self._lane_direction = 1.0 if self._lane_id % 2 == 0 else -1.0
        else:
            self._lane_direction = 1.0
        self._lane_start_position_m = self._axial_position_m()
        self._lane_target_position_m = (
            self._lane_start_position_m + self._lane_direction * self._lane_length_m
        )
        # Preserve the intended axial heading when it was already computed by
        # the previous maneuver sequence (for example after tangential indexing).
        # Otherwise AXIAL_SCAN would lock onto the current, slightly wrong yaw
        # and carry that bias for the whole lane.
        if self._desired_yaw_rad is None:
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
        self._last_index_surface_speed_mps = 0.0
        self._last_index_lateral_correction_mps = 0.0
        self._index_motion_committed = False
        self._index_bottom_loss_start_s = None

    def _prepare_tangential_rotation(self):
        if self._odom is None:
            return
        if self._desired_yaw_rad is not None:
            self._axial_yaw_target_rad = self._desired_yaw_rad
        elif self._axial_yaw_target_rad is None:
            self._axial_yaw_target_rad = self._odom.yaw_rad
        # Tangential offset is always in the same world direction regardless of
        # lane direction. The lawnmower reverses linear_x, not the heading.
        self._tangential_yaw_target_rad = _wrap_pi(
            self._axial_yaw_target_rad + self._tangential_yaw_offset_rad
        )

    def _prepare_next_axial_rotation(self):
        if self._tangential_yaw_target_rad is None:
            return
        # The second 90 deg rotation must continue in the same direction as
        # ROTATE_TO_TANGENTIAL, leaving the base 180 deg from the previous lane.
        self._axial_yaw_target_rad = _wrap_pi(
            self._tangential_yaw_target_rad + self._tangential_yaw_offset_rad
        )
        self._desired_yaw_rad = self._axial_yaw_target_rad

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
        self._last_axial_mode = 'idle'

    def _reset_align_control(self):
        self._align_lateral_error_integral = 0.0
        self._align_lock_start_s = None
        self._last_align_heading_error_deg = 0.0
        self._last_align_lateral_error_deg = 0.0
        self._last_align_ready = False
        self._last_align_ready_reason = 'not_evaluated'
        self._last_align_mode = 'idle'
        self._last_align_linear_cmd_mps = 0.0
        self._last_align_cmd_rad_s = 0.0

    def _recover_bottom_ready(self) -> bool:
        if self._odom is None:
            self._last_recover_bottom_ready = False
            self._last_recover_bottom_reason = 'missing_odom'
            return False
        if not self._fresh(self._stability, self._max_stability_age_s):
            self._last_recover_bottom_ready = False
            self._last_recover_bottom_reason = 'missing_stability'
            return False
        if not self._bottom_locked():
            self._last_recover_bottom_ready = False
            self._last_recover_bottom_reason = 'bottom_not_locked'
            return False
        if abs(self._stability.roll_deg) > self._recover_bottom_max_roll_deg:
            self._last_recover_bottom_ready = False
            self._last_recover_bottom_reason = 'roll_error'
            return False
        if (
            self._recover_bottom_require_pitch and
            abs(self._stability.pitch_deg) > self._recover_bottom_max_pitch_deg
        ):
            self._last_recover_bottom_ready = False
            self._last_recover_bottom_reason = 'pitch_error'
            return False
        if abs(self._stability.lateral_angle_deg) > self._recover_bottom_max_lateral_angle_deg:
            self._last_recover_bottom_ready = False
            self._last_recover_bottom_reason = 'lateral_error'
            return False
        if self._odom.linear_speed_mps > self._recover_bottom_max_linear_speed_mps:
            self._last_recover_bottom_ready = False
            self._last_recover_bottom_reason = 'linear_speed'
            return False
        if self._odom.yaw_rate_rad_s > self._recover_bottom_max_yaw_rate_rad_s:
            self._last_recover_bottom_ready = False
            self._last_recover_bottom_reason = 'yaw_rate'
            return False
        self._last_recover_bottom_ready = True
        self._last_recover_bottom_reason = 'ready'
        return True

    def _recover_bottom_ready_for_hold_time(self) -> bool:
        if self._recover_bottom_ready():
            if self._recover_bottom_lock_start_s is None:
                self._recover_bottom_lock_start_s = self._now_s()
            return self._now_s() - self._recover_bottom_lock_start_s >= self._recover_bottom_hold_s
        self._recover_bottom_lock_start_s = None
        return False

    def _descend_bottom_ready(self) -> bool:
        if self._odom is None:
            self._last_descend_ready = False
            self._last_descend_reason = 'missing_odom'
            return False
        if not self._fresh(self._stability, self._max_stability_age_s):
            self._last_descend_ready = False
            self._last_descend_reason = 'missing_stability'
            return False
        lateral_error_deg = abs(self._stability.roll_deg)
        if lateral_error_deg > self._descend_bottom_max_lateral_angle_deg:
            self._last_descend_ready = False
            self._last_descend_reason = 'lateral_error'
            return False
        self._last_descend_ready = True
        self._last_descend_reason = 'ready'
        return True

    def _descend_bottom_ready_for_hold_time(self) -> bool:
        if self._descend_bottom_ready():
            if self._descend_lock_start_s is None:
                self._descend_lock_start_s = self._now_s()
            return self._now_s() - self._descend_lock_start_s >= self._descend_bottom_hold_s
        self._descend_lock_start_s = None
        return False

    def _current_index_compensation_sign(self) -> float:
        sign = self._index_compensation_sign
        if self._index_compensation_alternate_by_lane and (self._lane_id % 2 == 1):
            sign *= -1.0
        return sign

    def _alignment_locked_for_hold_time(self) -> bool:
        if self._alignment_ready():
            if self._align_lock_start_s is None:
                self._align_lock_start_s = self._now_s()
            return self._now_s() - self._align_lock_start_s >= self._align_lock_hold_s
        self._align_lock_start_s = None
        return False

    def _alignment_ready(self) -> bool:
        # ALIGN_TO_BOTTOM_LANE is responsible for *recovering* bottom-lane lock.
        # Gating readiness on `_scan_allowed()` (which depends on
        # bottom_lane_locked / safe_to_scan) can create a circular dependency
        # that prevents ever leaving ALIGN_TO_BOTTOM_LANE.
        if self._odom is None:
            self._last_align_ready = False
            self._last_align_ready_reason = 'missing_odom'
            return False

        target_yaw = self._current_align_target_yaw()
        heading_error_deg = abs(math.degrees(_wrap_pi(target_yaw - self._odom.yaw_rad)))
        if heading_error_deg > self._align_yaw_tolerance_deg:
            self._last_align_ready = False
            self._last_align_ready_reason = 'yaw_error'
            return False

        if not self._fresh(self._stability, self._max_stability_age_s):
            self._last_align_ready = False
            self._last_align_ready_reason = 'missing_stability'
            return False
        lateral_error_deg = abs(self._stability.lateral_angle_deg)
        if lateral_error_deg > self._align_max_lateral_angle_deg:
            self._last_align_ready = False
            self._last_align_ready_reason = 'lateral_error'
            return False

        self._last_align_ready = True
        self._last_align_ready_reason = 'ready'
        return True

    def _publish_align_to_bottom_lane_cmd(self):
        if self._odom is None:
            self._stop_base()
            return
        dt = 1.0 / self._publish_rate_hz
        target_yaw = self._current_align_target_yaw()
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

        lateral_cmd = self._align_lateral_sign * (
            self._k_align_lateral_p * lateral_error +
            self._k_align_lateral_i * self._align_lateral_error_integral
        )
        heading_cmd = self._k_align_heading_p * heading_error

        lateral_p = self._align_lateral_sign * (self._k_align_lateral_p * lateral_error)
        lateral_i = self._align_lateral_sign * (self._k_align_lateral_i * self._align_lateral_error_integral)
        lateral_u_raw = lateral_p + lateral_i
        heading_p = heading_cmd

        # If the robot is not sitting on the bottom lane, do not let yaw
        # correction cancel the lateral recovery command. First recover the
        # stable generatrix, then clean up yaw.
        heading_error_deg = abs(math.degrees(heading_error))
        lateral_error_deg = abs(math.degrees(lateral_error))
        if self._align_post_axial_phase == 'yaw' and heading_error_deg <= self._align_yaw_tolerance_deg:
            self._align_force_yaw_phase = False
            self._align_post_axial_phase = 'lateral'
        if self._align_post_axial_phase == 'lateral' and lateral_error_deg <= self._align_max_lateral_angle_deg:
            self._align_post_axial_phase = None

        if heading_error_deg > self._align_yaw_reacquire_deg:
            angular_z = heading_cmd
            linear_x = 0.0
            self._last_align_mode = 'yaw_reacquire'
        elif self._align_post_axial_phase == 'yaw' and heading_error_deg > self._align_yaw_tolerance_deg:
            angular_z = heading_cmd
            linear_x = 0.0
            self._last_align_mode = 'yaw_capture_forced'
        elif self._align_post_axial_phase == 'lateral' and lateral_error_deg > self._align_max_lateral_angle_deg:
            angular_z = lateral_cmd
            linear_x = self._align_linear_speed_mps
            self._last_align_mode = 'lateral_capture_forced'
        elif (
            lateral_error_deg > self._align_max_lateral_angle_deg and
            heading_error_deg <= self._align_yaw_lateral_capture_deg
        ):
            # During lateral capture, prioritize recovering the bottom-lane
            # generatrix. Mixing heading correction here can weaken or even
            # oppose the lateral recovery when yaw is still non-zero after the
            # axial rotation.
            angular_z = lateral_cmd
            linear_x = self._align_linear_speed_mps
            self._last_align_mode = 'lateral_capture'
        elif heading_error_deg > self._align_yaw_tolerance_deg:
            angular_z = heading_cmd
            linear_x = 0.0
            self._last_align_mode = 'yaw_capture'
        else:
            angular_z = 0.0
            linear_x = 0.0
            self._last_align_mode = 'hold_ready'
        angular_z = max(
            -self._align_max_angular_z_rad_s,
            min(self._align_max_angular_z_rad_s, angular_z),
        )
        angular_z_unsat = angular_z
        # The clamp above already applied; infer saturation by recomputing unclamped.
        # (Keep it simple: compare to pre-clamp candidates.)
        # Note: angular_z here is the commanded output; lateral_u_raw is not directly
        # applied when modes select yaw-only.
        saturated = False
        if self._last_align_mode in ('yaw_reacquire', 'yaw_capture', 'yaw_capture_forced'):
            saturated = abs(heading_cmd) > self._align_max_angular_z_rad_s
        elif self._last_align_mode in ('lateral_capture', 'lateral_capture_forced'):
            saturated = abs(lateral_cmd) > self._align_max_angular_z_rad_s
        self._last_align_heading_error_deg = math.degrees(heading_error)
        self._last_align_lateral_error_deg = math.degrees(lateral_error)
        self._last_align_linear_cmd_mps = linear_x
        self._last_align_cmd_rad_s = angular_z
        self._publish_cmd_vel(linear_x, angular_z)

        controllers = [
            {
                'controller': 'align_heading_p',
                'target': {'yaw_rad': float(target_yaw)},
                'measured': {'yaw_rad': float(self._odom.yaw_rad)},
                'error': float(heading_error),
                'error_unit': 'rad',
                'p': float(heading_p),
                'i': 0.0,
                'u_raw': float(heading_p),
                'u_sat': float(angular_z) if self._last_align_mode in ('yaw_reacquire', 'yaw_capture', 'yaw_capture_forced') else None,
                'integrator': None,
                'integrator_unit': None,
                'dt': float(dt),
                'saturated': bool(saturated) if self._last_align_mode in ('yaw_reacquire', 'yaw_capture', 'yaw_capture_forced') else False,
                'notes': f'mode={self._last_align_mode}',
            },
            {
                'controller': 'align_lateral_pi',
                'target': {'lateral_error_rad': 0.0},
                'measured': {'lateral_error_rad': float(lateral_error)},
                'error': float(lateral_error),
                'error_unit': 'rad',
                'p': float(lateral_p),
                'i': float(lateral_i),
                'u_raw': float(lateral_u_raw),
                'u_sat': float(angular_z) if self._last_align_mode in ('lateral_capture', 'lateral_capture_forced') else None,
                'integrator': float(self._align_lateral_error_integral),
                'integrator_unit': 'rad*s',
                'dt': float(dt),
                'saturated': bool(saturated) if self._last_align_mode in ('lateral_capture', 'lateral_capture_forced') else False,
                'notes': f'mode={self._last_align_mode}',
            },
        ]
        self._publish_control_debug(controllers, cmd_linear_x=linear_x, cmd_angular_z=angular_z)

    def _publish_descend_to_bottom_lane_cmd(self):
        if self._odom is None:
            self._stop_base()
            return
        dt = 1.0 / self._publish_rate_hz
        target_yaw = self._current_realign_target_yaw()
        heading_error = _wrap_pi(target_yaw - self._odom.yaw_rad)
        lateral_error = self._signed_bottom_error_rad()

        self._align_lateral_error_integral += lateral_error * dt
        self._align_lateral_error_integral = max(
            -self._max_align_lateral_integral_rad_s,
            min(self._max_align_lateral_integral_rad_s, self._align_lateral_error_integral),
        )

        lateral_cmd = self._align_lateral_sign * (
            self._k_align_lateral_p * lateral_error +
            self._k_align_lateral_i * self._align_lateral_error_integral
        )
        heading_cmd = self._k_align_heading_p * heading_error
        heading_error_deg = abs(math.degrees(heading_error))
        lateral_error_deg = abs(math.degrees(lateral_error))

        if lateral_error_deg > self._descend_bottom_max_lateral_angle_deg:
            angular_z = lateral_cmd
            linear_x = self._align_linear_speed_mps
            self._last_align_mode = 'descend_bottom_lane'
        elif heading_error_deg > self._align_yaw_tolerance_deg:
            angular_z = heading_cmd
            linear_x = 0.0
            self._last_align_mode = 'descend_yaw_trim'
        else:
            angular_z = 0.0
            linear_x = 0.0
            self._last_align_mode = 'descend_hold_ready'

        angular_z = max(
            -self._align_max_angular_z_rad_s,
            min(self._align_max_angular_z_rad_s, angular_z),
        )
        lateral_p = self._align_lateral_sign * (self._k_align_lateral_p * lateral_error)
        lateral_i = self._align_lateral_sign * (self._k_align_lateral_i * self._align_lateral_error_integral)
        saturated = False
        if self._last_align_mode == 'descend_bottom_lane':
            saturated = abs(lateral_cmd) > self._align_max_angular_z_rad_s
        elif self._last_align_mode == 'descend_yaw_trim':
            saturated = abs(heading_cmd) > self._align_max_angular_z_rad_s

        self._last_align_heading_error_deg = math.degrees(heading_error)
        self._last_align_lateral_error_deg = math.degrees(lateral_error)
        self._last_align_linear_cmd_mps = linear_x
        self._last_align_cmd_rad_s = angular_z
        self._publish_cmd_vel(linear_x, angular_z)

        controllers = [
            {
                'controller': 'descend_heading_p',
                'target': {'yaw_rad': float(target_yaw)},
                'measured': {'yaw_rad': float(self._odom.yaw_rad)},
                'error': float(heading_error),
                'error_unit': 'rad',
                'p': float(heading_cmd),
                'i': 0.0,
                'u_raw': float(heading_cmd),
                'u_sat': float(angular_z) if self._last_align_mode == 'descend_yaw_trim' else None,
                'integrator': None,
                'integrator_unit': None,
                'dt': float(dt),
                'saturated': bool(saturated) if self._last_align_mode == 'descend_yaw_trim' else False,
                'notes': f'mode={self._last_align_mode}',
            },
            {
                'controller': 'descend_lateral_pi',
                'target': {'signed_bottom_error_rad': 0.0},
                'measured': {
                    'signed_bottom_error_rad': float(lateral_error),
                    'roll_deg': float(self._stability.roll_deg) if self._stability is not None else None,
                },
                'error': float(lateral_error),
                'error_unit': 'rad',
                'p': float(lateral_p),
                'i': float(lateral_i),
                'u_raw': float(lateral_p + lateral_i),
                'u_sat': float(angular_z) if self._last_align_mode == 'descend_bottom_lane' else None,
                'integrator': float(self._align_lateral_error_integral),
                'integrator_unit': 'rad*s',
                'dt': float(dt),
                'saturated': bool(saturated) if self._last_align_mode == 'descend_bottom_lane' else False,
                'notes': f'mode={self._last_align_mode}',
            },
        ]
        self._publish_control_debug(controllers, cmd_linear_x=linear_x, cmd_angular_z=angular_z)

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

        lateral_error = self._signed_bottom_error_rad()
        self._axial_lateral_error_integral += lateral_error * dt
        self._axial_lateral_error_integral = max(
            -self._max_axial_lateral_integral_rad_s,
            min(
                self._max_axial_lateral_integral_rad_s,
                self._axial_lateral_error_integral,
            ),
        )

        heading_p = self._k_axial_heading_p * heading_error
        heading_i = self._k_axial_heading_i * self._axial_heading_error_integral
        lateral_p = self._axial_lateral_sign * (self._k_axial_lateral_p * lateral_error)
        lateral_i = self._axial_lateral_sign * (self._k_axial_lateral_i * self._axial_lateral_error_integral)
        heading_error_deg = abs(math.degrees(heading_error))
        if heading_error_deg > self._axial_yaw_priority_deg:
            angular_z_raw = heading_p + heading_i
            axial_mode = 'yaw_priority'
        else:
            angular_z_raw = heading_p + heading_i + lateral_p + lateral_i
            axial_mode = 'coupled'
        angular_z = angular_z_raw
        angular_z = max(
            -self._max_angular_z_rad_s,
            min(self._max_angular_z_rad_s, angular_z),
        )
        self._last_axial_heading_error_deg = math.degrees(heading_error)
        self._last_axial_lateral_error_deg = math.degrees(lateral_error)
        self._last_axial_cmd_rad_s = angular_z
        self._last_axial_mode = axial_mode

        controllers = [
            {
                'controller': 'axial_heading_pi',
                'target': {'yaw_rad': float(target_yaw_rad)},
                'measured': {'yaw_rad': float(self._odom.yaw_rad)},
                'error': float(heading_error),
                'error_unit': 'rad',
                'p': float(heading_p),
                'i': float(heading_i),
                'u_raw': float(heading_p + heading_i),
                'u_sat': None,
                'integrator': float(self._axial_heading_error_integral),
                'integrator_unit': 'rad*s',
                'dt': float(dt),
                'saturated': False,
                'notes': f'contributes_to_cmd_angular_z mode={axial_mode}',
            },
            {
                'controller': 'axial_lateral_pi',
                'target': {'lateral_error_rad': 0.0},
                'measured': {
                    'lateral_error_rad': float(lateral_error),
                    'signed_roll_deg': (
                        float(self._stability.roll_deg)
                        if self._stability is not None else None
                    ),
                },
                'error': float(lateral_error),
                'error_unit': 'rad',
                'p': float(lateral_p),
                'i': float(lateral_i),
                'u_raw': float(lateral_p + lateral_i),
                'u_sat': None,
                'integrator': float(self._axial_lateral_error_integral),
                'integrator_unit': 'rad*s',
                'dt': float(dt),
                'saturated': False,
                'notes': f'signed_lateral_error_from_stability_roll_deg_converted_to_rad mode={axial_mode}',
            },
            {
                'controller': 'axial_sum',
                'target': None,
                'measured': None,
                'error': None,
                'error_unit': None,
                'p': float(heading_p + lateral_p),
                'i': float(heading_i + lateral_i),
                'u_raw': float(angular_z_raw),
                'u_sat': float(angular_z),
                'integrator': None,
                'integrator_unit': None,
                'dt': float(dt),
                'saturated': bool(abs(angular_z_raw) > self._max_angular_z_rad_s),
                'notes': f'cmd_angular_z_after_clamp mode={axial_mode}',
            },
        ]
        # cmd_linear_x is filled by the caller (AXIAL_SCAN) right after this.
        self._pending_axial_debug = controllers
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
        p = self._k_rotate_yaw_p * error
        i = self._k_rotate_yaw_i * self._yaw_error_integral
        u_raw = p + i
        angular_z = max(
            -self._rotate_angular_speed_rad_s,
            min(self._rotate_angular_speed_rad_s, u_raw),
        )
        self._last_yaw_cmd_rad_s = angular_z

        self._pending_yaw_debug = {
            'controller': 'yaw_pi',
            'target': {'yaw_rad': float(target_yaw_rad)},
            'measured': {'yaw_rad': float(self._odom.yaw_rad)},
            'error': float(error),
            'error_unit': 'rad',
            'p': float(p),
            'i': float(i),
            'u_raw': float(u_raw),
            'u_sat': float(angular_z),
            'integrator': float(self._yaw_error_integral),
            'integrator_unit': 'rad*s',
            'dt': float(dt),
            'saturated': bool(abs(u_raw) > self._rotate_angular_speed_rad_s),
            'notes': None,
        }
        return angular_z

    def _rotate_lateral_correction(self) -> float:
        if not self._fresh(self._stability, self._max_stability_age_s):
            return 0.0
        signed_error = self._signed_bottom_error_rad()
        return self._align_lateral_sign * self._k_rotate_lateral_p * signed_error

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
        angular_z_cmd = max(
            -self._rotate_angular_speed_rad_s,
            min(self._rotate_angular_speed_rad_s, angular_z + self._rotate_lateral_correction()),
        )
        self._publish_cmd_vel(0.0, angular_z_cmd)

        ctrl = getattr(self, '_pending_yaw_debug', None)
        if isinstance(ctrl, dict):
            ctrl = dict(ctrl)
            ctrl['controller'] = 'yaw_pi'
            ctrl['notes'] = 'rotate_to_yaw_with_signed_lateral_correction'
            self._publish_control_debug([ctrl], cmd_linear_x=0.0, cmd_angular_z=angular_z_cmd)
        return False

    def _publish_index_compensation(self, turner_velocity_rad_s: float):
        lateral_error_deg = 0.0
        if self._fresh(self._stability, self._max_stability_age_s):
            lateral_error_deg = self._stability.roll_deg
        dt = 1.0 / self._publish_rate_hz
        self._index_error_integral += lateral_error_deg * dt
        surface_speed_mps = self._tube_radius_m * turner_velocity_rad_s
        lateral_p = self._index_lateral_sign * (self._k_index_lateral_p * lateral_error_deg)
        lateral_i = self._index_lateral_sign * (self._k_index_lateral_i * self._index_error_integral)
        correction = lateral_p + lateral_i
        compensation_sign = self._current_index_compensation_sign()
        feedforward = (
            compensation_sign *
            self._index_compensation_gain *
            surface_speed_mps
        )
        linear_x = (
            feedforward + correction
        )
        linear_x_raw = linear_x
        linear_x = max(
            -self._max_index_linear_speed_mps,
            min(self._max_index_linear_speed_mps, linear_x),
        )
        yaw_target = self._tangential_yaw_target_rad
        angular_z = self._yaw_pi_command(yaw_target)
        if angular_z is None:
            angular_z = 0.0
        self._last_index_surface_speed_mps = surface_speed_mps
        self._last_index_lateral_correction_mps = correction
        self._last_index_base_cmd_mps = linear_x
        self._last_turner_cmd_rad_s = turner_velocity_rad_s
        self._publish_cmd_vel(linear_x, angular_z)

        controllers = []
        controllers.append({
            'controller': 'index_feedforward',
            'target': {'surface_speed_mps': float(surface_speed_mps)},
            'measured': None,
            'error': None,
            'error_unit': None,
            'p': None,
            'i': None,
            'u_raw': float(feedforward),
            'u_sat': float(feedforward),
            'integrator': None,
            'integrator_unit': None,
            'dt': float(dt),
            'saturated': False,
            'notes': f'u_raw = contextual_index_compensation_sign({compensation_sign}) * index_compensation_gain * (R * turner_w)',
        })
        controllers.append({
            'controller': 'index_lateral_pi',
            'target': {'lateral_error_deg': 0.0},
            'measured': {'lateral_error_deg': float(lateral_error_deg)},
            'error': float(lateral_error_deg),
            'error_unit': 'deg',
            'p': float(lateral_p),
            'i': float(lateral_i),
            'u_raw': float(correction),
            'u_sat': float(correction),
            'integrator': float(self._index_error_integral),
            'integrator_unit': 'deg*s',
            'dt': float(dt),
            'saturated': False,
            'notes': 'integrator_unit=deg*s (explicit)',
        })
        ctrl_yaw = getattr(self, '_pending_yaw_debug', None)
        if isinstance(ctrl_yaw, dict):
            ctrl_yaw = dict(ctrl_yaw)
            ctrl_yaw['controller'] = 'index_yaw_hold'
            ctrl_yaw['notes'] = 'yaw_pi holding tangential during INDEX_TUBE'
            controllers.append(ctrl_yaw)
        controllers.append({
            'controller': 'index_sum',
            'target': None,
            'measured': None,
            'error': None,
            'error_unit': None,
            'p': None,
            'i': None,
            'u_raw': float(linear_x_raw),
            'u_sat': float(linear_x),
            'integrator': None,
            'integrator_unit': None,
            'dt': float(dt),
            'saturated': bool(abs(linear_x_raw) > self._max_index_linear_speed_mps),
            'notes': 'cmd_linear_x_after_clamp',
        })
        self._publish_control_debug(controllers, cmd_linear_x=linear_x, cmd_angular_z=angular_z)

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
                RECOVER_BOTTOM_AFTER_INDEX,
                DESCEND_TO_BOTTOM_LANE,
                REALIGN_AXIAL_YAW,
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
                self._transition(DESCEND_TO_BOTTOM_LANE, 'bottom_lane_not_locked')

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
                if (
                    self._last_axial_mode == 'yaw_priority'
                ):
                    linear_speed = 0.0
                elif (
                    abs(self._last_axial_lateral_error_deg) > self._descend_bottom_max_lateral_angle_deg
                ):
                    linear_speed = min(self._axial_speed_mps, self._align_linear_speed_mps)
                else:
                    linear_speed = self._axial_speed_mps
                linear_x = linear_speed * self._lane_direction
                self._publish_cmd_vel(linear_x, angular_z)

                controllers = getattr(self, '_pending_axial_debug', None)
                if isinstance(controllers, list) and controllers:
                    # Update only the per-tick command fields; controllers already include dt.
                    self._publish_control_debug(controllers, cmd_linear_x=linear_x, cmd_angular_z=angular_z)

        elif self._state == RETURN_TO_BOTTOM_LANE:
            self._stop_base()
            self._stop_turner()
            if self._scan_allowed():
                self._transition(VERIFY_BOTTOM_LOCK, 'bottom_lane_recovered')
            else:
                self._transition(DESCEND_TO_BOTTOM_LANE, 'return_requires_active_alignment')

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
            # Pre-motion gate uses the strict safe_to_index signal. Once the
            # tube is moving, transient geometry loss should not abort the
            # index unless explicitly enabled.
            bottom_locked = self._bottom_locked()
            if self._index_motion_committed and not bottom_locked:
                if self._index_bottom_loss_start_s is None:
                    self._index_bottom_loss_start_s = self._now_s()
                bottom_loss_elapsed = self._now_s() - self._index_bottom_loss_start_s
                if (
                    self._abort_index_on_bottom_loss and
                    bottom_loss_elapsed >= self._index_bottom_loss_grace_s
                ):
                    self._stop_base()
                    self._stop_turner()
                    self._index_motion_committed = False
                    self._transition(DESCEND_TO_BOTTOM_LANE, 'index_bottom_lost')
                    self._publish_state()
                    return
            else:
                self._index_bottom_loss_start_s = None

            if not self._index_motion_committed and not self._index_allowed():
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
                        self._transition(RECOVER_BOTTOM_AFTER_INDEX, reason)
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

        elif self._state == RECOVER_BOTTOM_AFTER_INDEX:
            self._stop_turner()
            if self._recover_bottom_ready_for_hold_time():
                self._stop_base()
                self._transition(REALIGN_AXIAL_YAW, 'bottom_recovered_after_index')
            else:
                self._publish_align_to_bottom_lane_cmd()

        elif self._state == REALIGN_AXIAL_YAW:
            self._stop_turner()
            if self._rotate_to_yaw(self._axial_yaw_target_rad):
                self._transition(VERIFY_BOTTOM_LOCK, 'axial_yaw_reached')

        elif self._state == DESCEND_TO_BOTTOM_LANE:
            self._stop_turner()
            if self._descend_bottom_ready_for_hold_time():
                self._stop_base()
                self._transition(VERIFY_BOTTOM_LOCK, 'bottom_lane_descended')
            else:
                self._publish_descend_to_bottom_lane_cmd()

        elif self._state == ROTATE_TO_AXIAL:
            self._stop_turner()
            if self._rotate_to_yaw(self._axial_yaw_target_rad):
                self._transition(DESCEND_TO_BOTTOM_LANE, 'axial_yaw_reached')

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
                self._transition(DESCEND_TO_BOTTOM_LANE, 'indexed_position_not_aligned')

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
                'mode': self._last_align_mode,
                'k_heading_p': self._k_align_heading_p,
                'k_lateral_p': self._k_align_lateral_p,
                'k_lateral_i': self._k_align_lateral_i,
                'lateral_sign': self._align_lateral_sign,
                'lock_hold_s': self._align_lock_hold_s,
                'max_lateral_angle_deg': self._align_max_lateral_angle_deg,
                'yaw_tolerance_deg': self._align_yaw_tolerance_deg,
                'yaw_lateral_capture_deg': self._align_yaw_lateral_capture_deg,
                'yaw_reacquire_deg': self._align_yaw_reacquire_deg,
                'ready': self._last_align_ready,
                'ready_reason': self._last_align_ready_reason,
                'locked_hold_elapsed_s': (
                    self._now_s() - self._align_lock_start_s
                    if self._align_lock_start_s is not None else 0.0
                ),
            },
            'recover_bottom': {
                'ready': self._last_recover_bottom_ready,
                'ready_reason': self._last_recover_bottom_reason,
                'require_pitch': self._recover_bottom_require_pitch,
                'hold_s': self._recover_bottom_hold_s,
                'hold_elapsed_s': (
                    self._now_s() - self._recover_bottom_lock_start_s
                    if self._recover_bottom_lock_start_s is not None else 0.0
                ),
                'max_roll_deg': self._recover_bottom_max_roll_deg,
                'max_pitch_deg': self._recover_bottom_max_pitch_deg,
                'max_lateral_angle_deg': self._recover_bottom_max_lateral_angle_deg,
                'max_linear_speed_mps': self._recover_bottom_max_linear_speed_mps,
                'max_yaw_rate_rad_s': self._recover_bottom_max_yaw_rate_rad_s,
            },
            'descend_bottom': {
                'ready': self._last_descend_ready,
                'ready_reason': self._last_descend_reason,
                'hold_s': self._descend_bottom_hold_s,
                'hold_elapsed_s': (
                    self._now_s() - self._descend_lock_start_s
                    if self._descend_lock_start_s is not None else 0.0
                ),
                'max_lateral_angle_deg': self._descend_bottom_max_lateral_angle_deg,
            },
            'index_compensation': {
                'enabled': self._use_tangential_indexing,
                'tube_radius_m': self._tube_radius_m,
                'compensation_gain': self._index_compensation_gain,
                'compensation_sign': self._index_compensation_sign,
                'contextual_compensation_sign': self._current_index_compensation_sign(),
                'alternate_by_lane': self._index_compensation_alternate_by_lane,
                'surface_speed_mps': self._last_index_surface_speed_mps,
                'lateral_correction_mps': self._last_index_lateral_correction_mps,
                'lateral_sign': self._index_lateral_sign,
                'k_lateral_p': self._k_index_lateral_p,
                'k_lateral_i': self._k_index_lateral_i,
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
                'abort_index_on_bottom_loss': self._abort_index_on_bottom_loss,
                'index_bottom_loss_grace_s': self._index_bottom_loss_grace_s,
            },
            'index_safety': {
                'motion_committed': self._index_motion_committed,
                'bottom_loss_active': self._index_bottom_loss_start_s is not None,
                'bottom_loss_elapsed_s': (
                    self._now_s() - self._index_bottom_loss_start_s
                    if self._index_bottom_loss_start_s is not None else 0.0
                ),
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
