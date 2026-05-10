"""Minimal cylindrical coverage map for axial-lane inspection.

This node is passive: it does not move the robot or command the turner. It
records observed and nominal coverage cells using odometry, turner angle and
stability flags. Controllers and the state machine can consume these metrics
later to decide when to stop, index the tube or finish a lane.
"""

import json
import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy._rclpy_pybind11 import RCLError
from std_msgs.msg import Bool, Float64, String


@dataclass
class OdomPose:
    stamp_s: float
    x_m: float
    y_m: float
    yaw_rad: float


@dataclass
class TurnerPose:
    stamp_s: float
    theta_rad: float


@dataclass
class FlagState:
    stamp_s: float
    value: bool


@dataclass
class MissionState:
    stamp_s: float
    state: str


def _normalize_deg(angle_deg: float) -> float:
    return angle_deg % 360.0


def _quat_to_yaw(q) -> float:
    x, y, z, w = q.x, q.y, q.z, q.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class CylindricalMapNode(Node):
    """Build simple observed/nominal coverage metrics in (x, theta)."""

    def __init__(self):
        super().__init__('cylindrical_map')

        self._declare_parameters()
        self._load_parameters()
        self._derive_grid()

        self._odom: Optional[OdomPose] = None
        self._turner: Optional[TurnerPose] = None
        self._bottom_lane_locked: Optional[FlagState] = None
        self._safe_to_scan: Optional[FlagState] = None
        self._mission_state: Optional[MissionState] = None

        self._observed_cells: set[tuple[int, int]] = set()
        self._nominal_cells: set[tuple[int, int]] = set()
        self._last_cell: Optional[tuple[int, int]] = None

        self._pub_status = self.create_publisher(
            String, '/inspection/coverage_status', 10)
        self._pub_stats = self.create_publisher(
            String, '/inspection/cylindrical_map_stats', 10)
        self._pub_pose = self.create_publisher(
            String, '/inspection/cylindrical_pose', 10)

        self.create_subscription(Odometry, self._odom_topic, self._odom_cb, 10)
        self.create_subscription(
            Float64, self._turner_angle_topic, self._turner_cb, 10)
        self.create_subscription(
            Bool, self._bottom_lane_locked_topic, self._bottom_lane_cb, 10)
        self.create_subscription(
            Bool, self._safe_to_scan_topic, self._safe_to_scan_cb, 10)
        self.create_subscription(
            String, self._mission_state_topic, self._mission_state_cb, 10)

        self.create_timer(1.0 / self._publish_rate_hz, self._tick)

        self.get_logger().info(
            'Cylindrical map listo: '
            f'x=[{self._x_min_m:.2f}, {self._x_max_m:.2f}]m, '
            f'x_cell={self._x_cell_m:.3f}m, theta_cell={self._theta_cell_deg:.3f}deg, '
            f'grid={self._x_bins}x{self._theta_bins}, lanes≈{self._estimated_lanes}'
        )

    def _declare_parameters(self):
        self.declare_parameter('odom_topic', '/robot/platform/odom/filtered')
        self.declare_parameter('turner_angle_topic', '/turner/angle')
        self.declare_parameter(
            'bottom_lane_locked_topic', '/inspection/bottom_lane_locked')
        self.declare_parameter('safe_to_scan_topic', '/inspection/safe_to_scan')
        self.declare_parameter('mission_state_topic', '/inspection/state_text')

        self.declare_parameter('tube.radius_m', 4.0)
        self.declare_parameter('tube.length_m', 30.0)
        self.declare_parameter('tube.x_min_m', -15.0)
        self.declare_parameter('tube.x_max_m', 15.0)

        self.declare_parameter('lanes.useful_surface_width_m', 0.50)
        self.declare_parameter('lanes.overlap', 0.20)
        self.declare_parameter('lanes.theta_cell_deg', 5.0)
        self.declare_parameter('lanes.x_cell_m', 0.10)
        self.declare_parameter('lanes.lane_zero_theta_deg', 0.0)

        self.declare_parameter('coverage.nominal_only_when_safe_to_scan', True)
        self.declare_parameter('coverage.bypass_counts_as_nominal', False)
        self.declare_parameter('coverage.manual_default_state', 'AXIAL_SCAN')

        self.declare_parameter('freshness.max_odom_age_s', 0.5)
        self.declare_parameter('freshness.max_turner_age_s', 1.0)
        self.declare_parameter('freshness.max_flags_age_s', 0.5)

        self.declare_parameter('publish_rate_hz', 5.0)

    def _load_parameters(self):
        self._odom_topic = self.get_parameter('odom_topic').value
        self._turner_angle_topic = self.get_parameter('turner_angle_topic').value
        self._bottom_lane_locked_topic = self.get_parameter(
            'bottom_lane_locked_topic').value
        self._safe_to_scan_topic = self.get_parameter('safe_to_scan_topic').value
        self._mission_state_topic = self.get_parameter('mission_state_topic').value

        self._radius_m = float(self.get_parameter('tube.radius_m').value)
        self._length_m = float(self.get_parameter('tube.length_m').value)
        self._x_min_m = float(self.get_parameter('tube.x_min_m').value)
        self._x_max_m = float(self.get_parameter('tube.x_max_m').value)

        self._useful_surface_width_m = float(
            self.get_parameter('lanes.useful_surface_width_m').value)
        self._overlap = float(self.get_parameter('lanes.overlap').value)
        self._theta_cell_deg = float(
            self.get_parameter('lanes.theta_cell_deg').value)
        self._x_cell_m = float(self.get_parameter('lanes.x_cell_m').value)
        self._lane_zero_theta_deg = float(
            self.get_parameter('lanes.lane_zero_theta_deg').value)

        self._nominal_only_when_safe = bool(
            self.get_parameter('coverage.nominal_only_when_safe_to_scan').value)
        self._bypass_counts_as_nominal = bool(
            self.get_parameter('coverage.bypass_counts_as_nominal').value)
        self._manual_default_state = str(
            self.get_parameter('coverage.manual_default_state').value)

        self._max_odom_age_s = float(
            self.get_parameter('freshness.max_odom_age_s').value)
        self._max_turner_age_s = float(
            self.get_parameter('freshness.max_turner_age_s').value)
        self._max_flags_age_s = float(
            self.get_parameter('freshness.max_flags_age_s').value)
        self._publish_rate_hz = max(0.5, float(
            self.get_parameter('publish_rate_hz').value))

    def _derive_grid(self):
        if self._radius_m <= 0.0:
            raise ValueError('tube.radius_m must be positive')
        if self._x_cell_m <= 0.0:
            raise ValueError('lanes.x_cell_m must be positive')
        if self._theta_cell_deg <= 0.0:
            raise ValueError('lanes.theta_cell_deg must be positive')
        if self._x_max_m <= self._x_min_m:
            raise ValueError('tube.x_max_m must be greater than tube.x_min_m')

        self._x_bins = int(math.ceil((self._x_max_m - self._x_min_m) / self._x_cell_m))
        self._theta_bins = int(math.ceil(360.0 / self._theta_cell_deg))
        self._total_cells = self._x_bins * self._theta_bins

        delta_theta_rad = self._useful_surface_width_m / self._radius_m
        delta_theta_real_rad = delta_theta_rad * (1.0 - self._overlap)
        self._delta_theta_real_deg = math.degrees(delta_theta_real_rad)
        if self._delta_theta_real_deg > 0.0:
            self._estimated_lanes = int(math.ceil(360.0 / self._delta_theta_real_deg))
        else:
            self._estimated_lanes = 0

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _fresh(self, state, max_age_s: float) -> bool:
        return state is not None and (self._now_s() - state.stamp_s) <= max_age_s

    def _odom_cb(self, msg: Odometry):
        pos = msg.pose.pose.position
        self._odom = OdomPose(
            stamp_s=self._now_s(),
            x_m=float(pos.x),
            y_m=float(pos.y),
            yaw_rad=_quat_to_yaw(msg.pose.pose.orientation),
        )

    def _turner_cb(self, msg: Float64):
        self._turner = TurnerPose(
            stamp_s=self._now_s(),
            theta_rad=float(msg.data),
        )

    def _bottom_lane_cb(self, msg: Bool):
        self._bottom_lane_locked = FlagState(
            stamp_s=self._now_s(),
            value=bool(msg.data),
        )

    def _safe_to_scan_cb(self, msg: Bool):
        self._safe_to_scan = FlagState(
            stamp_s=self._now_s(),
            value=bool(msg.data),
        )

    def _mission_state_cb(self, msg: String):
        self._mission_state = MissionState(
            stamp_s=self._now_s(),
            state=str(msg.data).strip(),
        )

    def _current_state(self) -> str:
        if self._fresh(self._mission_state, self._max_flags_age_s):
            return self._mission_state.state
        return self._manual_default_state

    def _cell_from_pose(self, x_m: float, theta_deg: float) -> Optional[tuple[int, int]]:
        if x_m < self._x_min_m or x_m > self._x_max_m:
            return None
        x_idx = int((x_m - self._x_min_m) / self._x_cell_m)
        theta_idx = int(_normalize_deg(theta_deg) / self._theta_cell_deg)
        x_idx = min(max(x_idx, 0), self._x_bins - 1)
        theta_idx = min(max(theta_idx, 0), self._theta_bins - 1)
        return x_idx, theta_idx

    def _coverage_percent(self, cells: set[tuple[int, int]]) -> float:
        if self._total_cells <= 0:
            return 0.0
        return 100.0 * len(cells) / float(self._total_cells)

    def _tick(self):
        odom_fresh = self._fresh(self._odom, self._max_odom_age_s)
        turner_fresh = self._fresh(self._turner, self._max_turner_age_s)
        bottom_fresh = self._fresh(self._bottom_lane_locked, self._max_flags_age_s)
        scan_fresh = self._fresh(self._safe_to_scan, self._max_flags_age_s)

        if not odom_fresh or not turner_fresh:
            self._publish_status(
                odom_fresh=odom_fresh,
                turner_fresh=turner_fresh,
                bottom_fresh=bottom_fresh,
                scan_fresh=scan_fresh,
                current_cell=None,
                theta_surface_deg=None,
                x_m=None,
                mission_state=self._current_state(),
                in_tube_limits=False,
                nominal_enabled=False,
            )
            return

        theta_tube_deg = math.degrees(self._turner.theta_rad)
        theta_surface_deg = _normalize_deg(theta_tube_deg + self._lane_zero_theta_deg)
        current_cell = self._cell_from_pose(self._odom.x_m, theta_surface_deg)
        in_tube_limits = current_cell is not None

        mission_state = self._current_state()
        bottom_locked = bottom_fresh and self._bottom_lane_locked.value
        safe_to_scan = scan_fresh and self._safe_to_scan.value
        bypass_active = mission_state == 'BYPASS_OBSTACLE'

        nominal_enabled = (
            mission_state == 'AXIAL_SCAN' and
            in_tube_limits and
            bottom_locked and
            (safe_to_scan or not self._nominal_only_when_safe) and
            (not bypass_active or self._bypass_counts_as_nominal)
        )

        if current_cell is not None:
            self._observed_cells.add(current_cell)
            if nominal_enabled:
                self._nominal_cells.add(current_cell)
            self._last_cell = current_cell

        self._publish_pose(theta_surface_deg, mission_state, in_tube_limits)
        self._publish_status(
            odom_fresh=odom_fresh,
            turner_fresh=turner_fresh,
            bottom_fresh=bottom_fresh,
            scan_fresh=scan_fresh,
            current_cell=current_cell,
            theta_surface_deg=theta_surface_deg,
            x_m=self._odom.x_m,
            mission_state=mission_state,
            in_tube_limits=in_tube_limits,
            nominal_enabled=nominal_enabled,
        )

    def _publish_pose(self, theta_surface_deg: float, mission_state: str, in_tube_limits: bool):
        theta_tube_deg = math.degrees(self._turner.theta_rad)
        pose = {
            'x_m': self._odom.x_m,
            'theta_tube_deg': _normalize_deg(theta_tube_deg),
            'theta_tube_unwrapped_deg': theta_tube_deg,
            'theta_surface_deg': theta_surface_deg,
            'yaw_deg': math.degrees(self._odom.yaw_rad),
            'mission_state': mission_state,
            'in_tube_limits': in_tube_limits,
        }
        self._pub_pose.publish(String(data=json.dumps(pose)))

    def _publish_status(
        self,
        *,
        odom_fresh: bool,
        turner_fresh: bool,
        bottom_fresh: bool,
        scan_fresh: bool,
        current_cell: Optional[tuple[int, int]],
        theta_surface_deg: Optional[float],
        x_m: Optional[float],
        mission_state: str,
        in_tube_limits: bool,
        nominal_enabled: bool,
    ):
        observed_percent = self._coverage_percent(self._observed_cells)
        nominal_percent = self._coverage_percent(self._nominal_cells)
        uncovered_cells = max(0, self._total_cells - len(self._nominal_cells))
        x_progress = None
        if x_m is not None:
            x_progress = (x_m - self._x_min_m) / (self._x_max_m - self._x_min_m)
            x_progress = min(max(x_progress, 0.0), 1.0)

        status = {
            'nominal_coverage_percent': nominal_percent,
            'observed_coverage_percent': observed_percent,
            'nominal_cells': len(self._nominal_cells),
            'observed_cells': len(self._observed_cells),
            'uncovered_cells': uncovered_cells,
            'grid': {
                'x_bins': self._x_bins,
                'theta_bins': self._theta_bins,
                'total_cells': self._total_cells,
                'x_cell_m': self._x_cell_m,
                'theta_cell_deg': self._theta_cell_deg,
            },
            'lane_model': {
                'useful_surface_width_m': self._useful_surface_width_m,
                'overlap': self._overlap,
                'delta_theta_real_deg': self._delta_theta_real_deg,
                'estimated_lanes': self._estimated_lanes,
            },
            'current': {
                'mission_state': mission_state,
                'x_m': x_m,
                'theta_surface_deg': theta_surface_deg,
                'cell': list(current_cell) if current_cell is not None else None,
                'in_tube_limits': in_tube_limits,
                'x_progress_0_1': x_progress,
                'nominal_coverage_enabled': nominal_enabled,
            },
            'fresh': {
                'odom': odom_fresh,
                'turner': turner_fresh,
                'bottom_lane_locked': bottom_fresh,
                'safe_to_scan': scan_fresh,
            },
            'flags': {
                'bottom_lane_locked': (
                    self._bottom_lane_locked.value
                    if self._bottom_lane_locked is not None else None
                ),
                'safe_to_scan': (
                    self._safe_to_scan.value
                    if self._safe_to_scan is not None else None
                ),
            },
        }
        payload = json.dumps(status)
        self._pub_status.publish(String(data=payload))
        self._pub_stats.publish(String(data=payload))


def main(args=None):
    rclpy.init(args=args)
    node = CylindricalMapNode()
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
