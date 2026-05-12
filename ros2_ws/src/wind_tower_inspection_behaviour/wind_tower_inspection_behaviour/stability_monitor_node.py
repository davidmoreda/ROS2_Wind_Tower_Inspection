"""Monitor stability conditions for axial-lane tube inspection.

This node converts raw sensor signals into the first operational safety flags:
bottom_lane_locked, safe_to_scan and safe_to_index_tube.
"""

import json
import math
import re
from dataclasses import dataclass
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy._rclpy_pybind11 import RCLError
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float64, String


GRAVITY = 9.80665


@dataclass
class ImuState:
    stamp_s: float
    roll_deg: float
    pitch_deg: float
    lateral_angle_deg: float
    raw_roll_deg: float
    raw_pitch_deg: float
    raw_lateral_angle_deg: float
    accel_norm: float
    angular_speed: float


@dataclass
class OdomState:
    stamp_s: float
    linear_speed: float
    yaw_rate: float


@dataclass
class GeometryState:
    stamp_s: float
    fit_radius: float
    fit_rms: float
    wall_points: int


@dataclass
class TurnerState:
    stamp_s: float
    theta_rad: float


def _quat_to_roll_pitch(q) -> tuple[float, float]:
    x, y, z, w = q.x, q.y, q.z, q.w

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    return math.degrees(roll), math.degrees(pitch)


class StabilityMonitorNode(Node):
    """Publish inspection safety flags from IMU, odom and local tube geometry."""

    _STATS_RE = re.compile(
        r'fit_r=(?P<fit_r>[-+0-9.]+)m\s+'
        r'rms=(?P<rms>[-+0-9.]+)m.*'
        r'wall_pts=(?P<wall_pts>\d+)'
    )

    def __init__(self):
        super().__init__('stability_monitor')

        self._declare_parameters()
        self._load_parameters()

        self._imu: Optional[ImuState] = None
        self._odom: Optional[OdomState] = None
        self._geometry: Optional[GeometryState] = None
        self._turner: Optional[TurnerState] = None
        self._imu_calibration_samples = []
        self._imu_calibrated = not self._auto_calibrate_imu
        self._roll_offset_deg = self._manual_roll_offset_deg
        self._pitch_offset_deg = self._manual_pitch_offset_deg
        self._lateral_offset_deg = self._manual_lateral_offset_deg
        self._callback_counts = {
            'imu': 0,
            'odom': 0,
            'geometry': 0,
            'turner': 0,
        }

        self._pub_bottom_lane = self.create_publisher(
            Bool, '/inspection/bottom_lane_locked', 10)
        self._pub_safe_scan = self.create_publisher(
            Bool, '/inspection/safe_to_scan', 10)
        self._pub_safe_index = self.create_publisher(
            Bool, '/inspection/safe_to_index_tube', 10)
        self._pub_stability = self.create_publisher(
            String, '/inspection/stability', 10)

        self._imu_sub = self.create_subscription(
            Imu, self._imu_topic, self._imu_cb, 10)
        self._odom_sub = self.create_subscription(
            Odometry, self._odom_topic, self._odom_cb, 10)
        self._geometry_sub = self.create_subscription(
            String, self._cylinder_stats_topic, self._geometry_cb, 10)
        self._turner_sub = self.create_subscription(
            Float64, self._turner_angle_topic, self._turner_cb, 10)

        self.create_timer(0.1, self._publish_status)

        self.get_logger().info(
            'Stability monitor listo: '
            f'imu={self._imu_topic}, odom={self._odom_topic}, '
            f'geometry={self._cylinder_stats_topic}, '
            f'turner={self._turner_angle_topic}'
        )

    def _declare_parameters(self):
        self.declare_parameter('imu_topic', '/robot/sensors/imu_0/data')
        self.declare_parameter('odom_topic', '/robot/platform/odom/filtered')
        self.declare_parameter('cylinder_stats_topic', '/cylinder_fit/stats')
        self.declare_parameter('turner_angle_topic', '/turner/angle')

        self.declare_parameter('imu_calibration.auto_calibrate_on_start', True)
        self.declare_parameter('imu_calibration.sample_count', 50)
        self.declare_parameter('imu_calibration.roll_offset_deg', 0.0)
        self.declare_parameter('imu_calibration.pitch_offset_deg', 0.0)
        self.declare_parameter('imu_calibration.lateral_angle_offset_deg', 0.0)

        self.declare_parameter('bottom_lane.max_roll_deg', 5.0)
        self.declare_parameter('bottom_lane.max_pitch_deg', 5.0)
        self.declare_parameter('bottom_lane.max_lateral_angle_deg', 5.0)
        self.declare_parameter('bottom_lane.max_accel_error', 1.0)
        # How to compute lateral_angle_deg from IMU orientation.
        # - 'roll': abs(roll_deg)
        # - 'pitch': abs(pitch_deg)
        # - 'tilt': sqrt(roll_deg^2 + pitch_deg^2)
        self.declare_parameter('bottom_lane.lateral_angle_mode', 'roll')

        # If true, bottom-lane lock ignores pitch. Pitch can legitimately spike
        # during in-place yaw rotations due to contact/IMU mounting dynamics,
        # but that does not necessarily mean the robot left the bottom lane.
        self.declare_parameter('bottom_lane.ignore_pitch_for_lock', True)

        self.declare_parameter('geometry.min_wall_points', 300)
        self.declare_parameter('geometry.max_fit_rms', 0.35)
        self.declare_parameter('geometry.min_fit_radius', 3.4)
        self.declare_parameter('geometry.max_fit_radius', 4.2)

        self.declare_parameter('motion.max_scan_speed', 0.20)
        self.declare_parameter('motion.max_index_speed', 0.02)
        self.declare_parameter('motion.max_yaw_rate', 0.10)

        self.declare_parameter('freshness.max_imu_age_s', 0.5)
        self.declare_parameter('freshness.max_odom_age_s', 0.5)
        self.declare_parameter('freshness.max_geometry_age_s', 1.0)
        self.declare_parameter('freshness.max_turner_age_s', 1.0)

    def _load_parameters(self):
        self._imu_topic = self.get_parameter('imu_topic').value
        self._odom_topic = self.get_parameter('odom_topic').value
        self._cylinder_stats_topic = self.get_parameter(
            'cylinder_stats_topic').value
        self._turner_angle_topic = self.get_parameter('turner_angle_topic').value

        self._auto_calibrate_imu = bool(
            self.get_parameter('imu_calibration.auto_calibrate_on_start').value)
        self._imu_calibration_sample_count = max(
            1, int(self.get_parameter('imu_calibration.sample_count').value))
        self._manual_roll_offset_deg = float(
            self.get_parameter('imu_calibration.roll_offset_deg').value)
        self._manual_pitch_offset_deg = float(
            self.get_parameter('imu_calibration.pitch_offset_deg').value)
        self._manual_lateral_offset_deg = float(
            self.get_parameter(
                'imu_calibration.lateral_angle_offset_deg').value)

        self._max_roll_deg = self.get_parameter(
            'bottom_lane.max_roll_deg').value
        self._max_pitch_deg = self.get_parameter(
            'bottom_lane.max_pitch_deg').value
        self._max_lateral_angle_deg = self.get_parameter(
            'bottom_lane.max_lateral_angle_deg').value
        self._max_accel_error = self.get_parameter(
            'bottom_lane.max_accel_error').value

        self._lateral_angle_mode = str(
            self.get_parameter('bottom_lane.lateral_angle_mode').value
        ).strip().lower()

        self._ignore_pitch_for_lock = bool(
            self.get_parameter('bottom_lane.ignore_pitch_for_lock').value
        )

        self._min_wall_points = self.get_parameter(
            'geometry.min_wall_points').value
        self._max_fit_rms = self.get_parameter('geometry.max_fit_rms').value
        self._min_fit_radius = self.get_parameter(
            'geometry.min_fit_radius').value
        self._max_fit_radius = self.get_parameter(
            'geometry.max_fit_radius').value

        self._max_scan_speed = self.get_parameter(
            'motion.max_scan_speed').value
        self._max_index_speed = self.get_parameter(
            'motion.max_index_speed').value
        self._max_yaw_rate = self.get_parameter('motion.max_yaw_rate').value

        self._max_imu_age_s = self.get_parameter(
            'freshness.max_imu_age_s').value
        self._max_odom_age_s = self.get_parameter(
            'freshness.max_odom_age_s').value
        self._max_geometry_age_s = self.get_parameter(
            'freshness.max_geometry_age_s').value
        self._max_turner_age_s = self.get_parameter(
            'freshness.max_turner_age_s').value

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _msg_stamp_s(self, msg) -> float:
        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            return self._now_s()
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _imu_cb(self, msg: Imu):
        self._callback_counts['imu'] += 1
        raw_roll_deg, raw_pitch_deg = _quat_to_roll_pitch(msg.orientation)

        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        accel_norm = math.sqrt(ax * ax + ay * ay + az * az)
        # "Lateral angle" is intended to represent bottom-lane deviation.
        # Using the accelerometer directly flaps during yaw rotations due to
        # non-gravitational accelerations. Prefer orientation-derived signals.
        mode = getattr(self, '_lateral_angle_mode', 'roll')
        if mode == 'pitch':
            raw_lateral_angle = abs(raw_pitch_deg)
        elif mode == 'tilt':
            raw_lateral_angle = math.sqrt(raw_roll_deg * raw_roll_deg + raw_pitch_deg * raw_pitch_deg)
        else:
            raw_lateral_angle = abs(raw_roll_deg)

        wx = msg.angular_velocity.x
        wy = msg.angular_velocity.y
        wz = msg.angular_velocity.z
        angular_speed = math.sqrt(wx * wx + wy * wy + wz * wz)

        if self._auto_calibrate_imu and not self._imu_calibrated:
            self._imu_calibration_samples.append(
                (raw_roll_deg, raw_pitch_deg, raw_lateral_angle))
            if len(self._imu_calibration_samples) >= self._imu_calibration_sample_count:
                n = float(len(self._imu_calibration_samples))
                self._roll_offset_deg = sum(
                    sample[0] for sample in self._imu_calibration_samples) / n
                self._pitch_offset_deg = sum(
                    sample[1] for sample in self._imu_calibration_samples) / n
                self._lateral_offset_deg = sum(
                    sample[2] for sample in self._imu_calibration_samples) / n
                self._imu_calibrated = True
                self.get_logger().info(
                    'IMU bottom-lane calibrada: '
                    f'roll_offset={self._roll_offset_deg:.3f}deg, '
                    f'pitch_offset={self._pitch_offset_deg:.3f}deg, '
                    f'lateral_offset={self._lateral_offset_deg:.3f}deg'
                )

        roll_deg = raw_roll_deg - self._roll_offset_deg
        pitch_deg = raw_pitch_deg - self._pitch_offset_deg
        lateral_angle = abs(raw_lateral_angle - self._lateral_offset_deg)

        self._imu = ImuState(
            stamp_s=self._now_s(),
            roll_deg=roll_deg,
            pitch_deg=pitch_deg,
            lateral_angle_deg=lateral_angle,
            raw_roll_deg=raw_roll_deg,
            raw_pitch_deg=raw_pitch_deg,
            raw_lateral_angle_deg=raw_lateral_angle,
            accel_norm=accel_norm,
            angular_speed=angular_speed,
        )

    def _odom_cb(self, msg: Odometry):
        self._callback_counts['odom'] += 1
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z
        linear_speed = math.sqrt(vx * vx + vy * vy + vz * vz)
        yaw_rate = abs(msg.twist.twist.angular.z)

        self._odom = OdomState(
            stamp_s=self._now_s(),
            linear_speed=linear_speed,
            yaw_rate=yaw_rate,
        )

    def _geometry_cb(self, msg: String):
        self._callback_counts['geometry'] += 1
        match = self._STATS_RE.search(msg.data)
        if not match:
            self.get_logger().warn(
                f'No se pudo parsear cylinder stats: {msg.data}',
                throttle_duration_sec=3.0)
            return

        self._geometry = GeometryState(
            stamp_s=self._now_s(),
            fit_radius=float(match.group('fit_r')),
            fit_rms=float(match.group('rms')),
            wall_points=int(match.group('wall_pts')),
        )

    def _turner_cb(self, msg: Float64):
        self._callback_counts['turner'] += 1
        self._turner = TurnerState(
            stamp_s=self._now_s(),
            theta_rad=msg.data,
        )

    def _fresh(self, state, max_age_s: float) -> bool:
        return state is not None and (self._now_s() - state.stamp_s) <= max_age_s

    def _publish_status(self):
        imu_fresh = self._fresh(self._imu, self._max_imu_age_s)
        odom_fresh = self._fresh(self._odom, self._max_odom_age_s)
        geometry_fresh = self._fresh(self._geometry, self._max_geometry_age_s)
        turner_fresh = self._fresh(self._turner, self._max_turner_age_s)

        imu_ok = False
        if imu_fresh:
            # Bottom-lane lock is primarily about gravity alignment to the
            # cylinder generatrix (lateral angle) and overall stability.
            # Pitch is optionally ignored for lock to avoid flapping during
            # yaw rotations.
            pitch_ok = (
                True if self._ignore_pitch_for_lock
                else abs(self._imu.pitch_deg) <= self._max_pitch_deg
            )
            imu_ok = (
                self._imu_calibrated and
                abs(self._imu.roll_deg) <= self._max_roll_deg and
                pitch_ok and
                self._imu.lateral_angle_deg <= self._max_lateral_angle_deg and
                abs(self._imu.accel_norm - GRAVITY) <= self._max_accel_error
            )

        geometry_ok = False
        if geometry_fresh:
            geometry_ok = (
                self._geometry.wall_points >= self._min_wall_points and
                self._geometry.fit_rms <= self._max_fit_rms and
                self._min_fit_radius <= self._geometry.fit_radius <=
                self._max_fit_radius
            )

        scan_motion_ok = False
        index_motion_ok = False
        if odom_fresh:
            scan_motion_ok = (
                self._odom.linear_speed <= self._max_scan_speed and
                self._odom.yaw_rate <= self._max_yaw_rate
            )
            index_motion_ok = (
                self._odom.linear_speed <= self._max_index_speed and
                self._odom.yaw_rate <= self._max_yaw_rate
            )

        bottom_lane_locked = imu_ok and geometry_ok
        safe_to_scan = bottom_lane_locked and scan_motion_ok and turner_fresh
        safe_to_index = bottom_lane_locked and index_motion_ok and turner_fresh

        self._pub_bottom_lane.publish(Bool(data=bottom_lane_locked))
        self._pub_safe_scan.publish(Bool(data=safe_to_scan))
        self._pub_safe_index.publish(Bool(data=safe_to_index))

        status = {
            'bottom_lane_locked': bottom_lane_locked,
            'safe_to_scan': safe_to_scan,
            'safe_to_index_tube': safe_to_index,
            'imu_ok': imu_ok,
            'geometry_ok': geometry_ok,
            'scan_motion_ok': scan_motion_ok,
            'index_motion_ok': index_motion_ok,
            'fresh': {
                'imu': imu_fresh,
                'odom': odom_fresh,
                'geometry': geometry_fresh,
                'turner': turner_fresh,
            },
            'imu': self._imu.__dict__ if self._imu is not None else None,
            'odom': self._odom.__dict__ if self._odom is not None else None,
            'geometry': (
                self._geometry.__dict__ if self._geometry is not None else None
            ),
            'theta_tube_rad': (
                self._turner.theta_rad if self._turner is not None else None
            ),
            'callback_counts': self._callback_counts,
            'imu_calibration': {
                'auto_calibrate_on_start': self._auto_calibrate_imu,
                'calibrated': self._imu_calibrated,
                'samples': len(self._imu_calibration_samples),
                'sample_count': self._imu_calibration_sample_count,
                'roll_offset_deg': self._roll_offset_deg,
                'pitch_offset_deg': self._pitch_offset_deg,
                'lateral_angle_offset_deg': self._lateral_offset_deg,
            },
        }
        self._pub_stability.publish(String(data=json.dumps(status)))


def main(args=None):
    rclpy.init(args=args)
    node = StabilityMonitorNode()
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
