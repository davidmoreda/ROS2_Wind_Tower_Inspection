import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
import tf2_ros


class ScanGate(Node):
    def __init__(self):
        super().__init__('scan_gate')

        self.declare_parameter('input_scan_topic',  '/scan_raw')
        self.declare_parameter('output_scan_topic', '/scan')
        self.declare_parameter('target_frame',      'odom')
        self.declare_parameter('source_frame',      'lidar2d_0_laser')
        self.declare_parameter('check_period',      0.2)
        self.declare_parameter('startup_timeout',   60.0)
        self.declare_parameter('post_open_delay',   2.0)
        self.declare_parameter('restamp_offset_s',  0.1)

        in_topic  = self.get_parameter('input_scan_topic').value
        out_topic = self.get_parameter('output_scan_topic').value
        self._target = self.get_parameter('target_frame').value
        self._source = self.get_parameter('source_frame').value
        check_hz   = self.get_parameter('check_period').value
        self._timeout = self.get_parameter('startup_timeout').value

        pub_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        sub_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._pub  = self.create_publisher(LaserScan, out_topic, pub_qos)
        self._sub  = self.create_subscription(LaserScan, in_topic, self._cb, sub_qos)

        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._gate_open     = False
        self._tf_ready      = False
        self._elapsed       = 0.0
        self._post_delay    = self.get_parameter('post_open_delay').value
        self._restamp_ns    = int(self.get_parameter('restamp_offset_s').value * 1e9)
        self._check_timer   = self.create_timer(check_hz, self._check_tf)

        self.get_logger().info(
            f'scan_gate: waiting for TF {self._target}←{self._source} '
            f'before forwarding {in_topic} → {out_topic}'
        )

    def _check_tf(self):
        if self._gate_open:
            self._check_timer.cancel()
            return

        if self._tf_ready:
            # TF detectado — esperando post_open_delay antes de abrir
            self._elapsed += self.get_parameter('check_period').value
            if self._elapsed >= self._post_delay:
                self._gate_open = True
                self.get_logger().info('scan_gate: gate OPEN — scans forwarding to SLAM')
                self._check_timer.cancel()
            return

        try:
            self._tf_buffer.lookup_transform(
                self._target, self._source, rclpy.time.Time()
            )
            self._tf_ready = True
            self._elapsed  = 0.0
            self.get_logger().info(
                f'scan_gate: TF {self._target}←{self._source} available — '
                f'waiting {self._post_delay}s before opening gate'
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            self._elapsed += self.get_parameter('check_period').value
            if self._elapsed >= self._timeout:
                self.get_logger().warn(
                    f'scan_gate: timeout {self._timeout}s — opening gate anyway'
                )
                self._gate_open = True
                self._check_timer.cancel()

    def _cb(self, msg: LaserScan):
        if not self._gate_open:
            return
        # Restamp al tiempo actual - offset para que el TF ya esté en el buffer de SLAM
        now_ns = self.get_clock().now().nanoseconds
        stamp_ns = now_ns - self._restamp_ns
        msg.header.stamp.sec     = stamp_ns // 1_000_000_000
        msg.header.stamp.nanosec = stamp_ns %  1_000_000_000
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = ScanGate()
    rclpy.spin(node)
    rclpy.shutdown()
