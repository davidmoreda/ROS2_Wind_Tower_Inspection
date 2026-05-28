"""
Puente de QoS para LaserScan.

pointcloud_to_laserscan publica con SensorDataQoS (BEST_EFFORT).
slam_toolbox, nav2_amcl y costmap pueden necesitar RELIABLE. Este nodo actua
de puente configurable: suscribe BEST_EFFORT y republica RELIABLE.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan


class ScanQosBridge(Node):
    def __init__(self):
        super().__init__('scan_qos_bridge')

        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        self.declare_parameter('input_topic', '/scan_raw')
        self.declare_parameter('output_topic', '/scan')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self._pub = self.create_publisher(LaserScan, output_topic, pub_qos)
        self._sub = self.create_subscription(LaserScan, input_topic, self._cb, sub_qos)
        self.get_logger().info(
            f'scan_qos_bridge: {input_topic} (BE) -> {output_topic} (RELIABLE)'
        )

    def _cb(self, msg: LaserScan) -> None:
        self._pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(ScanQosBridge())
    rclpy.shutdown()
