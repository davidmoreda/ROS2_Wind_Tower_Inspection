"""
Puente de QoS para /scan.

pointcloud_to_laserscan publica con SensorDataQoS (BEST_EFFORT).
slam_toolbox y nav2_amcl suscriben con RELIABLE (SystemDefaultQoS).
Este nodo actúa de puente: suscribe BEST_EFFORT → republica RELIABLE.

Entrada:  /scan_raw  (BEST_EFFORT) ← desde pointcloud_to_laserscan
Salida:   /scan      (RELIABLE)    → slam_toolbox, nav2, RViz
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

        self._pub = self.create_publisher(LaserScan, '/scan', pub_qos)
        self._sub = self.create_subscription(LaserScan, '/scan_raw', self._cb, sub_qos)
        self.get_logger().info('scan_qos_bridge: /scan_raw (BE) → /scan (RELIABLE)')

    def _cb(self, msg: LaserScan) -> None:
        self._pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(ScanQosBridge())
    rclpy.shutdown()
