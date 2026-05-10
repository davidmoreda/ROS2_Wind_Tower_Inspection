import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy


def main():
    rclpy.init()
    node = Node('tf_static_relay')

    qos = QoSProfile(
        depth=100,
        history=HistoryPolicy.KEEP_LAST,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )

    pub = node.create_publisher(TFMessage, '/tf_static', qos)

    def cb(msg):
        pub.publish(msg)

    node.create_subscription(TFMessage, '/robot/tf_static', cb, qos)
    node.get_logger().info('tf_static_relay: /robot/tf_static → /tf_static (TRANSIENT_LOCAL)')
    rclpy.spin(node)
    rclpy.shutdown()
