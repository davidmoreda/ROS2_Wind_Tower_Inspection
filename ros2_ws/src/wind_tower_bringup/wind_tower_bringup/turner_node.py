"""
Turner node — virador del tubo.

Publica /turner/angle (Float64, rad acumulados desde inicio).
Lee /turner/joint_state si está disponible (bridge gz); si no, integra /turner/cmd_vel.
Publica también /turner/angle_deg para conveniencia.
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState


class TurnerNode(Node):

    def __init__(self):
        super().__init__('turner_node')

        self._angle_rad = 0.0          # ángulo acumulado desde inicio
        self._last_cmd = 0.0           # última velocidad comandada (rad/s)
        self._use_joint_state = False  # True cuando llega joint_state del bridge

        self._pub_angle = self.create_publisher(Float64, '/turner/angle', 10)
        self._pub_deg   = self.create_publisher(Float64, '/turner/angle_deg', 10)

        self.create_subscription(
            JointState, '/turner/joint_state', self._joint_state_cb, 10)
        self.create_subscription(
            Float64, '/turner/cmd_vel', self._cmd_cb, 10)

        # Timer 20 Hz: integra velocidad si no hay joint_state real
        self._dt = 0.05
        self.create_timer(self._dt, self._timer_cb)

        self.get_logger().info('Turner node iniciado. Esperando /turner/joint_state ...')

    def _joint_state_cb(self, msg: JointState):
        """Ángulo real desde Gazebo (más preciso que integración)."""
        if 'turner_joint' in msg.name:
            idx = msg.name.index('turner_joint')
            self._angle_rad = msg.position[idx]
            self._use_joint_state = True

    def _cmd_cb(self, msg: Float64):
        self._last_cmd = msg.data

    def _timer_cb(self):
        if not self._use_joint_state:
            self._angle_rad += self._last_cmd * self._dt

        angle_deg = math.degrees(self._angle_rad) % 360.0

        self._pub_angle.publish(Float64(data=self._angle_rad))
        self._pub_deg.publish(Float64(data=angle_deg))


def main(args=None):
    rclpy.init(args=args)
    node = TurnerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
