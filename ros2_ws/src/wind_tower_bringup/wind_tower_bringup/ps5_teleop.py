"""
PS5 DualSense teleop para Wind Tower Inspection.

Joystick derecho    → UR5e joints 0 y 1 (rotación base + hombro)
Gatillo L2          → habilitar brazo (dead-man switch)

Control a tasa fija (10 Hz) para velocidad predecible.
El Husky lo gestiona el teleop_twist_joy_node de Clearpath (R1 + stick izq).

Ejes /joy (DualSense en Linux HID):
  axes[0] = stick izq X
  axes[1] = stick izq Y
  axes[2] = stick der X  → shoulder_pan (joint 0)
  axes[3] = stick der Y  → shoulder_lift (joint 1)
  axes[4] = L2  (-1 sin pulsar, +1 pulsado)
  axes[5] = R2
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

CONTROL_HZ = 10                    # tasa del bucle de control
MAX_JOINT_VEL = 0.05               # rad por tick a 10 Hz → ~0.5 rad/s máximo
DEADZONE = 0.15


def apply_deadzone(value, dz=DEADZONE):
    if abs(value) < dz:
        return 0.0
    return value


class PS5Teleop(Node):

    def __init__(self):
        super().__init__('ps5_teleop')

        self._arm_pub = self.create_publisher(
            JointTrajectory,
            '/robot/arm_0_joint_trajectory_controller/joint_trajectory',
            10,
        )

        self._joy_sub = self.create_subscription(
            Joy, '/robot/joy_teleop/joy', self._joy_callback, 10)

        # Último estado del mando (actualizado por el callback)
        self._ax_right_x = 0.0
        self._ax_right_y = 0.0
        self._enabled = False

        # Posición acumulada de los joints (home)
        self._joint_positions = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
        self._arm_was_moving = False

        self._joint_names = [
            'arm_0_shoulder_pan_joint',
            'arm_0_shoulder_lift_joint',
            'arm_0_elbow_joint',
            'arm_0_wrist_1_joint',
            'arm_0_wrist_2_joint',
            'arm_0_wrist_3_joint',
        ]

        # Timer de control a tasa fija
        self.create_timer(1.0 / CONTROL_HZ, self._control_loop)

        self.get_logger().info(
            f'PS5 Teleop iniciado ({CONTROL_HZ} Hz). '
            'Mantén L2 + stick derecho para mover el brazo.'
        )

    def _joy_callback(self, msg: Joy):
        if len(msg.axes) < 5:
            return
        self._ax_right_x = apply_deadzone(msg.axes[2])
        self._ax_right_y = apply_deadzone(msg.axes[3])
        l2 = msg.axes[4]
        self._enabled = l2 > 0.0

    def _control_loop(self):
        arm_moving = (
            self._enabled
            and (abs(self._ax_right_x) > 0 or abs(self._ax_right_y) > 0)
        )

        if arm_moving:
            self._joint_positions[0] += self._ax_right_x * MAX_JOINT_VEL
            self._joint_positions[1] += self._ax_right_y * MAX_JOINT_VEL
            self._send_arm_trajectory(duration_ms=120)
            self._arm_was_moving = True
        elif self._arm_was_moving:
            self._send_arm_trajectory(duration_ms=300)
            self._arm_was_moving = False

    def _send_arm_trajectory(self, duration_ms):
        traj = JointTrajectory()
        traj.joint_names = self._joint_names
        point = JointTrajectoryPoint()
        point.positions = list(self._joint_positions)
        point.time_from_start = Duration(
            sec=0, nanosec=duration_ms * 1_000_000)
        traj.points = [point]
        self._arm_pub.publish(traj)


def main(args=None):
    rclpy.init(args=args)
    node = PS5Teleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
