"""
PS5 DualSense teleop para Wind Tower Inspection.

Joystick derecho    → UR5e joints 0 y 1 (rotación base + hombro)
Gatillo L2          → habilitar brazo (dead-man switch)
Botón X (Cruz)      → girar virador (tubo) a velocidad fija
Botón Cuadrado      → girar virador en sentido contrario
Botón Triángulo     → solicitar inicio de inspección autónoma
Botón Círculo       → solicitar parada de inspección autónoma

Control a tasa fija (10 Hz) para velocidad predecible.
El Husky lo gestiona el teleop_twist_joy_node de Clearpath (R1 + stick izq).

Ejes /joy (DualSense en Linux HID):
  axes[0] = stick izq X
  axes[1] = stick izq Y
  axes[2] = stick der X  → shoulder_pan (joint 0)
  axes[3] = stick der Y  → shoulder_lift (joint 1)
  axes[4] = L2  (-1 sin pulsar, +1 pulsado)
  axes[5] = R2

Botones /joy (DualSense en Linux HID, verificado):
  Los índices son parametrizables porque cambian según driver.
  Defaults actuales para las pruebas: X=1, Círculo=2, Triángulo=3.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float64, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

CONTROL_HZ = 10
MAX_JOINT_VEL = 0.05
DEADZONE = 0.15
TURNER_VEL = 0.15                  # rad/s velocidad del virador (≈8.6°/s)


def apply_deadzone(value, dz=DEADZONE):
    if abs(value) < dz:
        return 0.0
    return value


class PS5Teleop(Node):

    def __init__(self):
        super().__init__('ps5_teleop')

        self.declare_parameter('button_start_index', 3)
        self.declare_parameter('button_stop_index', 2)
        self.declare_parameter('button_turner_positive_index', 1)
        self.declare_parameter('button_turner_negative_index', 0)
        self._button_start_index = int(
            self.get_parameter('button_start_index').value)
        self._button_stop_index = int(
            self.get_parameter('button_stop_index').value)
        self._button_turner_positive_index = int(
            self.get_parameter('button_turner_positive_index').value)
        self._button_turner_negative_index = int(
            self.get_parameter('button_turner_negative_index').value)

        self._arm_pub = self.create_publisher(
            JointTrajectory,
            '/robot/arm_0_joint_trajectory_controller/joint_trajectory',
            10,
        )
        self._turner_pub = self.create_publisher(Float64, '/turner/cmd_vel', 10)
        self._mission_command_pub = self.create_publisher(
            String, '/inspection/mission_command', 10)

        self._joy_sub = self.create_subscription(
            Joy, '/robot/joy_teleop/joy', self._joy_callback, 10)
        self.create_subscription(
            Bool, '/inspection/autonomous_active', self._autonomous_active_cb, 10)

        self._ax_right_x = 0.0
        self._ax_right_y = 0.0
        self._enabled = False
        self._turner_vel = 0.0         # rad/s comandada al virador
        self._autonomous_active = False
        self._prev_circle = False
        self._prev_triangle = False

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
            'Mantén L2 + stick derecho para mover el brazo. '
            f'start_button={self._button_start_index}, '
            f'stop_button={self._button_stop_index}'
        )

    def _joy_callback(self, msg: Joy):
        if len(msg.axes) < 5:
            return
        btn_triangle = self._button_pressed(msg, self._button_start_index)
        btn_circle = self._button_pressed(msg, self._button_stop_index)
        if btn_triangle and not self._prev_triangle:
            self._mission_command_pub.publish(String(data='START_AUTO'))
            self.get_logger().info('Triángulo: START_AUTO enviado')
        if btn_circle and not self._prev_circle:
            self._mission_command_pub.publish(String(data='STOP'))
            self.get_logger().warn('Círculo: STOP enviado')
        self._prev_triangle = btn_triangle
        self._prev_circle = btn_circle

        if self._autonomous_active:
            self._ax_right_x = 0.0
            self._ax_right_y = 0.0
            self._enabled = False
            self._turner_vel = 0.0
            return

        self._ax_right_x = apply_deadzone(msg.axes[2])
        self._ax_right_y = apply_deadzone(msg.axes[3])
        l2 = msg.axes[4]
        self._enabled = l2 > 0.0
        btn_x = self._button_pressed(msg, self._button_turner_positive_index)
        btn_square = self._button_pressed(msg, self._button_turner_negative_index)
        if btn_x and not btn_square:
            self._turner_vel = TURNER_VEL
        elif btn_square and not btn_x:
            self._turner_vel = -TURNER_VEL
        else:
            self._turner_vel = 0.0

    def _button_pressed(self, msg: Joy, index: int) -> bool:
        return index >= 0 and len(msg.buttons) > index and bool(msg.buttons[index])

    def _autonomous_active_cb(self, msg: Bool):
        self._autonomous_active = bool(msg.data)
        if self._autonomous_active:
            self._turner_vel = 0.0
            self._enabled = False
            self._ax_right_x = 0.0
            self._ax_right_y = 0.0
            self._turner_pub.publish(Float64(data=0.0))

    def _control_loop(self):
        if self._autonomous_active:
            return
        self._turner_pub.publish(Float64(data=self._turner_vel))


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
