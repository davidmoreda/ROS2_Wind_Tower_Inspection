"""
PS5 DualSense teleop para Wind Tower Inspection.

Joystick derecho    → UR5e joints 0 y 1 (rotación base + hombro)
Gatillo L2          → habilitar brazo (dead-man switch)
Botón X (Cruz)      → girar virador (tubo) a velocidad fija
Botón Cuadrado      → girar virador en sentido contrario
Botón Triángulo     → solicitar inicio de inspección autónoma
Botón Círculo       → solicitar parada de inspección autónoma
D-pad arriba        → subir nivel de velocidad del Husky (5 niveles)
D-pad abajo         → bajar nivel de velocidad del Husky

Control a tasa fija (50 Hz) para velocidad predecible.
El Husky lo gestiona el teleop_twist_joy_node de Clearpath (R1 + stick izq).
La velocidad se controla interceptando /robot/cmd_vel y aplicando un factor
de escala interno (no depende de param set en runtime).

Ejes /joy (DualSense en Linux HID):
  axes[0] = stick izq X
  axes[1] = stick izq Y
  axes[2] = stick der X  → shoulder_pan (joint 0)
  axes[3] = stick der Y  → shoulder_lift (joint 1)
  axes[4] = L2  (-1 sin pulsar, +1 pulsado)
  axes[5] = R2
  axes[6] = D-pad X  (-1 izq, +1 der)
  axes[7] = D-pad Y  (+1 arriba, -1 abajo)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float64, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# Factores de escala por nivel (multiplicados sobre el cmd_vel de Clearpath)
# Clearpath publica con scale_linear=0.5 y scale_turbo=1.0 por defecto.
# Estos factores reducen/amplían esa salida.
SPEED_FACTORS = [0.15, 0.30, 0.50, 0.70, 1.00]   # niveles 1–5
DEFAULT_LEVEL  = 2   # nivel 3 (0.50×) — arranque con velocidad media

CONTROL_HZ   = 50
MAX_JOINT_VEL = 0.05
DEADZONE      = 0.15
TURNER_VEL    = 0.15   # rad/s velocidad del virador


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
        self._turner_pub   = self.create_publisher(Float64, '/turner/cmd_vel', 10)
        self._mission_pub  = self.create_publisher(String, '/inspection/mission_command', 10)

        # Intercept cmd_vel de Clearpath y republica escalado.
        # En ROS2 Jazzy/Clearpath el topic /robot/cmd_vel es TwistStamped
        # (velocity_smoother con enable_stamped_cmd_vel: true).
        # IMPORTANTE: NO suscribirse con dos tipos distintos al mismo topic —
        # eso causa "contains more than one type" en el grafo DDS y rompe
        # ros2 topic echo y otros herramientas de diagnóstico.
        self._cmd_vel_pub  = self.create_publisher(Twist, '/robot/cmd_vel_scaled', 10)
        self.create_subscription(
            TwistStamped, '/robot/cmd_vel', self._cmd_vel_stamped_cb, 10)

        self._joy_sub = self.create_subscription(
            Joy, '/robot/joy_teleop/joy', self._joy_callback, 10)
        self.create_subscription(
            Bool, '/inspection/autonomous_active', self._autonomous_active_cb, 10)

        self._ax_right_x = 0.0
        self._ax_right_y = 0.0
        self._enabled    = False
        self._turner_vel = 0.0
        self._autonomous_active = False
        self._prev_circle   = False
        self._prev_triangle = False
        self._speed_level   = DEFAULT_LEVEL
        self._prev_dpad_y   = 0.0

        self._joint_positions = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
        self._arm_was_moving  = False
        self._joint_names = [
            'arm_0_shoulder_pan_joint',
            'arm_0_shoulder_lift_joint',
            'arm_0_elbow_joint',
            'arm_0_wrist_1_joint',
            'arm_0_wrist_2_joint',
            'arm_0_wrist_3_joint',
        ]

        self.create_timer(1.0 / CONTROL_HZ, self._control_loop)

        self.get_logger().info(
            f'PS5 Teleop iniciado ({CONTROL_HZ} Hz). '
            f'Velocidad inicial nivel {DEFAULT_LEVEL + 1}/5 '
            f'(factor {SPEED_FACTORS[DEFAULT_LEVEL]:.2f}). '
            'D-pad ↑↓ para cambiar velocidad.'
        )

    # ── cmd_vel scaler ────────────────────────────────────────────────────────
    def _cmd_vel_stamped_cb(self, msg: TwistStamped):
        """Escala el cmd_vel (TwistStamped) de Clearpath Jazzy y republica como Twist."""
        if self._autonomous_active:
            return
        factor = SPEED_FACTORS[self._speed_level]
        scaled = Twist()
        scaled.linear.x  = msg.twist.linear.x  * factor
        scaled.linear.y  = msg.twist.linear.y  * factor
        scaled.angular.z = msg.twist.angular.z * factor
        self._cmd_vel_pub.publish(scaled)

    # ── joy ───────────────────────────────────────────────────────────────────
    def _joy_callback(self, msg: Joy):
        if len(msg.axes) < 5:
            return

        btn_triangle = self._button_pressed(msg, self._button_start_index)
        btn_circle   = self._button_pressed(msg, self._button_stop_index)
        if btn_triangle and not self._prev_triangle:
            self._mission_pub.publish(String(data='START_AUTO'))
            self.get_logger().info('Triángulo: START_AUTO enviado')
        if btn_circle and not self._prev_circle:
            self._mission_pub.publish(String(data='STOP'))
            self.get_logger().warn('Círculo: STOP enviado')
        self._prev_triangle = btn_triangle
        self._prev_circle   = btn_circle

        if self._autonomous_active:
            self._ax_right_x = 0.0
            self._ax_right_y = 0.0
            self._enabled    = False
            self._turner_vel = 0.0
            return

        self._ax_right_x = apply_deadzone(msg.axes[2])
        self._ax_right_y = apply_deadzone(msg.axes[3])
        self._enabled    = msg.axes[4] > 0.0

        btn_x      = self._button_pressed(msg, self._button_turner_positive_index)
        btn_square = self._button_pressed(msg, self._button_turner_negative_index)
        if btn_x and not btn_square:
            self._turner_vel = TURNER_VEL
        elif btn_square and not btn_x:
            self._turner_vel = -TURNER_VEL
        else:
            self._turner_vel = 0.0

        # D-pad velocidad — soporta axes[7] (analógico) y botones HID
        dpad_y = msg.axes[7] if len(msg.axes) >= 8 else 0.0
        if dpad_y > 0.5 and self._prev_dpad_y <= 0.5:
            self._speed_level = min(self._speed_level + 1, len(SPEED_FACTORS) - 1)
            self.get_logger().info(
                f'↑ Velocidad nivel {self._speed_level + 1}/5 '
                f'(factor {SPEED_FACTORS[self._speed_level]:.2f})'
            )
        elif dpad_y < -0.5 and self._prev_dpad_y >= -0.5:
            self._speed_level = max(self._speed_level - 1, 0)
            self.get_logger().info(
                f'↓ Velocidad nivel {self._speed_level + 1}/5 '
                f'(factor {SPEED_FACTORS[self._speed_level]:.2f})'
            )
        self._prev_dpad_y = dpad_y

    def _button_pressed(self, msg: Joy, index: int) -> bool:
        return index >= 0 and len(msg.buttons) > index and bool(msg.buttons[index])

    def _autonomous_active_cb(self, msg: Bool):
        self._autonomous_active = bool(msg.data)
        if self._autonomous_active:
            self._turner_vel = 0.0
            self._enabled    = False
            self._ax_right_x = 0.0
            self._ax_right_y = 0.0
            self._turner_pub.publish(Float64(data=0.0))

    # ── control loop ──────────────────────────────────────────────────────────
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
        point.time_from_start = Duration(sec=0, nanosec=duration_ms * 1_000_000)
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
