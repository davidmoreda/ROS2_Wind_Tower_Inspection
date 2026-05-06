"""
DualSense → sensor_msgs/Joy publisher (evdev, no joydev kernel module needed).

Publica en /joy con el mismo formato que joy_node de ROS 2.

Ejes publicados:
  axes[0] = joystick izquierdo X  (izq -1 / der +1)
  axes[1] = joystick izquierdo Y  (atrás -1 / adelante +1)
  axes[2] = joystick derecho X
  axes[3] = joystick derecho Y
  axes[4] = L2 analógico          (-1 sin pulsar / +1 pulsado)
  axes[5] = R2 analógico
Botones:
  buttons[0] = Cruz
  buttons[1] = Círculo
  buttons[2] = Cuadrado
  buttons[3] = Triángulo
  buttons[4] = L1
  buttons[5] = R1
  buttons[6] = L2 digital
  buttons[7] = R2 digital
  buttons[8] = Share
  buttons[9] = Options
  buttons[10] = PS
  buttons[11] = L3 (pulsar stick izq)
  buttons[12] = R3 (pulsar stick der)
"""

import threading

import evdev
from evdev import ecodes

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

DEVICE_PATH = '/dev/input/event0'

# Mapeo evdev ABS real del DualSense en Linux HID (verificado con evdev):
# ABS_Z  = stick der X  (center ~127)
# ABS_RZ = stick der Y  (center ~128)
# ABS_RX = L2 trigger   (0=sin pulsar, 255=fondo)
# ABS_RY = R2 trigger   (0=sin pulsar, 255=fondo)
ABS_MAP = {
    ecodes.ABS_X:   0,   # stick izq X
    ecodes.ABS_Y:   1,   # stick izq Y
    ecodes.ABS_Z:   2,   # stick der X
    ecodes.ABS_RZ:  3,   # stick der Y
    ecodes.ABS_RX:  4,   # L2 analógico
    ecodes.ABS_RY:  5,   # R2 analógico
}

# Mapeo evdev KEY → índice en buttons[]
BTN_MAP = {
    ecodes.BTN_SOUTH:  0,   # Cruz
    ecodes.BTN_EAST:   1,   # Círculo
    ecodes.BTN_WEST:   2,   # Cuadrado
    ecodes.BTN_NORTH:  3,   # Triángulo
    ecodes.BTN_TL:     4,   # L1
    ecodes.BTN_TR:     5,   # R1
    ecodes.BTN_TL2:    6,   # L2 digital
    ecodes.BTN_TR2:    7,   # R2 digital
    ecodes.BTN_SELECT: 8,   # Share
    ecodes.BTN_START:  9,   # Options
    ecodes.BTN_MODE:   10,  # PS
    ecodes.BTN_THUMBL: 11,  # L3
    ecodes.BTN_THUMBR: 12,  # R3
}


def _normalize_axis(value, info):
    """Normaliza un valor ABS a [-1.0, +1.0], invierte Y si necesario."""
    mid = (info.max + info.min) / 2.0
    half = (info.max - info.min) / 2.0
    if half == 0:
        return 0.0
    return -(value - mid) / half  # negado para convenio ROS (adelante = +1)


def _normalize_trigger(value, info):
    """Normaliza un gatillo a [-1.0 sin pulsar, +1.0 pulsado]."""
    span = info.max - info.min
    if span == 0:
        return -1.0
    normalized = (value - info.min) / span  # 0.0 … 1.0
    return normalized * 2.0 - 1.0           # -1.0 … +1.0


class DualSenseJoy(Node):

    def __init__(self):
        super().__init__('dualsense_joy')

        self._pub = self.create_publisher(Joy, '/robot/joy_teleop/joy', 10)

        self._axes = [0.0] * 6
        self._buttons = [0] * 13

        try:
            self._dev = evdev.InputDevice(DEVICE_PATH)
            # capabilities() devuelve {type: [(code, AbsInfo), ...]}, no un dict
            abs_codes = {
                code for code, _ in self._dev.capabilities().get(ecodes.EV_ABS, [])
            }
            self._abs_info = {
                code: self._dev.absinfo(code)
                for code in ABS_MAP
                if code in abs_codes
            }
            self.get_logger().info(f'DualSense conectado: {self._dev.name}')
        except Exception as e:
            self.get_logger().error(f'No se puede abrir {DEVICE_PATH}: {e}')
            raise

        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _read_loop(self):
        for event in self._dev.read_loop():
            if event.type == ecodes.EV_ABS and event.code in ABS_MAP:
                idx = ABS_MAP[event.code]
                info = self._abs_info.get(event.code)
                if info is None:
                    continue
                if event.code in (ecodes.ABS_RX, ecodes.ABS_RY):
                    self._axes[idx] = _normalize_trigger(event.value, info)
                else:
                    self._axes[idx] = _normalize_axis(event.value, info)
                self._publish()

            elif event.type == ecodes.EV_KEY and event.code in BTN_MAP:
                self._buttons[BTN_MAP[event.code]] = event.value
                self._publish()

    def _publish(self):
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = list(self._axes)
        msg.buttons = list(self._buttons)
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DualSenseJoy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
