#!/usr/bin/env python3
"""Test de integración YOLO: publica imágenes reales del dataset y muestra detecciones.

Uso — 2 terminales:

  T1 (detector con YOLO):
    cd ~/ROS2_Wind_Tower_Inspection/ros2_ws && source install/setup.bash
    ros2 run wind_tower_perception detector \
        --ros-args -p backend:=yolo \
        -p yolo.model_path:=$HOME/ROS2_Wind_Tower_Inspection/ros2_ws/models/best.pt

  T2 (este test):
    cd ~/ROS2_Wind_Tower_Inspection/ros2_ws && source install/setup.bash
    python3 ../tools/yolo_test_node.py

Publica a:   /inspection/camera/image_raw
Suscribe a:  /inspection/detections/text
             /inspection/detections/image_annotated  (guarda un JPEG por detección)
"""

import glob
import json
import os

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

DATASET_DIR = os.path.expanduser(
    '~/ROS2_Wind_Tower_Inspection/ros2_ws/datasets/roboflow_v2/images/val')
OUTPUT_DIR = os.path.expanduser(
    '~/ROS2_Wind_Tower_Inspection/inspections/test_frames')
PUBLISH_HZ = 1.0


class YoloTestNode(Node):

    def __init__(self):
        super().__init__('yolo_test')

        self._bridge = CvBridge()
        self._frame_count = 0
        self._detection_callbacks = 0

        self._images = sorted(glob.glob(os.path.join(DATASET_DIR, '*.jpg')))
        if not self._images:
            self.get_logger().error(f'No hay imágenes en {DATASET_DIR}')
            raise SystemExit(1)
        self._index = 0

        self._pub_img = self.create_publisher(
            Image, '/inspection/camera/image_raw', 10)
        self.create_subscription(
            String, '/inspection/detections/text', self._on_detections, 10)
        self.create_subscription(
            Image, '/inspection/detections/image_annotated', self._on_annotated, 1)

        os.makedirs(OUTPUT_DIR, exist_ok=True)

        self._timer = self.create_timer(1.0 / PUBLISH_HZ, self._publish)
        self.get_logger().info(
            f'Test YOLO — {len(self._images)} imágenes reales del dataset (val)\n'
            f'Publicando {PUBLISH_HZ:.0f} fps → /inspection/camera/image_raw\n'
            f'Frames anotados → {OUTPUT_DIR}/'
        )

    def _publish(self):
        if self._index >= len(self._images):
            self._timer.cancel()
            self.get_logger().info(
                f'\n--- Test completado ---\n'
                f'  Imágenes publicadas : {self._frame_count}\n'
                f'  Callbacks recibidos : {self._detection_callbacks}\n'
                f'  Guardados en        : {OUTPUT_DIR}/\n'
                + ('  ⚠ Sin detecciones — comprueba que T1 está corriendo con backend=yolo'
                   if self._detection_callbacks == 0 else '')
            )
            return

        path = self._images[self._index]
        num = self._index + 1
        self._index += 1

        frame = cv2.imread(path)
        if frame is None:
            return

        msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'inspection_camera_optical_frame'
        self._pub_img.publish(msg)
        self._frame_count += 1

        nombre = os.path.basename(path)
        self.get_logger().info(f'[{num}/{len(self._images)}] {nombre}')

    def _on_detections(self, msg: String):
        self._detection_callbacks += 1
        data = json.loads(msg.data)
        dets = data.get('detections', [])
        backend = data.get('backend', '?')

        if dets:
            resumen = ', '.join(f'{d["class_id"]} {d["score"]:.2f}' for d in dets)
            self.get_logger().info(f'  → [{backend}] {len(dets)} detección(es): {resumen}')
        else:
            self.get_logger().info(f'  → [{backend}] Sin detecciones')

    def _on_annotated(self, msg: Image):
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        idx = max(0, self._index - 1)
        nombre = os.path.basename(self._images[idx])
        path = os.path.join(OUTPUT_DIR, nombre)
        cv2.imwrite(path, frame)
        self.get_logger().info(f'  → guardado: {nombre}')


def main():
    rclpy.init()
    node = YoloTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f'Test finalizado — {node._frame_count} frames, '
            f'{node._detection_callbacks} callbacks de detección.'
        )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
