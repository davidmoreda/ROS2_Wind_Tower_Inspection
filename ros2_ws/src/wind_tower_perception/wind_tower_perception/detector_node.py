"""Circular-defect detector for wind-tower internal inspection.

Subscribes to the inspection camera and publishes detections of circular
features (rust spots, pitting, through-holes) using one of two interchangeable
backends:

* ``hough`` (default): OpenCV ``HoughCircles`` over a blurred grayscale image.
  Works out of the box with zero training. Good baseline for clean simulated
  imagery, weak under heavy texture/noise.
* ``yolo``: ``ultralytics.YOLO`` inference loaded from a ``.pt`` file. If the
  model file is missing or ``ultralytics`` is not installed, the node logs a
  warning and silently falls back to ``hough`` so the pipeline never goes dark.

Outputs:

* ``/inspection/detections/raw`` (``vision_msgs/Detection2DArray``): standard
  ROS detection format.
* ``/inspection/detections/text`` (``std_msgs/String`` JSON): same content,
  for nodes that prefer the project-wide JSON convention.
* ``/inspection/detections/image_annotated`` (``sensor_msgs/Image``):
  per-frame visualization with bounding boxes burned in.
"""

import json
import math
import os
from dataclasses import dataclass
from typing import List, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy._rclpy_pybind11 import RCLError
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)


@dataclass
class CircleDetection:
    """Backend-agnostic detection payload."""

    class_id: str
    score: float
    cx: float          # bounding-box center x (pixels)
    cy: float          # bounding-box center y (pixels)
    w: float           # bounding-box width (pixels)
    h: float           # bounding-box height (pixels)
    radius_px: float   # equivalent circular radius (pixels)


class DetectorNode(Node):
    """Detect circular defects from the inspection camera."""

    def __init__(self):
        super().__init__('defect_detector')

        self._declare_parameters()
        self._load_parameters()

        self._bridge = CvBridge()
        self._yolo_model = None
        self._yolo_ready = False
        self._effective_backend = self._backend

        if self._backend == 'yolo':
            self._yolo_ready = self._try_load_yolo()
            if not self._yolo_ready:
                self.get_logger().warn(
                    f'YOLO backend requested but model "{self._yolo_model_path}" '
                    'could not be loaded; falling back to HoughCircles.'
                )
                self._effective_backend = 'hough'

        self._pub_raw = self.create_publisher(
            Detection2DArray, '/inspection/detections/raw', 10)
        self._pub_text = self.create_publisher(
            String, '/inspection/detections/text', 10)
        self._pub_annotated = self.create_publisher(
            Image, '/inspection/detections/image_annotated', 1)

        self.create_subscription(
            Image, self._image_topic, self._image_cb, 5)

        self.get_logger().info(
            f'Defect detector ready (backend={self._effective_backend}, '
            f'topic={self._image_topic}).'
        )

    # ------------------------------------------------------------------ params
    def _declare_parameters(self):
        self.declare_parameter('image_topic', '/inspection/camera/image_raw')

        self.declare_parameter('backend', 'hough')
        self.declare_parameter('publish_annotated', True)

        # YOLO backend
        self.declare_parameter('yolo.model_path', '')
        self.declare_parameter('yolo.conf_threshold', 0.25)
        self.declare_parameter('yolo.iou_threshold', 0.45)
        self.declare_parameter('yolo.imgsz', 640)
        self.declare_parameter('yolo.device', 'cpu')

        # HoughCircles backend
        self.declare_parameter('hough.gaussian_blur_kernel', 7)
        self.declare_parameter('hough.dp', 1.2)
        self.declare_parameter('hough.min_dist_px', 40.0)
        self.declare_parameter('hough.canny_high', 120.0)
        self.declare_parameter('hough.accumulator_threshold', 30.0)
        self.declare_parameter('hough.min_radius_px', 8)
        self.declare_parameter('hough.max_radius_px', 120)
        self.declare_parameter('hough.score_constant', 0.6)

    def _load_parameters(self):
        self._image_topic = str(self.get_parameter('image_topic').value)
        self._backend = str(self.get_parameter('backend').value).lower()
        self._publish_annotated = bool(
            self.get_parameter('publish_annotated').value)

        self._yolo_model_path = str(
            self.get_parameter('yolo.model_path').value)
        self._yolo_conf = float(
            self.get_parameter('yolo.conf_threshold').value)
        self._yolo_iou = float(
            self.get_parameter('yolo.iou_threshold').value)
        self._yolo_imgsz = int(self.get_parameter('yolo.imgsz').value)
        self._yolo_device = str(self.get_parameter('yolo.device').value)

        self._h_blur = int(self.get_parameter('hough.gaussian_blur_kernel').value)
        if self._h_blur % 2 == 0:
            self._h_blur += 1
        self._h_dp = float(self.get_parameter('hough.dp').value)
        self._h_min_dist = float(self.get_parameter('hough.min_dist_px').value)
        self._h_canny_high = float(self.get_parameter('hough.canny_high').value)
        self._h_acc_thr = float(
            self.get_parameter('hough.accumulator_threshold').value)
        self._h_min_r = int(self.get_parameter('hough.min_radius_px').value)
        self._h_max_r = int(self.get_parameter('hough.max_radius_px').value)
        self._h_score_const = float(self.get_parameter('hough.score_constant').value)

    # -------------------------------------------------------------- yolo setup
    def _try_load_yolo(self) -> bool:
        if not self._yolo_model_path:
            return False
        self._yolo_model_path = os.path.expanduser(self._yolo_model_path)
        if not os.path.isfile(self._yolo_model_path):
            return False
        try:
            from ultralytics import YOLO  # noqa: WPS433 — lazy import on purpose
        except ImportError:
            self.get_logger().warn(
                '`ultralytics` is not installed. Run `pip install ultralytics` '
                'inside the workspace virtualenv to enable the YOLO backend.'
            )
            return False
        try:
            self._yolo_model = YOLO(self._yolo_model_path)
            return True
        except Exception as exc:  # noqa: BLE001 — surface any load failure
            self.get_logger().error(
                f'Failed to load YOLO weights at {self._yolo_model_path}: {exc}'
            )
            return False

    # ------------------------------------------------------------- callbacks
    def _image_cb(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001 — defensive against malformed images
            self.get_logger().warn(f'cv_bridge failed to decode image: {exc}')
            return

        if self._effective_backend == 'yolo' and self._yolo_ready:
            detections = self._detect_yolo(frame)
        else:
            detections = self._detect_hough(frame)

        self._publish(msg.header, frame, detections)

    # ----------------------------------------------------------- yolo backend
    def _detect_yolo(self, frame: np.ndarray) -> List[CircleDetection]:
        results = self._yolo_model.predict(
            source=frame,
            conf=self._yolo_conf,
            iou=self._yolo_iou,
            imgsz=self._yolo_imgsz,
            device=self._yolo_device,
            verbose=False,
        )
        out: List[CircleDetection] = []
        if not results:
            return out
        names = getattr(self._yolo_model, 'names', None) or {}
        boxes = results[0].boxes
        if boxes is None or boxes.shape[0] == 0:
            return out
        xyxy = boxes.xyxy.cpu().numpy()
        conf = boxes.conf.cpu().numpy()
        cls = boxes.cls.cpu().numpy().astype(int)
        for (x1, y1, x2, y2), c, k in zip(xyxy, conf, cls):
            w = float(x2 - x1)
            h = float(y2 - y1)
            cx = float(x1 + w * 0.5)
            cy = float(y1 + h * 0.5)
            class_id = str(names.get(int(k), int(k))) if isinstance(names, dict) else str(int(k))
            out.append(CircleDetection(
                class_id=class_id,
                score=float(c),
                cx=cx, cy=cy,
                w=w, h=h,
                radius_px=0.5 * math.hypot(w, h) * 0.5,
            ))
        return out

    # ---------------------------------------------------------- hough backend
    def _detect_hough(self, frame: np.ndarray) -> List[CircleDetection]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (self._h_blur, self._h_blur), 0)
        circles = cv2.HoughCircles(
            gray,
            cv2.HOUGH_GRADIENT,
            dp=self._h_dp,
            minDist=self._h_min_dist,
            param1=self._h_canny_high,
            param2=self._h_acc_thr,
            minRadius=self._h_min_r,
            maxRadius=self._h_max_r,
        )
        out: List[CircleDetection] = []
        if circles is None:
            return out
        for cx, cy, r in circles[0]:
            # Heuristic score: Hough does not return a real confidence, so we
            # map accumulator vs. configured threshold to a bounded score that
            # downstream nodes can rank without claiming probabilistic meaning.
            score = max(0.1, min(0.9, self._h_score_const))
            out.append(CircleDetection(
                class_id='circular_defect',
                score=score,
                cx=float(cx), cy=float(cy),
                w=float(r * 2.0), h=float(r * 2.0),
                radius_px=float(r),
            ))
        return out

    # ----------------------------------------------------------- publication
    def _publish(self, header, frame: np.ndarray, detections: List[CircleDetection]):
        array_msg = Detection2DArray()
        array_msg.header = header
        for d in detections:
            det = Detection2D()
            det.header = header
            bbox = BoundingBox2D()
            bbox.center.position.x = d.cx
            bbox.center.position.y = d.cy
            bbox.center.theta = 0.0
            bbox.size_x = max(1.0, d.w)
            bbox.size_y = max(1.0, d.h)
            det.bbox = bbox
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = d.class_id
            hyp.hypothesis.score = float(d.score)
            det.results.append(hyp)
            array_msg.detections.append(det)
        self._pub_raw.publish(array_msg)

        text_payload = {
            'stamp_s': self._now_s(),
            'frame_id': header.frame_id,
            'backend': self._effective_backend,
            'detections': [
                {
                    'class_id': d.class_id,
                    'score': d.score,
                    'cx_px': d.cx,
                    'cy_px': d.cy,
                    'w_px': d.w,
                    'h_px': d.h,
                    'radius_px': d.radius_px,
                }
                for d in detections
            ],
        }
        self._pub_text.publish(String(data=json.dumps(text_payload)))

        if self._publish_annotated:
            annotated = self._draw_annotations(frame, detections)
            ann_msg = self._bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            ann_msg.header = header
            self._pub_annotated.publish(ann_msg)

    def _draw_annotations(
        self,
        frame: np.ndarray,
        detections: List[CircleDetection],
    ) -> np.ndarray:
        annotated = frame.copy()
        for d in detections:
            x1 = int(d.cx - d.w * 0.5)
            y1 = int(d.cy - d.h * 0.5)
            x2 = int(d.cx + d.w * 0.5)
            y2 = int(d.cy + d.h * 0.5)
            color = (0, 200, 255)
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
            cv2.circle(annotated, (int(d.cx), int(d.cy)), int(d.radius_px), color, 1)
            label = f'{d.class_id} {d.score:.2f}'
            cv2.putText(
                annotated, label, (x1, max(0, y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA,
            )
        return annotated

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
