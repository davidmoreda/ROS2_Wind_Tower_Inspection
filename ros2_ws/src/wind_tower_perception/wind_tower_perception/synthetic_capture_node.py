"""Autolabel a YOLO-format dataset from the inspection camera in Gazebo.

The node consumes:

* the ground-truth list of defects produced by ``generate_synthetic_world``,
* the camera image + ``camera_info``,
* the static transform ``world -> inspection_camera_optical_frame`` (the
  perception launcher publishes ``world -> odom`` from the spawn pose).

For every incoming frame:

1. Compute the camera pose in the world frame.
2. For each ground-truth defect, project its sphere centre into the image
   plane. Discard defects that are behind the camera, beyond the configured
   range, or outside the image rectangle.
3. Compute the projected pixel radius from the defect's physical radius and
   the depth in camera coordinates.
4. If at least one defect is visible, write the image and a YOLO label file.

Frames alternate between ``images/train`` and ``images/val`` (and similarly
for ``labels``) using ``val_every_n`` modulo, so the dataset is ready for
``ultralytics`` training out of the box.
"""

import math
import os
import threading
from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy._rclpy_pybind11 import RCLError
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener, TransformException


@dataclass
class GroundTruthDefect:
    defect_id: int
    class_id: int
    class_name: str
    world: np.ndarray   # (3,) — defect centre in world frame
    radius_m: float


class SyntheticCaptureNode(Node):
    """Capture and autolabel a YOLO dataset from the simulated camera."""

    def __init__(self):
        super().__init__('synthetic_capture')

        self._declare_parameters()
        self._load_parameters()

        self._defects = self._load_ground_truth(self._ground_truth_path)
        if not self._defects:
            raise RuntimeError(
                f'No defects loaded from {self._ground_truth_path}; '
                'aborting capture.'
            )

        self._bridge = CvBridge()
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        self._K: Optional[np.ndarray] = None
        self._image_width = 0
        self._image_height = 0

        self._lock = threading.Lock()
        self._frame_counter = 0
        self._saved_train = 0
        self._saved_val = 0
        self._image_msgs = 0
        self._camera_info_msgs = 0
        self._frames_without_camera_info = 0
        self._fallback_camera_info_uses = 0
        self._tf_failures = 0
        self._cv_failures = 0
        self._frames_without_labels = 0
        self._last_tf_error = ''

        self._train_images = os.path.join(self._output_dir, 'images', 'train')
        self._val_images = os.path.join(self._output_dir, 'images', 'val')
        self._train_labels = os.path.join(self._output_dir, 'labels', 'train')
        self._val_labels = os.path.join(self._output_dir, 'labels', 'val')
        output_paths = [self._train_images, self._val_images]
        if self._write_label_files:
            output_paths += [self._train_labels, self._val_labels]
        for path in output_paths:
            os.makedirs(path, exist_ok=True)

        self.create_subscription(
            CameraInfo, self._camera_info_topic, self._camera_info_cb, 5)
        self.create_subscription(
            Image, self._image_topic, self._image_cb, 5)
        if self._diagnostics_period_s > 0.0:
            self.create_timer(self._diagnostics_period_s, self._diagnostics_cb)

        self.get_logger().info(
            f'Synthetic capture ready: {len(self._defects)} defects loaded, '
            f'output_dir={self._output_dir}, save_empty={self._save_empty}, '
            f'write_labels={self._write_label_files}'
        )

    # ------------------------------------------------------------------ params
    def _declare_parameters(self):
        self.declare_parameter('image_topic', '/inspection/camera/image_raw')
        self.declare_parameter(
            'camera_info_topic', '/inspection/camera/camera_info')

        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('camera_frame', 'auto')
        self.declare_parameter(
            'tf_fallback_camera_frame', 'inspection_camera_optical_frame')

        self.declare_parameter('ground_truth_path', '')
        self.declare_parameter(
            'output_dir',
            '~/ROS2_Wind_Tower_Inspection/ros2_ws/datasets/wind_tower_dataset',
        )
        self.declare_parameter('max_depth_m', 6.0)
        self.declare_parameter('min_pixel_radius', 4.0)
        self.declare_parameter('min_bbox_dimension_px', 16.0)
        self.declare_parameter('min_visible_bbox_fraction', 0.65)
        self.declare_parameter('min_label_visual_contrast', 12.0)
        self.declare_parameter('label_contrast_padding_px', 6.0)
        self.declare_parameter('val_every_n', 5)
        self.declare_parameter('jpeg_quality', 92)
        self.declare_parameter('min_frame_period_s', 0.2)
        self.declare_parameter('tf_timeout_s', 0.2)
        self.declare_parameter('save_empty_frames', False)
        self.declare_parameter('write_label_files', True)
        self.declare_parameter('diagnostics_period_s', 5.0)
        self.declare_parameter('allow_intrinsics_fallback', True)
        self.declare_parameter('fallback_horizontal_fov_rad', 0.95)

    def _load_parameters(self):
        self._image_topic = str(self.get_parameter('image_topic').value)
        self._camera_info_topic = str(
            self.get_parameter('camera_info_topic').value)
        self._world_frame = str(self.get_parameter('world_frame').value)
        self._camera_frame = str(self.get_parameter('camera_frame').value)
        self._tf_fallback_camera_frame = str(
            self.get_parameter('tf_fallback_camera_frame').value).strip()

        self._ground_truth_path = os.path.expanduser(
            str(self.get_parameter('ground_truth_path').value))
        self._output_dir = os.path.expanduser(
            str(self.get_parameter('output_dir').value))
        self._max_depth_m = float(self.get_parameter('max_depth_m').value)
        self._min_pixel_radius = float(
            self.get_parameter('min_pixel_radius').value)
        self._min_bbox_dimension_px = float(
            self.get_parameter('min_bbox_dimension_px').value)
        self._min_visible_bbox_fraction = float(
            self.get_parameter('min_visible_bbox_fraction').value)
        self._min_label_visual_contrast = float(
            self.get_parameter('min_label_visual_contrast').value)
        self._label_contrast_padding_px = float(
            self.get_parameter('label_contrast_padding_px').value)
        self._val_every_n = max(2, int(
            self.get_parameter('val_every_n').value))
        self._jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self._min_frame_period_s = float(
            self.get_parameter('min_frame_period_s').value)
        self._tf_timeout = float(self.get_parameter('tf_timeout_s').value)
        self._save_empty = bool(
            self.get_parameter('save_empty_frames').value)
        self._write_label_files = bool(
            self.get_parameter('write_label_files').value)
        self._diagnostics_period_s = float(
            self.get_parameter('diagnostics_period_s').value)
        self._allow_intrinsics_fallback = bool(
            self.get_parameter('allow_intrinsics_fallback').value)
        self._fallback_hfov = float(
            self.get_parameter('fallback_horizontal_fov_rad').value)
        self._resolved_camera_frame = ''

        if not self._ground_truth_path:
            raise RuntimeError(
                "Parameter 'ground_truth_path' is required (path to the YAML "
                'produced by generate_synthetic_world).'
            )

    # --------------------------------------------------------------- loading
    def _load_ground_truth(self, path: str) -> List[GroundTruthDefect]:
        with open(path, 'r', encoding='utf-8') as fh:
            payload = yaml.safe_load(fh)
        out: List[GroundTruthDefect] = []
        for entry in payload.get('defects', []):
            out.append(GroundTruthDefect(
                defect_id=int(entry['defect_id']),
                class_id=int(entry['class_id']),
                class_name=str(entry.get('class_name', 'defect')),
                world=np.array([
                    float(entry['world_x']),
                    float(entry['world_y']),
                    float(entry['world_z']),
                ], dtype=float),
                radius_m=float(entry['radius_m']),
            ))
        return out

    # ------------------------------------------------------------ callbacks
    def _camera_info_cb(self, msg: CameraInfo):
        self._camera_info_msgs += 1
        K = np.array(msg.k, dtype=float).reshape(3, 3)
        if K[0, 0] <= 0.0 or K[1, 1] <= 0.0:
            return
        self._K = K
        self._image_width = int(msg.width)
        self._image_height = int(msg.height)

    def _image_cb(self, msg: Image):
        self._image_msgs += 1
        if self._K is None:
            if not self._set_fallback_intrinsics(msg.width, msg.height):
                self._frames_without_camera_info += 1
                return
        now = self._now_s()
        with self._lock:
            since_last = now - getattr(self, '_last_frame_stamp_s', 0.0)
        if since_last < self._min_frame_period_s:
            return

        if not self._write_label_files and self._save_empty:
            try:
                frame = self._bridge.imgmsg_to_cv2(
                    msg, desired_encoding='bgr8')
            except Exception as exc:  # noqa: BLE001
                self._cv_failures += 1
                self.get_logger().warn(
                    f'cv_bridge failed to decode image: {exc}')
                return
            self._write_capture(frame, [])
            with self._lock:
                self._last_frame_stamp_s = now
            return

        camera_frame = self._resolve_camera_frame(msg.header.frame_id)
        if not camera_frame:
            self._tf_failures += 1
            self._last_tf_error = (
                'camera_frame:=auto but image header.frame_id is empty')
            return

        try:
            tf_stamped = self._buffer.lookup_transform(
                camera_frame,
                self._world_frame,
                msg.header.stamp,
                timeout=Duration(seconds=self._tf_timeout),
            )
        except TransformException as exc:
            self._tf_failures += 1
            self._last_tf_error = str(exc)
            return

        T_world_to_cam = self._tf_to_matrix(tf_stamped.transform)

        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001
            self._cv_failures += 1
            self.get_logger().warn(f'cv_bridge failed to decode image: {exc}')
            return

        labels = self._compute_labels(T_world_to_cam, frame)
        if not labels and not self._save_empty:
            self._frames_without_labels += 1
            return

        self._write_capture(frame, labels)
        with self._lock:
            self._last_frame_stamp_s = now

    def _diagnostics_cb(self):
        total_saved = self._saved_train + self._saved_val
        camera_frame = self._resolved_camera_frame or self._camera_frame
        msg = (
            'Synthetic capture stats: '
            f'images={self._image_msgs}, camera_info={self._camera_info_msgs}, '
            f'camera_frame={camera_frame}, '
            f'no_camera_info={self._frames_without_camera_info}, '
            f'intrinsics_fallback={self._fallback_camera_info_uses}, '
            f'tf_failures={self._tf_failures}, cv_failures={self._cv_failures}, '
            f'no_labels={self._frames_without_labels}, saved={total_saved} '
            f'({self._saved_train} train + {self._saved_val} val)'
        )
        if self._last_tf_error:
            msg += f', last_tf_error="{self._last_tf_error}"'
        self.get_logger().info(msg)

    def _resolve_camera_frame(self, image_frame_id: str) -> str:
        if self._camera_frame != 'auto':
            return self._camera_frame
        frame_id = image_frame_id.strip()
        if (
            self._looks_like_gazebo_sensor_frame(frame_id)
            and self._tf_fallback_camera_frame
        ):
            if self._tf_fallback_camera_frame != self._resolved_camera_frame:
                self._resolved_camera_frame = self._tf_fallback_camera_frame
                self.get_logger().info(
                    'Synthetic capture image frame_id looks like a Gazebo '
                    f'sensor path ({frame_id}); using TF fallback '
                    f'camera_frame: {self._tf_fallback_camera_frame}')
            return self._resolved_camera_frame
        if frame_id and frame_id != self._resolved_camera_frame:
            self._resolved_camera_frame = frame_id
            self.get_logger().info(
                f'Synthetic capture using image frame_id as camera_frame: '
                f'{frame_id}')
        return self._resolved_camera_frame

    def _looks_like_gazebo_sensor_frame(self, frame_id: str) -> bool:
        return '/' in frame_id and frame_id.endswith('/inspection_camera')

    def _set_fallback_intrinsics(self, width: int, height: int) -> bool:
        if not self._allow_intrinsics_fallback:
            return False
        if width <= 0 or height <= 0:
            return False
        fx = (float(width) * 0.5) / math.tan(self._fallback_hfov * 0.5)
        fy = fx
        cx = (float(width) - 1.0) * 0.5
        cy = (float(height) - 1.0) * 0.5
        self._K = np.array([
            [fx, 0.0, cx],
            [0.0, fy, cy],
            [0.0, 0.0, 1.0],
        ], dtype=float)
        self._image_width = int(width)
        self._image_height = int(height)
        self._fallback_camera_info_uses += 1
        self.get_logger().warn(
            'No CameraInfo received; using fallback intrinsics from image size '
            f'{width}x{height} and horizontal_fov={self._fallback_hfov:.3f} rad.'
        )
        return True

    # --------------------------------------------------------------- labels
    def _compute_labels(self, T_world_to_cam: np.ndarray, frame) -> List[Tuple[int, float, float, float, float]]:
        if self._image_width <= 0 or self._image_height <= 0:
            return []
        fx = self._K[0, 0]
        fy = self._K[1, 1]
        cx = self._K[0, 2]
        cy = self._K[1, 2]

        results: List[Tuple[int, float, float, float, float]] = []

        for defect in self._defects:
            pw = np.array([defect.world[0], defect.world[1], defect.world[2], 1.0])
            pc = T_world_to_cam @ pw
            z = pc[2]
            if z <= 0.1 or z > self._max_depth_m:
                continue
            u = fx * (pc[0] / z) + cx
            v = fy * (pc[1] / z) + cy
            # Projected radius of a sphere approximated by its great circle:
            # angular radius alpha ≈ asin(r/z); pixel radius ≈ fx * tan(alpha).
            if z <= defect.radius_m:
                continue
            alpha = math.asin(min(0.999, defect.radius_m / z))
            r_px = fx * math.tan(alpha)
            if r_px < self._min_pixel_radius:
                continue

            x1 = u - r_px
            y1 = v - r_px
            x2 = u + r_px
            y2 = v + r_px
            full_area = max(1.0, (x2 - x1) * (y2 - y1))

            # Clip to image; discard if the box is mostly out of view.
            cx1 = max(0.0, x1)
            cy1 = max(0.0, y1)
            cx2 = min(self._image_width - 1.0, x2)
            cy2 = min(self._image_height - 1.0, y2)
            visible_w = cx2 - cx1
            visible_h = cy2 - cy1
            visible_area = max(0.0, visible_w) * max(0.0, visible_h)
            visible_fraction = visible_area / full_area
            if visible_fraction < self._min_visible_bbox_fraction:
                continue
            if visible_w < self._min_bbox_dimension_px:
                continue
            if visible_h < self._min_bbox_dimension_px:
                continue
            if not self._bbox_has_visual_signal(frame, cx1, cy1, cx2, cy2):
                continue

            cx_norm = ((cx1 + cx2) * 0.5) / self._image_width
            cy_norm = ((cy1 + cy2) * 0.5) / self._image_height
            w_norm = (cx2 - cx1) / self._image_width
            h_norm = (cy2 - cy1) / self._image_height

            results.append((defect.class_id, cx_norm, cy_norm, w_norm, h_norm))

        return results

    def _bbox_has_visual_signal(self, frame, x1: float, y1: float, x2: float, y2: float) -> bool:
        if self._min_label_visual_contrast <= 0.0:
            return True
        pad = self._label_contrast_padding_px
        x1_i = max(0, int(math.floor(x1 - pad)))
        y1_i = max(0, int(math.floor(y1 - pad)))
        x2_i = min(self._image_width - 1, int(math.ceil(x2 + pad)))
        y2_i = min(self._image_height - 1, int(math.ceil(y2 + pad)))
        if x2_i <= x1_i or y2_i <= y1_i:
            return False
        crop = frame[y1_i:y2_i + 1, x1_i:x2_i + 1]
        if crop.size == 0:
            return False
        pixels = crop.reshape(-1, crop.shape[-1]).astype(float)
        channel_ranges = np.percentile(pixels, 95, axis=0) - np.percentile(
            pixels, 5, axis=0)
        return float(np.max(channel_ranges)) >= self._min_label_visual_contrast

    # --------------------------------------------------------------- writers
    def _write_capture(self, frame, labels):
        with self._lock:
            self._frame_counter += 1
            idx = self._frame_counter
            is_val = (idx % self._val_every_n) == 0

        stem = f'img_{idx:06d}'
        img_dir = self._val_images if is_val else self._train_images
        lbl_dir = self._val_labels if is_val else self._train_labels
        image_path = os.path.join(img_dir, f'{stem}.jpg')
        label_path = os.path.join(lbl_dir, f'{stem}.txt')

        encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality]
        cv2.imwrite(image_path, frame, encode_params)
        if self._write_label_files:
            with open(label_path, 'w', encoding='utf-8') as fh:
                for cls, cx_n, cy_n, w_n, h_n in labels:
                    fh.write(
                        f'{cls} {cx_n:.6f} {cy_n:.6f} {w_n:.6f} {h_n:.6f}\n'
                    )

        with self._lock:
            if is_val:
                self._saved_val += 1
            else:
                self._saved_train += 1
            if idx % 50 == 0:
                self.get_logger().info(
                    f'Saved {self._saved_train} train + {self._saved_val} val frames '
                    f'(total {idx}).'
                )

    # ----------------------------------------------------------------- utils
    def _tf_to_matrix(self, transform) -> np.ndarray:
        t = transform.translation
        q = transform.rotation
        R = _quat_to_rotmat(q.x, q.y, q.z, q.w)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [t.x, t.y, t.z]
        return T

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def _quat_to_rotmat(x, y, z, w):
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array([
        [1 - 2 * (yy + zz),     2 * (xy - wz),     2 * (xz + wy)],
        [    2 * (xy + wz), 1 - 2 * (xx + zz),     2 * (yz - wx)],
        [    2 * (xz - wy),     2 * (yz + wx), 1 - 2 * (xx + yy)],
    ], dtype=float)


def main(args=None):
    rclpy.init(args=args)
    node = SyntheticCaptureNode()
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
