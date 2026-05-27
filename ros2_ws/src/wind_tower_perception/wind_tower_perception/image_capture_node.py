"""Persist frames with detections and cylindrical metadata for offline review.

Each mission run becomes a directory under ``output_root`` named
``run_YYYYMMDD_HHMMSS``. Inside, the node writes:

* ``frames/frame_NNNNNN.jpg`` — captured image (JPEG, quality configurable).
* ``frames/frame_NNNNNN.json`` — sidecar with the metadata of that frame
  (timestamp, cylindrical pose, mission state, detections).
* ``detections.ndjson`` — append-only log with **one detection per line**.
  This is the file the LLM report generator reads.
* ``manifest.json`` — rolling summary of the run; rewritten on every save.

A frame is captured whenever:

* there is at least one detection (``save_on_detection``), or
* the periodic heartbeat fires (every ``heartbeat_period_s``) — useful for
  proving coverage even where no defects were found.

Both rules can be toggled independently. A minimum spacing between captures
(``min_capture_period_s``) prevents disk-bloat under high detection rates.
"""

import datetime
import json
import os
import threading
from dataclasses import dataclass
from typing import Optional

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy._rclpy_pybind11 import RCLError
from sensor_msgs.msg import Image
from std_msgs.msg import String


@dataclass
class CylindricalPose:
    stamp_s: float
    x_m: float
    theta_tube_deg: float
    theta_surface_deg: float
    yaw_deg: float
    mission_state: str
    in_tube_limits: bool


class ImageCaptureNode(Node):
    """Capture annotated frames with full cylindrical metadata."""

    def __init__(self):
        super().__init__('image_capture')

        self._declare_parameters()
        self._load_parameters()

        self._bridge = CvBridge()
        self._lock = threading.Lock()

        self._frame_counter = 0
        self._last_save_stamp_s = 0.0
        self._last_heartbeat_stamp_s = 0.0

        self._cylindrical: Optional[CylindricalPose] = None
        self._mission_state: str = 'UNKNOWN'
        self._last_detection_payload: Optional[dict] = None
        self._last_detection_stamp_s: float = 0.0
        self._last_projected_payload: Optional[dict] = None
        self._last_projected_stamp_s: float = 0.0

        self._run_id = self._make_run_id()
        self._run_dir = os.path.join(
            os.path.expanduser(self._output_root), self._run_id)
        self._frames_dir = os.path.join(self._run_dir, 'frames')
        self._detections_path = os.path.join(self._run_dir, 'detections.ndjson')
        self._manifest_path = os.path.join(self._run_dir, 'manifest.json')
        os.makedirs(self._frames_dir, exist_ok=True)
        self._write_manifest(initial=True)

        self.create_subscription(
            Image, self._image_topic, self._image_cb, 5)
        self.create_subscription(
            String, self._detections_topic, self._detections_cb, 10)
        self.create_subscription(
            String, self._cylindrical_pose_topic, self._cylindrical_cb, 10)
        self.create_subscription(
            String, self._mission_state_topic, self._mission_state_cb, 10)
        self.create_subscription(
            String, self._defects_projected_topic, self._projected_cb, 10)

        self.get_logger().info(
            f'Image capture ready (run_id={self._run_id}, dir={self._run_dir}).'
        )

    # ------------------------------------------------------------------ params
    def _declare_parameters(self):
        self.declare_parameter('image_topic', '/inspection/camera/image_raw')
        self.declare_parameter(
            'detections_topic', '/inspection/detections/text')
        self.declare_parameter(
            'cylindrical_pose_topic', '/inspection/cylindrical_pose')
        self.declare_parameter(
            'mission_state_topic', '/inspection/state_text')
        self.declare_parameter(
            'defects_projected_topic', '/inspection/defects/cylindrical')

        self.declare_parameter('output_root', '~/ROS2_Wind_Tower_Inspection/inspections')

        self.declare_parameter('save_on_detection', True)
        self.declare_parameter('save_heartbeat', True)
        self.declare_parameter('heartbeat_period_s', 5.0)
        self.declare_parameter('min_capture_period_s', 0.5)

        self.declare_parameter('jpeg_quality', 90)
        self.declare_parameter('require_in_tube_limits', False)
        self.declare_parameter(
            'detection_freshness_s', 1.0,
        )

    def _load_parameters(self):
        self._image_topic = str(self.get_parameter('image_topic').value)
        self._detections_topic = str(
            self.get_parameter('detections_topic').value)
        self._cylindrical_pose_topic = str(
            self.get_parameter('cylindrical_pose_topic').value)
        self._mission_state_topic = str(
            self.get_parameter('mission_state_topic').value)
        self._defects_projected_topic = str(
            self.get_parameter('defects_projected_topic').value)

        self._output_root = str(self.get_parameter('output_root').value)
        self._save_on_detection = bool(
            self.get_parameter('save_on_detection').value)
        self._save_heartbeat = bool(self.get_parameter('save_heartbeat').value)
        self._heartbeat_period_s = float(
            self.get_parameter('heartbeat_period_s').value)
        self._min_capture_period_s = float(
            self.get_parameter('min_capture_period_s').value)
        self._jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self._require_in_tube_limits = bool(
            self.get_parameter('require_in_tube_limits').value)
        self._detection_freshness_s = float(
            self.get_parameter('detection_freshness_s').value)

    # ------------------------------------------------------------- callbacks
    def _detections_cb(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('Could not decode detections JSON payload.')
            return
        with self._lock:
            self._last_detection_payload = payload
            self._last_detection_stamp_s = self._now_s()

    def _cylindrical_cb(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        try:
            pose = CylindricalPose(
                stamp_s=self._now_s(),
                x_m=float(payload.get('x_m', 0.0)),
                theta_tube_deg=float(payload.get('theta_tube_deg', 0.0)),
                theta_surface_deg=float(payload.get('theta_surface_deg', 0.0)),
                yaw_deg=float(payload.get('yaw_deg', 0.0)),
                mission_state=str(payload.get('mission_state', 'UNKNOWN')),
                in_tube_limits=bool(payload.get('in_tube_limits', False)),
            )
        except (TypeError, ValueError):
            return
        with self._lock:
            self._cylindrical = pose

    def _mission_state_cb(self, msg: String):
        with self._lock:
            self._mission_state = str(msg.data).strip() or 'UNKNOWN'

    def _projected_cb(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self._lock:
            self._last_projected_payload = payload
            self._last_projected_stamp_s = self._now_s()

    @staticmethod
    def _match_projected(det: dict, projected_payload: Optional[dict]) -> Optional[dict]:
        """Empareja una detección (cx_px, cy_px) con su proyección por píxel."""
        if not projected_payload:
            return None
        target_u = float(det.get('cx_px', 0.0))
        target_v = float(det.get('cy_px', 0.0))
        target_cls = str(det.get('class_id', ''))
        best: Optional[dict] = None
        best_dist2 = 25.0  # ≤ 5 px de tolerancia
        for entry in projected_payload.get('detections', []):
            if str(entry.get('class_id', '')) != target_cls:
                continue
            pc = entry.get('pixel_center') or {}
            u = float(pc.get('u', 0.0))
            v = float(pc.get('v', 0.0))
            d2 = (u - target_u) ** 2 + (v - target_v) ** 2
            if d2 < best_dist2:
                best_dist2 = d2
                best = entry
        return best

    def _image_cb(self, msg: Image):
        now = self._now_s()

        with self._lock:
            cyl = self._cylindrical
            detections_payload = self._last_detection_payload
            detections_fresh = (
                detections_payload is not None
                and (now - self._last_detection_stamp_s)
                <= self._detection_freshness_s
            )
            since_last_save = now - self._last_save_stamp_s
            since_heartbeat = now - self._last_heartbeat_stamp_s
            mission_state = (
                cyl.mission_state if cyl is not None else self._mission_state
            )

        if self._require_in_tube_limits and (cyl is None or not cyl.in_tube_limits):
            return

        detections = []
        if detections_fresh:
            detections = list(detections_payload.get('detections', []))
        has_detection = bool(detections)

        reason: Optional[str] = None
        if self._save_on_detection and has_detection:
            reason = 'detection'
        elif (
            self._save_heartbeat
            and since_heartbeat >= self._heartbeat_period_s
        ):
            reason = 'heartbeat'

        if reason is None:
            return
        if since_last_save < self._min_capture_period_s and reason != 'detection':
            return

        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'cv_bridge failed to decode image: {exc}')
            return

        self._save_capture(frame, msg, cyl, mission_state, detections, reason)

        with self._lock:
            self._last_save_stamp_s = now
            if reason == 'heartbeat':
                self._last_heartbeat_stamp_s = now

    # ---------------------------------------------------------------- writers
    def _save_capture(
        self,
        frame,
        image_msg: Image,
        cyl: Optional[CylindricalPose],
        mission_state: str,
        detections: list,
        reason: str,
    ):
        with self._lock:
            self._frame_counter += 1
            idx = self._frame_counter

        stem = f'frame_{idx:06d}'
        image_path = os.path.join(self._frames_dir, f'{stem}.jpg')
        sidecar_path = os.path.join(self._frames_dir, f'{stem}.json')

        encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), max(50, min(100, self._jpeg_quality))]
        ok = cv2.imwrite(image_path, frame, encode_params)
        if not ok:
            self.get_logger().error(f'Failed to write image {image_path}')
            return

        height, width = frame.shape[:2]
        image_relpath = os.path.relpath(image_path, self._run_dir)

        sidecar = {
            'run_id': self._run_id,
            'frame_index': idx,
            'image_path': image_relpath,
            'capture_reason': reason,
            'wall_clock_iso': datetime.datetime.utcnow().isoformat() + 'Z',
            'ros_stamp_s': (
                image_msg.header.stamp.sec
                + image_msg.header.stamp.nanosec * 1e-9
            ),
            'node_stamp_s': self._now_s(),
            'image': {
                'width': width,
                'height': height,
                'frame_id': image_msg.header.frame_id,
            },
            'mission_state': mission_state,
            'cylindrical_pose': (
                {
                    'x_m': cyl.x_m,
                    'theta_tube_deg': cyl.theta_tube_deg,
                    'theta_surface_deg': cyl.theta_surface_deg,
                    'yaw_deg': cyl.yaw_deg,
                    'in_tube_limits': cyl.in_tube_limits,
                }
                if cyl is not None
                else None
            ),
            'detections': detections,
        }
        with open(sidecar_path, 'w', encoding='utf-8') as fh:
            json.dump(sidecar, fh, indent=2)

        if detections:
            with self._lock:
                projected = self._last_projected_payload
                projected_fresh = (
                    projected is not None
                    and (self._now_s() - self._last_projected_stamp_s)
                    <= self._detection_freshness_s
                )
            projected_payload = projected if projected_fresh else None
            with open(self._detections_path, 'a', encoding='utf-8') as fh:
                for det in detections:
                    proj = self._match_projected(det, projected_payload)
                    defect_pose = None
                    if proj is not None:
                        defect_pose = {
                            'x_axial_m': float(proj.get('x_axial_m', 0.0)),
                            'theta_surface_deg': float(
                                proj.get('theta_surface_deg', 0.0)),
                            'theta_world_around_axis_deg': float(
                                proj.get('theta_world_around_axis_deg', 0.0)),
                        }
                    line = {
                        'run_id': self._run_id,
                        'frame_index': idx,
                        'image_path': image_relpath,
                        'wall_clock_iso': sidecar['wall_clock_iso'],
                        'mission_state': mission_state,
                        'cylindrical_pose': sidecar['cylindrical_pose'],
                        'defect_cylindrical_pose': defect_pose,
                        'detection': det,
                    }
                    fh.write(json.dumps(line) + '\n')

        self._write_manifest(
            initial=False,
            last_frame_index=idx,
            last_reason=reason,
            last_n_detections=len(detections),
        )

    def _write_manifest(self, *, initial: bool, **kwargs):
        try:
            existing = {}
            if not initial and os.path.isfile(self._manifest_path):
                with open(self._manifest_path, 'r', encoding='utf-8') as fh:
                    existing = json.load(fh)
            manifest = {
                'run_id': self._run_id,
                'run_dir': self._run_dir,
                'frames_dir': os.path.relpath(self._frames_dir, self._run_dir),
                'detections_log': os.path.relpath(
                    self._detections_path, self._run_dir),
                'started_at_iso': existing.get(
                    'started_at_iso',
                    datetime.datetime.utcnow().isoformat() + 'Z',
                ),
                'updated_at_iso': datetime.datetime.utcnow().isoformat() + 'Z',
                'total_frames_saved': self._frame_counter,
            }
            manifest.update({k: v for k, v in kwargs.items() if v is not None})
            with open(self._manifest_path, 'w', encoding='utf-8') as fh:
                json.dump(manifest, fh, indent=2)
        except OSError as exc:
            self.get_logger().warn(f'Could not update manifest: {exc}')

    # ----------------------------------------------------------------- utils
    def _make_run_id(self) -> str:
        return 'run_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S')

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = ImageCaptureNode()
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
