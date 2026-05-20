"""Project image-plane detections onto the cylindrical surface (x_axial, theta).

Pipeline per detection:

1. Read the bounding-box centre ``(u, v)`` from
   ``/inspection/detections/raw``.
2. Back-project ``(u, v)`` into a 3-D ray in the camera optical frame using
   the latest ``camera_info``.
3. Transform that ray into the cylinder reference frame (default: ``odom``,
   with the cylinder axis configured by parameters).
4. Intersect the ray with the cylindrical surface (inner wall).
5. Convert the intersection point to surface coordinates:

   * ``x_axial_m`` — distance along the tube axis.
   * ``theta_surface_deg`` — angle on the rotating surface, taking the
     current turner angle into account.

Two outputs are published:

* ``/inspection/defects/cylindrical`` (``std_msgs/String``, JSON): per-frame
  payload with the projected cylindrical coordinates of every detection.
* ``/inspection/defects/cumulative`` (``std_msgs/String``, JSON): rolling
  list of unique defects clustered by ``(x_axial, theta_surface)`` proximity.

The clustering radius (``cluster.x_tol_m``, ``cluster.theta_tol_deg``) is
configurable; new observations merge into the closest existing cluster, and
otherwise spawn a new one.
"""

import json
import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy._rclpy_pybind11 import RCLError
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float64, String
from tf2_ros import Buffer, TransformListener, TransformException
from vision_msgs.msg import Detection2DArray


@dataclass
class DefectCluster:
    cluster_id: int
    x_axial_m: float
    theta_surface_deg: float
    observations: int = 1
    max_score: float = 0.0
    class_id: str = 'circular_defect'
    last_seen_stamp_s: float = 0.0
    image_paths: List[str] = field(default_factory=list)


def _normalize_deg(angle_deg: float) -> float:
    return angle_deg % 360.0


def _angle_diff_deg(a: float, b: float) -> float:
    """Smallest unsigned angular distance between two angles, in degrees."""
    diff = (_normalize_deg(a) - _normalize_deg(b) + 540.0) % 360.0 - 180.0
    return abs(diff)


class DefectMapperNode(Node):
    """Project 2D detections to cylindrical (x_axial, theta_surface)."""

    def __init__(self):
        super().__init__('defect_mapper')

        self._declare_parameters()
        self._load_parameters()

        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)

        self._K: Optional[np.ndarray] = None
        self._image_width: int = 0
        self._image_height: int = 0
        self._theta_tube_deg: float = 0.0
        self._theta_surface_robot_deg: float = 0.0
        self._x_axial_robot_m: float = 0.0

        self._clusters: List[DefectCluster] = []
        self._next_cluster_id = 1

        self._pub_cyl = self.create_publisher(
            String, '/inspection/defects/cylindrical', 10)
        self._pub_cum = self.create_publisher(
            String, '/inspection/defects/cumulative', 10)

        self.create_subscription(
            CameraInfo, self._camera_info_topic, self._camera_info_cb, 5)
        self.create_subscription(
            Detection2DArray, self._detections_topic, self._detections_cb, 10)
        self.create_subscription(
            String, self._cylindrical_pose_topic, self._cylindrical_pose_cb, 10)
        self.create_subscription(
            Float64, self._turner_angle_topic, self._turner_cb, 10)

        self.get_logger().info(
            'Defect mapper ready: '
            f'cylinder_frame={self._cylinder_frame}, camera_frame={self._camera_frame}, '
            f'R={self._radius_m:.3f}m, axis={self._axis_direction}, '
            f'axis_point=({self._axis_origin[0]:.2f}, {self._axis_origin[1]:.2f}, '
            f'{self._axis_origin[2]:.2f})'
        )

    # ------------------------------------------------------------------ params
    def _declare_parameters(self):
        self.declare_parameter('detections_topic', '/inspection/detections/raw')
        self.declare_parameter(
            'camera_info_topic', '/inspection/camera/camera_info')
        self.declare_parameter(
            'cylindrical_pose_topic', '/inspection/cylindrical_pose')
        self.declare_parameter('turner_angle_topic', '/turner/angle')

        self.declare_parameter('camera_frame', 'inspection_camera_optical_frame')
        self.declare_parameter('cylinder_frame', 'world')

        # Cylinder geometry expressed in cylinder_frame coordinates. Defaults
        # match the SDF: tube axis along world Y, centered at world (0, 0, 4),
        # inner radius 3.925 m. The launcher publishes a static world→odom
        # transform from the spawn pose so this can stay in absolute world
        # coordinates.
        self.declare_parameter('cylinder.axis_origin_x_m', 0.0)
        self.declare_parameter('cylinder.axis_origin_y_m', 0.0)
        self.declare_parameter('cylinder.axis_origin_z_m', 4.0)
        self.declare_parameter('cylinder.axis_direction', 'y')
        self.declare_parameter('cylinder.radius_m', 3.925)
        self.declare_parameter('cylinder.x_axial_offset_m', 0.0)

        self.declare_parameter('cluster.x_tol_m', 0.30)
        self.declare_parameter('cluster.theta_tol_deg', 5.0)
        self.declare_parameter('cluster.max_observations_keep', 10000)

        self.declare_parameter('tf_timeout_s', 0.2)
        self.declare_parameter('min_detection_score', 0.0)

    def _load_parameters(self):
        self._detections_topic = str(self.get_parameter('detections_topic').value)
        self._camera_info_topic = str(
            self.get_parameter('camera_info_topic').value)
        self._cylindrical_pose_topic = str(
            self.get_parameter('cylindrical_pose_topic').value)
        self._turner_angle_topic = str(
            self.get_parameter('turner_angle_topic').value)

        self._camera_frame = str(self.get_parameter('camera_frame').value)
        self._cylinder_frame = str(self.get_parameter('cylinder_frame').value)

        self._axis_origin = (
            float(self.get_parameter('cylinder.axis_origin_x_m').value),
            float(self.get_parameter('cylinder.axis_origin_y_m').value),
            float(self.get_parameter('cylinder.axis_origin_z_m').value),
        )
        self._axis_direction = str(
            self.get_parameter('cylinder.axis_direction').value).lower()
        self._radius_m = float(self.get_parameter('cylinder.radius_m').value)
        self._x_axial_offset_m = float(
            self.get_parameter('cylinder.x_axial_offset_m').value)

        self._cluster_x_tol = float(self.get_parameter('cluster.x_tol_m').value)
        self._cluster_theta_tol = float(
            self.get_parameter('cluster.theta_tol_deg').value)
        self._cluster_keep = int(
            self.get_parameter('cluster.max_observations_keep').value)

        self._tf_timeout = float(self.get_parameter('tf_timeout_s').value)
        self._min_score = float(
            self.get_parameter('min_detection_score').value)

        if self._axis_direction not in ('x', 'y', 'z'):
            raise ValueError(
                "cylinder.axis_direction must be 'x', 'y' or 'z' "
                f'(got {self._axis_direction!r})'
            )

    # ------------------------------------------------------------- callbacks
    def _camera_info_cb(self, msg: CameraInfo):
        K = np.array(msg.k, dtype=float).reshape(3, 3)
        if K[0, 0] <= 0.0 or K[1, 1] <= 0.0:
            return
        self._K = K
        self._image_width = int(msg.width)
        self._image_height = int(msg.height)

    def _cylindrical_pose_cb(self, msg: String):
        try:
            payload = json.loads(msg.data)
            self._theta_surface_robot_deg = float(
                payload.get('theta_surface_deg', 0.0))
            self._theta_tube_deg = float(payload.get('theta_tube_deg', 0.0))
            self._x_axial_robot_m = float(payload.get('x_m', 0.0))
        except (json.JSONDecodeError, TypeError, ValueError):
            pass

    def _turner_cb(self, msg: Float64):
        self._theta_tube_deg = math.degrees(float(msg.data))

    def _detections_cb(self, msg: Detection2DArray):
        if self._K is None:
            return
        if not msg.detections:
            return

        try:
            tf_stamped = self._buffer.lookup_transform(
                self._cylinder_frame,
                self._camera_frame,
                msg.header.stamp,
                timeout=Duration(seconds=self._tf_timeout),
            )
        except TransformException as exc:
            self.get_logger().warn_once(
                f'TF unavailable from {self._cylinder_frame} to '
                f'{self._camera_frame}: {exc}. Detections will not be projected.'
            )
            return

        cam_pos, cam_rot = self._tf_to_pose(tf_stamped.transform)
        per_frame = []

        for det in msg.detections:
            if not det.results:
                continue
            score = float(det.results[0].hypothesis.score)
            if score < self._min_score:
                continue
            class_id = str(det.results[0].hypothesis.class_id)
            u = float(det.bbox.center.position.x)
            v = float(det.bbox.center.position.y)
            w_px = float(det.bbox.size_x)
            h_px = float(det.bbox.size_y)

            cyl_point = self._project_pixel_to_cylinder(u, v, cam_pos, cam_rot)
            if cyl_point is None:
                continue
            x_axial, theta_surface, theta_world = cyl_point

            per_frame.append({
                'class_id': class_id,
                'score': score,
                'pixel_center': {'u': u, 'v': v},
                'pixel_size': {'w': w_px, 'h': h_px},
                'x_axial_m': x_axial,
                'theta_surface_deg': theta_surface,
                'theta_world_around_axis_deg': math.degrees(theta_world),
            })

            self._absorb_observation(
                x_axial=x_axial,
                theta_surface=theta_surface,
                class_id=class_id,
                score=score,
            )

        if per_frame:
            self._pub_cyl.publish(String(data=json.dumps({
                'stamp_s': self._now_s(),
                'frame_id': msg.header.frame_id,
                'cylinder_frame': self._cylinder_frame,
                'theta_tube_deg': self._theta_tube_deg,
                'theta_surface_robot_deg': self._theta_surface_robot_deg,
                'detections': per_frame,
            })))
            self._publish_cumulative()

    # ----------------------------------------------------------- projection
    def _project_pixel_to_cylinder(
        self,
        u: float,
        v: float,
        cam_pos: np.ndarray,
        cam_rot: np.ndarray,
    ) -> Optional[Tuple[float, float, float]]:
        fx = self._K[0, 0]
        fy = self._K[1, 1]
        cx = self._K[0, 2]
        cy = self._K[1, 2]

        ray_cam = np.array([
            (u - cx) / fx,
            (v - cy) / fy,
            1.0,
        ], dtype=float)
        ray_cam /= np.linalg.norm(ray_cam)

        ray_world = cam_rot @ ray_cam
        origin = cam_pos

        intersection = self._intersect_ray_cylinder(origin, ray_world)
        if intersection is None:
            return None
        point = origin + intersection * ray_world
        return self._point_to_cylindrical(point)

    def _intersect_ray_cylinder(
        self,
        origin: np.ndarray,
        direction: np.ndarray,
    ) -> Optional[float]:
        """Return the smallest positive ray parameter t hitting the inner wall."""
        axis_map = {'x': 0, 'y': 1, 'z': 2}
        ai = axis_map[self._axis_direction]
        perp = [i for i in range(3) if i != ai]

        ox = origin[perp[0]] - self._axis_origin[perp[0]]
        oy = origin[perp[1]] - self._axis_origin[perp[1]]
        dx = direction[perp[0]]
        dy = direction[perp[1]]

        a = dx * dx + dy * dy
        b = 2.0 * (ox * dx + oy * dy)
        c = ox * ox + oy * oy - self._radius_m * self._radius_m
        disc = b * b - 4.0 * a * c
        if a < 1e-9 or disc < 0.0:
            return None
        sqrt_disc = math.sqrt(disc)
        t1 = (-b - sqrt_disc) / (2.0 * a)
        t2 = (-b + sqrt_disc) / (2.0 * a)

        # Camera is inside the cylinder, so the relevant intersection is the
        # first positive root (going outward to the wall).
        for t in sorted([t1, t2]):
            if t > 1e-3:
                return t
        return None

    def _point_to_cylindrical(
        self,
        point: np.ndarray,
    ) -> Tuple[float, float, float]:
        axis_map = {'x': 0, 'y': 1, 'z': 2}
        ai = axis_map[self._axis_direction]
        perp = [i for i in range(3) if i != ai]

        axial_local = point[ai] - self._axis_origin[ai]
        x_axial = axial_local + self._x_axial_offset_m

        px = point[perp[0]] - self._axis_origin[perp[0]]
        py = point[perp[1]] - self._axis_origin[perp[1]]

        # World-frame angle around the tube axis, measured from the bottom
        # (-perp[1] direction). The robot lives at world theta ≈ 0.
        theta_world = math.atan2(px, -py)
        theta_world_deg = math.degrees(theta_world)

        # Surface-frame angle: as the tube rotates by Δθ_tube, the surface
        # label that sits at world θ=0 is θ_tube. So for any defect:
        #   theta_surface = theta_world + theta_tube  (mod 360)
        # Equivalent to cylindrical_map_node's convention, where the robot
        # (at world θ=0) reports theta_surface_robot = theta_tube.
        theta_surface = _normalize_deg(theta_world_deg + self._theta_tube_deg)
        return x_axial, theta_surface, theta_world

    # ------------------------------------------------------------ clustering
    def _absorb_observation(
        self,
        *,
        x_axial: float,
        theta_surface: float,
        class_id: str,
        score: float,
    ) -> None:
        best: Optional[DefectCluster] = None
        best_cost = math.inf
        for cluster in self._clusters:
            dx = abs(cluster.x_axial_m - x_axial)
            dtheta = _angle_diff_deg(cluster.theta_surface_deg, theta_surface)
            if dx <= self._cluster_x_tol and dtheta <= self._cluster_theta_tol:
                cost = (dx / max(self._cluster_x_tol, 1e-6)) + (
                    dtheta / max(self._cluster_theta_tol, 1e-6))
                if cost < best_cost:
                    best_cost = cost
                    best = cluster
        now = self._now_s()
        if best is None:
            self._clusters.append(DefectCluster(
                cluster_id=self._next_cluster_id,
                x_axial_m=x_axial,
                theta_surface_deg=theta_surface,
                observations=1,
                max_score=score,
                class_id=class_id,
                last_seen_stamp_s=now,
            ))
            self._next_cluster_id += 1
        else:
            n = best.observations
            best.x_axial_m = (best.x_axial_m * n + x_axial) / (n + 1)
            best.theta_surface_deg = _normalize_deg(
                (best.theta_surface_deg * n + theta_surface) / (n + 1))
            best.observations = n + 1
            best.max_score = max(best.max_score, score)
            best.last_seen_stamp_s = now

        if len(self._clusters) > self._cluster_keep:
            self._clusters = sorted(
                self._clusters,
                key=lambda c: c.last_seen_stamp_s,
                reverse=True,
            )[: self._cluster_keep]

    def _publish_cumulative(self):
        payload = {
            'stamp_s': self._now_s(),
            'count': len(self._clusters),
            'cluster_tolerance': {
                'x_m': self._cluster_x_tol,
                'theta_deg': self._cluster_theta_tol,
            },
            'clusters': [
                {
                    'id': c.cluster_id,
                    'x_axial_m': c.x_axial_m,
                    'theta_surface_deg': c.theta_surface_deg,
                    'observations': c.observations,
                    'max_score': c.max_score,
                    'class_id': c.class_id,
                    'last_seen_stamp_s': c.last_seen_stamp_s,
                }
                for c in self._clusters
            ],
        }
        self._pub_cum.publish(String(data=json.dumps(payload)))

    # ----------------------------------------------------------------- utils
    def _tf_to_pose(self, transform) -> Tuple[np.ndarray, np.ndarray]:
        t = transform.translation
        q = transform.rotation
        pos = np.array([t.x, t.y, t.z], dtype=float)
        rot = _quat_to_rotmat(q.x, q.y, q.z, q.w)
        return pos, rot

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def _quat_to_rotmat(x: float, y: float, z: float, w: float) -> np.ndarray:
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return np.array([
        [1 - 2 * (yy + zz),     2 * (xy - wz),     2 * (xz + wy)],
        [    2 * (xy + wz), 1 - 2 * (xx + zz),     2 * (yz - wx)],
        [    2 * (xz - wy),     2 * (yz + wx), 1 - 2 * (xx + yy)],
    ], dtype=float)


def main(args=None):
    rclpy.init(args=args)
    node = DefectMapperNode()
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
