"""
Remove synthetic defect spheres only from the cloud used to build /scan.

The raw Gazebo GPU LiDAR point cloud is still published unchanged on
/velodyne_points. This node publishes a scan-clean cloud for
pointcloud_to_laserscan by dropping points that fall inside the defect sphere
models declared in the SDF world.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2

import tf2_ros


class DefectCloudFilter(Node):
    def __init__(self) -> None:
        super().__init__('defect_cloud_filter')

        self.declare_parameter('input_topic', '/velodyne_points')
        self.declare_parameter('output_topic', '/velodyne_points_scan_clean')
        self.declare_parameter('world_file', '')
        self.declare_parameter('fixed_frame', 'map')
        self.declare_parameter('radius_margin', 0.08)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.world_file = self.get_parameter('world_file').value
        self.fixed_frame = self.get_parameter('fixed_frame').value
        self.radius_margin = float(self.get_parameter('radius_margin').value)

        self.centers, self.radii = self._load_defects(self.world_file)
        self.radii = self.radii + self.radius_margin

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(
            PointCloud2, self.output_topic, qos_profile_sensor_data)
        self.sub = self.create_subscription(
            PointCloud2, self.input_topic, self._cb, qos_profile_sensor_data)

        self.get_logger().info(
            f'defect_cloud_filter: {self.input_topic} -> {self.output_topic} | '
            f'{len(self.centers)} defects, fixed_frame={self.fixed_frame}, '
            f'margin={self.radius_margin:.2f} m'
        )

    def _load_defects(self, world_file: str) -> tuple[np.ndarray, np.ndarray]:
        if not world_file:
            self.get_logger().warn('world_file vacío; no se filtrarán defectos')
            return np.empty((0, 3), dtype=np.float32), np.empty((0,), dtype=np.float32)

        path = Path(world_file).expanduser()
        root = ET.parse(path).getroot()
        centers = []
        radii = []
        for model in root.findall('.//model'):
            name = model.attrib.get('name', '')
            if not name.startswith('defect_'):
                continue
            pose_text = (model.findtext('pose') or '').strip()
            pose_vals = [float(v) for v in pose_text.split()[:3]]
            radius_text = model.findtext('.//visual/geometry/sphere/radius')
            if len(pose_vals) != 3 or radius_text is None:
                self.get_logger().warn(f'defecto {name} incompleto; omitido')
                continue
            centers.append(pose_vals)
            radii.append(float(radius_text))

        if not centers:
            self.get_logger().warn(f'no encontré modelos defect_* en {path}')
            return np.empty((0, 3), dtype=np.float32), np.empty((0,), dtype=np.float32)

        return (
            np.asarray(centers, dtype=np.float32),
            np.asarray(radii, dtype=np.float32),
        )

    def _cb(self, msg: PointCloud2) -> None:
        if self.centers.shape[0] == 0:
            self.pub.publish(msg)
            return

        try:
            pts = pc2.read_points_numpy(
                msg, field_names=('x', 'y', 'z'), skip_nans=True)
        except Exception as exc:
            self.get_logger().warn(
                f'no pude leer PointCloud2; paso nube original: {exc}',
                throttle_duration_sec=5.0,
            )
            self.pub.publish(msg)
            return

        if pts.dtype.names:
            xyz = np.column_stack([pts['x'], pts['y'], pts['z']]).astype(np.float32)
        else:
            xyz = np.asarray(pts, dtype=np.float32)
            if xyz.ndim == 1:
                xyz = xyz.reshape(-1, 3)

        if xyz.shape[0] == 0:
            self._publish(msg.header, xyz)
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                self.fixed_frame, msg.header.frame_id, msg.header.stamp,
                timeout=Duration(seconds=0.05))
        except Exception as exc:
            self.get_logger().warn(
                f'TF {self.fixed_frame}<-{msg.header.frame_id} no disponible; '
                f'paso nube original: {exc}',
                throttle_duration_sec=5.0,
            )
            self.pub.publish(msg)
            return

        world_xyz = self._transform_xyz(xyz, tf)
        keep = np.ones((xyz.shape[0],), dtype=bool)
        for center, radius in zip(self.centers, self.radii):
            delta = world_xyz - center
            keep &= np.einsum('ij,ij->i', delta, delta) > radius * radius

        self._publish(msg.header, xyz[keep])

    @staticmethod
    def _transform_xyz(xyz: np.ndarray, tf) -> np.ndarray:
        t = tf.transform.translation
        q = tf.transform.rotation
        xx, yy, zz, ww = q.x, q.y, q.z, q.w
        rot = np.array([
            [1 - 2 * (yy * yy + zz * zz), 2 * (xx * yy - zz * ww), 2 * (xx * zz + yy * ww)],
            [2 * (xx * yy + zz * ww), 1 - 2 * (xx * xx + zz * zz), 2 * (yy * zz - xx * ww)],
            [2 * (xx * zz - yy * ww), 2 * (yy * zz + xx * ww), 1 - 2 * (xx * xx + yy * yy)],
        ], dtype=np.float32)
        out = xyz @ rot.T
        out[:, 0] += t.x
        out[:, 1] += t.y
        out[:, 2] += t.z
        return out

    def _publish(self, header, xyz: np.ndarray) -> None:
        arr = np.ascontiguousarray(xyz, dtype=np.float32)
        out = PointCloud2()
        out.header = header
        out.height = 1
        out.width = arr.shape[0]
        out.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        out.is_bigendian = False
        out.point_step = 12
        out.row_step = 12 * arr.shape[0]
        out.is_dense = True
        out.data = arr.tobytes()
        self.pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DefectCloudFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
