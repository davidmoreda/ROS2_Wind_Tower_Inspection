"""
Acumula el mapa del tubo usando localización por cylinder fit.

Localización:
  /robot_in_tube  (PoseStamped) — x axial y φ angular del cylinder_localizer
  /turner/angle   (Float64)     — θ virador exacto

Usa TODOS los rings del VLP-16 para máxima densidad de puntos en el mapa.
Los puntos se colocan en el frame canónico del tubo des-rotando por −θ.

Salida:
  /map_cloud        PointCloud2  — mapa acumulado (frame odom)
  /map_cloud/stats  String       — estadísticas
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time as RCLTime
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64, String, Header
import tf2_ros
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped


def _quat_to_rot(q) -> np.ndarray:
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y-z*w),   2*(x*z+y*w)],
        [  2*(x*y+z*w), 1-2*(x*x+z*z),   2*(y*z-x*w)],
        [  2*(x*z-y*w),   2*(y*z+x*w), 1-2*(x*x+y*y)],
    ], dtype=np.float64)


def _rot_y(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[ c, 0, s],
                     [ 0, 1, 0],
                     [-s, 0, c]], dtype=np.float64)


class MapAccumulatorNode(Node):

    TUBE_CENTER = np.array([0.0, 0.0, 4.0])

    def __init__(self):
        super().__init__('map_accumulator')

        self.declare_parameter('downsample', 3)
        self.declare_parameter('max_points', 1_000_000)
        self.declare_parameter('publish_hz', 1.0)

        self._ds      = self.get_parameter('downsample').value
        self._max_pts = self.get_parameter('max_points').value

        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._R_s2b = None
        self._t_s2b = np.zeros(3)

        # Estado localización
        self._theta      = 0.0
        self._robot_pose = None   # de cylinder_localizer: (x_axial, phi)
        self._using_localizer = False

        # Publicar TF odom → tube_canonical
        self._static_tf = StaticTransformBroadcaster(self)
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id  = 'tube_canonical'
        tf_msg.transform.rotation.w = 1.0
        self._static_tf.sendTransform(tf_msg)

        self._pts        = np.empty((0, 3), dtype=np.float32)
        self._scan_count = 0

        self._pub_cloud = self.create_publisher(PointCloud2, '/map_cloud', 1)
        self._pub_stats = self.create_publisher(String, '/map_cloud/stats', 10)

        self.create_subscription(Float64,      '/turner/angle',    self._angle_cb,     10)
        self.create_subscription(PoseStamped,  '/robot_in_tube',   self._localizer_cb, 10)
        self.create_subscription(PointCloud2,  '/velodyne_points', self._cloud_cb,      5)

        period = 1.0 / self.get_parameter('publish_hz').value
        self.create_timer(period, self._publish_map)

        self.get_logger().info(
            f'MapAccumulator listo — downsample=1/{self._ds}, '
            f'max={self._max_pts} pts, todos los rings VLP-16'
        )

    def _angle_cb(self, msg: Float64):
        self._theta = msg.data

    def _localizer_cb(self, msg: PoseStamped):
        self._robot_pose = msg
        self._using_localizer = True

    def _cloud_cb(self, msg: PointCloud2):
        # TF sensor → base_link (estático, una sola vez)
        if self._R_s2b is None:
            try:
                tf = self._tf_buffer.lookup_transform(
                    'base_link', msg.header.frame_id,
                    RCLTime(), timeout=Duration(seconds=0.2))
                t = tf.transform.translation
                self._R_s2b = _quat_to_rot(tf.transform.rotation)
                self._t_s2b = np.array([t.x, t.y, t.z])
            except Exception as e:
                self.get_logger().warn(f'TF no disponible: {e}', throttle_duration_sec=3.0)
                return

        # Leer TODOS los puntos (todos los rings para máxima densidad)
        pts_raw = list(pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))
        if not pts_raw:
            return
        arr = np.array([[p[0], p[1], p[2]] for p in pts_raw], dtype=np.float64)

        # 1) sensor → base_link
        arr = arr @ self._R_s2b.T + self._t_s2b

        # 2) Colocar en frame del tubo usando localización
        if self._using_localizer and self._robot_pose is not None:
            # x_axial del robot (pose.position.x del cylinder_localizer)
            x_axial = self._robot_pose.pose.position.x
            # Traslación axial: el eje axial del tubo es Y en base_link
            arr[:, 1] += x_axial
        else:
            # Fallback: sin traslación axial (robot quieto en origen)
            pass

        # 3) Des-rotar por −θ (virador) alrededor eje Y para frame canónico tubo
        arr -= self.TUBE_CENTER
        arr  = arr @ _rot_y(-self._theta).T
        arr += self.TUBE_CENTER

        # Downsample
        arr = arr[::self._ds].astype(np.float32)
        self._pts = np.vstack([self._pts, arr]) if self._pts.size else arr
        self._scan_count += 1

        if len(self._pts) > self._max_pts:
            self._pts = self._pts[-self._max_pts:]

    def _publish_map(self):
        if len(self._pts) == 0:
            return

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'odom'

        cloud = pc2.create_cloud_xyz32(header, self._pts.tolist())
        self._pub_cloud.publish(cloud)

        src = 'cylinder_localizer' if self._using_localizer else 'sin-localizer'
        stats = (
            f'src={src} | θ={math.degrees(self._theta):.1f}° | '
            f'scans={self._scan_count} | pts={len(self._pts):,}'
        )
        self._pub_stats.publish(String(data=stats))
        self.get_logger().info(stats, throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = MapAccumulatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
