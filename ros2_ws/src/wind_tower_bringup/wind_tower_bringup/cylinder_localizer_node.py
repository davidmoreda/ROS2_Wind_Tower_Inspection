"""
Localización del robot dentro del tubo por ajuste de cilindro al scan LiDAR.

Por cada scan LiDAR ajusta un círculo a la sección XZ (sección transversal del tubo).
El centro del círculo ajustado = posición del robot respecto al eje del tubo.
No depende de ruedas ni IMU — el tubo es la referencia absoluta.

Publica:
  /robot_in_tube  (geometry_msgs/PoseStamped)
      position.x = x axial (m) desde inicio — componente Y del centroide en base_link
      position.y = φ angular robot en tubo (rad) — atan2 del offset radial
      position.z = r offset desde eje tubo (m) — debería ser ~4m
      orientation = cuaternión identidad (sin uso por ahora)

  /cylinder_fit/wall_points  (PointCloud2) — puntos de pared usados para el fit
  /cylinder_fit/stats        (String)      — info del fit en tiempo real
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


def _quat_to_rot(q) -> np.ndarray:
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y-z*w),   2*(x*z+y*w)],
        [  2*(x*y+z*w), 1-2*(x*x+z*z),   2*(y*z-x*w)],
        [  2*(x*z-y*w),   2*(y*z+x*w), 1-2*(x*x+y*y)],
    ], dtype=np.float64)


def fit_circle_2d(pts_xz: np.ndarray):
    """
    Ajuste algebraico de círculo a puntos 2D (x, z).
    Resuelve: x²+z² + Dx + Ez + F = 0  →  centro=(−D/2, −E/2), r=sqrt(D²/4+E²/4−F)
    Devuelve (cx, cz, r, residual_rms) o None si falla.
    """
    if len(pts_xz) < 6:
        return None
    x, z = pts_xz[:, 0], pts_xz[:, 1]
    A = np.column_stack([x, z, np.ones(len(x))])
    b = -(x**2 + z**2)
    try:
        result, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    except Exception:
        return None
    D, E, F = result
    cx, cz = -D / 2.0, -E / 2.0
    r2 = cx**2 + cz**2 - F
    if r2 <= 0:
        return None
    r = math.sqrt(r2)
    residuals = np.abs(np.sqrt((x - cx)**2 + (z - cz)**2) - r)
    return cx, cz, r, float(np.sqrt(np.mean(residuals**2)))


class CylinderLocalizerNode(Node):

    TUBE_RADIUS_NOM = 3.925  # radio efectivo medido del STL (m)
    WALL_MARGIN     = 0.5    # puntos dentro de [R-margin, R+margin] son pared
    RINGS_HOR       = set(range(2, 14))  # todos menos los más extremos (0,1,14,15)

    def __init__(self):
        super().__init__('cylinder_localizer')

        self.declare_parameter('tube_radius', self.TUBE_RADIUS_NOM)
        self.declare_parameter('wall_margin', self.WALL_MARGIN)
        self.declare_parameter('min_wall_points', 80)

        self._R_nom    = self.get_parameter('tube_radius').value
        self._margin   = self.get_parameter('wall_margin').value
        self._min_pts  = self.get_parameter('min_wall_points').value

        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._R_s2b = None
        self._t_s2b = np.zeros(3)

        self._theta      = 0.0
        self._last_pose  = None   # PoseStamped del último fit válido

        self._pub_pose   = self.create_publisher(PoseStamped,  '/robot_in_tube',          10)
        self._pub_wall   = self.create_publisher(PointCloud2,  '/cylinder_fit/wall_points', 1)
        self._pub_stats  = self.create_publisher(String,       '/cylinder_fit/stats',      10)

        self.create_subscription(Float64,     '/turner/angle',    self._angle_cb,  10)
        self.create_subscription(PointCloud2, '/velodyne_points', self._cloud_cb,  5)

        self.get_logger().info(
            f'CylinderLocalizer listo — R_nom={self._R_nom}m, margen=±{self._margin}m'
        )

    def _angle_cb(self, msg: Float64):
        self._theta = msg.data

    def _cloud_cb(self, msg: PointCloud2):
        # Obtener TF sensor→base_link (estático, una sola vez)
        if self._R_s2b is None:
            try:
                tf = self._tf_buffer.lookup_transform(
                    'base_link', msg.header.frame_id,
                    RCLTime(), timeout=Duration(seconds=0.2))
                t = tf.transform.translation
                self._R_s2b = _quat_to_rot(tf.transform.rotation)
                self._t_s2b = np.array([t.x, t.y, t.z])
                self.get_logger().info('TF sensor→base_link obtenido')
            except Exception as e:
                self.get_logger().warn(f'TF no disponible: {e}', throttle_duration_sec=3.0)
                return

        # Leer puntos — filtrar por ring si disponible (solo haces horizontales)
        fields = {f.name for f in msg.fields}
        if 'ring' in fields:
            pts_raw = [(p[0], p[1], p[2]) for p in
                       pc2.read_points(msg, field_names=('x', 'y', 'z', 'ring'), skip_nans=True)
                       if int(p[3]) in self.RINGS_HOR]
        else:
            pts_raw = list(pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))
        if not pts_raw:
            return
        arr = np.array([[p[0], p[1], p[2]] for p in pts_raw], dtype=np.float64)

        # Transformar sensor → base_link
        arr = arr @ self._R_s2b.T + self._t_s2b

        # Sección transversal: XZ del robot (eje Y = eje axial tubo en base_link)
        # Filtrar puntos de pared: radio en XZ cercano al radio nominal
        r_xz = np.sqrt(arr[:, 0]**2 + arr[:, 2]**2)
        wall_mask = (r_xz > self._R_nom - self._margin) & \
                    (r_xz < self._R_nom + self._margin)
        wall_pts = arr[wall_mask]

        if len(wall_pts) < self._min_pts:
            self.get_logger().warn(
                f'Pocos puntos de pared: {len(wall_pts)} < {self._min_pts}',
                throttle_duration_sec=2.0)
            return

        # Ajuste de círculo en XZ
        fit = fit_circle_2d(wall_pts[:, [0, 2]])
        if fit is None:
            return
        cx, cz, r_fit, rms = fit

        # Validar fit: radio debe ser razonable
        if abs(r_fit - self._R_nom) > 1.0:
            self.get_logger().warn(
                f'Fit dudoso: r={r_fit:.2f}m (nominal={self._R_nom}m), rms={rms:.3f}m',
                throttle_duration_sec=2.0)
            return

        # Posición del robot en el tubo:
        # El centro del círculo ajustado da el offset del robot respecto al eje del tubo
        # (cx, cz) = vector desde robot hasta eje tubo en plano XZ
        phi = math.atan2(-cz, -cx)      # ángulo angular del robot en tubo
        r_offset = math.sqrt(cx**2 + cz**2)  # distancia robot-eje (debería ser ~0 si centrado)
        x_axial = float(np.mean(wall_pts[:, 1]))  # posición axial ≈ media Y en base_link

        # Publicar pose en tubo
        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = 'tube_canonical'
        pose.pose.position.x = x_axial
        pose.pose.position.y = phi
        pose.pose.position.z = r_offset
        pose.pose.orientation.w = 1.0
        self._pub_pose.publish(pose)
        self._last_pose = pose

        # Publicar puntos de pared usados (para RViz)
        hdr = Header()
        hdr.stamp = msg.header.stamp
        hdr.frame_id = 'base_link'
        wall_cloud = pc2.create_cloud_xyz32(hdr, wall_pts.tolist())
        self._pub_wall.publish(wall_cloud)

        # Stats
        stats = (
            f'fit_r={r_fit:.3f}m rms={rms:.4f}m | '
            f'cx={cx:.3f} cz={cz:.3f} | '
            f'x_axial={x_axial:.3f}m φ={math.degrees(phi):.1f}° | '
            f'θ_virador={math.degrees(self._theta):.1f}° | '
            f'wall_pts={len(wall_pts)}'
        )
        self._pub_stats.publish(String(data=stats))
        self.get_logger().info(stats, throttle_duration_sec=3.0)


def main(args=None):
    rclpy.init(args=args)
    node = CylinderLocalizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
