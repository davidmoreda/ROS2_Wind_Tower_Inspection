"""
Filtro de obstáculos consciente de pendiente (ramp-aware ground removal).

PROBLEMA QUE RESUELVE
---------------------
El obstacle_layer de Nav2 filtra por altura (min_obstacle_height) en un frame
alineado con la gravedad. Una rampa inclinada sube de forma continua, así que su
superficie supera cualquier umbral fijo más allá de cierta distancia y se marca
como obstáculo letal → el planner aborta. Bajar el umbral, en cambio, deja de
detectar objetos bajos (palés, cajas) y mete ruido del suelo.

SOLUCIÓN
--------
En vez de un umbral de altura absoluto, estimamos el SUELO LOCAL por celdas y
marcamos como obstáculo solo lo que SOBRESALE de su propio suelo. Así una
pendiente suave (rampa) queda como suelo, mientras que un escalón/objeto/pared
sobresale y sí se marca:

  1. Proyectamos la nube a una rejilla XY (en el frame del sensor) de paso
     `cell_size`.
  2. En cada celda, suelo = z mínima de la celda.
  3. Un punto es obstáculo si  (z - suelo_celda) > ground_threshold.

POR QUÉ TOLERA LA RAMPA
-----------------------
Dentro de una celda de tamaño C, una pendiente de ángulo α sube como mucho
C·tan(α). Si C·tan(α) < ground_threshold, la rampa NUNCA se marca. La pendiente
máxima tolerada es  atan(ground_threshold / cell_size).
  Con cell_size=0.20 y ground_threshold=0.12  →  rampa hasta atan(0.6)=31°.
  La rampa real (13°) sube solo 0.20·tan13°=0.046 m por celda → muy por debajo.

POR QUÉ SIGUE VIENDO OBJETOS BAJOS Y PAREDES
--------------------------------------------
Un objeto de 0.20 m sobre suelo plano sobresale 0.20 m > 0.12 m → se marca.
Una pared/objeto alto tiene gran extensión vertical → se marca. Funciona en 360°
y a cualquier distancia porque el umbral es RELATIVO al suelo de cada celda
(no importa la altura absoluta del terreno).

Entrada : /velodyne_points (PointCloud2, frame del LiDAR)
Salida  : /obstacle_points  (PointCloud2, mismo frame) → obstacle_layer del costmap
"""

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Header

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2

import tf2_ros


class ObstacleCloudFilter(Node):

    def __init__(self):
        super().__init__('obstacle_cloud_filter')

        # ── Parámetros ──────────────────────────────────────────────────────
        self.declare_parameter('input_topic', '/velodyne_points')
        self.declare_parameter('output_topic', '/obstacle_points')
        # Tamaño de celda de la rejilla XY (m). Más pequeño = más tolerante a
        # pendiente pero más sensible al ruido. 0.20 da margen hasta ~31°.
        self.declare_parameter('cell_size', 0.20)
        # Altura que un punto debe sobresalir del suelo de su celda para contar
        # como obstáculo (m). Debe ser > subida-por-celda de la rampa (0.046 m)
        # y < altura del objeto bajo más pequeño que quieras detectar.
        self.declare_parameter('ground_threshold', 0.12)
        # Recorte de rango horizontal (m) para no procesar ruido lejano.
        self.declare_parameter('min_range', 0.4)
        self.declare_parameter('max_range', 10.0)
        # Techo: ignora puntos demasiado altos sobre el suelo de la celda
        # (estructura del techo de la nave) para no marcarlos como obstáculo.
        self.declare_parameter('max_obstacle_height', 2.0)

        # ── Zona de exclusión: recinto del tubo ─────────────────────────────
        # Los defectos sintéticos son ~80 esferas pequeñas (r≈2-12 cm) pegadas a
        # la superficie del tubo, dentro de las barreras (x=±5). El robot NUNCA
        # entra ahí, pero el gpu_lidar SÍ ve esas esferas (ve <visual>, no
        # colisión) y las marca como obstáculo/ruido de localización. Aquí se
        # descartan TODOS los puntos cuyo XY (en el frame fijo `exclude_frame`)
        # caiga dentro del rectángulo del recinto. Las barreras (x=±5) quedan
        # FUERA del rectángulo → se conservan como límite real de navegación.
        self.declare_parameter('exclude_zone_enabled', True)
        self.declare_parameter('exclude_frame', 'map')
        self.declare_parameter('exclude_xmin', -4.6)
        self.declare_parameter('exclude_xmax',  4.6)
        self.declare_parameter('exclude_ymin', -14.0)
        self.declare_parameter('exclude_ymax',  14.0)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.cell_size = float(self.get_parameter('cell_size').value)
        self.ground_threshold = float(self.get_parameter('ground_threshold').value)
        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.max_obstacle_height = float(self.get_parameter('max_obstacle_height').value)

        self.exclude_zone_enabled = bool(self.get_parameter('exclude_zone_enabled').value)
        self.exclude_frame = self.get_parameter('exclude_frame').value
        self.exclude_xmin = float(self.get_parameter('exclude_xmin').value)
        self.exclude_xmax = float(self.get_parameter('exclude_xmax').value)
        self.exclude_ymin = float(self.get_parameter('exclude_ymin').value)
        self.exclude_ymax = float(self.get_parameter('exclude_ymax').value)

        # Buffer TF para transformar la nube (frame del sensor) → frame fijo y
        # decidir qué puntos caen en el recinto del tubo.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.max_slope_deg = np.degrees(np.arctan2(self.ground_threshold, self.cell_size))

        # QoS sensor (BEST_EFFORT) para casar con el bridge de Gazebo y con el
        # consumidor del costmap (igual que la nube cruda /velodyne_points).
        self.pub = self.create_publisher(
            PointCloud2, self.output_topic, qos_profile_sensor_data)
        self.sub = self.create_subscription(
            PointCloud2, self.input_topic, self._cb, qos_profile_sensor_data)

        # Watchdog: si /velodyne_points deja de llegar, publica nube vacía para
        # que el costmap haga raytrace clearing y no quede bloqueado con marcas antiguas.
        self._last_input_time = self.get_clock().now()
        self._last_header: Header | None = None
        self._watchdog_timeout = 0.5   # segundos sin datos → publicar vacío
        self.create_timer(0.2, self._watchdog_cb)

        self.get_logger().info(
            f'obstacle_cloud_filter: {self.input_topic} → {self.output_topic} | '
            f'cell={self.cell_size} m, umbral={self.ground_threshold} m, '
            f'pendiente máx tolerada≈{self.max_slope_deg:.0f}°')

    def _watchdog_cb(self):
        elapsed = (self.get_clock().now() - self._last_input_time).nanoseconds * 1e-9
        if elapsed > self._watchdog_timeout and self._last_header is not None:
            # Publica nube vacía: el costmap ejecuta raytrace sin marcar nada
            # → limpia marcas antiguas que bloquearían la navegación.
            self._publish_empty(self._last_header)

    def _cb(self, msg: PointCloud2):
        self._last_input_time = self.get_clock().now()
        self._last_header = msg.header

        try:
            pts = pc2.read_points_numpy(
                msg, field_names=('x', 'y', 'z'), skip_nans=True)
        except Exception as e:
            self.get_logger().warn(f'read_points_numpy falló: {e}', throttle_duration_sec=5.0)
            self._publish_empty(msg.header)
            return

        # read_points_numpy puede devolver array estructurado (N,) o 2D (N,3)
        if pts.dtype.names:
            xyz = np.column_stack([pts['x'], pts['y'], pts['z']]).astype(np.float32)
        else:
            xyz = np.asarray(pts, dtype=np.float32)
            if xyz.ndim == 1:
                xyz = xyz.reshape(-1, 3)

        if xyz.shape[0] == 0:
            self._publish_empty(msg.header)
            return

        x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]

        # Recorte por rango horizontal.
        rng = np.hypot(x, y)
        keep = (rng >= self.min_range) & (rng <= self.max_range)
        if not np.any(keep):
            self._publish_empty(msg.header)
            return
        x, y, z, xyz = x[keep], y[keep], z[keep], xyz[keep]

        # Índice de celda XY. Offset grande para evitar negativos.
        OFF = 10000
        ix = np.floor(x / self.cell_size).astype(np.int64) + OFF
        iy = np.floor(y / self.cell_size).astype(np.int64) + OFF
        cell_id = ix * 100000 + iy  # clave única por celda

        # Suelo por celda = z mínima de la celda. Vectorizado con np.minimum.at.
        uniq, inv = np.unique(cell_id, return_inverse=True)
        ground = np.full(uniq.shape[0], np.inf, dtype=np.float64)
        np.minimum.at(ground, inv, z)

        height_above_ground = z - ground[inv]

        # Obstáculo: sobresale del suelo de su celda pero por debajo del techo.
        is_obstacle = (
            (height_above_ground > self.ground_threshold)
            & (height_above_ground <= self.max_obstacle_height)
        )

        obstacle_xyz = xyz[is_obstacle]

        # Descartar los puntos del recinto del tubo (defectos + superficie del
        # tubo). Se aplica solo a los obstáculos → barato.
        if self.exclude_zone_enabled and obstacle_xyz.shape[0] > 0:
            obstacle_xyz = self._drop_tube_zone(obstacle_xyz, msg.header)

        self._publish(msg.header, obstacle_xyz)

    def _drop_tube_zone(self, xyz: np.ndarray, header: Header) -> np.ndarray:
        """
        Elimina los puntos cuyo XY (en `exclude_frame`) caiga dentro del
        rectángulo del recinto del tubo. Si la TF no está disponible todavía
        (p.ej. antes de que AMCL publique map→odom), devuelve la nube intacta
        para no perder obstáculos reales por accidente.
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                self.exclude_frame, header.frame_id, header.stamp,
                timeout=Duration(seconds=0.05))
        except Exception as e:
            self.get_logger().warn(
                f'TF {self.exclude_frame}<-{header.frame_id} no disponible, '
                f'sin exclusión del tubo este ciclo: {e}',
                throttle_duration_sec=5.0)
            return xyz

        t = tf.transform.translation
        q = tf.transform.rotation
        # Cuaternión → matriz de rotación 3x3.
        xx, yy, zz, ww = q.x, q.y, q.z, q.w
        R = np.array([
            [1 - 2 * (yy * yy + zz * zz), 2 * (xx * yy - zz * ww),     2 * (xx * zz + yy * ww)],
            [2 * (xx * yy + zz * ww),     1 - 2 * (xx * xx + zz * zz), 2 * (yy * zz - xx * ww)],
            [2 * (xx * zz - yy * ww),     2 * (yy * zz + xx * ww),     1 - 2 * (xx * xx + yy * yy)],
        ], dtype=np.float32)

        # Puntos en el frame fijo (solo necesitamos XY).
        world = xyz @ R.T
        wx = world[:, 0] + t.x
        wy = world[:, 1] + t.y

        in_tube = (
            (wx >= self.exclude_xmin) & (wx <= self.exclude_xmax)
            & (wy >= self.exclude_ymin) & (wy <= self.exclude_ymax)
        )
        return xyz[~in_tube]

    def _publish(self, header, xyz: np.ndarray):
        # Empaqueta directamente desde numpy sin pasar por tolist() — 10× más rápido.
        arr = np.ascontiguousarray(xyz, dtype=np.float32)
        out = PointCloud2()
        out.header = header
        out.height = 1
        out.width = arr.shape[0]
        out.fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        out.is_bigendian = False
        out.point_step = 12
        out.row_step = 12 * arr.shape[0]
        out.is_dense = True
        out.data = arr.tobytes()
        self.pub.publish(out)

    def _publish_empty(self, header):
        out = PointCloud2()
        out.header = header
        out.height = 1
        out.width = 0
        out.fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        out.is_bigendian = False
        out.point_step = 12
        out.row_step = 0
        out.is_dense = True
        out.data = b''
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleCloudFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
