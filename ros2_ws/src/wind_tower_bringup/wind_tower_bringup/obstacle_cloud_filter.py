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
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2


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

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.cell_size = float(self.get_parameter('cell_size').value)
        self.ground_threshold = float(self.get_parameter('ground_threshold').value)
        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.max_obstacle_height = float(self.get_parameter('max_obstacle_height').value)

        self.max_slope_deg = np.degrees(np.arctan2(self.ground_threshold, self.cell_size))

        # QoS sensor (BEST_EFFORT) para casar con el bridge de Gazebo y con el
        # consumidor del costmap (igual que la nube cruda /velodyne_points).
        self.pub = self.create_publisher(
            PointCloud2, self.output_topic, qos_profile_sensor_data)
        self.sub = self.create_subscription(
            PointCloud2, self.input_topic, self._cb, qos_profile_sensor_data)

        self.get_logger().info(
            f'obstacle_cloud_filter: {self.input_topic} → {self.output_topic} | '
            f'cell={self.cell_size} m, umbral={self.ground_threshold} m, '
            f'pendiente máx tolerada≈{self.max_slope_deg:.0f}°')

    def _cb(self, msg: PointCloud2):
        # Nx3 (x, y, z), descartando NaN/inf.
        pts = pc2.read_points_numpy(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if pts.shape[0] == 0:
            self._publish(msg.header, pts)
            return

        x = pts[:, 0]
        y = pts[:, 1]
        z = pts[:, 2]

        # Recorte por rango horizontal.
        rng = np.hypot(x, y)
        keep = (rng >= self.min_range) & (rng <= self.max_range)
        if not np.any(keep):
            self._publish(msg.header, pts[:0])
            return
        x, y, z, pts = x[keep], y[keep], z[keep], pts[keep]

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

        self._publish(msg.header, pts[is_obstacle])

    def _publish(self, header, points_xyz):
        out = pc2.create_cloud_xyz32(header, points_xyz.tolist())
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
