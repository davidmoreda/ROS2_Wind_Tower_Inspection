#!/usr/bin/env python3
"""
people_collision_sync — sincroniza cápsulas de colisión invisibles con los
<actor> animados que caminan por el mundo Gazebo.

PROBLEMA QUE RESUELVE
  Los <actor> de Gazebo Sim mueven las piernas (animación skeletal walk.dae) y
  se desplazan suavemente por <trajectory> nativa, pero son CINEMÁTICOS y NO
  tienen geometría de colisión → el LiDAR VLP-16 (raycast contra <collision>)
  no los detecta y Nav2 no los esquiva.

SOLUCIÓN (híbrida)
  Por cada actor_N existe un modelo invisible person_col_N con cápsula de
  colisión. Este nodo:
    1. Se suscribe a las poses dinámicas de Gazebo
       /world/<world>/dynamic_pose/info  (bridge → tf2_msgs/TFMessage).
       Cada TransformStamped.child_frame_id es el nombre de la entidad (actor_N).
    2. Guarda la última (x, y, yaw) de cada actor.
    3. A update_rate Hz, teletransporta cada person_col_N a la pose de su actor
       vía el servicio SetEntityPose (canal ROS async, SIN subprocesos → sin el
       parpadeo del nodo random_walk_people anterior).

  El desacople suscripción↔timer evita saturar el servicio: la entrada llega a
  ~60 Hz pero solo emitimos a update_rate (15 Hz por defecto).

Parámetros ROS 2:
  world_name      (string)  Mundo Gazebo.            Default: wind_tower_world
  num_people      (int)     Nº de pares actor/cápsula. Default: 12
  actor_prefix    (string)  Prefijo de los actores.   Default: actor_
  capsule_prefix  (string)  Prefijo de las cápsulas.  Default: person_col_
  update_rate     (float)   Hz de actualización de cápsulas. Default: 15.0
  capsule_z       (float)   Altura del modelo cápsula (suelo). Default: 0.0
"""

import math
from typing import Dict, Tuple

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Pose, Point, Quaternion
from tf2_msgs.msg import TFMessage
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Extrae yaw (rotación en Z) de un cuaternión."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class PeopleCollisionSync(Node):

    def __init__(self) -> None:
        super().__init__('people_collision_sync')

        self.declare_parameter('world_name', 'wind_tower_world')
        self.declare_parameter('num_people', 12)
        self.declare_parameter('actor_prefix', 'actor_')
        self.declare_parameter('capsule_prefix', 'person_col_')
        self.declare_parameter('update_rate', 15.0)
        self.declare_parameter('capsule_z', 0.0)

        self._world_name = self.get_parameter('world_name').value
        self._num = int(self.get_parameter('num_people').value)
        self._actor_prefix = self.get_parameter('actor_prefix').value
        self._capsule_prefix = self.get_parameter('capsule_prefix').value
        update_rate = float(self.get_parameter('update_rate').value)
        self._capsule_z = float(self.get_parameter('capsule_z').value)

        # Mapa actor_N → person_col_N
        self._actor_to_capsule: Dict[str, str] = {
            f'{self._actor_prefix}{i}': f'{self._capsule_prefix}{i}'
            for i in range(1, self._num + 1)
        }

        # Última pose conocida de cada actor: nombre → (x, y, yaw)
        self._latest: Dict[str, Tuple[float, float, float]] = {}

        # Cliente del servicio SetEntityPose (callback group separado → sin
        # deadlock con el timer en el MultiThreadedExecutor).
        self._srv_cbg = MutuallyExclusiveCallbackGroup()
        self._set_pose = self.create_client(
            SetEntityPose,
            f'/world/{self._world_name}/set_pose',
            callback_group=self._srv_cbg,
        )

        # Suscripción a poses dinámicas de Gazebo (bridge → TFMessage).
        self._sub = self.create_subscription(
            TFMessage,
            f'/world/{self._world_name}/dynamic_pose/info',
            self._on_dynamic_pose,
            10,
        )

        # Timer de emisión desacoplado de la tasa de entrada.
        self._timer_cbg = MutuallyExclusiveCallbackGroup()
        self._timer = self.create_timer(
            1.0 / update_rate,
            self._tick,
            callback_group=self._timer_cbg,
        )

        self.get_logger().info(
            f'people_collision_sync iniciado: {self._num} pares '
            f'{self._actor_prefix}N→{self._capsule_prefix}N, '
            f'update={update_rate} Hz, '
            f'topic=/world/{self._world_name}/dynamic_pose/info'
        )

    def _on_dynamic_pose(self, msg: TFMessage) -> None:
        """Cachea la pose de cada actor que nos interesa."""
        for tf in msg.transforms:
            name = tf.child_frame_id
            if name not in self._actor_to_capsule:
                continue
            t = tf.transform.translation
            r = tf.transform.rotation
            yaw = yaw_from_quaternion(r.x, r.y, r.z, r.w)
            self._latest[name] = (t.x, t.y, yaw)

    def _tick(self) -> None:
        """Reposiciona cada cápsula sobre su actor a update_rate Hz."""
        if not self._set_pose.service_is_ready():
            self.get_logger().warn(
                f'/world/{self._world_name}/set_pose no disponible aún',
                throttle_duration_sec=10.0,
            )
            return

        for actor_name, (x, y, yaw) in self._latest.items():
            capsule = self._actor_to_capsule[actor_name]
            req = SetEntityPose.Request()
            req.entity.name = capsule
            req.entity.type = Entity.MODEL
            pose = Pose()
            pose.position = Point(x=x, y=y, z=self._capsule_z)
            pose.orientation = yaw_to_quaternion(yaw)
            req.pose = pose
            self._set_pose.call_async(req)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PeopleCollisionSync()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
