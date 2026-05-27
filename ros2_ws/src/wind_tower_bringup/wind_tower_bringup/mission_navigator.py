"""
Nodo de misión: navega a puntos nombrados (estación de carga, mantenimiento, etc.)

Uso desde CLI:
  # Ir a la estación de carga:
  ros2 run wind_tower_bringup mission_navigator --ros-args -p target:=charging_station

  # Ir a mantenimiento:
  ros2 run wind_tower_bringup mission_navigator --ros-args -p target:=maintenance_station

  # Desde Python / otro nodo → servicio /go_to_waypoint (string con nombre del waypoint)

Uso desde servicio ROS2:
  ros2 service call /go_to_waypoint std_srvs/srv/Trigger '{}'
  # (ver parámetro 'target' para cambiar destino en tiempo de ejecución)

Lectura de coordenadas:
  El nodo carga config/waypoints.yaml desde el paquete wind_tower_bringup.
  Edita ese fichero con los valores reales obtenidos del mapa.
"""

import math
from math import sin, cos

import rclpy
from rclpy.node import Node

import yaml
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_srvs.srv import Trigger


class MissionNavigator(Node):
    """Nodo que expone un servicio /go_to_waypoint para navegar a puntos nombrados."""

    def __init__(self):
        super().__init__('mission_navigator')

        # ── Parámetros ────────────────────────────────────────────────────────
        self.declare_parameter('target', 'charging_station')
        self.declare_parameter('waypoints_file', '')   # vacío → usa el del paquete

        # Cargar waypoints
        wp_file = self.get_parameter('waypoints_file').get_parameter_value().string_value
        if not wp_file:
            pkg = get_package_share_directory('wind_tower_bringup')
            wp_file = f'{pkg}/config/waypoints.yaml'

        with open(wp_file, 'r') as f:
            data = yaml.safe_load(f)
        self._waypoints = data.get('waypoints', {})
        self.get_logger().info(
            f'Waypoints cargados: {list(self._waypoints.keys())}'
        )

        # ── Nav2 BasicNavigator ───────────────────────────────────────────────
        self._navigator = BasicNavigator()

        # ── Servicio /go_to_waypoint ──────────────────────────────────────────
        self._srv = self.create_service(
            Trigger,
            'go_to_waypoint',
            self._handle_go_to_waypoint,
        )
        self.get_logger().info(
            'MissionNavigator listo. '
            'Llama al servicio /go_to_waypoint o arranca con -p target:=<nombre>'
        )

        # Si se pasa target por parámetro al arrancar, navega inmediatamente
        target = self.get_parameter('target').get_parameter_value().string_value
        if target:
            self.get_logger().info(f'Target inicial: {target}')
            # Pequeño timer para que el executor arranque antes de navegar
            self._timer = self.create_timer(2.0, lambda: self._navigate_to(target))

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _build_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self._navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = sin(yaw / 2.0)
        pose.pose.orientation.w = cos(yaw / 2.0)
        return pose

    def _navigate_to(self, name: str) -> bool:
        """Navega al waypoint con ese nombre. Devuelve True si llega con éxito."""
        if name not in self._waypoints:
            self.get_logger().error(
                f'Waypoint "{name}" no existe. Disponibles: {list(self._waypoints.keys())}'
            )
            return False

        wp = self._waypoints[name]
        x   = float(wp.get('x', 0.0))
        y   = float(wp.get('y', 0.0))
        yaw = float(wp.get('yaw', 0.0))
        desc = wp.get('description', name)

        self.get_logger().info(f'Navegando a "{name}" ({desc})  → x={x}, y={y}, yaw={yaw:.2f}')

        # Esperar a que Nav2 esté activo
        self._navigator.waitUntilNav2Active()

        pose = self._build_pose(x, y, yaw)
        self._navigator.goToPose(pose)

        # Esperar resultado (bloqueante en este hilo — rclpy.spin corre en otro)
        while not self._navigator.isTaskComplete():
            feedback = self._navigator.getFeedback()
            if feedback:
                dist = feedback.distance_remaining
                self.get_logger().info(f'  Distancia restante: {dist:.2f} m', throttle_duration_sec=2.0)

        result = self._navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f'Llegado a "{name}" correctamente.')
            return True
        else:
            self.get_logger().warn(f'Fallo al navegar a "{name}": {result}')
            return False

    def _handle_go_to_waypoint(self, request, response):
        target = self.get_parameter('target').get_parameter_value().string_value
        success = self._navigate_to(target)
        response.success = success
        response.message = f'Waypoint: {target}, resultado: {"OK" if success else "FALLO"}'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MissionNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
