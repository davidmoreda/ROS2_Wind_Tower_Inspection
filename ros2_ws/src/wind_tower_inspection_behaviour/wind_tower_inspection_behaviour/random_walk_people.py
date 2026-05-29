#!/usr/bin/env python3
"""
random_walk_people — nodo ROS 2 que mueve trabajadores y controla la carretilla
en el mundo Gazebo de forma realista por la planta industrial.

Estrategia de movimiento (doble canal, con fallback robusto):
  1. Primero intenta usar el servicio ROS SetEntityPose
     (/world/wind_tower_world/set_pose) expuesto por ros_gz_bridge.
  2. Si el servicio no está disponible o falla, cae automáticamente a
     gz service via subprocess (método más fiable en Gazebo Sim 8 / WSL2):
       gz service -s /world/wind_tower_world/set_pose \
         --reqtype gz.msgs.Pose \
         --reptype gz.msgs.Boolean \
         --timeout 200 \
         --req 'name: "<model>" position { x: X y: Y z: 0 } ...'

MOVIMIENTO SEGURO — sin atravesar paredes ni el tubo:
  Los trabajadores se quedan dentro de zonas conectadas reales del mapa.
  Antes de aceptar cada waypoint se comprueba que el segmento de trayectoria
  no cruce ningún obstáculo rectangular, barrera, rodillo, rampa ni zona del tubo.
  La carretilla patrulla por el pasillo este libre (x≈16) y no entra al recinto
  central del tubo.

Distribución de zonas de trabajo:
  Pasillo OESTE:   x in [-18, -5.8], y in [-19, 13.2]
  Pasillo ESTE:    x in [ 5.8, 14.2], y in [-19, 13.2]
  Franja sur:      x in [-3.5, 3.5],  y in [-19, -16.8]
  Pasillo carretilla: x=16, y in [-13, 13]

Parámetros ROS 2:
  world_name       (string)  Nombre del mundo Gazebo.  Default: wind_tower_world
  worker_names     (string)  Nombres separados por coma. Default: worker_1,...,4
  speed            (float)   Velocidad de caminata [m/s].  Default: 0.8
  tick_rate        (float)   Hz de actualización de poses.  Default: 10.0
  waypoint_timeout (float)   Segundos máx para alcanzar un waypoint. Default: 30.0
  use_gz_service   (bool)    Usar gz service subprocess en vez de ROS srv. Default: false
"""

import math
import random
import subprocess
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Pose, Point, Quaternion
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose


# ---------------------------------------------------------------------------
# OBSTÁCULOS — rectángulos que los trabajadores NO deben cruzar.
# Cada entrada es (x_min, x_max, y_min, y_max).
# Se usan para validar que el segmento current→goal no cruce ningún obstáculo.
# Se añade un margen de 0.5 m a todos los lados para seguridad del robot.
# ---------------------------------------------------------------------------
_MARGIN = 0.6  # metros de margen de seguridad alrededor de cada obstáculo

OBSTACLE_RECTS: List[Tuple[float, float, float, float]] = [
    # Paredes exteriores (X:[-20,+20], Y:[-30,+30])
    (-20.4, 20.4, 29.6, 30.4),    # pared norte
    (-20.4, 20.4, -30.4, -29.6),  # pared sur
    (19.6, 20.4, -30.4, 30.4),    # pared este
    (-20.4, -19.6, -30.4, 30.4),  # pared oeste

    # Separador norte Y=+20 (con puertas en X=0 y X=±12, ancho 3 m)
    # Bloqueamos los segmentos sólidos: X=[-20,-13.5], X=[-10.5,-1.5], X=[+1.5,+10.5], X=[+13.5,+20]
    (-20.0 - _MARGIN, -13.5 + _MARGIN, 19.7 - _MARGIN, 20.3 + _MARGIN),
    (-10.5 - _MARGIN, -1.5 + _MARGIN,  19.7 - _MARGIN, 20.3 + _MARGIN),
    ( 1.5 - _MARGIN,  10.5 + _MARGIN,  19.7 - _MARGIN, 20.3 + _MARGIN),
    (13.5 - _MARGIN,  20.0 + _MARGIN,  19.7 - _MARGIN, 20.3 + _MARGIN),

    # Separador sur Y=-20 (simétrico)
    (-20.0 - _MARGIN, -13.5 + _MARGIN, -20.3 - _MARGIN, -19.7 + _MARGIN),
    (-10.5 - _MARGIN, -1.5 + _MARGIN,  -20.3 - _MARGIN, -19.7 + _MARGIN),
    ( 1.5 - _MARGIN,  10.5 + _MARGIN,  -20.3 - _MARGIN, -19.7 + _MARGIN),
    (13.5 - _MARGIN,  20.0 + _MARGIN,  -20.3 - _MARGIN, -19.7 + _MARGIN),

    # Recinto central del tubo, rampas y barreras naranjas. Las rutas de
    # personas/carretilla no deben entrar aquí; el Husky sí usa esta zona.
    (-5.1 - _MARGIN, 5.1 + _MARGIN, -16.1 - _MARGIN, 20.1 + _MARGIN),

    # Tubo y apoyos/rodillos: mantener despejado todo el volumen central.
    (-4.5 - _MARGIN, 4.5 + _MARGIN, -11.0 - _MARGIN, 16.0 + _MARGIN),

    # Columnas principales.
    ( 7.7 - _MARGIN,  8.3 + _MARGIN,  7.7 - _MARGIN,  8.3 + _MARGIN),
    (-8.3 - _MARGIN, -7.7 + _MARGIN,  7.7 - _MARGIN,  8.3 + _MARGIN),
    ( 7.7 - _MARGIN,  8.3 + _MARGIN, -8.3 - _MARGIN, -7.7 + _MARGIN),
    (-8.3 - _MARGIN, -7.7 + _MARGIN, -8.3 - _MARGIN, -7.7 + _MARGIN),

    # Estanterías metálicas lado oeste (X≈-12, en Y=-10, -4, +8)
    (-13.0, -11.0, -11.0, -9.0),
    (-13.0, -11.0,  -5.0, -3.0),
    (-13.0, -11.0,   7.0,  9.0),

    # Estanterías metálicas lado este (X≈+12)
    (11.0, 13.0, -11.0,  -9.0),
    (11.0, 13.0,  -5.0,  -3.0),
    (11.0, 13.0,   7.0,   9.0),

    # Pallets lado oeste (X≈-10.5)
    (-11.5,  -9.5, -14.0, -12.0),
    (-11.5,  -9.5, -11.0,  -9.0),
    (-11.5,  -9.5,   7.5,   8.5),
    (-11.5,  -9.5,   9.5,  10.5),

    # Pallets lado este (X≈+10.5)
    ( 9.5,  11.5, -14.0, -12.0),
    ( 9.5,  11.5, -12.0, -10.0),
    ( 9.5,  11.5,   7.5,   8.5),
    ( 9.5,  11.5,   9.5,  10.5),

    # Bidones lado oeste (X≈-11.5, Y≈-7.15)
    (-12.5, -10.5, -8.0, -6.5),

    # Bidones lado este (X≈+11.5)
    (10.5,  12.5, -8.0, -6.5),
]


# ---------------------------------------------------------------------------
# ZONAS NAVEGABLES — rectángulos donde los trabajadores SÍ pueden ir.
# Cada zona es (x_min, x_max, y_min, y_max).
# Se eligen waypoints dentro de estas zonas; además se verifica que el
# trayecto no cruce ningún obstáculo de OBSTACLE_RECTS.
# ---------------------------------------------------------------------------
Zone = Tuple[float, float, float, float]

NAVIGABLE_ZONES: Dict[str, Zone] = {
    # Cada zona es conexa sin cruzar las barreras del mundo.
    'west': (-18.0, -5.8, -19.0, 13.2),
    'east_workers': (5.8, 14.2, -19.0, 13.2),
    'south': (-3.5, 3.5, -19.0, -16.8),
}

WORKER_ZONE_BY_INDEX = ['west', 'east_workers', 'west', 'east_workers']

FORKLIFT_X = 16.0
FORKLIFT_Y_MIN = -13.0
FORKLIFT_Y_MAX = 13.0

# Mínima separación entre dos trabajadores
MIN_WORKER_SEPARATION = 1.5  # metros

# Mínima distancia a la carretilla (cuando está en movimiento)
MIN_FORKLIFT_DISTANCE = 3.0  # metros

# Huellas de seguridad usadas por el controlador cinemático. Gazebo tiene
# colisiones reales, pero SetEntityPose mueve los modelos de forma cinemática,
# así que el nodo debe rechazar pasos que invadan esas huellas.
WORKER_RADIUS = 0.35
FORKLIFT_HALF_WIDTH = 0.8
FORKLIFT_HALF_LENGTH = 1.8


# ---------------------------------------------------------------------------
# Funciones de geometría
# ---------------------------------------------------------------------------

def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convierte yaw (rad) a geometry_msgs/Quaternion."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def distance_2d(x1: float, y1: float, x2: float, y2: float) -> float:
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def random_point_in_zone(zone_name: str) -> Tuple[float, float]:
    """Elige un punto (x, y) dentro de una zona navegable conectada."""
    z = NAVIGABLE_ZONES[zone_name]
    return random.uniform(z[0], z[1]), random.uniform(z[2], z[3])


def point_in_rect(x: float, y: float, rect: Tuple[float, float, float, float]) -> bool:
    """Comprueba si el punto (x,y) está dentro del rectángulo (x_min,x_max,y_min,y_max)."""
    return rect[0] <= x <= rect[1] and rect[2] <= y <= rect[3]


def segment_intersects_rect(
    x1: float, y1: float,
    x2: float, y2: float,
    rect: Tuple[float, float, float, float]
) -> bool:
    """
    Comprueba si el segmento [(x1,y1)-(x2,y2)] intersecta el rectángulo.
    Usa separating-axis theorem (SAT) para AABB vs segmento.
    """
    rx_min, rx_max, ry_min, ry_max = rect

    # Salida rápida: si ambos puntos están dentro del rectángulo
    if point_in_rect(x1, y1, rect) or point_in_rect(x2, y2, rect):
        return True

    dx = x2 - x1
    dy = y2 - y1

    # Parametrize t=[0,1] y clipear con cada cara del AABB (Cohen-Sutherland 1D)
    t_min = 0.0
    t_max = 1.0

    def clip(p: float, q: float) -> bool:
        nonlocal t_min, t_max
        if abs(p) < 1e-9:
            return q >= 0.0  # paralelo: comprobar si dentro del slab
        t = q / p
        if p < 0:
            if t > t_max:
                return False
            if t > t_min:
                t_min = t
        else:
            if t < t_min:
                return False
            if t < t_max:
                t_max = t
        return True

    if not clip(-dx, x1 - rx_min):
        return False
    if not clip( dx, rx_max - x1):
        return False
    if not clip(-dy, y1 - ry_min):
        return False
    if not clip( dy, ry_max - y1):
        return False
    return t_min <= t_max


def path_is_clear(
    x1: float, y1: float,
    x2: float, y2: float,
    extra_rects: Optional[List[Tuple[float, float, float, float]]] = None
) -> bool:
    """
    Devuelve True si el segmento (x1,y1)→(x2,y2) no cruza ningún obstáculo.
    extra_rects se añade a OBSTACLE_RECTS (p.ej. posición actual de la carretilla).
    """
    rects = OBSTACLE_RECTS
    if extra_rects:
        rects = OBSTACLE_RECTS + extra_rects
    for rect in rects:
        if segment_intersects_rect(x1, y1, x2, y2, rect):
            return False
    return True


def point_is_free(
    x: float, y: float,
    extra_rects: Optional[List[Tuple[float, float, float, float]]] = None
) -> bool:
    """Devuelve True si el punto (x,y) no está dentro de ningún obstáculo."""
    rects = OBSTACLE_RECTS
    if extra_rects:
        rects = OBSTACLE_RECTS + extra_rects
    for rect in rects:
        if point_in_rect(x, y, rect):
            return False
    return True


# ---------------------------------------------------------------------------
# Estado por trabajador
# ---------------------------------------------------------------------------

class WorkerState:
    """Máquina de estados por trabajador."""

    def __init__(self, name: str, init_x: float, init_y: float, zone_name: str):
        self.name = name
        self.zone_name = zone_name
        self.x: float = init_x
        self.y: float = init_y
        self.yaw: float = 0.0
        self.goal_x: float = init_x
        self.goal_y: float = init_y
        self.goal_timestamp: float = time.monotonic()
        self.arrived: bool = True  # empezar seleccionando nuevo waypoint

        # Registro de cuántas veces se rechazó un waypoint (debug)
        self.rejected_waypoints: int = 0


# ---------------------------------------------------------------------------
# Nodo ROS 2
# ---------------------------------------------------------------------------

class RandomWalkPeople(Node):

    def __init__(self) -> None:
        super().__init__('random_walk_people')

        # Parámetros
        self.declare_parameter('world_name', 'wind_tower_world')
        self.declare_parameter('worker_names', 'worker_1,worker_2,worker_3,worker_4')
        self.declare_parameter('speed', 0.8)
        self.declare_parameter('tick_rate', 10.0)
        self.declare_parameter('waypoint_timeout', 30.0)
        self.declare_parameter('use_gz_service', False)
        # Parámetro: mover también la carretilla vía este nodo (redundante si ya tiene
        # TrajectoryFollower en el SDF, pero útil si se quiere control ROS)
        self.declare_parameter('control_forklift', False)

        self._world_name: str = self.get_parameter('world_name').value
        names_str: str = self.get_parameter('worker_names').value
        self.speed: float = self.get_parameter('speed').value
        tick_rate: float = self.get_parameter('tick_rate').value
        self.waypoint_timeout: float = self.get_parameter('waypoint_timeout').value
        self._use_gz_service: bool = self.get_parameter('use_gz_service').value
        self._control_forklift: bool = self.get_parameter('control_forklift').value

        self.worker_names: List[str] = [n.strip() for n in names_str.split(',') if n.strip()]
        self.dt: float = 1.0 / tick_rate

        # Posiciones iniciales en los pasillos laterales (deben coincidir con el SDF)
        initial_positions = [
            (-10.0, -15.0),   # worker_1 — pasillo oeste, zona sur
            ( 10.0,  12.0),   # worker_2 — pasillo este, zona norte
            (-12.0,   5.0),   # worker_3 — pasillo oeste, zona centro
            ( 12.0,  -8.0),   # worker_4 — pasillo este, zona sur
        ]

        self.workers: Dict[str, WorkerState] = {}
        for i, name in enumerate(self.worker_names):
            ix, iy = initial_positions[i] if i < len(initial_positions) else (0.0, 0.0)
            zone_name = WORKER_ZONE_BY_INDEX[i] if i < len(WORKER_ZONE_BY_INDEX) else 'west'
            self.workers[name] = WorkerState(name, ix, iy, zone_name)

        # Estado de la carretilla (posición para evitarla)
        # La carretilla sigue su TrajectoryFollower en el SDF por el pasillo este;
        # aquí la modelamos analíticamente para que los trabajadores la eviten.
        self._forklift_x: float = FORKLIFT_X
        self._forklift_y: float = FORKLIFT_Y_MIN
        self._forklift_dir: float = 1.0  # +1 → norte, -1 → sur
        self._forklift_speed: float = (
            (FORKLIFT_Y_MAX - FORKLIFT_Y_MIN) / 52.0
        )  # 26 m en 52 s ≈ 0.5 m/s (igual que TrajectoryFollower)

        # Cliente de servicio — callback group separado para evitar deadlock (AI pitfall #1)
        self._srv_cbg = MutuallyExclusiveCallbackGroup()
        self._set_pose_client = self.create_client(
            SetEntityPose,
            f'/world/{self._world_name}/set_pose',
            callback_group=self._srv_cbg,
        )

        self._ros_service_ok: bool = False
        self._ros_failures: int = 0
        self._ROS_FAILURE_THRESHOLD = 5

        # Timer principal de movimiento
        self._timer_cbg = MutuallyExclusiveCallbackGroup()
        self._timer = self.create_timer(
            self.dt,
            self._tick,
            callback_group=self._timer_cbg,
        )

        mode = 'gz-service subprocess' if self._use_gz_service else 'ROS SetEntityPose (fallback gz-service)'
        self.get_logger().info(
            f'random_walk_people started: {self.worker_names}, '
            f'speed={self.speed} m/s, tick={tick_rate} Hz, mode={mode}, '
            f'obstacle-aware navigation ACTIVE ({len(OBSTACLE_RECTS)} rects)'
        )

    # -------------------------------------------------------------------------
    # Tick principal — llamado a tick_rate Hz
    # -------------------------------------------------------------------------

    def _tick(self) -> None:
        """Mueve cada trabajador un paso hacia su objetivo."""
        # Verificar disponibilidad del servicio ROS (no bloqueante)
        if not self._use_gz_service and not self._ros_service_ok:
            if self._set_pose_client.service_is_ready():
                self._ros_service_ok = True
                self.get_logger().info(
                    '/world/*/set_pose ROS service ready — using ROS channel'
                )
            else:
                if not self._gz_service_available():
                    return  # Esperando a que Gazebo arranque
                self.get_logger().info(
                    '/world/*/set_pose ROS service NOT ready, '
                    'falling back to gz service subprocess',
                    throttle_duration_sec=10.0,
                )
                self._use_gz_service = True

        if (self._ros_failures >= self._ROS_FAILURE_THRESHOLD
                and not self._use_gz_service):
            self.get_logger().warn(
                f'{self._ros_failures} ROS SetEntityPose failures — '
                'switching permanently to gz service subprocess'
            )
            self._use_gz_service = True

        # Avanzar posición estimada de la carretilla (modelo analítico)
        self._update_forklift_estimate()

        # Rectángulo dinámico de la carretilla (1.2×2.4m + margen)
        fk_rect = self._forklift_rect()

        now = time.monotonic()

        for name, w in self.workers.items():
            dynamic_obstacles = [fk_rect] + self._other_worker_rects(name)
            timed_out = (now - w.goal_timestamp) > self.waypoint_timeout
            dist_to_goal = distance_2d(w.x, w.y, w.goal_x, w.goal_y)

            if w.arrived or timed_out or dist_to_goal < 0.5:
                self._assign_new_waypoint(w, extra_obstacles=dynamic_obstacles)

            # Avanzar posición hacia el objetivo
            dx = w.goal_x - w.x
            dy = w.goal_y - w.y
            d = math.sqrt(dx * dx + dy * dy)

            if d < 0.01:
                w.arrived = True
                continue

            # Calcular siguiente posición
            step = min(self.speed * self.dt, d)
            nx = w.x + (dx / d) * step
            ny = w.y + (dy / d) * step

            # Verificar que el paso completo no entra en obstáculos estáticos
            # ni en huellas dinámicas de otros trabajadores/carretilla.
            if (not point_is_free(nx, ny, extra_rects=dynamic_obstacles)
                    or not path_is_clear(w.x, w.y, nx, ny, extra_rects=dynamic_obstacles)
                    or not self._keeps_worker_separation(name, nx, ny)):
                self._assign_new_waypoint(w, extra_obstacles=dynamic_obstacles)
                continue

            w.x = nx
            w.y = ny
            w.yaw = math.atan2(dy, dx)
            w.arrived = False

            self._send_pose(name, w.x, w.y, w.yaw)

    def _update_forklift_estimate(self) -> None:
        """Avanza el modelo analítico de la posición de la carretilla."""
        self._forklift_y += self._forklift_dir * self._forklift_speed * self.dt
        if self._forklift_y >= FORKLIFT_Y_MAX:
            self._forklift_y = FORKLIFT_Y_MAX
            self._forklift_dir = -1.0
        elif self._forklift_y <= FORKLIFT_Y_MIN:
            self._forklift_y = FORKLIFT_Y_MIN
            self._forklift_dir = 1.0

    def _forklift_rect(self) -> Tuple[float, float, float, float]:
        """
        Rectángulo de exclusión dinámico de la carretilla con margen humano.
        La carretilla real mide aprox. 1.2×2.4 m, pero el controlador usa una
        huella ampliada para que los trabajadores no esperen al contacto físico.
        """
        margin = MIN_FORKLIFT_DISTANCE
        fx = self._forklift_x
        fy = self._forklift_y
        return (
            fx - FORKLIFT_HALF_WIDTH - margin,
            fx + FORKLIFT_HALF_WIDTH + margin,
            fy - FORKLIFT_HALF_LENGTH - margin,
            fy + FORKLIFT_HALF_LENGTH + margin,
        )

    def _worker_rect(self, x: float, y: float) -> Tuple[float, float, float, float]:
        """Huella AABB conservadora de un trabajador."""
        return (
            x - WORKER_RADIUS,
            x + WORKER_RADIUS,
            y - WORKER_RADIUS,
            y + WORKER_RADIUS,
        )

    def _other_worker_rects(self, name: str) -> List[Tuple[float, float, float, float]]:
        """Huellas dinámicas de todos los trabajadores salvo `name`."""
        rects: List[Tuple[float, float, float, float]] = []
        for other_name, other in self.workers.items():
            if other_name == name:
                continue
            rects.append(self._worker_rect(other.x, other.y))
        return rects

    def _keeps_worker_separation(self, name: str, x: float, y: float) -> bool:
        """Comprueba separación circular entre trabajadores para el siguiente paso."""
        for other_name, other in self.workers.items():
            if other_name == name:
                continue
            if distance_2d(x, y, other.x, other.y) < MIN_WORKER_SEPARATION:
                return False
        return True

    # -------------------------------------------------------------------------
    # Asignación de waypoints con comprobación de obstáculos
    # -------------------------------------------------------------------------

    def _assign_new_waypoint(
        self,
        w: WorkerState,
        extra_obstacles: Optional[List[Tuple[float, float, float, float]]] = None,
    ) -> None:
        """
        Elige un waypoint aleatorio en las zonas navegables tal que:
        1. El punto destino no está dentro de ningún obstáculo.
        2. El segmento desde la posición actual hasta el destino no cruza ningún obstáculo.
        3. El destino está suficientemente lejos de otros trabajadores.
        4. El destino está suficientemente lejos de la carretilla.
        """
        max_attempts = 20
        for _ in range(max_attempts):
            gx, gy = random_point_in_zone(w.zone_name)

            # Comprobar punto libre
            if not point_is_free(gx, gy, extra_rects=extra_obstacles):
                w.rejected_waypoints += 1
                continue

            # Comprobar separación respecto a otros trabajadores
            too_close_worker = False
            for other_name, other in self.workers.items():
                if other_name == w.name:
                    continue
                if distance_2d(gx, gy, other.x, other.y) < MIN_WORKER_SEPARATION:
                    too_close_worker = True
                    break
            if too_close_worker:
                continue

            # Comprobar que el trayecto no cruza obstáculos
            if not path_is_clear(w.x, w.y, gx, gy, extra_rects=extra_obstacles):
                w.rejected_waypoints += 1
                # Si el camino directo está bloqueado, buscar un punto intermedio
                # en la misma zona del trabajador
                mid = self._find_intermediate(w.x, w.y, gx, gy, extra_obstacles)
                if mid is not None:
                    gx, gy = mid
                else:
                    continue

            w.goal_x = gx
            w.goal_y = gy
            w.goal_timestamp = time.monotonic()
            w.arrived = False
            self.get_logger().debug(
                f'{w.name} → nuevo waypoint ({gx:.1f}, {gy:.1f})'
            )
            return

        # Fallback: elegir punto cercano en la misma zona (sin comprobación de ruta)
        for _ in range(5):
            gx, gy = random_point_in_zone(w.zone_name)
            if (point_is_free(gx, gy, extra_rects=extra_obstacles)
                    and path_is_clear(w.x, w.y, gx, gy, extra_rects=extra_obstacles)):
                w.goal_x = gx
                w.goal_y = gy
                w.goal_timestamp = time.monotonic()
                w.arrived = False
                self.get_logger().debug(
                    f'{w.name} → waypoint fallback ({gx:.1f}, {gy:.1f}) '
                    f'(rechazados: {w.rejected_waypoints})'
                )
                w.rejected_waypoints = 0
                return

        # Último recurso: quedarse quieto
        w.goal_x = w.x
        w.goal_y = w.y
        w.arrived = True

    def _find_intermediate(
        self,
        x1: float, y1: float,
        x2: float, y2: float,
        extra_rects: Optional[List[Tuple[float, float, float, float]]] = None,
    ) -> Optional[Tuple[float, float]]:
        """
        Busca un punto intermedio en el espacio libre entre (x1,y1) y (x2,y2).
        Prueba puntos en la zona cercana al origen que tengan visibilidad libre.
        """
        for _ in range(10):
            mx = random.uniform(min(x1, x2) - 1.0, max(x1, x2) + 1.0)
            my = random.uniform(min(y1, y2) - 1.0, max(y1, y2) + 1.0)
            if (point_is_free(mx, my, extra_rects=extra_rects)
                    and path_is_clear(x1, y1, mx, my, extra_rects=extra_rects)):
                return mx, my
        return None

    # -------------------------------------------------------------------------
    # Envío de poses — canal ROS service
    # -------------------------------------------------------------------------

    def _send_pose(self, model_name: str, x: float, y: float, yaw: float) -> None:
        """Despacha una actualización de pose por el canal adecuado."""
        if self._use_gz_service:
            self._send_pose_gz(model_name, x, y, yaw)
        else:
            self._send_pose_ros(model_name, x, y, yaw)

    def _send_pose_ros(self, model_name: str, x: float, y: float, yaw: float) -> None:
        """Envía una petición asíncrona SetEntityPose vía ROS service bridge."""
        if not self._set_pose_client.service_is_ready():
            return

        req = SetEntityPose.Request()
        req.entity.name = model_name
        req.entity.type = Entity.MODEL

        pose = Pose()
        pose.position = Point(x=x, y=y, z=0.0)
        pose.orientation = yaw_to_quaternion(yaw)
        req.pose = pose

        future = self._set_pose_client.call_async(req)
        future.add_done_callback(
            lambda f, n=model_name: self._pose_ros_callback(f, n)
        )

    def _pose_ros_callback(self, future, model_name: str) -> None:
        try:
            result = future.result()
            if not result.success:
                self._ros_failures += 1
                self.get_logger().warning(
                    f'SetEntityPose ROS failed for {model_name} '
                    f'(failures: {self._ros_failures})',
                    throttle_duration_sec=5.0,
                )
            else:
                if self._ros_failures > 0:
                    self._ros_failures = max(0, self._ros_failures - 1)
        except Exception as exc:
            self._ros_failures += 1
            self.get_logger().error(
                f'SetEntityPose ROS exception for {model_name}: {exc} '
                f'(failures: {self._ros_failures})',
                throttle_duration_sec=5.0,
            )

    # -------------------------------------------------------------------------
    # Envío de poses — canal gz service subprocess (fallback robusto)
    # -------------------------------------------------------------------------

    def _gz_service_available(self) -> bool:
        """Comprueba si gz reporta el servicio set_pose (no bloqueante)."""
        try:
            result = subprocess.run(
                ['gz', 'service', '--list'],
                capture_output=True,
                text=True,
                timeout=2.0,
            )
            return f'/world/{self._world_name}/set_pose' in result.stdout
        except Exception:
            return False

    def _send_pose_gz(self, model_name: str, x: float, y: float, yaw: float) -> None:
        """
        Mueve un modelo Gazebo llamando gz service en un hilo daemon
        (fire-and-forget — no bloquea el callback del timer ROS).
        """
        import threading

        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        req_str = (
            f'name: "{model_name}" '
            f'position {{ x: {x:.4f} y: {y:.4f} z: 0.0 }} '
            f'orientation {{ x: 0.0 y: 0.0 z: {qz:.6f} w: {qw:.6f} }}'
        )

        cmd = [
            'gz', 'service',
            '-s', f'/world/{self._world_name}/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '200',
            '--req', req_str,
        ]

        def _run():
            try:
                subprocess.run(cmd, capture_output=True, timeout=1.0)
            except Exception as exc:
                self.get_logger().debug(
                    f'gz service pose error for {model_name}: {exc}'
                )

        t = threading.Thread(target=_run, daemon=True)
        t.start()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RandomWalkPeople()
    # MultiThreadedExecutor para que los callbacks de servicio y los timers
    # corran concurrentemente sin bloquearse (AI pitfall #1).
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
