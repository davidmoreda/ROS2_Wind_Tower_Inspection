#!/usr/bin/env python3
"""
Mission Controller — Wind Tower Inspection
Nodo ROS2 + servidor web Flask (http://localhost:5000).

Funcionalidades:
- Navega a charging, maintenance, home, inspeccion con via-points seguros.
- Simula bateria: se drena al moverse, se carga al llegar a charging_station.
- Publica markers en RViz: waypoints, gates, ruta de inspeccion, indicador bateria.
- Web UI con 4 botones para disparar cada mision.
"""

import math
import subprocess
import sys
import threading
import yaml
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped, TwistStamped
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from sensor_msgs.msg import Image, Imu, PointCloud2
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker, MarkerArray

from flask import (
    Flask,
    Response,
    abort,
    jsonify,
    redirect,
    render_template_string,
    request,
    send_from_directory,
)

from wind_tower_inspection_behaviour.inspection_bt import build_mission_tree
from wind_tower_inspection_behaviour.natural_language_commands import (
    COMMAND_LABELS,
    NaturalLanguageCommandParser,
)

MISSION_COMMANDS = {
    'charging_station',
    'battery_critical',
    'home_inspection',
    'maintenance_station',
    'start_inspection',
}

# ── Parametros ─────────────────────────────────────────────────────────────────
BATTERY_DRAIN  = 0.05   # % / tick (0.5 s) mientras se mueve  → ~0.1%/s
BATTERY_CHARGE = 1.0    # % / tick mientras carga
BATTERY_LOW    = 20.0
BATTERY_CRITICAL = 10.0
BATTERY_FULL   = 100.0

# Gate waypoints: obligatorios para entrar en zona del tubo
GATE_RIGHT = ( 12.015,  5.035)
GATE_LEFT  = (-12.078,  5.698)
POST_GATE  = (  0.245,  8.5  )   # dentro de la zona del tubo, tras cruzar el gate

# Tramo de inspeccion
INSP_CENTER_X = 0.245
INSP_START = (INSP_CENTER_X, 10.246)
INSP_END   = (INSP_CENTER_X, 39.579)
INSP_STEP  = 1.0    # metros entre paradas de inspeccion
INSP_PAUSE_SEC = 1.0
# Inspección del brazo (UR5e) en cada parada:
#  - Publicamos state=INSPECTING → arm_inspection_node arranca sweep 360°
#    (home → 7 waypoints @45° cada uno → home) vía MoveIt.
#  - Esperamos /arm/inspection_ready=True antes de avanzar al siguiente
#    punto. Si en INSP_ARM_TIMEOUT_S segundos no llega (MoveIt no planifica,
#    p.ej.), registramos el fallo y seguimos. Polling cada INSP_ARM_POLL_S.
INSP_ARM_TIMEOUT_S = 15.0   # sweep simple 360° ~8 s con velocity_scaling=0.15
                            # (8 waypoints + retorno a home módulo 2π).
INSP_ARM_POLL_S    = 0.2
INSP_MAX_LATERAL_ERROR = 1.6   # m — tubo 4 m ancho, Husky 0.67 m; margen real para nav entre puntos
INSP_MAX_ROLL_DEG = 18.0       # Husky muy estable; ~8 deg rozando pared NO es vuelco. Solo abortar ante inclinacion sostenida y grande
INSP_MAX_PITCH_DEG = 20.0      # margen para transitorios, rampa y frenadas
# Antirrebote del guard: la condicion debe mantenerse N ticks seguidos
# (0.2 s/tick) antes de cancelar. Asi un pico transitorio al rozar una
# pared (y recuperarse) no aborta la mision entera; solo una inclinacion
# o desvio SOSTENIDO dispara la parada de seguridad.
INSP_GUARD_STRIKES_REQUIRED = 5   # 5 x 0.2 s = 1.0 s sostenido
TUBE_ROUTE_HALF_WIDTH = 1.25
TUBE_ROUTE_MIN_Y = 6.5

STUCK_TIMEOUT_SEC = 18.0
STUCK_MIN_PROGRESS_M = 0.08
STUCK_MIN_YAW_PROGRESS_RAD = 0.08
STUCK_RECOVERY_ATTEMPTS = 3
STUCK_RECOVERY_FREE_M = 0.18
RECOVERY_CMD_PERIOD_SEC = 0.1
RELOCALIZATION_SPIN_SEC = 14.0
RELOCALIZATION_SPIN_RAD_S = 0.35
RELOCALIZATION_SPIN_SEGMENT_SEC = 2.0
RELOCALIZATION_STOP_SEGMENT_SEC = 1.2
MAP_REALIGNMENT_SETTLE_SEC = 4.0
MAP_REALIGNMENT_UPDATE_PERIOD_SEC = 1.0
STUCK_LOCALIZATION_STD_XY = 0.80
STUCK_LOCALIZATION_STD_YAW = 0.90
STUCK_RECOVERY_MAX_LOCAL_OFFSET = 0.75
CHARGE_CONFIRM_TOLERANCE_M = 0.75
CHARGE_CONFIRM_MAX_COV_XY = 0.35
CHARGE_SEARCH_SPIN_SEC = 16.0
CHARGE_SEARCH_RETRY_DELAY_SEC = 2.0

# ── Informe de inspección ──────────────────────────────────────────────────────
INSPECTIONS_ROOT = Path('~/ROS2_Wind_Tower_Inspection/inspections').expanduser()
REPORT_SUBPROCESS_TIMEOUT_SEC = 240.0
REPORT_MODULE = 'wind_tower_perception.scripts.generate_inspection_report'


# ── Nodo ROS2 ──────────────────────────────────────────────────────────────────
class MissionController(Node):

    def __init__(self):
        super().__init__('mission_controller')

        self._wp    = self._load_waypoints()
        self._batt  = 80.0
        self._charging  = False
        self._moving    = False
        self._dest      = 'idle'
        self._status    = 'Listo'
        self._alarm     = ''
        self._error_state = ''
        self._nl_parser = NaturalLanguageCommandParser()
        self._last_text_command = ''
        self._last_llm_command = ''
        self._last_llm_reason = ''
        self._last_llm_source = ''
        self._last_llm_confidence = 0.0
        self._mission_queue = []
        self._active_queue_item = None
        self._queue_paused = False
        self._mission_generation = 0
        self._active_send_goal_future = None
        self._lock      = threading.Lock()
        self._robot_x   = 0.0
        self._robot_y   = 0.0
        self._robot_yaw = 0.0
        self._amcl_cov_x = float('inf')
        self._amcl_cov_y = float('inf')
        self._amcl_cov_yaw = float('inf')
        self._last_amcl_time = 0.0
        self._imu_roll_deg = 0.0
        self._imu_pitch_deg = 0.0
        self._inspection_approach = []
        self._inspection_approach_index = 0
        self._inspection_points = []
        self._inspection_index = 0
        self._inspection_active = False
        self._inspection_paused = False
        self._inspection_guard_tripped = False
        self._inspection_guard_strikes = 0
        self._inspection_returning = False
        self._inspection_pause_timer = None
        # Estado del brazo durante una parada de inspección.
        #
        # Patrón "doble flanco": el arm publica /arm/inspection_ready=True
        # CONSTANTEMENTE cuando cualquier behaviour deseado está alcanzado
        # (también hold_lookat). Si solo miramos "ready==True" se cumple
        # inmediatamente al entrar en pausa (residuo del hold_lookat previo)
        # y el robot avanza antes de que el sweep arranque siquiera.
        #
        # Solución: tras publicar state=INSPECTING:
        #   1) Esperar a ver ready=False  → confirma que arm cambió a
        #      desired=sweep y empezó a moverse (busy)
        #   2) Después esperar ready=True → sweep + vuelta a home OK
        #
        # _arm_pause_failures: lista de y (mundo) donde MoveIt no terminó
        #                      en INSP_ARM_TIMEOUT_S — se vuelca al reporte
        self._arm_inspection_ready = False     # último valor del topic
        self._arm_pause_start_s = 0.0
        self._arm_pause_failures = []
        self._arm_pause_saw_false = False      # se cumplió el primer flanco
        self._arm_pause_completed = False      # se cumplió el segundo flanco
        now = self._now_sec()
        self._last_progress_x = 0.0
        self._last_progress_y = 0.0
        self._last_progress_yaw = 0.0
        self._last_progress_time = now
        self._stuck_alarm_tripped = False
        self._stuck_recovery_active = False
        self._stuck_recovery_attempt = 0
        self._stuck_recovery_step = 0
        self._stuck_recovery_step_start = now
        self._stuck_detect_x = 0.0
        self._stuck_detect_y = 0.0
        self._stuck_detect_yaw = 0.0
        self._stuck_recovery_start_x = 0.0
        self._stuck_recovery_start_y = 0.0
        self._stuck_recovery_timer = None
        self._relocalization_active = False
        self._relocalization_start_time = now
        self._relocalization_last_stop_index = -1
        self._map_realignment_last_update_time = now
        self._relocalization_timer = None
        self._charging_search_active = False
        self._charging_search_attempt = 0
        self._charging_search_start_time = now
        self._charging_search_timer = None
        self._charging_search_retry_timer = None

        # Estado del informe post-inspección
        self._report_status = 'idle'   # idle | running | done | done_no_llm | error
        self._report_run_id = ''
        self._report_run_dir = ''
        self._report_summary_md = ''
        self._report_full_md = ''
        self._report_has_defect_map = False
        self._report_message = ''
        self._report_started_at = 0.0
        self._report_thread: Optional[threading.Thread] = None

        self._nav1 = ActionClient(self, NavigateToPose,       'navigate_to_pose')
        self._navN = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self._global_localization_cli = self.create_client(
            Empty,
            '/reinitialize_global_localization',
        )
        self._nomotion_update_cli = self.create_client(
            Empty,
            '/request_nomotion_update',
        )
        self._active_goal_handle = None   # para poder cancelar
        self._active_goal_kind = None

        self._marker_pub = self.create_publisher(MarkerArray, '/mission_markers', 10)
        self._batt_pub   = self.create_publisher(Float32,     '/battery_percent',  10)
        self._cmd_pub    = self.create_publisher(String,      '/mission_command',  10)
        self._recovery_cmd_pub = self.create_publisher(TwistStamped, '/robot/cmd_vel', 10)
        self._initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10,
        )
        self._state_text_pub = self.create_publisher(String, '/inspection/state_text', 10)
        self._mission_status_pub = self.create_publisher(String, '/inspection/mission_status', 10)
        self._autonomous_pub = self.create_publisher(Bool, '/inspection/autonomous_active', 10)
        self._relocalization_pub = self.create_publisher(
            String,
            '/inspection/relocalization_request',
            10,
        )

        self.create_subscription(String, '/mission_command', self._on_command, 10)
        self.create_subscription(String, '/mission_command_text', self._on_voice_text, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl_pose, 10
        )
        self.create_subscription(Imu, '/robot/sensors/imu_0/data', self._on_imu, 10)
        # Señal de arm_inspection_node: True cuando el brazo ha terminado el
        # sweep y ha vuelto a home — momento en que el robot puede avanzar.
        self.create_subscription(
            Bool, '/arm/inspection_ready', self._on_arm_inspection_ready, 10,
        )

        # Camara — frame JPEG para streaming web
        self._camera_frame_lock = threading.Lock()
        self._camera_jpeg: Optional[bytes] = None
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        _cam_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            Image,
            '/inspection/camera/image_raw',
            self._on_camera_image,
            _cam_qos,
        )
        self.create_subscription(
            Image,
            '/robot/sensors/inspection_camera/image',
            self._on_camera_image,
            _cam_qos,
        )

        # Nube de puntos LiDAR — downsampled para streaming web
        self._pointcloud_lock = threading.Lock()
        self._pointcloud_pts: list = []
        self.create_subscription(
            PointCloud2,
            '/velodyne_points',
            self._on_pointcloud,
            _cam_qos,
        )

        # URDF del robot — TRANSIENT_LOCAL para recibir el último valor publicado
        self._robot_description_lock = threading.Lock()
        self._robot_description = ''
        from rclpy.qos import DurabilityPolicy
        _urdf_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String,
            '/robot_description',
            self._on_robot_description,
            _urdf_qos,
        )

        self.create_timer(0.5, self._battery_tick)
        self.create_timer(1.0, self._publish_markers)
        self.create_timer(0.2, self._inspection_guard_tick)
        self.create_timer(0.5, self._progress_watchdog_tick)

        self.get_logger().info('MissionController listo — web UI en http://localhost:5000')

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    @staticmethod
    def _yaw_from_quat(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _angle_delta(a: float, b: float) -> float:
        return math.atan2(math.sin(a - b), math.cos(a - b))

    # ── Pose del robot ─────────────────────────────────────────────────────────
    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped):
        with self._lock:
            self._robot_x = msg.pose.pose.position.x
            self._robot_y = msg.pose.pose.position.y
            self._robot_yaw = self._yaw_from_quat(msg.pose.pose.orientation)
            self._amcl_cov_x = msg.pose.covariance[0]
            self._amcl_cov_y = msg.pose.covariance[7]
            self._amcl_cov_yaw = msg.pose.covariance[35]
            self._last_amcl_time = self._now_sec()

    def _on_imu(self, msg: Imu):
        q = msg.orientation
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        with self._lock:
            self._imu_roll_deg = math.degrees(roll)
            self._imu_pitch_deg = math.degrees(pitch)

    @staticmethod
    def _ensure_cv2_path():
        """Añade dist-packages al path si cv2 no está importable directamente."""
        try:
            import cv2  # noqa: F401
        except ImportError:
            import sys
            _dist = '/usr/lib/python3/dist-packages'
            if _dist not in sys.path:
                sys.path.insert(0, _dist)

    def _on_camera_image(self, msg: Image):
        try:
            self._ensure_cv2_path()
            import cv2
            import numpy as np
            # Soporta encoding bgr8, rgb8 y bgra8 habituales en Gazebo
            enc = msg.encoding.lower()
            channels = len(msg.data) // (msg.height * msg.width) if msg.height and msg.width else 3
            raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, channels
            )
            if enc in ('rgb8', 'rgb'):
                frame = cv2.cvtColor(raw, cv2.COLOR_RGB2BGR)
            elif enc in ('bgra8',):
                frame = cv2.cvtColor(raw, cv2.COLOR_BGRA2BGR)
            elif enc in ('rgba8',):
                frame = cv2.cvtColor(raw, cv2.COLOR_RGBA2BGR)
            elif enc in ('mono8',):
                frame = cv2.cvtColor(raw, cv2.COLOR_GRAY2BGR)
            else:
                frame = raw  # ya es BGR
            _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            with self._camera_frame_lock:
                self._camera_jpeg = buf.tobytes()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Error codificando frame de camara: {exc}', throttle_duration_sec=5.0)

    def get_camera_jpeg(self) -> Optional[bytes]:
        with self._camera_frame_lock:
            return self._camera_jpeg

    def _on_pointcloud(self, msg: PointCloud2):
        import struct
        try:
            fields = {f.name: f.offset for f in msg.fields}
            x_off = fields.get('x', 0)
            y_off = fields.get('y', 4)
            z_off = fields.get('z', 8)
            step = msg.point_step
            n = msg.width * msg.height
            stride = max(1, n // 500)
            pts = []
            data = bytes(msg.data)
            for i in range(0, n, stride):
                base = i * step
                if base + z_off + 4 > len(data):
                    break
                x = struct.unpack_from('<f', data, base + x_off)[0]
                y = struct.unpack_from('<f', data, base + y_off)[0]
                z = struct.unpack_from('<f', data, base + z_off)[0]
                if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                    r = math.hypot(x, y)
                    if 0.3 < r < 25.0:
                        pts.append([round(x, 2), round(y, 2), round(z, 2)])
            with self._pointcloud_lock:
                self._pointcloud_pts = pts
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f'Error parseando pointcloud: {exc}', throttle_duration_sec=5.0
            )

    def get_pointcloud(self) -> list:
        with self._pointcloud_lock:
            return list(self._pointcloud_pts)

    def _on_robot_description(self, msg: String):
        with self._robot_description_lock:
            self._robot_description = msg.data
        self.get_logger().info('robot_description recibido (%d bytes)' % len(msg.data))

    def get_robot_description(self) -> str:
        with self._robot_description_lock:
            return self._robot_description

    def _get_robot_y(self) -> float:
        with self._lock:
            return self._robot_y

    def _get_robot_x(self) -> float:
        with self._lock:
            return self._robot_x

    def _is_robot_in_tube_area(self) -> bool:
        with self._lock:
            lateral_error = abs(self._robot_x - INSP_CENTER_X)
            return self._robot_y >= TUBE_ROUTE_MIN_Y and lateral_error <= TUBE_ROUTE_HALF_WIDTH

    def _charging_station_confirmation(self) -> tuple[bool, str]:
        wp = self._wp.get('charging_station')
        if not wp:
            return False, 'waypoint charging_station no existe'

        with self._lock:
            dx = self._robot_x - float(wp['x'])
            dy = self._robot_y - float(wp['y'])
            dist = math.hypot(dx, dy)
            cov_xy = max(self._amcl_cov_x, self._amcl_cov_y)
            amcl_age = self._now_sec() - self._last_amcl_time

        if amcl_age > 3.0:
            return False, f'AMCL sin pose reciente ({amcl_age:.1f}s)'
        if cov_xy > CHARGE_CONFIRM_MAX_COV_XY:
            return (
                False,
                f'AMCL poco fiable cov_xy={cov_xy:.3f} '
                f'(max {CHARGE_CONFIRM_MAX_COV_XY:.3f})',
            )
        if dist > CHARGE_CONFIRM_TOLERANCE_M:
            return (
                False,
                f'AMCL dice que esta a {dist:.2f} m de carga '
                f'(max {CHARGE_CONFIRM_TOLERANCE_M:.2f} m)',
            )
        return True, f'AMCL confirmado dist={dist:.2f} m cov_xy={cov_xy:.3f}'

    def _publish_inspection_state(self, state: str, detail: str = ''):
        # /inspection/state_text → solo el state bare (sin detalle). Esto es
        # lo que consume arm_inspection_node, que hace dict.get(state,
        # default) con clave exacta — si publicas 'INSPECTING: punto 6/30'
        # no matchea con 'INSPECTING' y el brazo se queda en default=home.
        self._state_text_pub.publish(String(data=state))
        # /inspection/mission_status → state + detail para la UI/log humano.
        text = state if not detail else f'{state}: {detail}'
        self._mission_status_pub.publish(String(data=text))

    def _set_alarm(self, state: str, detail: str):
        with self._lock:
            self._error_state = state
            self._alarm = detail
            self._status = f'{state}: {detail}'
        self._publish_inspection_state(state, detail)

    def _clear_alarm_locked(self):
        self._alarm = ''
        self._error_state = ''

    # ── Waypoints ──────────────────────────────────────────────────────────────
    def _load_waypoints(self) -> dict:
        candidates = [
            Path(__file__).parents[3] / 'wind_tower_bringup' / 'config' / 'waypoints.yaml',
        ]
        try:
            import ament_index_python.packages as aip
            share = aip.get_package_share_directory('wind_tower_bringup')
            candidates.insert(0, Path(share) / 'config' / 'waypoints.yaml')
        except Exception:
            pass
        for p in candidates:
            if p.exists():
                with open(p) as f:
                    data = yaml.safe_load(f)
                self.get_logger().info(f'Waypoints cargados: {list(data["waypoints"].keys())}')
                return data['waypoints']
        self.get_logger().error('waypoints.yaml no encontrado')
        return {}

    # ── Bateria ────────────────────────────────────────────────────────────────
    def _battery_tick(self):
        go_charge = False
        charge_completed = False
        with self._lock:
            if self._charging:
                self._batt = min(BATTERY_FULL, self._batt + BATTERY_CHARGE)
                if self._batt >= BATTERY_FULL:
                    self._charging = False
                    self._status   = 'Carga completa'
                    charge_completed = True
            elif self._moving:
                self._batt = max(0.0, self._batt - BATTERY_DRAIN)

            low_battery = self._batt <= BATTERY_LOW
            critical_battery = self._batt <= BATTERY_CRITICAL
            going_to_charge = self._dest in {
                'charging_station',
                'battery_critical',
                'relocalizing',
                'map_realigning',
                'searching_charging_station',
            }
            if (
                (critical_battery or low_battery)
                and not self._charging
                and not going_to_charge
            ):
                if critical_battery:
                    self.get_logger().error(
                        f'Bateria critica ({self._batt:.0f}%) — carga obligatoria'
                    )
                    self._status = (
                        f'BATERIA CRITICA ({self._batt:.0f}%) — '
                        'interrumpiendo y yendo a carga'
                    )
                    self._error_state = 'BATTERY_CRITICAL'
                    self._alarm = 'Bateria <10%; retorno a carga obligatorio'
                    go_charge = 'battery_critical'
                else:
                    self.get_logger().warn(
                        f'Bateria baja ({self._batt:.0f}%) — yendo a carga'
                    )
                    self._status = f'BATERIA BAJA ({self._batt:.0f}%) — yendo a carga'
                    go_charge = 'charging_station'

        msg = Float32()
        msg.data = float(self._batt)
        self._batt_pub.publish(msg)
        self._autonomous_pub.publish(Bool(data=bool(
            self._moving
            or self._inspection_active
            or self._stuck_recovery_active
            or self._relocalization_active
            or self._charging_search_active
        )))
        if go_charge:
            self._dispatch(go_charge)
        elif charge_completed:
            self._complete_active_queue_item(success=True)
            self._start_next_queued_if_idle()

    # ── Comandos ───────────────────────────────────────────────────────────────
    def send_command(self, cmd: str):
        msg = String()
        msg.data = cmd
        self._cmd_pub.publish(msg)

    def send_text_command(self, text: str) -> dict:
        intents = self._nl_parser.parse_many(text)
        accepted = []
        rejected = []
        first_intent = intents[0]

        with self._lock:
            self._last_text_command = text
            self._last_llm_command = ', '.join(intent.command for intent in intents)
            self._last_llm_reason = '; '.join(
                intent.reason for intent in intents if intent.reason
            )[:240]
            self._last_llm_source = first_intent.source
            self._last_llm_confidence = round(
                min(intent.confidence for intent in intents),
                2,
            )

        for intent in intents:
            label = COMMAND_LABELS.get(intent.command, intent.command)
            if intent.command == 'unknown':
                rejected.append({
                    'command': intent.command,
                    'label': label,
                    'confidence': round(intent.confidence, 2),
                    'reason': intent.reason,
                    'source': intent.source,
                })
                self.get_logger().warn(
                    f'Comando texto no entendido: "{text}" ({intent.reason})'
                )
            elif intent.command == 'cancel':
                self.get_logger().info(
                    f'Comando texto "{text}" -> cancel '
                    f'({intent.source}, conf={intent.confidence:.2f})'
                )
                self.cancel()
                accepted.append({
                    'command': intent.command,
                    'label': label,
                    'confidence': round(intent.confidence, 2),
                    'reason': intent.reason,
                    'source': intent.source,
                })
            else:
                result = self.enqueue_command(
                    intent.command,
                    source_text=text,
                    source=intent.source,
                    confidence=intent.confidence,
                )
                accepted.append({
                    'command': intent.command,
                    'label': label,
                    'confidence': round(intent.confidence, 2),
                    'reason': intent.reason,
                    'source': intent.source,
                    'queued': result['queued'],
                })

        if not accepted and rejected:
            with self._lock:
                self._status = f'Texto no entendido: {rejected[0]["reason"]}'

        return {
            'text': text,
            'accepted': accepted,
            'rejected': rejected,
            'queue': self.queue_state(),
        }

    def _on_command(self, msg: String):
        self.get_logger().info(f'Comando: {msg.data}')
        self.enqueue_command(msg.data.strip().lower(), source='ros_topic')

    def _on_voice_text(self, msg: String):
        text = (msg.data or '').strip()
        if not text:
            return
        self.get_logger().info(f'Comando voz: "{text}"')
        self.send_text_command(text)

    def _mission_is_active_locked(self) -> bool:
        return bool(
            self._moving
            or self._charging
            or self._inspection_active
            or self._inspection_paused
            or self._stuck_recovery_active
            or self._relocalization_active
            or self._charging_search_active
            or self._active_goal_handle is not None
            or self._active_send_goal_future is not None
            or self._active_queue_item is not None
        )

    def _make_queue_item(
        self,
        command: str,
        source_text: str = '',
        source: str = 'manual',
        confidence: float = 1.0,
    ) -> dict:
        return {
            'command': command,
            'label': COMMAND_LABELS.get(command, command),
            'source_text': source_text,
            'source': source,
            'confidence': round(float(confidence), 2),
        }

    def enqueue_command(
        self,
        command: str,
        source_text: str = '',
        source: str = 'manual',
        confidence: float = 1.0,
    ) -> dict:
        command = (command or '').strip().lower()
        if command == 'cancel':
            self.cancel()
            return {'accepted': True, 'queued': False, 'command': command}
        if command not in MISSION_COMMANDS:
            with self._lock:
                self._status = f'Comando desconocido: {command}'
            self.get_logger().warn(f'Comando desconocido: {command}')
            return {'accepted': False, 'queued': False, 'command': command}

        item = self._make_queue_item(command, source_text, source, confidence)
        start_now = False
        with self._lock:
            self._queue_paused = False
            if self._mission_is_active_locked():
                self._mission_queue.append(item)
                self._status = (
                    f'En cola: {item["label"]} '
                    f'({len(self._mission_queue)} pendiente/s)'
                )
                queued = True
            else:
                self._active_queue_item = item
                start_now = True
                queued = False

        if start_now:
            self._dispatch(command)
        return {'accepted': True, 'queued': queued, 'command': command}

    def clear_queue(self) -> int:
        with self._lock:
            removed = len(self._mission_queue)
            self._mission_queue = []
            self._queue_paused = False
            if removed:
                self._status = f'Cola vaciada ({removed} comando/s eliminado/s)'
        return removed

    def queue_state(self) -> dict:
        with self._lock:
            return {
                'active': dict(self._active_queue_item) if self._active_queue_item else None,
                'pending': [dict(item) for item in self._mission_queue],
                'paused': self._queue_paused,
            }

    def _complete_active_queue_item(self, success: bool):
        with self._lock:
            self._active_queue_item = None
            if not success and self._mission_queue:
                self._queue_paused = True

    def _start_next_queued_if_idle(self):
        with self._lock:
            if self._queue_paused or self._mission_is_active_locked() or not self._mission_queue:
                return
            item = self._mission_queue.pop(0)
            self._active_queue_item = item
            command = item['command']
            self._status = f'Iniciando siguiente cola -> {item["label"]}'
        self._dispatch(command)

    def _reset_inspection(self):
        if self._inspection_pause_timer is not None:
            self._inspection_pause_timer.cancel()
            self._inspection_pause_timer = None
        self._inspection_approach = []
        self._inspection_approach_index = 0
        self._inspection_points = []
        self._inspection_index = 0
        self._inspection_active = False
        self._inspection_paused = False
        self._inspection_guard_tripped = False
        self._inspection_guard_strikes = 0
        self._inspection_returning = False

    def _reset_progress_watchdog_locked(self):
        self._last_progress_x = self._robot_x
        self._last_progress_y = self._robot_y
        self._last_progress_yaw = self._robot_yaw
        self._last_progress_time = self._now_sec()
        self._stuck_alarm_tripped = False

    def _reset_stuck_recovery_locked(self):
        if self._stuck_recovery_timer is not None:
            self._stuck_recovery_timer.cancel()
            self._stuck_recovery_timer = None
        self._stuck_recovery_active = False
        self._stuck_recovery_attempt = 0
        self._stuck_recovery_step = 0

    def _reset_relocalization_locked(self):
        if self._relocalization_timer is not None:
            self._relocalization_timer.cancel()
            self._relocalization_timer = None
        self._relocalization_active = False

    def _reset_charging_search_locked(self):
        if self._charging_search_timer is not None:
            self._charging_search_timer.cancel()
            self._charging_search_timer = None
        if self._charging_search_retry_timer is not None:
            self._charging_search_retry_timer.cancel()
            self._charging_search_retry_timer = None
        self._charging_search_active = False

    def _publish_recovery_cmd(self, linear_x: float = 0.0, angular_z: float = 0.0):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(linear_x)
        msg.twist.angular.z = float(angular_z)
        self._recovery_cmd_pub.publish(msg)

    def _publish_local_initial_pose(self, x: float, y: float, yaw: float):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance[0] = STUCK_LOCALIZATION_STD_XY ** 2
        msg.pose.covariance[7] = STUCK_LOCALIZATION_STD_XY ** 2
        msg.pose.covariance[35] = STUCK_LOCALIZATION_STD_YAW ** 2
        self._initial_pose_pub.publish(msg)
        self.get_logger().info(
            'Relocalizacion local: /initialpose centrado en '
            f'({x:.2f}, {y:.2f}, yaw={yaw:.2f})'
        )

    def _dispatch(self, dest: str):
        old_handle = None
        old_future = None
        with self._lock:
            old_handle = self._active_goal_handle
            old_future = self._active_send_goal_future
            self._mission_generation += 1
            self._active_goal_handle = None
            self._active_goal_kind = None
            self._active_send_goal_future = None
            if dest != 'start_inspection':
                self._reset_inspection()
            self._reset_stuck_recovery_locked()
            self._reset_relocalization_locked()
            if dest not in {'charging_station', 'battery_critical'}:
                self._reset_charging_search_locked()
                self._charging_search_attempt = 0
            self._clear_alarm_locked()
            self._dest   = dest
            self._status = f'Navegando -> {dest}'
            self._moving = True
            self._reset_progress_watchdog_locked()

        if old_future is not None and not old_future.done():
            old_future.cancel()
        if old_handle is not None:
            self.get_logger().info('Cancelando goal Nav2 anterior antes de nueva mision')
            old_handle.cancel_goal_async()

        if dest in {
            'charging_station',
            'battery_critical',
            'home_inspection',
            'maintenance_station',
            'start_inspection',
        }:
            self._run_mission_tree(dest)
        else:
            self.get_logger().warn(f'Destino desconocido: {dest}')
            with self._lock:
                self._moving = False
            self._complete_active_queue_item(success=False)

    # ── Logica de navegacion ───────────────────────────────────────────────────
    def _run_mission_tree(self, command: str):
        """Evalua el BT completo y dispara la accion asociada al comando."""
        tree = build_mission_tree(
            command=command,
            get_robot_x    = self._get_robot_x,
            get_robot_y    = self._get_robot_y,
            send_through_fn= self._send_through,
            send_to_fn     = self._send_to,
            make_pose_fn   = self._pose,
            waypoints      = self._wp,
            gate_left      = GATE_LEFT,
            gate_right     = GATE_RIGHT,
            post_gate      = POST_GATE,
            start_inspection_fn = self._go_inspection,
        )
        tree.setup()
        tree.tick_once()
        self.get_logger().info(
            f'Mission BT[{command}] → {tree.tip().name if tree.tip() else "?"} '
            f'(robot=({self._get_robot_x():.2f}, {self._get_robot_y():.2f}))'
        )

    def _go_direct(self, name: str):
        wp = self._wp.get(name)
        if not wp:
            return
        vias = wp.get('via_points') or []
        if vias:
            poses = [self._pose(v['x'], v['y'], v.get('yaw', 0.0)) for v in vias]
            poses.append(self._pose(wp['x'], wp['y'], wp.get('yaw', 0.0)))
            self._send_through(poses)
        else:
            self._send_to(wp)

    def _go_via_gate(self, name: str):
        wp = self._wp.get(name)
        if not wp:
            return
        poses = [
            self._pose(*GATE_RIGHT),
            self._pose(*POST_GATE),
        ]
        for v in (wp.get('via_points') or []):
            poses.append(self._pose(v['x'], v['y'], v.get('yaw', 0.0)))
        poses.append(self._pose(wp['x'], wp['y'], wp.get('yaw', 0.0)))
        self._send_through(poses)

    def _go_inspection(self):
        ramp = self._wp.get('ramp_top', {})
        inicio = self._wp.get('inicio_tramo', {})
        fin = self._wp.get('fin_tramo', {})

        # Leer coordenadas desde waypoints.yaml (fuente única de verdad)
        x0 = inicio.get('x', INSP_START[0])
        y0 = inicio.get('y', INSP_START[1])
        x1 = fin.get('x', INSP_END[0])
        y1 = fin.get('y', INSP_END[1])

        n = max(1, math.ceil(math.hypot(x1 - x0, y1 - y0) / INSP_STEP))

        # Aproximación: ramp_top → inicio_tramo
        approach = [
            self._pose(ramp.get('x', 0.245), ramp.get('y', 6.901), math.pi / 2),
            self._pose(x0, y0, math.pi / 2),
        ]
        points = []
        for i in range(n + 1):
            t = i / n
            points.append((
                x0 + t * (x1 - x0),
                y0 + t * (y1 - y0),
            ))

        with self._lock:
            self._reset_inspection()
            self._inspection_approach = approach
            self._inspection_points = points
            self._inspection_active = True
            self._inspection_guard_tripped = False
            self._inspection_guard_strikes = 0
            self._status = (
                f'Aproximando a inspeccion axial: {len(points)} paradas '
                f'cada {INSP_STEP:.1f} m'
            )
        self._publish_inspection_state('APPROACHING_INSPECTION', 'rampa superior -> inicio tramo')
        self._send_next_inspection_goal()

    # ── Nav2 helpers ───────────────────────────────────────────────────────────
    @staticmethod
    def _pose(x: float, y: float, yaw: float = 0.0) -> PoseStamped:
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.orientation.z = math.sin(yaw / 2.0)
        p.pose.orientation.w = math.cos(yaw / 2.0)
        return p

    def _send_next_inspection_goal(self):
        finished = False
        continue_return = False
        pose = phase = label = None
        with self._lock:
            if not self._inspection_active:
                return
            if self._inspection_approach_index < len(self._inspection_approach):
                pose = self._inspection_approach[self._inspection_approach_index]
                self._inspection_approach_index += 1
                phase = 'inspection_approach'
                label = f'aproximacion {self._inspection_approach_index}/{len(self._inspection_approach)}'
            elif self._inspection_index < len(self._inspection_points):
                x, y = self._inspection_points[self._inspection_index]
                yaw = -math.pi / 2 if self._inspection_returning else math.pi / 2
                pose = self._pose(x, y, yaw)
                phase = 'inspection_scan'
                direction = 'vuelta' if self._inspection_returning else 'ida'
                label = f'{direction} {self._inspection_index + 1}/{len(self._inspection_points)} y={y:.2f}'
            else:
                truly_done = self._finish_inspection_locked()
                if truly_done:
                    finished = True
                else:
                    continue_return = True

            if not finished and not continue_return:
                self._moving = True
                self._inspection_paused = False
                self._status = f'Inspeccion axial: navegando a {label}'

        if finished:
            self._complete_active_queue_item(success=True)
            self._trigger_report_generation()
            self._start_next_queued_if_idle()
            return
        if continue_return:
            self._send_next_inspection_goal()
            return
        self._publish_inspection_state('MOVING_TO_INSPECTION_POINT', label)
        self._send_pose_goal(pose, phase)

    def _start_inspection_pause(self):
        """Robot parado en un punto de inspección.

        Dispara el sweep 360° del UR5e publicando state=INSPECTING (el
        arm_inspection_node lo mapea a behaviour='sweep'). Luego espera
        el patrón doble flanco en /arm/inspection_ready:
          1) False → arm cambió a sweep y está moviéndose
          2) True  → sweep completo + vuelta a home
        Failsafe: tras INSP_ARM_TIMEOUT_S sigue igualmente y registra el
        punto como 'arm_failed_at_y=X.XX' para el reporte.
        """
        with self._lock:
            if not self._inspection_active:
                return
            point_no = self._inspection_index + 1
            total = len(self._inspection_points)
            x, y = self._inspection_points[self._inspection_index]
            self._moving = False
            self._inspection_paused = True
            # Reseteo para esta parada: ignoramos el "ready=True" residual
            # del hold_lookat previo y esperamos a ver primero False
            # (sweep en marcha) y después True (sweep+home OK).
            self._arm_inspection_ready = False
            self._arm_pause_saw_false = False
            self._arm_pause_completed = False
            self._arm_pause_start_s = self._now_sec()
            self._status = (
                f'INSPECTING punto {point_no}/{total} '
                f'(x={x:.2f}, y={y:.2f}) — esperando sweep brazo '
                f'(timeout {INSP_ARM_TIMEOUT_S:.0f}s)'
            )
            if self._inspection_pause_timer is not None:
                self._inspection_pause_timer.cancel()
            # Polling cada 200 ms hasta que el brazo termine o timeout.
            self._inspection_pause_timer = self.create_timer(
                INSP_ARM_POLL_S,
                self._inspection_pause_tick,
            )
        self._publish_inspection_state(
            'INSPECTING',
            f'punto {point_no}/{total}; sweep brazo, max {INSP_ARM_TIMEOUT_S:.0f}s',
        )

    def _on_arm_inspection_ready(self, msg: Bool):
        """Callback /arm/inspection_ready.

        Implementa el patrón doble flanco para distinguir el ready=True
        residual del hold_lookat previo del ready=True real tras un sweep.
        Solo importa cuando estamos en pausa (_inspection_paused=True);
        fuera de pausa solo actualizamos _arm_inspection_ready por si
        otros usuarios lo consultan.
        """
        value = bool(msg.data)
        self._arm_inspection_ready = value
        # Solo cuenta el doble flanco dentro de una pausa activa.
        if not self._inspection_paused:
            return
        if not value:
            # Primer flanco: arm dejó ready (empezó a moverse para sweep).
            self._arm_pause_saw_false = True
        elif self._arm_pause_saw_false:
            # Segundo flanco: arm dice ready tras haber estado moviéndose.
            self._arm_pause_completed = True

    def _inspection_pause_tick(self):
        """Polling: avanza cuando se completa el doble flanco, o timeout."""
        with self._lock:
            if not self._inspection_active or not self._inspection_paused:
                if self._inspection_pause_timer is not None:
                    self._inspection_pause_timer.cancel()
                    self._inspection_pause_timer = None
                return
            elapsed = self._now_sec() - self._arm_pause_start_s
            done = self._arm_pause_completed
            timed_out = elapsed >= INSP_ARM_TIMEOUT_S
            if not done and not timed_out:
                return  # seguimos esperando
            if timed_out and not done:
                # Registra el fallo y sigue (no aborta la misión).
                x, y = self._inspection_points[self._inspection_index]
                phase = (
                    'sweep no inició' if not self._arm_pause_saw_false
                    else 'sweep no terminó'
                )
                self.get_logger().warn(
                    f'Arm sweep timeout ({INSP_ARM_TIMEOUT_S:.0f}s, '
                    f'{phase}) en punto y={y:.2f}; sigo al siguiente.'
                )
                self._arm_pause_failures.append({
                    'point_index': self._inspection_index,
                    'x': float(x),
                    'y': float(y),
                    'reason': f'arm_sweep_timeout ({phase})',
                })
        # Llama fuera del lock para no anidar en _finish.
        self._finish_inspection_pause()

    def _finish_inspection_pause(self):
        with self._lock:
            if self._inspection_pause_timer is not None:
                self._inspection_pause_timer.cancel()
                self._inspection_pause_timer = None
            if not self._inspection_active:
                return
            self._inspection_index += 1
            self._inspection_paused = False
            # Reset por seguridad de cara a la siguiente parada.
            self._arm_inspection_ready = False
            self._arm_pause_saw_false = False
            self._arm_pause_completed = False
        self._send_next_inspection_goal()

    def _finish_inspection_locked(self) -> bool:
        """Devuelve True si la inspeccion ha terminado del todo, False si hay vuelta pendiente."""
        if not self._inspection_returning:
            self._inspection_returning = True
            self._inspection_points = list(reversed(self._inspection_points))
            self._inspection_index = 1  # ya estamos en el punto [0] invertido
            self._inspection_paused = False
            self._status = 'Inspeccion: iniciando vuelta al sur'
            return False
        self._inspection_returning = False
        self._inspection_active = False
        self._inspection_paused = False
        self._inspection_guard_tripped = False
        self._inspection_guard_strikes = 0
        self._moving = False
        self._dest = 'idle'
        self._status = 'Inspeccion axial completada (ida + vuelta)'
        return True

    def _abort_inspection(self, reason: str):
        with self._lock:
            self._reset_inspection()
            self._moving = False
            self._dest = 'idle'
            self._status = f'Inspeccion abortada: {reason}'
        self._publish_inspection_state('INSPECTION_ABORTED', reason)

    def _inspection_guard_tick(self):
        with self._lock:
            if (
                not self._inspection_active
                or self._inspection_paused
                or self._inspection_guard_tripped
                or self._inspection_approach_index < len(self._inspection_approach)
            ):
                return
            lateral_error = self._robot_x - INSP_CENTER_X
            roll = self._imu_roll_deg
            pitch = self._imu_pitch_deg
            handle = self._active_goal_handle

            reason = ''
            if abs(lateral_error) > INSP_MAX_LATERAL_ERROR:
                reason = (
                    f'desvio lateral {lateral_error:+.2f} m '
                    f'(limite {INSP_MAX_LATERAL_ERROR:.2f} m)'
                )
            elif abs(roll) > INSP_MAX_ROLL_DEG:
                reason = (
                    f'IMU roll {roll:+.1f} deg '
                    f'(limite {INSP_MAX_ROLL_DEG:.1f} deg)'
                )
            elif abs(pitch) > INSP_MAX_PITCH_DEG:
                reason = (
                    f'IMU pitch {pitch:+.1f} deg '
                    f'(limite {INSP_MAX_PITCH_DEG:.1f} deg)'
                )

            if not reason:
                # Condicion normal: limpiamos los strikes acumulados para que
                # los picos transitorios no se sumen entre paradas.
                self._inspection_guard_strikes = 0
                return

            # Antirrebote: exigimos que la condicion se mantenga varios ticks
            # seguidos antes de abortar. Un roce puntual con la pared (roll que
            # sube y baja) no cancela la mision.
            self._inspection_guard_strikes += 1
            if self._inspection_guard_strikes < INSP_GUARD_STRIKES_REQUIRED:
                self.get_logger().warn(
                    f'Guardia inspeccion: {reason} '
                    f'[{self._inspection_guard_strikes}/'
                    f'{INSP_GUARD_STRIKES_REQUIRED} ticks; '
                    f'aun no cancelo]'
                )
                return

            self._inspection_guard_tripped = True
            self._status = (
                f'Guardia inspeccion: cancelando por {reason} '
                f'(sostenido {self._inspection_guard_strikes} ticks)'
            )

        self.get_logger().warn(self._status)
        self._publish_inspection_state('INSPECTION_GUARD_STOP', reason)
        if handle is not None:
            handle.cancel_goal_async()
        else:
            self._abort_inspection(reason)

    def _progress_watchdog_tick(self):
        with self._lock:
            if (
                not self._moving
                or self._inspection_paused
                or self._stuck_recovery_active
                or self._relocalization_active
                or self._stuck_alarm_tripped
                or self._active_goal_handle is None
            ):
                return

            moved = math.hypot(
                self._robot_x - self._last_progress_x,
                self._robot_y - self._last_progress_y,
            )
            yaw_progress = abs(self._angle_delta(
                self._robot_yaw,
                self._last_progress_yaw,
            ))
            now = self._now_sec()
            if moved >= STUCK_MIN_PROGRESS_M or yaw_progress >= STUCK_MIN_YAW_PROGRESS_RAD:
                self._last_progress_x = self._robot_x
                self._last_progress_y = self._robot_y
                self._last_progress_yaw = self._robot_yaw
                self._last_progress_time = now
                return

            stalled_for = now - self._last_progress_time
            if stalled_for < STUCK_TIMEOUT_SEC:
                return

            handle = self._active_goal_handle
            self._stuck_alarm_tripped = True
            self._stuck_detect_x = self._robot_x
            self._stuck_detect_y = self._robot_y
            self._stuck_detect_yaw = self._robot_yaw
            self._status = (
                f'ERROR_STUCK: sin progreso {stalled_for:.0f}s '
                f'(avance < {STUCK_MIN_PROGRESS_M:.2f} m, '
                f'giro < {math.degrees(STUCK_MIN_YAW_PROGRESS_RAD):.1f} deg)'
            )
            self._error_state = 'ERROR_STUCK'
            self._alarm = 'Ruedas girando sin progreso global; cancelando Nav2'

        self.get_logger().error(self._status)
        self._publish_inspection_state('ERROR_STUCK', 'ruedas giran pero no avanza')
        if handle is not None:
            handle.cancel_goal_async()
        self._start_stuck_recovery()

    def _start_stuck_recovery(self):
        with self._lock:
            if self._stuck_recovery_active:
                return
            self._reset_inspection()
            self._moving = False
            self._dest = 'error_stuck_recovery'
            self._stuck_recovery_active = True
            self._stuck_recovery_attempt = 1
            self._stuck_recovery_step = 0
            self._stuck_recovery_step_start = self._now_sec()
            self._stuck_recovery_start_x = self._stuck_detect_x
            self._stuck_recovery_start_y = self._stuck_detect_y
            self._status = 'ERROR_STUCK_RECOVERY: intentando liberar el robot'
            self._error_state = 'ERROR_STUCK_RECOVERY'
            self._alarm = 'Recuperacion activa: marcha atras y giros cortos'
            if self._stuck_recovery_timer is not None:
                self._stuck_recovery_timer.cancel()
            self._stuck_recovery_timer = self.create_timer(
                RECOVERY_CMD_PERIOD_SEC,
                self._stuck_recovery_tick,
            )
        self._publish_inspection_state(
            'ERROR_STUCK_RECOVERY',
            f'intento 1/{STUCK_RECOVERY_ATTEMPTS}',
        )

    def _stuck_recovery_tick(self):
        sequence = [
            (3.0, -0.16, 0.0, 'marcha atras'),
            (2.0, 0.0, 0.45, 'giro izquierda'),
            (2.0, -0.12, 0.0, 'marcha atras corta'),
            (2.0, 0.0, -0.45, 'giro derecha'),
            (1.0, 0.0, 0.0, 'parada'),
        ]

        with self._lock:
            if not self._stuck_recovery_active:
                self._publish_recovery_cmd()
                return

            freed_distance = math.hypot(
                self._robot_x - self._stuck_recovery_start_x,
                self._robot_y - self._stuck_recovery_start_y,
            )
            local_offset = math.hypot(
                self._robot_x - self._stuck_detect_x,
                self._robot_y - self._stuck_detect_y,
            )
            if freed_distance >= STUCK_RECOVERY_FREE_M:
                self._finish_stuck_recovery_locked(
                    f'liberado tras mover {freed_distance:.2f} m'
                )
                go_charge = True
                cmd = (0.0, 0.0)
            elif local_offset >= STUCK_RECOVERY_MAX_LOCAL_OFFSET:
                self._finish_stuck_recovery_locked(
                    f'limite local {local_offset:.2f} m alcanzado'
                )
                go_charge = True
                cmd = (0.0, 0.0)
            else:
                now = self._now_sec()
                duration, linear_x, angular_z, label = sequence[self._stuck_recovery_step]
                if now - self._stuck_recovery_step_start >= duration:
                    self._stuck_recovery_step += 1
                    self._stuck_recovery_step_start = now
                    if self._stuck_recovery_step >= len(sequence):
                        if self._stuck_recovery_attempt >= STUCK_RECOVERY_ATTEMPTS:
                            self._reset_stuck_recovery_locked()
                            self._moving = False
                            self._dest = 'error_stuck_blocked'
                            self._status = (
                                'ERROR_STUCK_BLOCKED: no se pudo liberar; '
                                'requiere intervencion'
                            )
                            self._alarm = self._status
                            self._error_state = 'ERROR_STUCK_BLOCKED'
                            go_charge = False
                            cmd = (0.0, 0.0)
                        else:
                            self._stuck_recovery_attempt += 1
                            self._stuck_recovery_step = 0
                            self._stuck_recovery_step_start = now
                            self._stuck_recovery_start_x = self._robot_x
                            self._stuck_recovery_start_y = self._robot_y
                            self._status = (
                                'ERROR_STUCK_RECOVERY: '
                                f'intento {self._stuck_recovery_attempt}/'
                                f'{STUCK_RECOVERY_ATTEMPTS}'
                            )
                            self._alarm = self._status
                            go_charge = False
                            cmd = (-0.16, 0.0)
                    else:
                        duration, linear_x, angular_z, label = sequence[self._stuck_recovery_step]
                        self._status = (
                            'ERROR_STUCK_RECOVERY: '
                            f'intento {self._stuck_recovery_attempt}/'
                            f'{STUCK_RECOVERY_ATTEMPTS}, {label}'
                        )
                        self._alarm = self._status
                        go_charge = False
                        cmd = (linear_x, angular_z)
                else:
                    go_charge = False
                    cmd = (linear_x, angular_z)

        self._publish_recovery_cmd(*cmd)
        if go_charge:
            self._request_relocalization_and_charge()
        elif self._error_state == 'ERROR_STUCK_BLOCKED':
            self._publish_inspection_state(
                'ERROR_STUCK_BLOCKED',
                'recovery agotado; requiere intervencion',
            )

    def _finish_stuck_recovery_locked(self, detail: str):
        self._reset_stuck_recovery_locked()
        self._moving = False
        self._dest = 'localization_reset'
        self._status = f'STUCK_RECOVERY_OK: {detail}; yendo a carga'
        self._alarm = ''
        self._error_state = ''

    def _request_relocalization_and_charge(self):
        self._publish_recovery_cmd()
        self._relocalization_pub.publish(
            String(data='STUCK_RECOVERY_OK: local_relocalization_near_stuck')
        )
        self._start_relocalization_sequence()

    def _call_empty_service_if_available(self, client, label: str):
        if client.service_is_ready() or client.wait_for_service(timeout_sec=0.1):
            client.call_async(Empty.Request())
            self.get_logger().info(f'Relocalizacion: solicitado {label}')
            return True
        self.get_logger().warn(
            f'Relocalizacion: servicio {label} no disponible; '
            'continuo con giro de observacion'
        )
        return False

    def _start_relocalization_sequence(self):
        with self._lock:
            if self._relocalization_active:
                return
            self._relocalization_active = True
            self._relocalization_start_time = self._now_sec()
            self._relocalization_last_stop_index = -1
            self._map_realignment_last_update_time = 0.0
            self._dest = 'relocalizing'
            self._moving = False
            self._status = (
                'LOCAL_RELOCALIZATION: buscando coincidencias cerca del stuck'
            )
            if self._relocalization_timer is not None:
                self._relocalization_timer.cancel()
            self._relocalization_timer = self.create_timer(
                RECOVERY_CMD_PERIOD_SEC,
                self._relocalization_tick,
            )
            stuck_x = self._stuck_detect_x
            stuck_y = self._stuck_detect_y
            stuck_yaw = self._stuck_detect_yaw

        self._publish_local_initial_pose(stuck_x, stuck_y, stuck_yaw)
        self._call_empty_service_if_available(
            self._nomotion_update_cli,
            '/request_nomotion_update',
        )
        self._publish_inspection_state(
            'LOCAL_RELOCALIZATION',
            'busqueda local alrededor del stuck; giros con paradas',
        )

    def _relocalization_tick(self):
        with self._lock:
            if not self._relocalization_active:
                self._publish_recovery_cmd()
                return
            now = self._now_sec()
            elapsed = now - self._relocalization_start_time
            if elapsed < RELOCALIZATION_SPIN_SEC:
                cycle = RELOCALIZATION_SPIN_SEGMENT_SEC + RELOCALIZATION_STOP_SEGMENT_SEC
                cycle_index = int(elapsed // cycle)
                phase = elapsed - cycle_index * cycle
                if phase < RELOCALIZATION_SPIN_SEGMENT_SEC:
                    self._status = (
                        'LOCAL_RELOCALIZATION: giro corto para casar LiDAR '
                        f'({elapsed:.0f}/{RELOCALIZATION_SPIN_SEC:.0f}s)'
                    )
                    cmd = (0.0, RELOCALIZATION_SPIN_RAD_S)
                    request_update = False
                else:
                    self._status = (
                        'LOCAL_RELOCALIZATION: parada de calibracion '
                        f'({elapsed:.0f}/{RELOCALIZATION_SPIN_SEC:.0f}s)'
                    )
                    cmd = (0.0, 0.0)
                    request_update = cycle_index != self._relocalization_last_stop_index
                    self._relocalization_last_stop_index = cycle_index
                done = False
            elif elapsed < RELOCALIZATION_SPIN_SEC + MAP_REALIGNMENT_SETTLE_SEC:
                settle_elapsed = elapsed - RELOCALIZATION_SPIN_SEC
                self._dest = 'map_realigning'
                self._status = (
                    'MAP_REALIGNMENT: robot parado; ajustando map->odom '
                    f'({settle_elapsed:.1f}/{MAP_REALIGNMENT_SETTLE_SEC:.1f}s)'
                )
                cmd = (0.0, 0.0)
                request_update = (
                    now - self._map_realignment_last_update_time
                    >= MAP_REALIGNMENT_UPDATE_PERIOD_SEC
                )
                if request_update:
                    self._map_realignment_last_update_time = now
                done = False
            else:
                self._reset_relocalization_locked()
                self._dest = 'charging_station'
                self._status = 'MAP_REALIGNED: navegando a carga'
                cmd = (0.0, 0.0)
                request_update = True
                done = True

        self._publish_recovery_cmd(*cmd)
        if request_update:
            self._call_empty_service_if_available(
                self._nomotion_update_cli,
                '/request_nomotion_update',
            )
        if done:
            self._relocalization_pub.publish(
                String(data='MAP_REALIGNED: navigating_to_charging_station')
            )
            self._publish_inspection_state(
                'MAP_REALIGNED',
                'map->odom estabilizado; navegando a charging_station',
            )
            self._dispatch('charging_station')

    def _handle_charging_arrival(self):
        confirmed, detail = self._charging_station_confirmation()
        if confirmed:
            with self._lock:
                self._reset_charging_search_locked()
                self._charging_search_attempt = 0
                self._charging = True
                self._moving = False
                self._dest = 'charging_station'
                self._status = f'Cargando... ({detail})'
                self._alarm = ''
                self._error_state = ''
            self._publish_inspection_state('CHARGING_CONFIRMED', detail)
            self.get_logger().info(f'Estacion de carga confirmada: {detail}')
            return

        self._start_charging_search(detail)

    def _start_charging_search(self, reason: str):
        with self._lock:
            if self._charging_search_active:
                return
            self._reset_relocalization_locked()
            self._charging_search_active = True
            self._charging_search_attempt += 1
            self._charging_search_start_time = self._now_sec()
            self._moving = False
            self._charging = False
            self._dest = 'searching_charging_station'
            self._error_state = 'SEARCHING_CHARGING_STATION'
            self._alarm = (
                'Nav2 no confirma carga con AMCL/LiDAR; buscando referencia '
                'de estacion'
            )
            self._status = (
                'SEARCHING_CHARGING_STATION: '
                f'intento {self._charging_search_attempt}, {reason}'
            )
            if self._charging_search_timer is not None:
                self._charging_search_timer.cancel()
            self._charging_search_timer = self.create_timer(
                RECOVERY_CMD_PERIOD_SEC,
                self._charging_search_tick,
            )

        self._relocalization_pub.publish(
            String(data=f'SEARCHING_CHARGING_STATION: {reason}')
        )
        self._call_empty_service_if_available(
            self._global_localization_cli,
            '/reinitialize_global_localization',
        )
        self._publish_inspection_state('SEARCHING_CHARGING_STATION', reason)

    def _charging_search_tick(self):
        with self._lock:
            if not self._charging_search_active:
                self._publish_recovery_cmd()
                return
            elapsed = self._now_sec() - self._charging_search_start_time
            if elapsed < CHARGE_SEARCH_SPIN_SEC:
                self._status = (
                    'SEARCHING_CHARGING_STATION: giro lento para casar LiDAR '
                    f'con mapa ({elapsed:.0f}/{CHARGE_SEARCH_SPIN_SEC:.0f}s)'
                )
                cmd = (0.0, RELOCALIZATION_SPIN_RAD_S)
                done = False
            else:
                self._reset_charging_search_locked()
                self._status = (
                    'SEARCHING_CHARGING_STATION: reintentando ruta de carga '
                    f'(intento {self._charging_search_attempt})'
                )
                cmd = (0.0, 0.0)
                done = True

        self._publish_recovery_cmd(*cmd)
        if not done:
            return

        self._call_empty_service_if_available(
            self._nomotion_update_cli,
            '/request_nomotion_update',
        )
        confirmed, detail = self._charging_station_confirmation()
        if confirmed:
            self._handle_charging_arrival()
            return

        self._publish_inspection_state(
            'CHARGING_STATION_RETRY',
            f'{detail}; reintentando Nav2 hacia carga',
        )
        with self._lock:
            if self._charging_search_retry_timer is not None:
                self._charging_search_retry_timer.cancel()
            self._charging_search_retry_timer = self.create_timer(
                CHARGE_SEARCH_RETRY_DELAY_SEC,
                self._retry_charging_route,
            )

    def _retry_charging_route(self):
        with self._lock:
            if self._charging_search_retry_timer is not None:
                self._charging_search_retry_timer.cancel()
                self._charging_search_retry_timer = None
        self._dispatch('charging_station')

    def cancel(self):
        with self._lock:
            handle = self._active_goal_handle
            future = self._active_send_goal_future
            self._mission_generation += 1
            self._active_goal_handle = None
            self._active_goal_kind = None
            self._active_send_goal_future = None
            self._active_queue_item = None
            self._queue_paused = bool(self._mission_queue)
            self._reset_stuck_recovery_locked()
            self._reset_relocalization_locked()
            self._reset_charging_search_locked()
        if future is not None and not future.done():
            future.cancel()
        if handle is not None:
            self.get_logger().info('E-STOP: cancelando goal activo')
            handle.cancel_goal_async()
        for _ in range(3):
            self._publish_recovery_cmd()
        with self._lock:
            self._reset_inspection()
            self._moving   = False
            self._charging = False
            self._dest     = 'idle'
            self._status   = 'E-STOP — navegacion cancelada'
            if self._mission_queue:
                self._status += '; cola pausada'

    def _send_to(self, wp: dict):
        self._send_pose_goal(
            self._pose(wp['x'], wp['y'], wp.get('yaw', 0.0)),
            'normal',
        )

    def _send_pose_goal(self, pose: PoseStamped, kind: str):
        pose.header.stamp = self.get_clock().now().to_msg()
        goal = NavigateToPose.Goal()
        goal.pose = pose
        if not self._nav1.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('NavigateToPose no disponible')
            with self._lock:
                self._moving = False
            self._complete_active_queue_item(success=False)
            return
        with self._lock:
            generation = self._mission_generation
            self._active_goal_kind = kind
            self._reset_progress_watchdog_locked()
        future = self._nav1.send_goal_async(goal)
        with self._lock:
            if generation == self._mission_generation:
                self._active_send_goal_future = future
        future.add_done_callback(
            lambda f: self._goal_cb(f, kind, generation)
        )

    def _send_through(self, poses: list):
        for pose in poses:
            pose.header.stamp = self.get_clock().now().to_msg()
        goal = NavigateThroughPoses.Goal()
        goal.poses = poses
        if not self._navN.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('NavigateThroughPoses no disponible')
            with self._lock:
                self._moving = False
            self._complete_active_queue_item(success=False)
            return
        with self._lock:
            generation = self._mission_generation
            self._active_goal_kind = 'normal'
            self._reset_progress_watchdog_locked()
        future = self._navN.send_goal_async(goal)
        with self._lock:
            if generation == self._mission_generation:
                self._active_send_goal_future = future
        future.add_done_callback(
            lambda f: self._goal_cb(f, 'normal', generation)
        )

    def _goal_cb(self, f, kind: str, generation: int):
        try:
            handle = f.result()
        except Exception as exc:  # noqa: BLE001 - action futures can be cancelled during E-STOP
            should_complete = False
            with self._lock:
                if generation == self._mission_generation:
                    self._active_send_goal_future = None
                    self._moving = False
                    self._status = f'Error enviando goal Nav2 ({kind}): {exc}'
                    should_complete = True
            if should_complete:
                self._complete_active_queue_item(success=False)
            return

        with self._lock:
            stale = generation != self._mission_generation
            if not stale:
                self._active_send_goal_future = None
        if stale:
            if handle.accepted:
                self.get_logger().warn('Goal Nav2 obsoleto aceptado tras E-STOP; cancelando')
                handle.cancel_goal_async()
            return

        if not handle.accepted:
            with self._lock:
                self._active_goal_handle = None
                self._moving = False
                self._status = f'Goal rechazado por Nav2 ({kind})'
            self._publish_inspection_state('NAV2_GOAL_REJECTED', kind)
            self._complete_active_queue_item(success=False)
            return
        with self._lock:
            self._active_goal_handle = handle
            self._active_goal_kind = kind
        handle.get_result_async().add_done_callback(
            lambda r: self._result_cb(r, kind, generation)
        )

    def _result_cb(self, f, kind: str, generation: int):
        if generation != self._mission_generation:
            return
        try:
            result = f.result()
        except Exception as exc:  # noqa: BLE001 - action result futures can fail during cancellation
            with self._lock:
                if generation != self._mission_generation:
                    return
                self._active_goal_handle = None
                self._active_goal_kind = None
                self._moving = False
                self._status = f'Error recibiendo resultado Nav2 ({kind}): {exc}'
            self._complete_active_queue_item(success=False)
            return
        status = result.status
        verify_charging = False
        search_charging_reason = ''
        queue_success = False
        queue_terminal = False
        send_next = False
        start_pause = False
        with self._lock:
            self._active_goal_handle = None
            self._active_goal_kind = None
            if self._stuck_recovery_active:
                return
            if status != GoalStatus.STATUS_SUCCEEDED:
                if self._inspection_guard_tripped:
                    reason = self._status.replace('Guardia inspeccion: cancelando por ', '')
                    self._reset_inspection()
                    self._moving = False
                    self._dest = 'idle'
                    self._status = f'Inspeccion abortada: {reason}'
                    publish = ('INSPECTION_ABORTED', reason)
                    queue_terminal = True
                elif self._stuck_alarm_tripped:
                    publish = None
                elif self._dest in {'charging_station', 'battery_critical'}:
                    self._moving = False
                    self._status = (
                        f'Nav2 no completo carga ({kind}), status={status}; '
                        'buscando estacion'
                    )
                    search_charging_reason = f'Nav2 status={status}'
                    publish = None
                else:
                    self._moving = False
                    self._status = f'Nav2 no completo goal ({kind}), status={status}'
                    publish = ('NAV2_GOAL_FAILED', f'{kind} status={status}')
                    queue_terminal = True
                    if kind in ('inspection_approach', 'inspection_scan'):
                        self._reset_inspection()
            elif kind == 'inspection_approach':
                publish = None
                send_next = True
            elif kind == 'inspection_scan':
                publish = None
                start_pause = True
            else:
                self._moving = False
                if self._dest in {'charging_station', 'battery_critical'}:
                    self._status = 'Verificando estacion de carga con AMCL...'
                    verify_charging = True
                else:
                    self._status = f'Llegado: {self._dest}'
                    queue_success = True
                    queue_terminal = True
                publish = None

        if verify_charging:
            self._handle_charging_arrival()
            return
        if search_charging_reason:
            self._start_charging_search(search_charging_reason)
            return
        if send_next:
            self._send_next_inspection_goal()
            return
        if start_pause:
            self._start_inspection_pause()
            return
        if publish:
            self._publish_inspection_state(*publish)
        self.get_logger().info(f'Navegacion completada: {self._dest}')
        if queue_terminal:
            self._complete_active_queue_item(success=queue_success)
            if queue_success:
                self._start_next_queued_if_idle()

    # ── RViz markers ───────────────────────────────────────────────────────────
    def _publish_markers(self):
        ma  = MarkerArray()
        now = self.get_clock().now().to_msg()

        COLORS = {
            'charging_station':    (0.1, 0.9, 0.1),
            'home_inspection':     (0.2, 0.5, 1.0),
            'maintenance_station': (0.9, 0.5, 0.0),
            'inicio_tramo':        (0.0, 0.9, 0.9),
            'fin_tramo':           (0.0, 0.9, 0.9),
        }

        for i, (name, wp) in enumerate(self._wp.items()):
            r, g, b = COLORS.get(name, (0.7, 0.7, 0.7))
            ma.markers += self._cyl_label(i, now, wp['x'], wp['y'], r, g, b, name)

        # Gates (amarillo)
        for gi, (gx, gy, lbl) in enumerate([
            (*GATE_LEFT,  'Puerta Izq'),
            (*GATE_RIGHT, 'Puerta Der'),
        ]):
            ma.markers += self._cyl_label(80 + gi, now, gx, gy, 1.0, 1.0, 0.0, lbl, s=0.3)

        # Linea de inspeccion (cian)
        ma.markers.append(self._insp_line(now))

        # Indicador de bateria
        with self._lock:
            batt, status = self._batt, self._status
            alarm = self._alarm
        ma.markers.append(self._batt_marker(now, batt, status, alarm))

        self._marker_pub.publish(ma)

    def _mk(self, ns, mid, now) -> Marker:
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp    = now
        m.ns = ns
        m.id = mid
        m.action   = Marker.ADD
        m.lifetime = Duration(sec=2)
        m.pose.orientation.w = 1.0
        return m

    def _cyl_label(self, i, now, x, y, r, g, b, label, s=0.5):
        cyl = self._mk('wp_cyl', i, now)
        cyl.type = Marker.CYLINDER
        cyl.pose.position.x = float(x)
        cyl.pose.position.y = float(y)
        cyl.pose.position.z = 0.5
        cyl.scale.x = s; cyl.scale.y = s; cyl.scale.z = 1.0
        cyl.color.r = r; cyl.color.g = g; cyl.color.b = b; cyl.color.a = 0.85

        txt = self._mk('wp_txt', i + 200, now)
        txt.type = Marker.TEXT_VIEW_FACING
        txt.pose.position.x = float(x)
        txt.pose.position.y = float(y)
        txt.pose.position.z = 1.9
        txt.scale.z = 0.55
        txt.color.r = 1.0; txt.color.g = 1.0; txt.color.b = 1.0; txt.color.a = 1.0
        txt.text = label
        return [cyl, txt]

    def _insp_line(self, now) -> Marker:
        x0, y0 = INSP_START
        x1, y1 = INSP_END
        n = max(2, int(math.hypot(x1 - x0, y1 - y0) / INSP_STEP))
        line = self._mk('insp_line', 0, now)
        line.type    = Marker.LINE_STRIP
        line.scale.x = 0.12
        line.color.r = 0.0; line.color.g = 0.85; line.color.b = 1.0; line.color.a = 0.8
        for i in range(n + 1):
            t = i / n
            pt = Point()
            pt.x = x0 + t * (x1 - x0)
            pt.y = y0 + t * (y1 - y0)
            pt.z = 0.05
            line.points.append(pt)
        return line

    def _batt_marker(self, now, batt: float, status: str, alarm: str = '') -> Marker:
        m = self._mk('battery', 0, now)
        m.type = Marker.TEXT_VIEW_FACING
        m.pose.position.x = -14.0
        m.pose.position.y = -10.0
        m.pose.position.z = 2.5
        m.scale.z = 1.1
        if alarm:
            m.color.r = 1.0; m.color.g = 0.05; m.color.b = 0.05
        elif batt > 50:
            m.color.r = 0.1; m.color.g = 0.9; m.color.b = 0.2
        elif batt > 20:
            m.color.r = 1.0; m.color.g = 0.75; m.color.b = 0.0
        else:
            m.color.r = 0.9; m.color.g = 0.1; m.color.b = 0.1
        m.color.a = 1.0
        alarm_text = f'\nALARMA: {alarm}' if alarm else ''
        m.text    = f'Bateria: {batt:.0f}%\n{status}{alarm_text}'
        return m

    # ── Generación del informe post-inspección ─────────────────────────────────
    def _latest_run_dir(self) -> Optional[Path]:
        try:
            candidates = [p for p in INSPECTIONS_ROOT.iterdir()
                          if p.is_dir() and p.name.startswith('run_')]
        except FileNotFoundError:
            return None
        if not candidates:
            return None
        candidates.sort(key=lambda p: p.name, reverse=True)
        return candidates[0]

    def _trigger_report_generation(self):
        """Lanza generate_inspection_report en un hilo al terminar la inspección."""
        run_dir = self._latest_run_dir()
        if run_dir is None:
            self.get_logger().warn(
                f'No se encontró ninguna run-dir en {INSPECTIONS_ROOT}; '
                'omito generación de informe.'
            )
            return
        with self._lock:
            if self._report_thread is not None and self._report_thread.is_alive():
                self.get_logger().info(
                    'Ya hay un informe en curso; ignoro nuevo disparador.'
                )
                return
            self._report_status = 'running'
            self._report_run_id = run_dir.name
            self._report_run_dir = str(run_dir)
            self._report_summary_md = ''
            self._report_full_md = ''
            self._report_has_defect_map = False
            self._report_message = 'Generando resumen e informe…'
            self._report_started_at = self._now_sec()
            self._report_thread = threading.Thread(
                target=self._run_report_subprocess,
                args=(run_dir,),
                daemon=True,
            )
            self._report_thread.start()
        self.get_logger().info(
            f'Lanzo generación de informe para {run_dir.name}'
        )

    def _run_report_subprocess(self, run_dir: Path):
        cmd = [
            sys.executable, '-m', REPORT_MODULE,
            '--run-dir', str(run_dir),
        ]
        rc = -1
        stderr = ''
        try:
            proc = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=REPORT_SUBPROCESS_TIMEOUT_SEC,
            )
            rc = proc.returncode
            stderr = proc.stderr or ''
        except subprocess.TimeoutExpired:
            stderr = f'Timeout tras {REPORT_SUBPROCESS_TIMEOUT_SEC:.0f} s'
        except FileNotFoundError as exc:
            stderr = f'No se pudo ejecutar el generador: {exc}'

        report_dir = run_dir / 'report'
        summary_path = report_dir / 'inspection_summary.md'
        report_path = report_dir / 'inspection_report.md'
        defect_map_path = report_dir / 'defect_map.png'

        summary_md = ''
        report_md = ''
        if summary_path.is_file():
            try:
                summary_md = summary_path.read_text(encoding='utf-8')
            except OSError:
                pass
        if report_path.is_file():
            try:
                report_md = report_path.read_text(encoding='utf-8')
            except OSError:
                pass

        if rc == 0 and summary_md and report_md:
            status = 'done'
            message = 'Informe generado correctamente.'
        elif summary_md:
            # rc == 2 → script terminó sin API key, pero summary+map existen
            status = 'done_no_llm'
            if rc == 2:
                message = ('Resumen y mapa listos. Falta GEMINI_API_KEY '
                           'para generar el informe completo.')
            else:
                snippet = stderr.strip().splitlines()[-1] if stderr.strip() else ''
                message = (f'Resumen y mapa listos; informe LLM no disponible'
                           f' ({snippet})' if snippet
                           else 'Resumen y mapa listos; informe LLM no disponible.')
        else:
            status = 'error'
            snippet = stderr.strip().splitlines()[-1] if stderr.strip() else 'sin detalles'
            message = f'No se pudo generar el resumen (rc={rc}): {snippet}'

        with self._lock:
            self._report_status = status
            self._report_summary_md = summary_md
            self._report_full_md = report_md
            self._report_has_defect_map = defect_map_path.is_file()
            self._report_message = message

        log = self.get_logger().info if status != 'error' else self.get_logger().error
        log(f'[report] {status}: {message}')

    # ── Estado para Flask ──────────────────────────────────────────────────────
    def state(self) -> dict:
        with self._lock:
            return {
                'battery':     round(self._batt, 1),
                'status':      self._status,
                'destination': self._dest,
                'charging':    self._charging,
                'moving':      self._moving,
                'alarm':       self._alarm,
                'error_state': self._error_state,
                'inspection_active': self._inspection_active,
                'inspection_paused': self._inspection_paused,
                'inspection_index':  self._inspection_index,
                'inspection_count':  len(self._inspection_points),
                'stuck_recovery_active': self._stuck_recovery_active,
                'relocalization_active': self._relocalization_active,
                'robot_x':   round(self._robot_x, 3),
                'robot_y':   round(self._robot_y, 3),
                'robot_yaw': round(self._robot_yaw, 4),
                'imu_roll_deg':  round(self._imu_roll_deg, 1),
                'imu_pitch_deg': round(self._imu_pitch_deg, 1),
                'last_text_command': self._last_text_command,
                'last_llm_command': self._last_llm_command,
                'last_llm_reason': self._last_llm_reason,
                'last_llm_source': self._last_llm_source,
                'last_llm_confidence': self._last_llm_confidence,
                'queue_active': dict(self._active_queue_item) if self._active_queue_item else None,
                'queue_pending': [dict(item) for item in self._mission_queue],
                'queue_paused': self._queue_paused,
                'report': {
                    'status': self._report_status,
                    'run_id': self._report_run_id,
                    'message': self._report_message,
                    'summary_md': self._report_summary_md,
                    'report_md': self._report_full_md,
                    'defect_map_url': (
                        f'/inspections/{self._report_run_id}/report/defect_map.png'
                        if self._report_has_defect_map and self._report_run_id
                        else ''
                    ),
                    'run_url': (
                        f'/inspections/{self._report_run_id}/'
                        if self._report_run_id else ''
                    ),
                },
            }


# ── Flask Web UI ───────────────────────────────────────────────────────────────

_HTML = r'''<!DOCTYPE html>
<html lang="es">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>WTIS — Wind Tower Inspection System</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link href="https://fonts.googleapis.com/css2?family=Rajdhani:wght@400;500;600;700&family=Share+Tech+Mono&display=swap" rel="stylesheet">
<style>
*,*::before,*::after{box-sizing:border-box;margin:0;padding:0}
:root{
  --bg:#040910;--panel:#060c18;--border:#0c2840;
  --cyan:#00cfff;--cyan-dim:#00344a;
  --amber:#ff6b00;--green:#00e87a;--red:#ff2244;
  --text:#b8d4e8;--dim:#3a5068;
  --font-ui:'Rajdhani',sans-serif;--font-mono:'Share Tech Mono',monospace;
}
html,body{height:100%;background:var(--bg);color:var(--text);font-family:var(--font-ui);font-size:15px;overflow-x:hidden}
body::after{content:'';position:fixed;inset:0;background:repeating-linear-gradient(0deg,transparent,transparent 3px,rgba(0,0,0,0.025) 3px,rgba(0,0,0,0.025) 4px);pointer-events:none;z-index:9999}
::-webkit-scrollbar{width:4px}::-webkit-scrollbar-track{background:transparent}::-webkit-scrollbar-thumb{background:var(--border);border-radius:2px}

/* ─── TOP BAR ─── */
.topbar{display:flex;align-items:center;gap:12px;padding:8px 18px;border-bottom:1px solid var(--border);background:linear-gradient(90deg,#060c18,#030609);position:sticky;top:0;z-index:100;flex-wrap:wrap}
.logo{font-size:1.05rem;font-weight:700;letter-spacing:3px;color:var(--cyan);text-transform:uppercase;text-shadow:0 0 24px rgba(0,207,255,.4);white-space:nowrap}
.logo span{color:var(--text);font-weight:400}
.topbar-spacer{flex:1}
.status-pill{display:flex;align-items:center;gap:6px;padding:3px 10px;border:1px solid var(--border);border-radius:3px;font-family:var(--font-mono);font-size:.76rem;white-space:nowrap}
.pulse-dot{width:7px;height:7px;border-radius:50%;background:var(--green);box-shadow:0 0 8px var(--green);animation:pdot 2s ease-in-out infinite;flex-shrink:0}
.pulse-dot.warn{background:var(--amber);box-shadow:0 0 8px var(--amber)}
.pulse-dot.err{background:var(--red);box-shadow:0 0 8px var(--red);animation:blink .5s step-end infinite}
@keyframes pdot{0%,100%{opacity:1}50%{opacity:.35}}
@keyframes blink{0%,100%{opacity:1}50%{opacity:0}}
/* .batt-wrap/.batt-fill/.batt-text removed — replaced by inline SVG battery */

/* ─── LAYOUT ─── */
.main{padding:14px 18px;display:grid;gap:12px;max-width:1700px;margin:0 auto}
.viz-row{display:grid;grid-template-columns:1fr 1fr 1fr;gap:12px;height:370px}
.ctrl-row{display:grid;grid-template-columns:1fr 1fr;gap:12px}
@media(max-width:1100px){.viz-row{grid-template-columns:1fr 1fr;height:auto}.viz-row .panel:last-child{grid-column:1/-1}}
@media(max-width:700px){.viz-row,.ctrl-row{grid-template-columns:1fr}.viz-row{height:auto}}

/* ─── PANELS ─── */
.panel{position:relative;background:var(--panel);border:1px solid var(--border);border-radius:2px;overflow:hidden;display:flex;flex-direction:column}
.panel::before{content:'';position:absolute;top:-1px;left:-1px;width:14px;height:14px;border-top:2px solid var(--cyan);border-left:2px solid var(--cyan);z-index:2;pointer-events:none}
.panel::after{content:'';position:absolute;bottom:-1px;right:-1px;width:14px;height:14px;border-bottom:2px solid var(--cyan);border-right:2px solid var(--cyan);z-index:2;pointer-events:none}
.panel-hd{display:flex;align-items:center;gap:8px;padding:6px 14px;border-bottom:1px solid var(--border);background:rgba(0,20,40,.5);flex-shrink:0}
.panel-title{font-size:.7rem;font-weight:700;letter-spacing:2px;text-transform:uppercase;color:var(--cyan)}
.panel-sub{font-size:.67rem;color:var(--dim);font-family:var(--font-mono);margin-left:auto}
.panel-body{flex:1;overflow:hidden;position:relative}

/* ─── 3D VIEWER ─── */
#robot-canvas{width:100%!important;height:100%!important;display:block;cursor:grab}
#robot-canvas:active{cursor:grabbing}
.pc-overlay{position:absolute;bottom:7px;left:10px;font-family:var(--font-mono);font-size:.63rem;color:var(--dim);pointer-events:none;line-height:1.6}

/* ─── CAMERA ─── */
.cam-feed{width:100%;height:100%;object-fit:contain;background:#020508;display:block}
.cam-dot{width:7px;height:7px;border-radius:50%;background:var(--dim);transition:background .5s,box-shadow .5s;flex-shrink:0}
.cam-dot.live{background:var(--green);box-shadow:0 0 8px var(--green)}

/* ─── MAP ─── */
#map-canvas{width:100%;height:100%;display:block}

/* ─── STATUS ─── */
.data-grid{display:grid;grid-template-columns:1fr 1fr;gap:7px;padding:10px 12px}
.data-cell{display:flex;flex-direction:column;gap:2px}
.data-label{font-size:.62rem;color:var(--dim);letter-spacing:1.5px;text-transform:uppercase}
.data-value{font-family:var(--font-mono);font-size:.88rem;color:var(--cyan)}
.data-value.ok{color:var(--green)}.data-value.warn{color:var(--amber)}.data-value.err{color:var(--red)}
.status-readout{padding:8px 12px;font-family:var(--font-mono);font-size:.78rem;line-height:1.6;color:var(--text);border-top:1px solid var(--border);flex:1;overflow-y:auto}
.alarm-bar{margin:0 12px 10px;padding:7px 11px;background:rgba(255,34,68,.08);border:1px solid rgba(255,34,68,.35);border-radius:2px;font-size:.78rem;color:var(--red);display:none}
.alarm-bar.on{display:block}

/* ─── COMMAND ─── */
.cmd-section{padding:11px 12px;display:flex;flex-direction:column;gap:9px;flex:1}
.cmd-row{display:flex;gap:6px}
.cmd-input{flex:1;background:#030710;border:1px solid var(--cyan-dim);border-radius:3px;padding:8px 11px;color:var(--text);font-family:var(--font-mono);font-size:.83rem;outline:none;transition:border-color .2s}
.cmd-input:focus{border-color:var(--cyan)}
.cmd-input::placeholder{color:var(--dim)}
.btn-sm{padding:0 13px;background:var(--cyan-dim);color:var(--cyan);border:1px solid var(--cyan-dim);border-radius:3px;font-family:var(--font-ui);font-weight:600;font-size:.78rem;letter-spacing:1px;cursor:pointer;transition:background .15s,color .15s;white-space:nowrap}
.btn-sm:hover{background:var(--cyan);color:#000}
.mic-btn{padding:0 11px;background:#0a1428;color:var(--text);border:1px solid var(--border);border-radius:3px;font-size:1.05rem;cursor:pointer;transition:background .2s}
.mic-btn.listening{background:rgba(255,34,68,.2);border-color:var(--red);animation:pdot 1s infinite}
.voice-hint{font-family:var(--font-mono);font-size:.68rem;color:var(--dim);min-height:1.1em}
.intent-row{font-family:var(--font-mono);font-size:.71rem;color:#6a84a0;line-height:1.5;border-top:1px solid var(--border);padding-top:8px;display:none}
.intent-row.on{display:block}

/* ─── QUEUE ─── */
.queue-section{padding:11px 12px;flex:1;display:flex;flex-direction:column;gap:7px}
.queue-active{padding:6px 9px;background:rgba(0,207,255,.04);border:1px solid var(--cyan-dim);border-radius:2px;font-size:.78rem;font-family:var(--font-mono)}
.queue-list{list-style:none;display:flex;flex-direction:column;gap:4px;max-height:90px;overflow-y:auto}
.queue-list li{padding:4px 9px;background:rgba(255,255,255,.025);border-left:2px solid var(--border);font-size:.76rem;font-family:var(--font-mono)}
.queue-paused{color:var(--amber);font-size:.73rem;font-family:var(--font-mono);display:none}
.queue-paused.on{display:block}
.btn-clear{width:100%;padding:6px;background:rgba(80,20,80,.25);color:#c080d0;border:1px solid #3a1050;border-radius:3px;font-family:var(--font-ui);font-weight:600;font-size:.76rem;letter-spacing:1px;cursor:pointer;transition:background .15s;margin-top:auto}
.btn-clear:hover{background:rgba(130,30,130,.35)}

/* ─── MISSION BUTTONS ─── */
.mission-row{padding:11px 12px;display:grid;grid-template-columns:1fr 1fr 1fr 1fr 1.8fr;gap:9px}
@media(max-width:900px){.mission-row{grid-template-columns:1fr 1fr}.btn-estop{grid-column:1/-1}}
.mbtn{padding:14px 7px;border:1px solid;border-radius:2px;font-family:var(--font-ui);font-weight:700;font-size:.82rem;letter-spacing:.5px;text-transform:uppercase;cursor:pointer;transition:filter .15s,transform .1s;display:flex;flex-direction:column;align-items:center;gap:3px;line-height:1.3;width:100%}
.mbtn:hover{filter:brightness(1.3)}.mbtn:active{transform:scale(.97)}
.mbtn .sub{font-size:.6rem;opacity:.65;font-weight:400;letter-spacing:0}
.btn-g{background:rgba(0,70,35,.35);border-color:#004a25;color:var(--green)}
.btn-o{background:rgba(70,35,0,.35);border-color:#4a2500;color:var(--amber)}
.btn-b{background:rgba(0,35,70,.35);border-color:#002a50;color:#60b0ff}
.btn-p{background:rgba(55,20,75,.35);border-color:#3a1055;color:#c080ff}
.btn-estop{background:rgba(130,0,18,.25);border-color:rgba(255,34,68,.4);color:var(--red);letter-spacing:1.5px;font-size:.85rem;box-shadow:0 0 18px rgba(255,34,68,.08)}
.btn-estop:hover{background:rgba(180,0,25,.4);box-shadow:0 0 28px rgba(255,34,68,.25)}

/* ─── REPORT ─── */
.report-body{padding:13px}
.report-hd{display:flex;align-items:center;justify-content:space-between;margin-bottom:10px}
.report-badge{padding:2px 9px;border-radius:999px;font-family:var(--font-mono);font-size:.66rem;letter-spacing:1px;background:var(--border);color:var(--dim)}
.report-badge.running{background:rgba(0,55,20,.5);color:var(--green)}
.report-badge.done{background:rgba(0,60,20,.8);color:var(--green)}
.report-badge.done_no_llm{background:rgba(75,55,0,.5);color:var(--amber)}
.report-badge.error{background:rgba(75,0,10,.5);color:var(--red)}
.report-msg{font-family:var(--font-mono);font-size:.76rem;color:var(--dim);margin-bottom:9px}
.report-links a{color:var(--cyan);font-family:var(--font-mono);font-size:.73rem;text-decoration:none;margin-right:14px}
.report-links a:hover{text-decoration:underline}
.report-map img{width:100%;border-radius:3px;margin:9px 0}
.report-empty{font-family:var(--font-mono);font-size:.78rem;color:var(--dim);font-style:italic}
.md{font-size:.82rem;line-height:1.6;color:var(--text)}
.md h1,.md h2,.md h3{color:var(--cyan);margin:9px 0 4px}
.md h1{font-size:.98rem;border-bottom:1px solid var(--border);padding-bottom:4px}
.md h2{font-size:.9rem}.md h3{font-size:.83rem}
.md p{margin:5px 0}.md ul,.md ol{margin:5px 0 5px 20px}.md li{margin:2px 0}
.md code{background:#0a1020;padding:1px 4px;border-radius:3px;font-family:var(--font-mono);font-size:.78rem}
.md table{border-collapse:collapse;margin:7px 0;font-size:.76rem;width:100%}
.md th,.md td{border:1px solid var(--border);padding:3px 7px}
.md th{background:#0a1828;color:var(--cyan)}.md strong{color:#fff}
.md details{margin:9px 0;background:#07101e;border:1px solid var(--border);border-radius:3px;padding:7px 11px}
.md details summary{cursor:pointer;color:var(--cyan);font-weight:700}
</style>
</head>
<body>

<header class="topbar">
  <div class="logo">WTIS<span> / Wind Tower Inspection System</span></div>
  <div class="topbar-spacer"></div>
  <div class="status-pill"><div id="sys-dot" class="pulse-dot"></div><span id="sys-label" style="color:var(--green)">ONLINE</span></div>
  <div class="status-pill" style="gap:5px">
    <!-- Inline SVG battery icon with animated fill level -->
    <svg id="batt-svg" width="38" height="18" viewBox="0 0 38 18" fill="none" xmlns="http://www.w3.org/2000/svg" style="flex-shrink:0">
      <!-- Body outline -->
      <rect x="1" y="3" width="32" height="12" rx="2" ry="2" stroke="var(--border)" stroke-width="1.5" fill="#020608"/>
      <!-- Positive terminal nub -->
      <rect x="33" y="7" width="4" height="4" rx="1" fill="var(--border)"/>
      <!-- Fill bar (width driven by JS, 0-30px range) -->
      <rect id="batt-fill-rect" x="2.5" y="4.5" width="0" height="9" rx="1" fill="var(--green)" style="transition:width .6s,fill .6s"/>
    </svg>
    <span style="font-family:var(--font-mono);font-size:.76rem"><span id="batt-pct">{{ s.battery }}</span>%</span>
  </div>
  <div class="status-pill" style="max-width:340px;overflow:hidden">
    <span id="topbar-status" style="font-family:var(--font-mono);font-size:.72rem;overflow:hidden;text-overflow:ellipsis;white-space:nowrap">{{ s.status }}</span>
  </div>
</header>

<div class="main">

  <!-- ── VIZ ROW ── -->
  <div class="viz-row">

    <div class="panel">
      <div class="panel-hd">
        <span class="panel-title">3D Viewer</span>
        <span class="panel-sub">Husky A200 + UR5e + VLP-16</span>
      </div>
      <div class="panel-body">
        <canvas id="robot-canvas"></canvas>
        <div class="pc-overlay">LiDAR pts: <span id="pc-pts">—</span><br>Yaw: <span id="ov-yaw">—</span></div>
      </div>
    </div>

    <div class="panel">
      <div class="panel-hd">
        <div id="cam-dot" class="cam-dot"></div>
        <span class="panel-title">Camara Inspección</span>
        <a href="/api/snapshot" target="_blank" class="panel-sub" style="color:var(--dim);text-decoration:none" title="Snapshot">⬡</a>
      </div>
      <div class="panel-body">
        <img id="camera-feed" class="cam-feed" src="/video_feed" alt="Sin señal"
             onerror="this._t=setTimeout(()=>{this.src='/video_feed?t='+Date.now()},2000)">
      </div>
    </div>

    <div class="panel">
      <div class="panel-hd">
        <span class="panel-title">Mapa de Misión</span>
        <span id="map-pose" class="panel-sub">x:— y:—</span>
      </div>
      <div class="panel-body">
        <canvas id="map-canvas"></canvas>
      </div>
    </div>

  </div>

  <!-- ── CTRL ROW ── -->
  <div class="ctrl-row">

    <div class="panel" style="min-height:200px">
      <div class="panel-hd">
        <span class="panel-title">Estado del Sistema</span>
        <span id="dest-badge" class="panel-sub">idle</span>
      </div>
      <div class="data-grid">
        <div class="data-cell"><span class="data-label">Pose X</span><span class="data-value" id="pose-x">—</span></div>
        <div class="data-cell"><span class="data-label">Pose Y</span><span class="data-value" id="pose-y">—</span></div>
        <div class="data-cell"><span class="data-label">IMU Roll</span><span class="data-value" id="imu-roll">—</span></div>
        <div class="data-cell"><span class="data-label">IMU Pitch</span><span class="data-value" id="imu-pitch">—</span></div>
        <div class="data-cell" style="grid-column:1/-1"><span class="data-label">Yaw</span><span class="data-value" id="pose-yaw">—</span></div>
      </div>
      <div id="alarm-bar" class="alarm-bar"></div>
      <div id="status-readout" class="status-readout">{{ s.status }}</div>
    </div>

    <div style="display:flex;flex-direction:column;gap:12px">

      <div class="panel" style="flex:0 0 auto">
        <div class="panel-hd"><span class="panel-title">Comando de Misión</span></div>
        <div class="cmd-section">
          <div class="cmd-row">
            <input id="cmd-input" class="cmd-input" type="text" placeholder="ve a mantenimiento…" autocomplete="off">
            <button id="mic-btn" class="mic-btn" type="button" title="Voz">🎙</button>
            <button id="cmd-send" class="btn-sm">ENVIAR</button>
          </div>
          <div id="voice-hint" class="voice-hint"></div>
          <div id="intent-row" class="intent-row"></div>
        </div>
      </div>

      <div class="panel" style="flex:1">
        <div class="panel-hd">
          <span class="panel-title">Cola de Misiones</span>
          <span id="queue-badge" class="panel-sub">0 pendientes</span>
        </div>
        <div class="queue-section">
          <div id="queue-active" class="queue-active">Sin misión activa</div>
          <ul id="queue-list" class="queue-list"></ul>
          <div id="queue-paused" class="queue-paused">⚠ Cola pausada por fallo</div>
          <form method="post" action="/clear_queue">
            <button class="btn-clear" type="submit">VACIAR COLA</button>
          </form>
        </div>
      </div>

    </div>
  </div>

  <!-- ── MISSION BUTTONS ── -->
  <div class="panel" style="padding:0">
    <div class="panel-hd"><span class="panel-title">Control de Misiones</span></div>
    <div class="mission-row">
      <form method="post" action="/cmd" style="display:contents">
        <button class="mbtn btn-g" name="cmd" value="charging_station">⚡ CARGA<span class="sub">Estación norte</span></button>
      </form>
      <form method="post" action="/cmd" style="display:contents">
        <button class="mbtn btn-o" name="cmd" value="maintenance_station"><svg width="15" height="15" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round" style="vertical-align:-2px;margin-right:3px"><path d="M14.7 6.3a1 1 0 0 0 0 1.4l1.6 1.6a1 1 0 0 0 1.4 0l3.77-3.77a6 6 0 0 1-7.94 7.94l-6.91 6.91a2.12 2.12 0 0 1-3-3l6.91-6.91a6 6 0 0 1 7.94-7.94l-3.76 3.76z"/></svg> MANT.<span class="sub">Via gate der.</span></button>
      </form>
      <form method="post" action="/cmd" style="display:contents">
        <button class="mbtn btn-b" name="cmd" value="home_inspection">⌂ HOME<span class="sub">Base rampa</span></button>
      </form>
      <form method="post" action="/cmd" style="display:contents">
        <button class="mbtn btn-p" name="cmd" value="start_inspection">🔍 INSP.<span class="sub">Tramo completo</span></button>
      </form>
      <form method="post" action="/cancel" style="display:contents">
        <button class="mbtn btn-estop" type="submit">⚠ E-STOP — CANCELAR</button>
      </form>
    </div>
  </div>

  <!-- ── INSPECTION REPORT ── -->
  <div class="panel">
    <div class="panel-hd">
      <span class="panel-title">Informe de Inspección</span>
      <span id="report-badge" class="report-badge panel-sub">SIN DATOS</span>
    </div>
    <div class="report-body">
      <div id="report-msg" class="report-msg"></div>
      <div id="report-links" class="report-links"></div>
      <div id="report-map" style="display:none"><img id="report-map-img" alt="Mapa defectos"></div>
      <div id="report-summary" class="md"></div>
      <div id="report-full" class="md"></div>
      <div id="report-empty" class="report-empty">Aún no hay informe. Se generará automáticamente al terminar una inspección.</div>
    </div>
  </div>

</div>

<script type="importmap">
{
  "imports": {
    "three": "https://cdn.jsdelivr.net/npm/three@0.150.1/build/three.module.js",
    "three/examples/jsm/": "https://cdn.jsdelivr.net/npm/three@0.150.1/examples/jsm/"
  }
}
</script>
<script type="module">
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
import { ColladaLoader } from 'three/examples/jsm/loaders/ColladaLoader.js';
import URDFLoader from 'https://cdn.jsdelivr.net/npm/urdf-loader@0.12.1/src/URDFLoader.js';

// ═══════════════════════════════════════════════════════
//  THREE.JS ROBOT VIEWER — carga URDF real, fallback simplificado
// ═══════════════════════════════════════════════════════
(function () {
  const canvas = document.getElementById('robot-canvas');
  const wrap = canvas.parentElement;

  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.setClearColor(0x040910);
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.3;

  const scene = new THREE.Scene();
  scene.fog = new THREE.Fog(0x040910, 14, 32);

  const cam = new THREE.PerspectiveCamera(44, 1, 0.01, 80);
  cam.position.set(3.2, 2.4, 3.2);

  const controls = new OrbitControls(cam, canvas);
  controls.enableDamping = true; controls.dampingFactor = 0.08;
  controls.minDistance = 0.8; controls.maxDistance = 10;
  controls.target.set(0, 0.25, 0);

  scene.add(new THREE.AmbientLight(0x0d1c2e, 1.5));
  const dir = new THREE.DirectionalLight(0x90c8ff, 2.8);
  dir.position.set(4,6,3); dir.castShadow = true;
  dir.shadow.mapSize.set(1024,1024); scene.add(dir);
  scene.add(new THREE.HemisphereLight(0x001224, 0x050a14, 0.9));
  const rim = new THREE.DirectionalLight(0x00cfff, 0.5);
  rim.position.set(-3,2,-3); scene.add(rim);

  scene.add(new THREE.GridHelper(22,22,0x0a2030,0x081525));
  const gnd = new THREE.Mesh(new THREE.PlaneGeometry(30,30),
    new THREE.MeshStandardMaterial({color:0x020609,roughness:1}));
  gnd.rotation.x = -Math.PI/2; gnd.receiveShadow = true; scene.add(gnd);

  // ── Modelo simplificado (fallback mientras carga el URDF) ──
  const M = {
    body: new THREE.MeshStandardMaterial({color:0x1a1a1a,roughness:.65,metalness:.45}),
    wheel:new THREE.MeshStandardMaterial({color:0x080808,roughness:.9,metalness:.1}),
    rim:  new THREE.MeshStandardMaterial({color:0x2a2a2a,roughness:.5,metalness:.65}),
    arm:  new THREE.MeshStandardMaterial({color:0xd4a520,roughness:.35,metalness:.75,emissive:0x1a0800,emissiveIntensity:.2}),
    sens: new THREE.MeshStandardMaterial({color:0x00cfff,roughness:.15,metalness:.85,emissive:0x004466,emissiveIntensity:.9}),
    dark: new THREE.MeshStandardMaterial({color:0x111111,roughness:.6,metalness:.5}),
    glow: new THREE.MeshStandardMaterial({color:0x00cfff,emissive:0x00cfff,emissiveIntensity:1,transparent:true,opacity:.55}),
  };
  const fallback = new THREE.Group(); scene.add(fallback);
  const body = new THREE.Mesh(new THREE.BoxGeometry(.99,.28,.57),M.body);
  body.position.y=.305; body.castShadow=true; fallback.add(body);
  const deck= new THREE.Mesh(new THREE.BoxGeometry(.88,.04,.50),M.dark);
  deck.position.y=.465; fallback.add(deck);
  const wheels=[]; [[.256,.165,.34],[.256,.165,-.34],[-.256,.165,.34],[-.256,.165,-.34]].forEach(([x,y,z])=>{
    const w=new THREE.Mesh(new THREE.CylinderGeometry(.165,.165,.09,24),M.wheel);
    w.rotation.z=Math.PI/2; w.position.set(x,y,z); w.castShadow=true; fallback.add(w); wheels.push(w);
    const r=new THREE.Mesh(new THREE.TorusGeometry(.12,.014,8,24),M.rim);
    r.position.set(x,y,z); r.rotation.x=Math.PI/2; fallback.add(r);
  });
  const armBase=new THREE.Group(); armBase.position.set(.15,.49,0); fallback.add(armBase);
  const link1=new THREE.Group(); armBase.add(link1);
  const l1m=new THREE.Mesh(new THREE.BoxGeometry(.06,.24,.06),M.arm); l1m.position.y=.12; link1.add(l1m);
  const link2=new THREE.Group(); link2.position.y=.24; link1.add(link2);
  const l2m=new THREE.Mesh(new THREE.BoxGeometry(.22,.05,.05),M.arm); l2m.position.x=.11; link2.add(l2m);
  const link3=new THREE.Group(); link3.position.x=.22; link2.add(link3);
  const l3m=new THREE.Mesh(new THREE.BoxGeometry(.18,.05,.05),M.arm);
  l3m.position.set(.09,0,0);
  link3.add(l3m);
  const vlpH=new THREE.Mesh(new THREE.CylinderGeometry(.035,.04,.06,20),M.sens);
  vlpH.position.set(-.25,.505,0); fallback.add(vlpH);
  const vlpR=new THREE.Mesh(new THREE.TorusGeometry(.035,.003,8,24),M.glow);
  vlpR.rotation.x=Math.PI/2; vlpR.position.set(-.25,.509,0); fallback.add(vlpR);

  // ── URDF real desde /api/robot_description ──
  let urdfRobot = null;
  let urdfLoaded = false;
  const badge = document.getElementById('pc-count');

  function tryLoadURDF() {
    fetch('/api/robot_description', {cache:'no-store'})
      .then(r => { if (!r.ok) throw new Error('no urdf'); return r.text(); })
      .then(urdfText => {
        const loader = new URDFLoader();
        // Resuelve package://pkg/path → /api/mesh/pkg/path
        loader.packages = pkg => `/api/mesh/${pkg}`;
        // Usa STLLoader y ColladaLoader para los meshes
        loader.loadMeshCb = (path, manager, onLoad) => {
          const ext = path.split('.').pop().toLowerCase();
          if (ext === 'stl') {
            new STLLoader(manager).load(path, geom => {
              const mesh = new THREE.Mesh(geom,
                new THREE.MeshStandardMaterial({color:0x888888,roughness:.6,metalness:.4}));
              onLoad(mesh);
            }, undefined, () => onLoad(null));
          } else if (ext === 'dae') {
            new ColladaLoader(manager).load(path, result => {
              onLoad(result.scene);
            }, undefined, () => onLoad(null));
          } else {
            onLoad(null);
          }
        };
        const robot = loader.parse(urdfText);
        robot.rotation.x = -Math.PI / 2; // Roll -90 deg: URDF upright vs point cloud frame
        robot.rotation.z = Math.PI;      // ROS → Three.js frame
        scene.add(robot);
        urdfRobot = robot;
        urdfLoaded = true;
        fallback.visible = false;
        if (badge) badge.textContent = 'URDF real cargado';
      })
      .catch(err => {
        console.warn('No se pudo cargar URDF real; se mantiene fallback 3D.', err);
        // Sigue mostrando fallback, reintenta en 5s
        setTimeout(tryLoadURDF, 5000);
      });
  }
  tryLoadURDF();

  // ── Point cloud ──
  const MAX_PC=800;
  const pcGeo=new THREE.BufferGeometry();
  const pcPos=new Float32Array(MAX_PC*3),pcCol=new Float32Array(MAX_PC*3);
  pcGeo.setAttribute('position',new THREE.BufferAttribute(pcPos,3));
  pcGeo.setAttribute('color',new THREE.BufferAttribute(pcCol,3));
  pcGeo.setDrawRange(0,0);
  const pcMesh=new THREE.Points(pcGeo,
    new THREE.PointsMaterial({size:.045,vertexColors:true,transparent:true,opacity:.75}));
  pcMesh.position.set(-.25,.505,0); scene.add(pcMesh);

  let isMoving=false, armT=0;

  window._updateRobot3D = function(s) {
    isMoving = !!s.moving;
    const yaw = s.robot_yaw || 0;
    // Keep the viewer in the Husky base frame: the robot body stays fixed and
    // only local sensor data, wheels and articulated parts update.
    document.getElementById('ov-yaw').textContent = (yaw*180/Math.PI).toFixed(1)+'°';
    // Actualiza joints del URDF si están disponibles (ruedas)
    if (urdfRobot && isMoving) {
      const speed = armT * 8;
      ['front_left_wheel','front_right_wheel','rear_left_wheel','rear_right_wheel'].forEach(n => {
        const j = urdfRobot.joints[n];
        if (j) j.setJointValue(speed);
      });
    }
  };

  window._updatePC = function(pts) {
    const n=Math.min(pts.length,MAX_PC);
    document.getElementById('pc-pts').textContent=pts.length;
    for(let i=0;i<n;i++){
      const[x,y,z]=pts[i];
      pcPos[i*3]=x;pcPos[i*3+1]=z;pcPos[i*3+2]=-y;
      const t=Math.max(0,Math.min(1,(z+.3)/2));
      pcCol[i*3]=t*.2;pcCol[i*3+1]=.45+t*.55;pcCol[i*3+2]=1;
    }
    pcGeo.attributes.position.needsUpdate=true;
    pcGeo.attributes.color.needsUpdate=true;
    pcGeo.setDrawRange(0,n);
  };

  function resize(){
    const w=wrap.clientWidth,h=wrap.clientHeight;
    cam.aspect=w/h; cam.updateProjectionMatrix(); renderer.setSize(w,h);
  }
  resize(); new ResizeObserver(resize).observe(wrap);

  let prev=performance.now();
  (function animate(){
    requestAnimationFrame(animate);
    const dt=(performance.now()-prev)/1000; prev=performance.now();
    armT+=dt;
    if(!urdfLoaded){
      if(isMoving)wheels.forEach(w=>{w.rotation.y+=dt*9;});
      link1.rotation.y=Math.sin(armT*.38)*.28;
      link2.rotation.z=-.4+Math.sin(armT*.52+1)*.13;
      link3.rotation.z=-.28+Math.sin(armT*.31+2)*.09;
    }
    vlpH.rotation.y+=dt*3.5; vlpR.rotation.y+=dt*3.5;
    controls.update(); renderer.render(scene,cam);
  })();
})();

// ═══════════════════════════════════════════════════════
//  2D MAP CANVAS
// ═══════════════════════════════════════════════════════
(function () {
  const canvas = document.getElementById('map-canvas');
  const ctx = canvas.getContext('2d');
  const W = {xMin:-16,xMax:16,yMin:-3,yMax:44};
  let rState = {x:0,y:0,yaw:0,moving:false,alarm:false,inspActive:false,inspIdx:0,inspCount:0};

  function resize() {
    const s = devicePixelRatio;
    canvas.width  = canvas.offsetWidth  * s;
    canvas.height = canvas.offsetHeight * s;
    ctx.setTransform(s,0,0,s,0,0);
  }
  new ResizeObserver(()=>{resize();draw();}).observe(canvas);
  resize();

  function toC(wx,wy) {
    const cw=canvas.offsetWidth, ch=canvas.offsetHeight;
    return [((wx-W.xMin)/(W.xMax-W.xMin))*cw, ch-((wy-W.yMin)/(W.yMax-W.yMin))*ch];
  }

  function draw() {
    const cw=canvas.offsetWidth, ch=canvas.offsetHeight;
    ctx.clearRect(0,0,cw,ch);
    ctx.fillStyle='#040910'; ctx.fillRect(0,0,cw,ch);

    // Grid 5m
    ctx.strokeStyle='#0a1c2c'; ctx.lineWidth=.5;
    for(let x=-15;x<=15;x+=5){const[cx]=toC(x,0);ctx.beginPath();ctx.moveTo(cx,0);ctx.lineTo(cx,ch);ctx.stroke();}
    for(let y=0;y<=40;y+=5){const[,cy]=toC(0,y);ctx.beginPath();ctx.moveTo(0,cy);ctx.lineTo(cw,cy);ctx.stroke();}

    // Tube zone
    ctx.fillStyle='rgba(0,100,80,.055)';ctx.strokeStyle='rgba(0,150,100,.12)';ctx.lineWidth=1;
    const[tx1,ty1]=toC(-2,6),[tx2,ty2]=toC(2.5,41);
    ctx.fillRect(tx1,ty2,tx2-tx1,ty1-ty2);ctx.strokeRect(tx1,ty2,tx2-tx1,ty1-ty2);

    // Inspection route
    const[isx,isy]=toC(.245,10.246),[iex,iey]=toC(.245,39.579);
    ctx.strokeStyle='#00cfff';ctx.lineWidth=2;ctx.setLineDash([5,4]);
    ctx.beginPath();ctx.moveTo(isx,isy);ctx.lineTo(iex,iey);ctx.stroke();
    ctx.setLineDash([]);

    // Inspection progress overlay
    if(rState.inspActive && rState.inspCount>0){
      const pct=rState.inspIdx/rState.inspCount;
      const py=10.246+(39.579-10.246)*pct;
      const[px,ppyC]=toC(.245,py);
      ctx.strokeStyle='rgba(0,232,122,.6)';ctx.lineWidth=3;
      ctx.beginPath();ctx.moveTo(isx,isy);ctx.lineTo(px,ppyC);ctx.stroke();
    }

    // Endpoint dots
    [[.245,10.246,'#00cfff','INICIO'],[.245,39.579,'#00cfff','FIN']].forEach(([wx,wy,c,l])=>{
      const[cx,cy]=toC(wx,wy);
      ctx.fillStyle=c;ctx.beginPath();ctx.arc(cx,cy,4,0,Math.PI*2);ctx.fill();
      ctx.font='9px "Share Tech Mono"';ctx.fillStyle=c;ctx.fillText(l,cx+6,cy+3);
    });

    // Gates
    [[12.015,5.035,'#ffcc00','GR'],[-12.078,5.698,'#ffcc00','GL'],[.245,8.5,'#ff9900','PG']].forEach(([wx,wy,c,l])=>{
      const[cx,cy]=toC(wx,wy);
      ctx.strokeStyle=c;ctx.lineWidth=1.5;
      ctx.beginPath();ctx.moveTo(cx-5,cy);ctx.lineTo(cx+5,cy);ctx.moveTo(cx,cy-5);ctx.lineTo(cx,cy+5);ctx.stroke();
      ctx.fillStyle=c;ctx.font='8px "Share Tech Mono"';ctx.fillText(l,cx+6,cy-2);
    });

    // Robot
    const[rx,ry]=toC(rState.x,rState.y);
    const col=rState.alarm?'#ff2244':(rState.moving?'#00e87a':'#00cfff');
    ctx.strokeStyle='rgba(0,207,255,.12)';ctx.lineWidth=1;
    ctx.beginPath();ctx.arc(rx,ry,15,0,Math.PI*2);ctx.stroke();
    ctx.save();ctx.translate(rx,ry);ctx.rotate(Math.PI/2-rState.yaw);
    ctx.fillStyle=col;ctx.shadowColor=col;ctx.shadowBlur=10;
    ctx.beginPath();ctx.moveTo(0,-10);ctx.lineTo(7,5);ctx.lineTo(0,1);ctx.lineTo(-7,5);ctx.closePath();ctx.fill();
    ctx.restore();
    ctx.fillStyle=col;ctx.font='bold 8px "Share Tech Mono"';
    ctx.fillText(`(${rState.x.toFixed(1)},${rState.y.toFixed(1)})`,rx+12,ry);

    // Scale bar
    const sp=5*(cw/(W.xMax-W.xMin));
    ctx.strokeStyle='rgba(180,180,180,.25)';ctx.lineWidth=1;
    ctx.beginPath();ctx.moveTo(12,ch-13);ctx.lineTo(12+sp,ch-13);ctx.stroke();
    ctx.fillStyle='rgba(180,180,180,.4)';ctx.font='8px "Share Tech Mono"';ctx.fillText('5m',14+sp,ch-9);
    ctx.fillStyle='rgba(0,207,255,.4)';ctx.fillText('N↑',cw-22,18);
  }

  window._updateMap = function(s) {
    rState={x:s.robot_x||0,y:s.robot_y||0,yaw:s.robot_yaw||0,moving:!!s.moving,alarm:!!s.alarm,
            inspActive:!!s.inspection_active,inspIdx:s.inspection_index||0,inspCount:s.inspection_count||0};
    document.getElementById('map-pose').textContent=`x:${rState.x.toFixed(2)} y:${rState.y.toFixed(2)}`;
    draw();
  };
  draw();
})();

// ═══════════════════════════════════════════════════════
//  STATE POLLING & UI
// ═══════════════════════════════════════════════════════
function esc(v){return String(v??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));}
function battCol(v){return v>50?'#00e87a':v>20?'#ff6b00':'#ff2244';}

const RL={idle:'SIN DATOS',running:'GENERANDO',done:'COMPLETO',done_no_llm:'PARCIAL',error:'ERROR'};

function renderMd(src){
  if(!src)return'';
  const lines=String(src).replace(/\r\n?/g,'\n').split('\n'),out=[];let i=0;
  const fmt=s=>esc(s).replace(/`([^`]+)`/g,'<code>$1</code>').replace(/\*\*([^*]+)\*\*/g,'<strong>$1</strong>').replace(/(^|[\s(])\*([^*\n]+)\*/g,'$1<em>$2</em>');
  while(i<lines.length){
    const l=lines[i];
    if(!l.trim()){i++;continue;}
    const h=l.match(/^(#{1,6})\s+(.*)/);
    if(h){const lv=Math.min(h[1].length,3);out.push(`<h${lv}>${fmt(h[2])}</h${lv}>`);i++;continue;}
    if(l.includes('|')&&i+1<lines.length&&/^\s*\|?[-: |]+\|/.test(lines[i+1])){
      const sr=r=>r.replace(/^\s*\|/,'').replace(/\|\s*$/,'').split('|').map(c=>c.trim());
      const hdrs=sr(l);i+=2;const rows=[];
      while(i<lines.length&&lines[i].includes('|'))rows.push(sr(lines[i++]));
      out.push(`<table><thead><tr>${hdrs.map(c=>`<th>${fmt(c)}</th>`).join('')}</tr></thead><tbody>${rows.map(r=>`<tr>${r.map(c=>`<td>${fmt(c)}</td>`).join('')}</tr>`).join('')}</tbody></table>`);
      continue;
    }
    if(/^\s*[-*]\s+/.test(l)){const it=[];while(i<lines.length&&/^\s*[-*]\s+/.test(lines[i]))it.push(`<li>${fmt(lines[i++].replace(/^\s*[-*]\s+/,''))}</li>`);out.push(`<ul>${it.join('')}</ul>`);continue;}
    if(/^\s*\d+\.\s+/.test(l)){const it=[];while(i<lines.length&&/^\s*\d+\.\s+/.test(lines[i]))it.push(`<li>${fmt(lines[i++].replace(/^\s*\d+\.\s+/,''))}</li>`);out.push(`<ol>${it.join('')}</ol>`);continue;}
    const p=[];while(i<lines.length&&lines[i].trim()&&!/^[#\-*\d]/.test(lines[i])&&!lines[i].includes('|'))p.push(lines[i++]);
    if(p.length)out.push(`<p>${fmt(p.join(' '))}</p>`);
  }
  return out.join('\n');
}

function renderState(s) {
  const pct=Number(s.battery||0);
  // Update inline SVG battery: fill rect max width = 29px (body interior 30px minus 1px margin)
  const fillW=Math.round(Math.max(0,Math.min(100,pct))*0.29);
  const fillEl=document.getElementById('batt-fill-rect');
  if(fillEl){fillEl.setAttribute('width',fillW);fillEl.setAttribute('fill',battCol(pct));}
  document.getElementById('batt-pct').textContent=pct.toFixed(1);
  document.getElementById('topbar-status').textContent=s.status||'';

  const dot=document.getElementById('sys-dot'),lbl=document.getElementById('sys-label');
  if(s.alarm){dot.className='pulse-dot err';lbl.style.color='var(--red)';lbl.textContent='ALARM';}
  else if(s.moving||s.inspection_active){dot.className='pulse-dot';lbl.style.color='var(--green)';lbl.textContent='ACTIVE';}
  else if(s.charging){dot.className='pulse-dot warn';lbl.style.color='var(--amber)';lbl.textContent='CHARGING';}
  else{dot.className='pulse-dot';lbl.style.color='var(--green)';lbl.textContent='ONLINE';}

  document.getElementById('dest-badge').textContent=s.destination||'idle';
  document.getElementById('status-readout').textContent=s.status||'';
  document.getElementById('pose-x').textContent=s.robot_x!=null?s.robot_x.toFixed(3)+' m':'—';
  document.getElementById('pose-y').textContent=s.robot_y!=null?s.robot_y.toFixed(3)+' m':'—';
  document.getElementById('pose-yaw').textContent=s.robot_yaw!=null?((s.robot_yaw*180/Math.PI).toFixed(1))+'°':'—';
  document.getElementById('imu-roll').textContent=s.imu_roll_deg!=null?s.imu_roll_deg.toFixed(1)+'°':'—';
  document.getElementById('imu-pitch').textContent=s.imu_pitch_deg!=null?s.imu_pitch_deg.toFixed(1)+'°':'—';

  const ab=document.getElementById('alarm-bar');
  if(s.alarm){ab.textContent=s.alarm;ab.classList.add('on');}else{ab.classList.remove('on');}

  const qa=document.getElementById('queue-active');
  qa.textContent=s.queue_active?('▶ '+(s.queue_active.label||s.queue_active.command)):'Sin misión activa';
  const ql=document.getElementById('queue-list');
  ql.innerHTML='';
  (s.queue_pending||[]).forEach((it,i)=>{
    const li=document.createElement('li');
    li.textContent=(i+1)+'. '+(it.label||it.command)+(it.source_text?` — "${it.source_text}"`:'')+
      (it.confidence<1?' [conf:'+it.confidence+']':'');
    ql.appendChild(li);
  });
  document.getElementById('queue-badge').textContent=(s.queue_pending||[]).length+' pendientes';
  document.getElementById('queue-paused').classList.toggle('on',!!s.queue_paused);

  const ir=document.getElementById('intent-row');
  if(s.last_text_command){
    ir.classList.add('on');
    ir.innerHTML=`"${esc(s.last_text_command)}" → <strong style="color:var(--cyan)">${esc(s.last_llm_command)}</strong> (${esc(s.last_llm_source)}, conf ${s.last_llm_confidence})<br><span style="color:var(--dim)">${esc(s.last_llm_reason)}</span>`;
  }else{ir.classList.remove('on');}

  if(window._updateRobot3D)window._updateRobot3D(s);
  if(window._updateMap)window._updateMap(s);
  renderReport(s.report);
}

function renderReport(r){
  r=r||{};
  const status=r.status||'idle';
  const badge=document.getElementById('report-badge');
  badge.className='report-badge panel-sub '+status;
  badge.textContent=RL[status]||status;
  document.getElementById('report-msg').textContent=r.message||'';
  const links=[];
  if(r.run_url)links.push(`<a href="${esc(r.run_url)}" target="_blank">Carpeta run</a>`);
  if(r.defect_map_url)links.push(`<a href="${esc(r.defect_map_url)}" target="_blank">Mapa defectos</a>`);
  document.getElementById('report-links').innerHTML=links.join(' ');
  const mw=document.getElementById('report-map'),mi=document.getElementById('report-map-img');
  if(r.defect_map_url){mi.src=r.defect_map_url+'?t='+encodeURIComponent(r.run_id||'');mw.style.display='block';}else{mw.style.display='none';}
  document.getElementById('report-summary').innerHTML=r.summary_md?renderMd(r.summary_md):'';
  document.getElementById('report-full').innerHTML=r.report_md?'<details><summary>Informe completo (LLM)</summary>'+renderMd(r.report_md)+'</details>':'';
  document.getElementById('report-empty').style.display=status==='idle'?'block':'none';
}

async function pollState(){
  try{const r=await fetch('/api/state',{cache:'no-store'});if(r.ok)renderState(await r.json());}catch(e){}
}
async function pollPC(){
  try{const r=await fetch('/api/pointcloud',{cache:'no-store'});if(r.ok&&window._updatePC)window._updatePC(await r.json());}catch(e){}
}

pollState();setInterval(pollState,1000);
setInterval(pollPC,250);

// Camera dot
(function(){
  const dot=document.getElementById('cam-dot');
  function chk(){fetch('/api/snapshot',{method:'HEAD',cache:'no-store'}).then(r=>dot.classList.toggle('live',r.ok)).catch(()=>dot.classList.remove('live'));}
  chk();setInterval(chk,3000);
})();

// Command
document.getElementById('cmd-send').addEventListener('click',async()=>{
  const inp=document.getElementById('cmd-input'),text=inp.value.trim();
  if(!text)return;
  const fd=new FormData();fd.append('text',text);
  const r=await fetch('/text_cmd',{method:'POST',body:fd});
  if(r.ok){inp.value='';pollState();}
});
document.getElementById('cmd-input').addEventListener('keydown',e=>{if(e.key==='Enter')document.getElementById('cmd-send').click();});

// Web Speech API
(function(){
  const mb=document.getElementById('mic-btn'),inp=document.getElementById('cmd-input'),hint=document.getElementById('voice-hint');
  const SR=window.SpeechRecognition||window.webkitSpeechRecognition;
  if(!SR){mb.style.opacity='.3';mb.style.cursor='not-allowed';return;}
  const rec=new SR();rec.lang='es-ES';rec.interimResults=true;rec.continuous=false;
  let active=false;
  mb.addEventListener('click',()=>active?rec.stop():(inp.value='',hint.textContent='Escuchando…',rec.start()));
  rec.onstart=()=>{active=true;mb.classList.add('listening');};
  rec.onend=()=>{active=false;mb.classList.remove('listening');if(!inp.value.trim())hint.textContent='';};
  rec.onerror=ev=>{active=false;mb.classList.remove('listening');hint.textContent=ev.error==='not-allowed'?'Micrófono denegado':`Error: ${ev.error}`;};
  rec.onresult=ev=>{
    const res=ev.results[ev.results.length-1],text=res[0].transcript.trim();
    inp.value=text;hint.textContent=res.isFinal?`✓ "${text}"`:` "${text}"`;
    if(res.isFinal&&text){
      const fd=new FormData();fd.append('text',text);
      fetch('/text_cmd',{method:'POST',body:fd}).then(r=>r.ok?pollState():null);
      inp.value='';
    }
  };
})();
</script>
</body>
</html>'''


def _flask_thread(node: MissionController):
    app = Flask(__name__)

    @app.route('/')
    def index():
        return render_template_string(_HTML, s=node.state())

    @app.route('/cmd', methods=['POST'])
    def cmd():
        node.enqueue_command(request.form.get('cmd', ''), source='web_button')
        return redirect('/')

    @app.route('/text_cmd', methods=['POST'])
    def text_cmd():
        result = node.send_text_command(request.form.get('text', ''))
        wants_json = (
            request.headers.get('X-Requested-With') == 'fetch'
            or 'application/json' in request.headers.get('Accept', '')
        )
        if wants_json:
            return jsonify(result)
        return redirect('/')

    @app.route('/cancel', methods=['POST'])
    def cancel():
        node.cancel()
        return redirect('/')

    @app.route('/clear_queue', methods=['POST'])
    def clear_queue():
        node.clear_queue()
        return redirect('/')

    @app.route('/api/state')
    def api_state():
        return jsonify(node.state())

    @app.route('/api/text_command', methods=['POST'])
    def api_text_command():
        payload = request.get_json(silent=True) or {}
        text = payload.get('text') or request.form.get('text', '')
        return jsonify(node.send_text_command(text))

    @app.route('/api/clear_queue', methods=['POST'])
    def api_clear_queue():
        return jsonify({'removed': node.clear_queue(), 'queue': node.queue_state()})

    @app.route('/api/pointcloud')
    def api_pointcloud():
        return jsonify(node.get_pointcloud())

    @app.route('/api/robot_description')
    def api_robot_description():
        urdf = node.get_robot_description()
        if not urdf:
            abort(503)
        return Response(urdf, mimetype='application/xml')

    @app.route('/api/mesh/<package>/<path:rest>')
    def api_mesh(package: str, rest: str):
        import os as _os
        try:
            import ament_index_python.packages as _aip
            share = _aip.get_package_share_directory(package)
            target = _os.path.realpath(_os.path.join(share, rest))
            if not target.startswith(_os.path.realpath(share)):
                abort(403)
            if not _os.path.isfile(target):
                abort(404)
            return send_from_directory(_os.path.dirname(target), _os.path.basename(target))
        except Exception:
            abort(404)

    def _mjpeg_generator():
        """Genera un stream MJPEG con el ultimo frame de la camara."""
        _placeholder = None  # frame gris de 320x240 cuando no hay imagen
        while True:
            jpeg = node.get_camera_jpeg()
            if jpeg is None:
                # Genera placeholder gris si aun no hay frames
                if _placeholder is None:
                    try:
                        MissionController._ensure_cv2_path()
                        import cv2
                        import numpy as np
                        grey = np.full((240, 320, 3), 60, dtype=np.uint8)
                        cv2.putText(grey, 'Sin senal de camara', (20, 120),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (180, 180, 180), 1)
                        _, buf = cv2.imencode('.jpg', grey)
                        _placeholder = buf.tobytes()
                    except Exception:
                        _placeholder = b''
                jpeg = _placeholder or b''
            if jpeg:
                yield (
                    b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' + jpeg + b'\r\n'
                )
            threading.Event().wait(0.04)   # ~25 fps max

    @app.route('/video_feed')
    def video_feed():
        return Response(
            _mjpeg_generator(),
            mimetype='multipart/x-mixed-replace; boundary=frame',
        )

    @app.route('/api/snapshot')
    def api_snapshot():
        jpeg = node.get_camera_jpeg()
        if jpeg is None:
            abort(503)
        return Response(jpeg, mimetype='image/jpeg')

    @app.route('/inspections/<path:rel>')
    def serve_inspection_file(rel: str):
        root = INSPECTIONS_ROOT.resolve()
        try:
            target = (root / rel).resolve()
        except OSError:
            abort(404)
        if root not in target.parents and target != root:
            abort(404)
        if not target.is_file():
            abort(404)
        return send_from_directory(str(target.parent), target.name)

    import logging
    log = logging.getLogger('werkzeug')
    log.setLevel(logging.ERROR)
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)


# ── Entrypoint ─────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = MissionController()

    threading.Thread(target=_flask_thread, args=(node,), daemon=True).start()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
