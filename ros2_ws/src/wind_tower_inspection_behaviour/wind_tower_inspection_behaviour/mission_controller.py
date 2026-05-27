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
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker, MarkerArray

from flask import (
    Flask,
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
INSP_MAX_LATERAL_ERROR = 1.2   # m — tubo 4 m ancho, Husky 0.67 m; margen real para nav entre puntos
INSP_MAX_ROLL_DEG = 8.0        # gravedad vertical: roll casi plano en calle axial
INSP_MAX_PITCH_DEG = 20.0      # margen para transitorios, rampa y frenadas
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
        self._inspection_returning = False
        self._inspection_pause_timer = None
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
        text = state if not detail else f'{state}: {detail}'
        self._state_text_pub.publish(String(data=text))
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
            self._pose(ramp.get('x', 0.160), ramp.get('y', 6.901), math.pi / 2),
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
        with self._lock:
            if not self._inspection_active:
                return
            point_no = self._inspection_index + 1
            total = len(self._inspection_points)
            x, y = self._inspection_points[self._inspection_index]
            self._moving = False
            self._inspection_paused = True
            self._status = (
                f'INSPECTING punto {point_no}/{total} '
                f'(x={x:.2f}, y={y:.2f}) durante {INSP_PAUSE_SEC:.0f}s'
            )
            if self._inspection_pause_timer is not None:
                self._inspection_pause_timer.cancel()
            self._inspection_pause_timer = self.create_timer(
                INSP_PAUSE_SEC,
                self._finish_inspection_pause,
            )
        self._publish_inspection_state(
            'INSPECTING',
            f'punto {point_no}/{total}; parada {INSP_PAUSE_SEC:.0f}s',
        )

    def _finish_inspection_pause(self):
        with self._lock:
            if self._inspection_pause_timer is not None:
                self._inspection_pause_timer.cancel()
                self._inspection_pause_timer = None
            if not self._inspection_active:
                return
            self._inspection_index += 1
            self._inspection_paused = False
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
                return

            self._inspection_guard_tripped = True
            self._status = f'Guardia inspeccion: cancelando por {reason}'

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
                'stuck_recovery_active': self._stuck_recovery_active,
                'relocalization_active': self._relocalization_active,
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
<title>Wind Tower Mission</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Segoe UI',sans-serif;background:#0d0d1a;color:#ddd;padding:24px;max-width:720px;margin:auto}
h1{color:#00d4ff;margin-bottom:18px;font-size:1.25rem;letter-spacing:1px}
.bar-wrap{background:#222;border-radius:12px;overflow:hidden;height:34px;margin-bottom:14px;border:1px solid #333}
.bar-fill{height:100%;display:flex;align-items:center;justify-content:center;font-weight:bold;font-size:.9rem;transition:width .6s}
.status{background:#131326;border:1px solid #1e2d5a;border-radius:8px;padding:12px 14px;margin-bottom:18px;font-size:.88rem;line-height:1.7}
.alarm{background:#3a1010;border:1px solid #d63b3b;color:#fff;border-radius:8px;padding:12px 14px;margin-bottom:18px;font-size:.9rem;line-height:1.5}
.text-box{background:#101020;border:1px solid #26366d;border-radius:8px;padding:12px;margin-bottom:18px}
.text-row{display:flex;gap:8px}
.text-row input{flex:1;min-width:0;background:#080812;color:#fff;border:1px solid #36427a;border-radius:8px;padding:12px;font-size:.95rem}
.text-row button{background:#0c7a78;color:#fff;border:0;border-radius:8px;padding:0 14px;font-weight:bold;cursor:pointer}
.text-row .mic-btn{background:#1a3a6b;color:#fff;border:0;border-radius:8px;padding:0 13px;font-size:1.2rem;cursor:pointer;transition:background .2s}
.text-row .mic-btn.listening{background:#8b0000;animation:pulse 1s infinite}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.6}}
.voice-status{margin-top:6px;font-size:.76rem;color:#7fc4ff;min-height:1.2em}
.intent{margin-top:8px;font-size:.78rem;color:#a9b7ff;line-height:1.4}
.queue{background:#11111f;border:1px solid #2a3158;border-radius:8px;padding:12px;margin-bottom:18px;font-size:.86rem}
.queue h2{font-size:.95rem;color:#b9d7ff;margin-bottom:8px}
.queue ol{padding-left:22px;line-height:1.7}
.queue .empty{color:#7f89a8}
.queue .paused{color:#ffbe64;margin-top:6px;font-weight:bold}
.queue-clear{margin-top:10px;width:100%;background:#4b234f;color:#fff;border:0;border-radius:8px;padding:10px;font-weight:bold;cursor:pointer}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.btn{width:100%;padding:22px 8px;font-size:.95rem;font-weight:bold;border:none;border-radius:10px;
     cursor:pointer;transition:filter .15s,transform .1s;line-height:1.4}
.btn:hover{filter:brightness(1.2)}
.btn:active{transform:scale(.96)}
.g{background:#1a6b30;color:#fff}
.o{background:#8a3800;color:#fff}
.b{background:#144d7a;color:#fff}
.p{background:#5b2282;color:#fff}
.r{background:#8b0000;color:#fff;grid-column:1/-1;font-size:1.05rem;letter-spacing:1px}
.sub{display:block;font-size:.7rem;opacity:.7;font-weight:normal;margin-top:4px}
.ok{color:#2ecc71}
.report{margin-top:22px;background:#0f0f1e;border:1px solid #2a3158;border-radius:10px;padding:14px}
.report-head{display:flex;align-items:baseline;justify-content:space-between;gap:8px;margin-bottom:10px}
.report-head h2{font-size:1rem;color:#b9d7ff;letter-spacing:.5px}
.report-status{font-size:.78rem;padding:3px 8px;border-radius:999px;background:#1a3a6b;color:#cfe1ff}
.report-status.running{background:#3a5a1a}
.report-status.done{background:#1a6b30}
.report-status.done_no_llm{background:#8a6b00}
.report-status.error{background:#7b1a1a}
.report-message{font-size:.82rem;color:#a9b7ff;margin-bottom:10px}
.report-links{font-size:.78rem;margin-bottom:10px}
.report-links a{color:#7fc4ff;text-decoration:none;margin-right:12px}
.report-links a:hover{text-decoration:underline}
.report-map{margin:10px 0}
.report-map img{width:100%;border-radius:6px;background:#fff}
.md{font-size:.86rem;line-height:1.55;color:#dfe5f7}
.md h1,.md h2,.md h3{color:#cfe1ff;margin:10px 0 6px;line-height:1.3}
.md h1{font-size:1.05rem;border-bottom:1px solid #2a3158;padding-bottom:4px}
.md h2{font-size:.98rem}
.md h3{font-size:.9rem}
.md p{margin:6px 0}
.md ul,.md ol{margin:6px 0 6px 22px}
.md li{margin:2px 0}
.md code{background:#1b1b2e;padding:1px 5px;border-radius:4px;font-size:.82rem}
.md table{border-collapse:collapse;margin:8px 0;font-size:.78rem;width:100%}
.md th,.md td{border:1px solid #2a3158;padding:4px 6px;text-align:left}
.md th{background:#1a2245;color:#cfe1ff}
.md strong{color:#fff}
.md details{margin:10px 0;background:#11121f;border:1px solid #2a3158;border-radius:6px;padding:8px 10px}
.md details summary{cursor:pointer;color:#cfe1ff;font-weight:bold}
.report-empty{font-size:.82rem;color:#7f89a8;font-style:italic}
</style>
</head>
<body>
<h1>Wind Tower — Mission Control</h1>

<div class="bar-wrap">
  <div id="battery-fill" class="bar-fill" style="width:{{ s.battery }}%;
    background:{{'#1a6b30' if s.battery>50 else ('#9a6200' if s.battery>20 else '#7b1a1a')}}">
    Bateria <span id="battery-value">{{ s.battery }}</span>%
  </div>
</div>

<div id="status-box" class="status">
  <b>Estado:</b> <span id="status-text">{{ s.status }}</span><br>
  <b>Destino activo:</b> <span id="destination-text">{{ s.destination }}</span>
  <span id="charging-line">{% if s.charging %}<br><span class="ok">&#9889; CARGANDO</span>{% endif %}</span>
  <span id="moving-line">{% if s.moving %}<br>&#8658; En movimiento{% endif %}</span>
</div>

<div id="alarm-box" class="alarm" style="{% if not s.alarm %}display:none{% endif %}">
  <b>ALARMA:</b> <span id="error-state">{{ s.error_state }}</span><br>
  <span id="alarm-text">{{ s.alarm }}</span>
</div>

<form id="text-command-form" class="text-box" method="post" action="/text_cmd">
  <div class="text-row">
    <input id="text-command-input" name="text" type="text" placeholder="ve a mantenimiento" autocomplete="off">
    <button id="mic-btn" class="mic-btn" type="button" title="Hablar comando (Web Speech API)">&#127908;</button>
    <button type="submit">Enviar</button>
  </div>
  <div id="voice-status" class="voice-status"></div>
  <div id="intent-box" class="intent" style="{% if not s.last_text_command %}display:none{% endif %}">
    "<span id="last-text-command">{{ s.last_text_command }}</span>" ->
    <span id="last-llm-command">{{ s.last_llm_command }}</span>
    (<span id="last-llm-source">{{ s.last_llm_source }}</span>,
    conf <span id="last-llm-confidence">{{ s.last_llm_confidence }}</span>)<br>
    <span id="last-llm-reason">{{ s.last_llm_reason }}</span>
  </div>
</form>

<div class="queue">
  <h2>Cola de misiones</h2>
  <div id="queue-active">
    {% if s.queue_active %}
      <div><b>Activo:</b> {{ s.queue_active.label }}</div>
    {% else %}
      <div class="empty">Sin mision activa</div>
    {% endif %}
  </div>
  <div id="queue-pending">
    {% if s.queue_pending %}
      <ol>
      {% for item in s.queue_pending %}
        <li>{{ item.label }}{% if item.source_text %} - "{{ item.source_text }}"{% endif %}</li>
      {% endfor %}
      </ol>
    {% else %}
      <div class="empty">Sin comandos pendientes</div>
    {% endif %}
  </div>
  <div id="queue-paused" class="paused" style="{% if not s.queue_paused %}display:none{% endif %}">
    Cola pausada por E-STOP o fallo
  </div>
  <form method="post" action="/clear_queue">
    <button class="queue-clear" type="submit">Vaciar cola</button>
  </form>
</div>

<div class="grid">
  <form method="post" action="/cmd">
    <button class="btn g" name="cmd" value="charging_station">
      Estacion de Carga
      <span class="sub">Habitacion norte</span>
    </button>
  </form>
  <form method="post" action="/cmd">
    <button class="btn o" name="cmd" value="maintenance_station">
      Mantenimiento
      <span class="sub">Via gate derecho</span>
    </button>
  </form>
  <form method="post" action="/cmd">
    <button class="btn b" name="cmd" value="home_inspection">
      Home Inspeccion
      <span class="sub">Base de la rampa</span>
    </button>
  </form>
  <form method="post" action="/cmd">
    <button class="btn p" name="cmd" value="start_inspection">
      Iniciar Inspeccion
      <span class="sub">Recorre el tramo completo</span>
    </button>
  </form>
  <form method="post" action="/cancel" style="grid-column:1/-1">
    <button class="btn r" type="submit">
      E-STOP — CANCELAR RUTA
    </button>
  </form>
</div>

<div class="report">
  <div class="report-head">
    <h2>Resumen de inspección</h2>
    <span id="report-status" class="report-status idle">sin datos</span>
  </div>
  <div id="report-run" class="report-message"></div>
  <div id="report-message" class="report-message"></div>
  <div id="report-links" class="report-links"></div>
  <div id="report-map" class="report-map" style="display:none">
    <img id="report-map-img" alt="Mapa de defectos">
  </div>
  <div id="report-summary" class="md"></div>
  <div id="report-full" class="md"></div>
  <div id="report-empty" class="report-empty">
    Aún no hay informe. Se generará automáticamente al terminar una inspección.
  </div>
</div>

<script>
function esc(value) {
  return String(value ?? '').replace(/[&<>"']/g, (ch) => ({
    '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;'
  }[ch]));
}

function batteryColor(value) {
  if (value > 50) return '#1a6b30';
  if (value > 20) return '#9a6200';
  return '#7b1a1a';
}

function renderQueue(state) {
  const active = document.getElementById('queue-active');
  const pending = document.getElementById('queue-pending');
  const paused = document.getElementById('queue-paused');

  if (state.queue_active) {
    active.innerHTML = `<div><b>Activo:</b> ${esc(state.queue_active.label)}</div>`;
  } else {
    active.innerHTML = '<div class="empty">Sin mision activa</div>';
  }

  if (state.queue_pending && state.queue_pending.length) {
    pending.innerHTML = '<ol>' + state.queue_pending.map((item) => {
      const source = item.source_text ? ` - "${esc(item.source_text)}"` : '';
      return `<li>${esc(item.label)}${source}</li>`;
    }).join('') + '</ol>';
  } else {
    pending.innerHTML = '<div class="empty">Sin comandos pendientes</div>';
  }

  paused.style.display = state.queue_paused ? 'block' : 'none';
}

// ── Minimal Markdown -> HTML (headings, bold/italic, lists, tables, code) ──
function renderMarkdown(src) {
  if (!src) return '';
  const lines = String(src).replace(/\r\n?/g, '\n').split('\n');
  const out = [];
  let i = 0;
  const inlineFmt = (s) => esc(s)
    .replace(/`([^`]+)`/g, '<code>$1</code>')
    .replace(/\*\*([^*]+)\*\*/g, '<strong>$1</strong>')
    .replace(/(^|[\s(])\*([^*\n]+)\*/g, '$1<em>$2</em>');
  while (i < lines.length) {
    let line = lines[i];
    if (!line.trim()) { i++; continue; }
    // Heading
    const h = line.match(/^(#{1,6})\s+(.*)$/);
    if (h) {
      const lvl = Math.min(h[1].length, 3);
      out.push(`<h${lvl}>${inlineFmt(h[2].trim())}</h${lvl}>`);
      i++;
      continue;
    }
    // Table (line with |, next line is separator)
    if (line.includes('|') && i + 1 < lines.length && /^\s*\|?[-: |]+\|[-: |]+/.test(lines[i + 1])) {
      const splitRow = (row) => row.replace(/^\s*\|/, '').replace(/\|\s*$/, '').split('|').map((c) => c.trim());
      const headers = splitRow(line);
      i += 2;
      const rows = [];
      while (i < lines.length && lines[i].includes('|')) {
        rows.push(splitRow(lines[i]));
        i++;
      }
      const th = headers.map((c) => `<th>${inlineFmt(c)}</th>`).join('');
      const trs = rows.map((r) => '<tr>' + r.map((c) => `<td>${inlineFmt(c)}</td>`).join('') + '</tr>').join('');
      out.push(`<table><thead><tr>${th}</tr></thead><tbody>${trs}</tbody></table>`);
      continue;
    }
    // Bullet list
    if (/^\s*[-*]\s+/.test(line)) {
      const items = [];
      while (i < lines.length && /^\s*[-*]\s+/.test(lines[i])) {
        items.push(`<li>${inlineFmt(lines[i].replace(/^\s*[-*]\s+/, ''))}</li>`);
        i++;
      }
      out.push(`<ul>${items.join('')}</ul>`);
      continue;
    }
    // Numbered list
    if (/^\s*\d+\.\s+/.test(line)) {
      const items = [];
      while (i < lines.length && /^\s*\d+\.\s+/.test(lines[i])) {
        items.push(`<li>${inlineFmt(lines[i].replace(/^\s*\d+\.\s+/, ''))}</li>`);
        i++;
      }
      out.push(`<ol>${items.join('')}</ol>`);
      continue;
    }
    // Paragraph
    const para = [];
    while (i < lines.length && lines[i].trim() && !/^(#{1,6})\s+/.test(lines[i]) && !/^\s*[-*]\s+/.test(lines[i]) && !/^\s*\d+\.\s+/.test(lines[i]) && !lines[i].includes('|')) {
      para.push(lines[i]);
      i++;
    }
    if (para.length) out.push(`<p>${inlineFmt(para.join(' '))}</p>`);
  }
  return out.join('\n');
}

const REPORT_STATUS_LABEL = {
  idle: 'sin datos',
  running: 'generando…',
  done: 'completo',
  done_no_llm: 'sin LLM',
  error: 'error',
};

function renderReport(report) {
  const statusEl = document.getElementById('report-status');
  const runEl = document.getElementById('report-run');
  const msgEl = document.getElementById('report-message');
  const linksEl = document.getElementById('report-links');
  const mapWrap = document.getElementById('report-map');
  const mapImg = document.getElementById('report-map-img');
  const summaryEl = document.getElementById('report-summary');
  const fullEl = document.getElementById('report-full');
  const emptyEl = document.getElementById('report-empty');

  const r = report || {};
  const status = r.status || 'idle';
  statusEl.className = `report-status ${status}`;
  statusEl.textContent = REPORT_STATUS_LABEL[status] || status;

  if (status === 'idle') {
    emptyEl.style.display = 'block';
    runEl.textContent = '';
    msgEl.textContent = '';
    linksEl.innerHTML = '';
    mapWrap.style.display = 'none';
    summaryEl.innerHTML = '';
    fullEl.innerHTML = '';
    return;
  }
  emptyEl.style.display = 'none';
  runEl.innerHTML = r.run_id ? `<b>Run:</b> ${esc(r.run_id)}` : '';
  msgEl.textContent = r.message || '';

  const links = [];
  if (r.run_url) links.push(`<a href="${esc(r.run_url)}" target="_blank">carpeta del run</a>`);
  if (r.defect_map_url) links.push(`<a href="${esc(r.defect_map_url)}" target="_blank">mapa de defectos</a>`);
  linksEl.innerHTML = links.join('');

  if (r.defect_map_url) {
    mapImg.src = r.defect_map_url + `?t=${encodeURIComponent(r.run_id || '')}`;
    mapWrap.style.display = 'block';
  } else {
    mapWrap.style.display = 'none';
  }

  summaryEl.innerHTML = r.summary_md ? renderMarkdown(r.summary_md) : '';
  if (r.report_md) {
    fullEl.innerHTML =
      '<details open><summary>Informe completo (LLM)</summary>' +
      renderMarkdown(r.report_md) +
      '</details>';
  } else {
    fullEl.innerHTML = '';
  }
}

function renderState(state) {
  const battery = Number(state.battery || 0);
  const fill = document.getElementById('battery-fill');
  document.getElementById('battery-value').textContent = battery.toFixed(1);
  fill.style.width = `${Math.max(0, Math.min(100, battery))}%`;
  fill.style.background = batteryColor(battery);

  document.getElementById('status-text').textContent = state.status || '';
  document.getElementById('destination-text').textContent = state.destination || '';
  document.getElementById('charging-line').innerHTML =
    state.charging ? '<br><span class="ok">&#9889; CARGANDO</span>' : '';
  document.getElementById('moving-line').innerHTML =
    state.moving ? '<br>&#8658; En movimiento' : '';

  const alarmBox = document.getElementById('alarm-box');
  if (state.alarm) {
    alarmBox.style.display = 'block';
    document.getElementById('error-state').textContent = state.error_state || '';
    document.getElementById('alarm-text').textContent = state.alarm || '';
  } else {
    alarmBox.style.display = 'none';
  }

  const intentBox = document.getElementById('intent-box');
  if (state.last_text_command) {
    intentBox.style.display = 'block';
    document.getElementById('last-text-command').textContent = state.last_text_command;
    document.getElementById('last-llm-command').textContent = state.last_llm_command || '';
    document.getElementById('last-llm-source').textContent = state.last_llm_source || '';
    document.getElementById('last-llm-confidence').textContent = state.last_llm_confidence ?? '';
    document.getElementById('last-llm-reason').textContent = state.last_llm_reason || '';
  } else {
    intentBox.style.display = 'none';
  }

  renderQueue(state);
  renderReport(state.report);
}

async function refreshState() {
  try {
    const response = await fetch('/api/state', {cache: 'no-store'});
    if (response.ok) renderState(await response.json());
  } catch (err) {
    console.warn('No se pudo actualizar estado', err);
  }
}

document.getElementById('text-command-form').addEventListener('submit', async (event) => {
  event.preventDefault();
  const input = document.getElementById('text-command-input');
  const text = input.value.trim();
  if (!text) return;
  const form = new FormData();
  form.append('text', text);
  const response = await fetch('/text_cmd', {method: 'POST', body: form});
  if (response.ok) {
    input.value = '';
    refreshState();
  }
});

refreshState();
setInterval(refreshState, 1000);

// ── Web Speech API (voz por navegador) ───────────────────────────────────────
(function () {
  const micBtn = document.getElementById('mic-btn');
  const input  = document.getElementById('text-command-input');
  const voiceStatus = document.getElementById('voice-status');

  const SpeechRecognition = window.SpeechRecognition || window.webkitSpeechRecognition;
  if (!SpeechRecognition) {
    micBtn.title = 'Web Speech API no soportada en este navegador (usa Chrome/Edge)';
    micBtn.style.opacity = '0.35';
    micBtn.style.cursor  = 'not-allowed';
    return;
  }

  const recog = new SpeechRecognition();
  recog.lang = 'es-ES';
  recog.interimResults = true;
  recog.continuous = false;
  recog.maxAlternatives = 1;

  let active = false;

  micBtn.addEventListener('click', () => {
    if (active) {
      recog.stop();
    } else {
      input.value = '';
      voiceStatus.textContent = 'Escuchando...';
      recog.start();
    }
  });

  recog.onstart = () => {
    active = true;
    micBtn.classList.add('listening');
    micBtn.title = 'Haz clic para parar';
  };

  recog.onend = () => {
    active = false;
    micBtn.classList.remove('listening');
    micBtn.title = 'Hablar comando (Web Speech API)';
    if (!input.value.trim()) voiceStatus.textContent = '';
  };

  recog.onerror = (ev) => {
    active = false;
    micBtn.classList.remove('listening');
    const msg = ev.error === 'not-allowed'
      ? 'Permiso de micrófono denegado'
      : `Error: ${ev.error}`;
    voiceStatus.textContent = msg;
  };

  recog.onresult = (ev) => {
    const result = ev.results[ev.results.length - 1];
    const transcript = result[0].transcript.trim();
    input.value = transcript;
    voiceStatus.textContent = result.isFinal
      ? `Reconocido: "${transcript}"`
      : `Interim: "${transcript}"`;

    if (result.isFinal && transcript) {
      // Enviar automáticamente al servidor
      const form = new FormData();
      form.append('text', transcript);
      fetch('/text_cmd', {method: 'POST', body: form})
        .then((r) => r.ok ? refreshState() : null)
        .catch((err) => { voiceStatus.textContent = `Error enviando: ${err}`; });
      input.value = '';
    }
  };
}());
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
