#!/usr/bin/env python3

"""Black-box recorder for wind_tower_inspection autonomous mission debug.

Standalone script (no ROS package install required). It subscribes to key topics
and stores compact JSONL logs plus a short summary.md for agent-friendly review.
"""

import argparse
import datetime as _dt
import json
import math
import os
import pathlib
import signal
import subprocess
import sys
import time
from typing import Any, Optional


def _utc_timestamp_folder() -> str:
    # Example: 20260511_142233Z
    return _dt.datetime.utcnow().strftime('%Y%m%d_%H%M%SZ')


def _safe_json_dumps(obj: Any) -> str:
    return json.dumps(obj, separators=(',', ':'), ensure_ascii=True, default=str)


def _prune_stability_payload(parsed: Any) -> Optional[dict]:
    if not isinstance(parsed, dict):
        return None
    # Keep only the fields needed to diagnose why safety flags are false.
    keep: dict[str, Any] = {}
    for k in (
        'bottom_lane_locked',
        'safe_to_scan',
        'safe_to_index_tube',
        'imu_ok',
        'geometry_ok',
        'scan_motion_ok',
        'index_motion_ok',
        'fresh',
        'imu',
        'odom',
        'geometry',
        'imu_calibration',
    ):
        if k in parsed:
            keep[k] = parsed.get(k)
    return keep


def _controller_mode(sample: Any) -> Optional[str]:
    if not isinstance(sample, dict):
        return None
    ctrls = sample.get('controllers')
    if not isinstance(ctrls, list):
        return None
    for ctrl in ctrls:
        if isinstance(ctrl, dict):
            notes = ctrl.get('notes')
            if isinstance(notes, str) and notes.startswith('mode='):
                return notes.split('=', 1)[1]
    return None


def _sample_key_line(state: str, sample: Any, stability_sample: Any = None) -> Optional[str]:
    if not isinstance(sample, dict):
        return None
    angles = sample.get('angles_deg') or {}
    targets = sample.get('targets_deg') or {}
    stability_imu = {}
    if isinstance(stability_sample, dict):
        stability_imu = (stability_sample.get('imu') or {}) if isinstance(stability_sample.get('imu'), dict) else {}
    mode = _controller_mode(sample)
    controllers = sample.get('controllers') if isinstance(sample.get('controllers'), list) else []
    yaw_deg = angles.get('yaw')
    align_yaw_target_deg = targets.get('align_yaw')
    if yaw_deg is None or align_yaw_target_deg is None:
        for ctrl in controllers:
            if isinstance(ctrl, dict):
                measured = ctrl.get('measured') or {}
                target = ctrl.get('target') or {}
                if yaw_deg is None and isinstance(measured, dict) and measured.get('yaw_rad') is not None:
                    yaw_deg = round(math.degrees(float(measured.get('yaw_rad'))), 6)
                if align_yaw_target_deg is None and isinstance(target, dict) and target.get('yaw_rad') is not None:
                    align_yaw_target_deg = round(math.degrees(float(target.get('yaw_rad'))), 6)
    return _safe_json_dumps({
        'state': state,
        'mode': mode,
        'yaw_deg': yaw_deg,
        'roll_deg': angles.get('roll', stability_imu.get('roll_deg')),
        'pitch_deg': angles.get('pitch', stability_imu.get('pitch_deg')),
        'lateral_angle_deg': angles.get('lateral_angle', stability_imu.get('lateral_angle_deg')),
        'align_yaw_target_deg': align_yaw_target_deg,
        'axial_yaw_target_deg': targets.get('axial_yaw'),
        'tangential_yaw_target_deg': targets.get('tangential_yaw'),
        'cmd_linear_x': sample.get('cmd_linear_x'),
        'cmd_angular_z': sample.get('cmd_angular_z'),
        'bottom_lane_locked': sample.get('bottom_lane_locked'),
        'safe_to_scan': sample.get('safe_to_scan'),
        'safe_to_index_tube': sample.get('safe_to_index_tube'),
        'recover_bottom_ready': sample.get('recover_bottom_ready'),
        'recover_bottom_reason': sample.get('recover_bottom_reason'),
    })


def _try_parse_json(s: str) -> Optional[Any]:
    try:
        return json.loads(s)
    except Exception:
        return None


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def _quat_to_euler_deg(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    # Standard aerospace sequence (roll=X, pitch=Y, yaw=Z) from quaternion.
    # Matches typical ROS usage.
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    sinp = _clamp(sinp, -1.0, 1.0)
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))


class _MinMax:
    def __init__(self):
        self.min = None
        self.max = None

    def update(self, v: Optional[float]):
        if v is None:
            return
        try:
            v = float(v)
        except Exception:
            return
        if self.min is None or v < self.min:
            self.min = v
        if self.max is None or v > self.max:
            self.max = v

    def as_dict(self):
        return {'min': self.min, 'max': self.max}


def _run_publishers_snapshot(out_path: pathlib.Path) -> None:
    cmds = [
        ['ros2', 'topic', 'info', '-v', '/robot/platform/cmd_vel'],
        ['ros2', 'topic', 'info', '-v', '/turner/cmd_vel'],
    ]
    lines = []
    for cmd in cmds:
        lines.append(f"$ {' '.join(cmd)}\n")
        try:
            proc = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=5.0,
                check=False,
            )
            if proc.stdout:
                lines.append(proc.stdout)
            if proc.stderr:
                lines.append(proc.stderr)
            lines.append(f"[exit_code={proc.returncode}]\n\n")
        except Exception as e:
            lines.append(f"ERROR running command: {e}\n\n")
    out_path.write_text(''.join(lines), encoding='ascii', errors='replace')


def _extract_publishers_summary(text: str) -> list[str]:
    # Best-effort extraction of publisher node names per topic.
    # Keeps output short and robust to format changes.
    lines = []
    current = None
    for raw in text.splitlines():
        s = raw.strip()
        if s.startswith('$ ros2 topic info -v '):
            current = s.split()[-1]
        if s.startswith('Publisher count:') and current:
            lines.append(f"{current} {s}")
        if s.startswith('Node name:') and current:
            # Publisher/subscriber blocks both contain Node name; include all.
            # Caller can still interpret from count lines.
            name = s.split(':', 1)[-1].strip()
            lines.append(f"{current} node={name}")
    # De-duplicate while preserving order.
    seen = set()
    out = []
    for l in lines:
        if l not in seen:
            seen.add(l)
            out.append(l)
    return out


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=90.0, help='Capture duration in seconds.')
    args = parser.parse_args()

    duration_s = max(1.0, float(args.duration))
    root = pathlib.Path('debug_runs')
    run_dir = root / _utc_timestamp_folder()
    run_dir.mkdir(parents=True, exist_ok=False)

    paths = {
        'control_debug': run_dir / 'control_debug.jsonl',
        'stability': run_dir / 'stability.jsonl',
        'state_text': run_dir / 'state_text.jsonl',
        'imu': run_dir / 'imu.jsonl',
        'odom': run_dir / 'odom.jsonl',
        'cmd_vel': run_dir / 'cmd_vel.jsonl',
        'turner': run_dir / 'turner.jsonl',
        'publishers': run_dir / 'publishers.txt',
        'publishers_end': run_dir / 'publishers_end.txt',
        'summary': run_dir / 'summary.md',
        'agent_payload': run_dir / 'agent_payload.md',
    }

    # Capture publishers snapshot best-effort. Do not fail the run if it errors.
    try:
        _run_publishers_snapshot(paths['publishers'])
    except Exception as e:
        paths['publishers'].write_text(f'ERROR: {e}\n', encoding='ascii', errors='replace')

    # Lazy import rclpy and message types so this file can still create a folder/summary on failure.
    start_wall = time.time()
    start_mono = time.monotonic()
    received = {
        '/inspection/control_debug': 0,
        '/inspection/stability': 0,
        '/inspection/state_text': 0,
        '/robot/platform/cmd_vel': 0,
        '/turner/cmd_vel': 0,
        '/turner/angle_deg': 0,
        '/robot/sensors/imu_0/data': 0,
        '/robot/platform/odom/filtered': 0,
    }
    missing_notes = []

    state_timeline = []  # list of {'t':..., 'state':...}
    last_state = None
    state_samples = {}  # state -> first representative control_debug payload
    state_last_samples = {}  # state -> last seen control_debug payload
    stability_samples = {}  # state -> first pruned /inspection/stability payload
    stability_last_samples = {}  # state -> last seen pruned /inspection/stability payload

    # Track current state for annotating other topic logs.
    current_state = {'value': None, 't': 0.0}

    # Per-state quick stats for the turner to avoid manual checking.
    per_state_turner = {}  # state -> stats

    def _get_state() -> Optional[str]:
        v = current_state['value']
        return v if isinstance(v, str) and v else None

    def _turner_state_stats(state: str) -> dict:
        st = per_state_turner.get(state)
        if st is None:
            st = {
                'cmd_total': 0,
                'cmd_nonzero': 0,
                'cmd_min': None,
                'cmd_max': None,
                'angle_min': None,
                'angle_max': None,
            }
            per_state_turner[state] = st
        return st

    controller_stats = {}  # controller -> stats dict
    saturated_controllers = set()

    safety_counts = {
        'bottom_lane_locked': {'true': 0, 'false': 0},
        'safe_to_scan': {'true': 0, 'false': 0},
        'safe_to_index_tube': {'true': 0, 'false': 0},
    }

    imu_roll = _MinMax()
    imu_pitch = _MinMax()
    imu_yaw = _MinMax()
    imu_wz = _MinMax()
    imu_ax = _MinMax()
    imu_ay = _MinMax()
    imu_az = _MinMax()

    odom_yaw = _MinMax()
    odom_v_lin = _MinMax()
    odom_wz = _MinMax()

    cmd_vx = _MinMax()
    cmd_wz = _MinMax()
    turner_cmd = _MinMax()
    turner_angle_deg = _MinMax()

    # Open files early (even if no messages arrive).
    f_control = paths['control_debug'].open('w', encoding='ascii', errors='replace')
    f_stability = paths['stability'].open('w', encoding='ascii', errors='replace')
    f_state = paths['state_text'].open('w', encoding='ascii', errors='replace')
    f_imu = paths['imu'].open('w', encoding='ascii', errors='replace')
    f_odom = paths['odom'].open('w', encoding='ascii', errors='replace')
    f_cmd = paths['cmd_vel'].open('w', encoding='ascii', errors='replace')
    f_turner = paths['turner'].open('w', encoding='ascii', errors='replace')

    def t_rel() -> float:
        return time.monotonic() - start_mono

    def write_jsonl(fh, obj: dict):
        fh.write(_safe_json_dumps(obj) + '\n')
        fh.flush()

    def note_state(s: str):
        nonlocal last_state
        if not s:
            return
        if s != last_state:
            state_timeline.append({'t': round(t_rel(), 3), 'state': s})
            last_state = s
        current_state['value'] = s
        current_state['t'] = round(t_rel(), 6)

    def update_controller_stats(ctrl: dict):
        name = ctrl.get('controller')
        if not isinstance(name, str) or not name:
            return
        st = controller_stats.setdefault(
            name,
            {
                'max_abs_error': 0.0,
                'max_abs_u_raw': 0.0,
                'max_abs_u_sat': 0.0,
                'saturated': False,
                'last_integrator': None,
                'last_integrator_unit': None,
            },
        )

        def _abs_float(v) -> Optional[float]:
            try:
                if v is None:
                    return None
                return abs(float(v))
            except Exception:
                return None

        ae = _abs_float(ctrl.get('error'))
        if ae is not None and ae > st['max_abs_error']:
            st['max_abs_error'] = ae
        aur = _abs_float(ctrl.get('u_raw'))
        if aur is not None and aur > st['max_abs_u_raw']:
            st['max_abs_u_raw'] = aur
        aus = _abs_float(ctrl.get('u_sat'))
        if aus is not None and aus > st['max_abs_u_sat']:
            st['max_abs_u_sat'] = aus
        sat = ctrl.get('saturated')
        if isinstance(sat, bool) and sat:
            st['saturated'] = True
            saturated_controllers.add(name)
        integ = ctrl.get('integrator')
        if integ is not None:
            try:
                st['last_integrator'] = float(integ)
            except Exception:
                st['last_integrator'] = integ
        iu = ctrl.get('integrator_unit')
        if isinstance(iu, str):
            st['last_integrator_unit'] = iu

    rclpy_error = None

    try:
        import rclpy
        from rclpy.node import Node

        from std_msgs.msg import String as StringMsg
        from std_msgs.msg import Float64 as Float64Msg
        from sensor_msgs.msg import Imu as ImuMsg
        from nav_msgs.msg import Odometry as OdometryMsg
        from geometry_msgs.msg import TwistStamped as TwistStampedMsg

        class Recorder(Node):
            def __init__(self):
                super().__init__('capture_inspection_debug')

                # Subscriptions
                self.create_subscription(StringMsg, '/inspection/control_debug', self.on_control_debug, 50)
                self.create_subscription(StringMsg, '/inspection/stability', self.on_stability, 50)
                self.create_subscription(StringMsg, '/inspection/state_text', self.on_state_text, 50)
                self.create_subscription(TwistStampedMsg, '/robot/platform/cmd_vel', self.on_cmd_vel, 50)
                self.create_subscription(Float64Msg, '/turner/cmd_vel', self.on_turner_cmd, 50)
                self.create_subscription(Float64Msg, '/turner/angle_deg', self.on_turner_angle, 50)
                self.create_subscription(ImuMsg, '/robot/sensors/imu_0/data', self.on_imu, 50)
                self.create_subscription(OdometryMsg, '/robot/platform/odom/filtered', self.on_odom, 50)

                # Stop timer
                self._stop_timer = self.create_timer(duration_s, self.request_stop)
                self._stop_requested = False

            def request_stop(self):
                self._stop_requested = True

            def should_stop(self) -> bool:
                return self._stop_requested

            def on_state_text(self, msg: StringMsg):
                received['/inspection/state_text'] += 1
                s = str(msg.data).strip()
                note_state(s)
                write_jsonl(f_state, {'t': round(t_rel(), 6), 'state': s})

            def on_control_debug(self, msg: StringMsg):
                received['/inspection/control_debug'] += 1
                raw = str(msg.data)
                parsed = _try_parse_json(raw)
                state = None
                if isinstance(parsed, dict):
                    state = parsed.get('state')
                    if isinstance(state, str):
                        note_state(state)
                    ctrls = parsed.get('controllers')
                    if isinstance(ctrls, list):
                        for c in ctrls:
                            if isinstance(c, dict):
                                update_controller_stats(c)
                    # Representative sample per state
                    if isinstance(state, str) and state and state not in state_samples:
                        state_samples[state] = parsed
                    if isinstance(state, str) and state:
                        state_last_samples[state] = parsed
                obj = {
                    't': round(t_rel(), 6),
                    'state': _get_state(),
                    'raw': raw,
                    'parsed': parsed,
                }
                write_jsonl(f_control, obj)

            def on_stability(self, msg: StringMsg):
                received['/inspection/stability'] += 1
                raw = str(msg.data)
                parsed = _try_parse_json(raw)
                if isinstance(parsed, dict):
                    for k in ('bottom_lane_locked', 'safe_to_scan', 'safe_to_index_tube'):
                        v = parsed.get(k)
                        if isinstance(v, bool):
                            safety_counts[k]['true' if v else 'false'] += 1
                    st_name = _get_state()
                    if isinstance(st_name, str) and st_name and st_name not in stability_samples:
                        pruned = _prune_stability_payload(parsed)
                        if pruned is not None:
                            stability_samples[st_name] = pruned
                    if isinstance(st_name, str) and st_name:
                        pruned = _prune_stability_payload(parsed)
                        if pruned is not None:
                            stability_last_samples[st_name] = pruned
                obj = {
                    't': round(t_rel(), 6),
                    'state': _get_state(),
                    'raw': raw,
                    'parsed': parsed,
                }
                write_jsonl(f_stability, obj)

            def on_cmd_vel(self, msg: TwistStampedMsg):
                received['/robot/platform/cmd_vel'] += 1
                vx = float(msg.twist.linear.x)
                wz = float(msg.twist.angular.z)
                cmd_vx.update(vx)
                cmd_wz.update(wz)
                obj = {
                    't': round(t_rel(), 6),
                    'state': _get_state(),
                    'linear_x': vx,
                    'angular_z': wz,
                    'stamp': {
                        'sec': int(msg.header.stamp.sec),
                        'nanosec': int(msg.header.stamp.nanosec),
                    },
                    'frame_id': str(msg.header.frame_id),
                }
                write_jsonl(f_cmd, obj)

            def on_turner_cmd(self, msg: Float64Msg):
                received['/turner/cmd_vel'] += 1
                v = float(msg.data)
                turner_cmd.update(v)
                st_name = _get_state() or 'UNKNOWN'
                st = _turner_state_stats(st_name)
                st['cmd_total'] += 1
                if abs(v) > 1e-9:
                    st['cmd_nonzero'] += 1
                st['cmd_min'] = v if st['cmd_min'] is None else min(st['cmd_min'], v)
                st['cmd_max'] = v if st['cmd_max'] is None else max(st['cmd_max'], v)
                write_jsonl(f_turner, {'t': round(t_rel(), 6), 'state': st_name, 'turner_cmd_vel': v})

            def on_turner_angle(self, msg: Float64Msg):
                received['/turner/angle_deg'] += 1
                v = float(msg.data)
                turner_angle_deg.update(v)
                st_name = _get_state() or 'UNKNOWN'
                st = _turner_state_stats(st_name)
                st['angle_min'] = v if st['angle_min'] is None else min(st['angle_min'], v)
                st['angle_max'] = v if st['angle_max'] is None else max(st['angle_max'], v)
                write_jsonl(f_turner, {'t': round(t_rel(), 6), 'state': st_name, 'turner_angle_deg': v})

            def on_imu(self, msg: ImuMsg):
                received['/robot/sensors/imu_0/data'] += 1
                q = msg.orientation
                roll_deg, pitch_deg, yaw_deg = _quat_to_euler_deg(q.x, q.y, q.z, q.w)
                imu_roll.update(roll_deg)
                imu_pitch.update(pitch_deg)
                imu_yaw.update(yaw_deg)
                imu_wz.update(float(msg.angular_velocity.z))
                imu_ax.update(float(msg.linear_acceleration.x))
                imu_ay.update(float(msg.linear_acceleration.y))
                imu_az.update(float(msg.linear_acceleration.z))
                obj = {
                    't': round(t_rel(), 6),
                    'state': _get_state(),
                    'rpy_deg': {'roll': roll_deg, 'pitch': pitch_deg, 'yaw': yaw_deg},
                    'angular_velocity': {
                        'x': float(msg.angular_velocity.x),
                        'y': float(msg.angular_velocity.y),
                        'z': float(msg.angular_velocity.z),
                    },
                    'linear_acceleration': {
                        'x': float(msg.linear_acceleration.x),
                        'y': float(msg.linear_acceleration.y),
                        'z': float(msg.linear_acceleration.z),
                    },
                    'stamp': {
                        'sec': int(msg.header.stamp.sec),
                        'nanosec': int(msg.header.stamp.nanosec),
                    },
                    'frame_id': str(msg.header.frame_id),
                }
                write_jsonl(f_imu, obj)

            def on_odom(self, msg: OdometryMsg):
                received['/robot/platform/odom/filtered'] += 1
                q = msg.pose.pose.orientation
                _, _, yaw_deg = _quat_to_euler_deg(q.x, q.y, q.z, q.w)
                odom_yaw.update(yaw_deg)
                vx = float(msg.twist.twist.linear.x)
                vy = float(msg.twist.twist.linear.y)
                vz = float(msg.twist.twist.linear.z)
                v_lin = math.sqrt(vx * vx + vy * vy + vz * vz)
                odom_v_lin.update(v_lin)
                wz = float(msg.twist.twist.angular.z)
                odom_wz.update(wz)
                obj = {
                    't': round(t_rel(), 6),
                    'state': _get_state(),
                    'pose': {
                        'x': float(msg.pose.pose.position.x),
                        'y': float(msg.pose.pose.position.y),
                        'z': float(msg.pose.pose.position.z),
                        'yaw_deg': yaw_deg,
                    },
                    'twist': {
                        'linear': {'x': vx, 'y': vy, 'z': vz, 'speed': v_lin},
                        'angular': {'z': wz},
                    },
                    'stamp': {
                        'sec': int(msg.header.stamp.sec),
                        'nanosec': int(msg.header.stamp.nanosec),
                    },
                    'frame_id': str(msg.header.frame_id),
                    'child_frame_id': str(msg.child_frame_id),
                }
                write_jsonl(f_odom, obj)

        rclpy.init(args=None)
        node = Recorder()

        # Minimize terminal noise
        node.get_logger().set_level(50)  # FATAL

        try:
            while rclpy.ok() and not node.should_stop():
                rclpy.spin_once(node, timeout_sec=0.1)
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    except KeyboardInterrupt:
        pass
    except Exception as e:
        rclpy_error = str(e)

    # Close files
    for fh in (f_control, f_stability, f_state, f_imu, f_odom, f_cmd, f_turner):
        try:
            fh.close()
        except Exception:
            pass

    # Summary
    # Capture publishers at end as well (helps catch transient competing publishers).
    try:
        _run_publishers_snapshot(paths['publishers_end'])
    except Exception as e:
        paths['publishers_end'].write_text(f'ERROR: {e}\n', encoding='ascii', errors='replace')

    elapsed = time.time() - start_wall
    summary_lines = []
    summary_lines.append('# Inspection Debug Capture Summary\n')
    summary_lines.append(f"- Run folder: {run_dir.as_posix()}\n")
    summary_lines.append(f"- Requested duration_s: {duration_s}\n")
    summary_lines.append(f"- Wall elapsed_s: {round(elapsed, 3)}\n")
    if rclpy_error:
        summary_lines.append(f"- rclpy_error: {rclpy_error}\n")
    summary_lines.append('\n')

    summary_lines.append('## Messages Received\n')
    for topic, n in received.items():
        summary_lines.append(f"- {topic}: {n}\n")
    summary_lines.append('\n')

    summary_lines.append('## State Timeline\n')
    if state_timeline:
        for e in state_timeline:
            summary_lines.append(f"- t={e['t']} state={e['state']}\n")
    else:
        summary_lines.append('- (no states detected)\n')
    summary_lines.append('\n')

    summary_lines.append('## Control Debug Samples By State\n')
    if state_samples:
        # Order: preferred states first, then any others.
        preferred = [
            'AXIAL_SCAN',
            'ROTATE_TO_TANGENTIAL',
            'ROTATE_TO_AXIAL',
            'INDEX_TUBE',
            'ALIGN_TO_BOTTOM_LANE',
        ]
        ordered = []
        for s in preferred:
            if s in state_samples:
                ordered.append(s)
        for s in sorted(state_samples.keys()):
            if s not in ordered:
                ordered.append(s)
        for s in ordered:
            sample = state_samples.get(s)
            summary_lines.append(f"- state={s}\n")
            # Keep it short: show one-line compact JSON of parsed sample.
            summary_lines.append(f"  sample={_safe_json_dumps(sample)}\n")
    else:
        summary_lines.append('- (no /inspection/control_debug samples parsed)\n')
    summary_lines.append('\n')

    summary_lines.append('## Controller Stats\n')
    if controller_stats:
        for name in sorted(controller_stats.keys()):
            st = controller_stats[name]
            summary_lines.append(f"- controller={name}\n")
            summary_lines.append(f"  max_abs_error={st['max_abs_error']}\n")
            summary_lines.append(f"  max_abs_u_raw={st['max_abs_u_raw']}\n")
            summary_lines.append(f"  max_abs_u_sat={st['max_abs_u_sat']}\n")
            summary_lines.append(f"  saturated={st['saturated']}\n")
            summary_lines.append(f"  last_integrator={st['last_integrator']}\n")
            summary_lines.append(f"  last_integrator_unit={st['last_integrator_unit']}\n")
    else:
        summary_lines.append('- (no controllers detected)\n')
    summary_lines.append('\n')

    summary_lines.append('## IMU Summary\n')
    summary_lines.append(f"- roll_deg: {imu_roll.as_dict()}\n")
    summary_lines.append(f"- pitch_deg: {imu_pitch.as_dict()}\n")
    summary_lines.append(f"- yaw_deg: {imu_yaw.as_dict()}\n")
    summary_lines.append(f"- angular_velocity.z: {imu_wz.as_dict()}\n")
    summary_lines.append(f"- linear_acceleration.x: {imu_ax.as_dict()}\n")
    summary_lines.append(f"- linear_acceleration.y: {imu_ay.as_dict()}\n")
    summary_lines.append(f"- linear_acceleration.z: {imu_az.as_dict()}\n")
    summary_lines.append('\n')

    summary_lines.append('## Odom Summary\n')
    summary_lines.append(f"- yaw_deg: {odom_yaw.as_dict()}\n")
    summary_lines.append(f"- linear_speed: {odom_v_lin.as_dict()}\n")
    summary_lines.append(f"- angular_velocity.z: {odom_wz.as_dict()}\n")
    summary_lines.append('\n')

    summary_lines.append('## Safety Summary (from /inspection/stability)\n')
    for k, d in safety_counts.items():
        summary_lines.append(f"- {k}: true={d['true']} false={d['false']}\n")
    summary_lines.append('\n')

    summary_lines.append('## Command Summary\n')
    summary_lines.append(f"- /robot/platform/cmd_vel linear.x: {cmd_vx.as_dict()}\n")
    summary_lines.append(f"- /robot/platform/cmd_vel angular.z: {cmd_wz.as_dict()}\n")
    summary_lines.append(f"- /turner/cmd_vel: {turner_cmd.as_dict()}\n")
    summary_lines.append(f"- /turner/angle_deg: {turner_angle_deg.as_dict()}\n")
    summary_lines.append('\n')

    summary_lines.append('## Turner Per-State Summary\n')
    if per_state_turner:
        for s in sorted(per_state_turner.keys()):
            st = per_state_turner[s]
            nz_ratio = None
            if st['cmd_total'] > 0:
                nz_ratio = st['cmd_nonzero'] / float(st['cmd_total'])
            angle_delta = None
            if st['angle_min'] is not None and st['angle_max'] is not None:
                angle_delta = st['angle_max'] - st['angle_min']
            summary_lines.append(f"- state={s} cmd_nonzero_ratio={nz_ratio} cmd_min={st['cmd_min']} cmd_max={st['cmd_max']} angle_delta_deg={angle_delta}\n")
    else:
        summary_lines.append('- (no turner data)\n')
    summary_lines.append('\n')

    summary_lines.append('## Que Copiar Al Agente\n')
    # Max 20 lines total.
    copy_lines = []
    copy_lines.append(f"folder={run_dir.as_posix()}")
    if state_timeline:
        for e in state_timeline[:8]:
            copy_lines.append(f"t={e['t']} state={e['state']}")
    # One sample per state (preferred first)
    for s in ['AXIAL_SCAN', 'ROTATE_TO_TANGENTIAL', 'ROTATE_TO_AXIAL', 'INDEX_TUBE', 'ALIGN_TO_BOTTOM_LANE']:
        if s in state_samples:
            copy_lines.append(f"sample_state={s} { _safe_json_dumps(state_samples[s]).replace('\n','') }")
    # Saturated controllers
    if saturated_controllers:
        copy_lines.append('saturated_controllers=' + ','.join(sorted(saturated_controllers)))
    # IMU + safety summaries
    copy_lines.append(f"imu_roll_deg={imu_roll.as_dict()} pitch_deg={imu_pitch.as_dict()}")
    copy_lines.append('safety=' + _safe_json_dumps(safety_counts))
    copy_lines.append(f"publishers_file={paths['publishers'].as_posix()}")
    copy_lines.append(f"publishers_end_file={paths['publishers_end'].as_posix()}")
    copy_lines = copy_lines[:20]
    for line in copy_lines:
        summary_lines.append(f"- {line}\n")

    paths['summary'].write_text(''.join(summary_lines), encoding='ascii', errors='replace')

    # A single short payload intended for copy/paste into an agent.
    pub_start_txt = ''
    pub_end_txt = ''
    try:
        pub_start_txt = paths['publishers'].read_text(encoding='ascii', errors='replace')
    except Exception:
        pass
    try:
        pub_end_txt = paths['publishers_end'].read_text(encoding='ascii', errors='replace')
    except Exception:
        pass

    agent_lines = []
    agent_lines.append('# Agent Payload (copy/paste)\n')
    agent_lines.append(f"- folder={run_dir.as_posix()}\n")
    agent_lines.append(f"- duration_s={duration_s} elapsed_s={round(elapsed,3)}\n")
    if rclpy_error:
        agent_lines.append(f"- rclpy_error={rclpy_error}\n")
    agent_lines.append('- msg_counts=' + _safe_json_dumps(received) + '\n')

    # State timeline (cap)
    if state_timeline:
        tl = [f"t={e['t']} state={e['state']}" for e in state_timeline[:12]]
        agent_lines.append('- timeline=' + ' | '.join(tl) + '\n')
    else:
        agent_lines.append('- timeline=(none)\n')

    # One compact sample per state (preferred first)
    preferred = [
        'AXIAL_SCAN',
        'ROTATE_TO_TANGENTIAL',
        'ROTATE_TO_AXIAL',
        'INDEX_TUBE',
        'RECOVER_BOTTOM_AFTER_INDEX',
        'REALIGN_AXIAL_YAW',
        'DESCEND_TO_BOTTOM_LANE',
        'ALIGN_TO_BOTTOM_LANE',
    ]
    for s in preferred:
        if s in state_samples:
            agent_lines.append(f"- sample_{s}={_safe_json_dumps(state_samples[s])}\n")
            key_line = _sample_key_line(s, state_samples[s], stability_samples.get(s))
            if key_line is not None:
                agent_lines.append(f"- key_{s}={key_line}\n")
        if s in stability_samples:
            agent_lines.append(f"- stability_{s}={_safe_json_dumps(stability_samples[s])}\n")
        # Also include the last samples for states that often drift over time.
        if s in ('AXIAL_SCAN', 'REALIGN_AXIAL_YAW', 'ALIGN_TO_BOTTOM_LANE'):
            if s in state_last_samples:
                agent_lines.append(f"- sample_last_{s}={_safe_json_dumps(state_last_samples[s])}\n")
                key_line = _sample_key_line(f'last_{s}', state_last_samples[s], stability_last_samples.get(s))
                if key_line is not None:
                    agent_lines.append(f"- key_last_{s}={key_line}\n")
            if s in stability_last_samples:
                agent_lines.append(f"- stability_last_{s}={_safe_json_dumps(stability_last_samples[s])}\n")
    # Any other states (cap)
    extra_states = [s for s in sorted(state_samples.keys()) if s not in preferred]
    for s in extra_states[:5]:
        agent_lines.append(f"- sample_{s}={_safe_json_dumps(state_samples[s])}\n")

    extra_stab_states = [s for s in sorted(stability_samples.keys()) if s not in preferred]
    for s in extra_stab_states[:3]:
        agent_lines.append(f"- stability_{s}={_safe_json_dumps(stability_samples[s])}\n")

    # Controller saturation
    if saturated_controllers:
        agent_lines.append('- saturated_controllers=' + ','.join(sorted(saturated_controllers)) + '\n')
    else:
        agent_lines.append('- saturated_controllers=(none)\n')

    # Turner per-state summary (cap to key states)
    for s in ['INDEX_TUBE', 'ROTATE_TO_TANGENTIAL', 'ROTATE_TO_AXIAL']:
        if s in per_state_turner:
            st = per_state_turner[s]
            nz_ratio = None
            if st['cmd_total'] > 0:
                nz_ratio = round(st['cmd_nonzero'] / float(st['cmd_total']), 3)
            angle_delta = None
            if st['angle_min'] is not None and st['angle_max'] is not None:
                angle_delta = round(st['angle_max'] - st['angle_min'], 6)
            agent_lines.append(
                f"- turner_state={s} cmd_nonzero_ratio={nz_ratio} cmd_min={st['cmd_min']} cmd_max={st['cmd_max']} angle_delta_deg={angle_delta}\n"
            )

    # Safety + IMU quick
    agent_lines.append('- safety_counts=' + _safe_json_dumps(safety_counts) + '\n')
    agent_lines.append(f"- imu_roll_deg={imu_roll.as_dict()} imu_pitch_deg={imu_pitch.as_dict()}\n")

    # Publishers summary (keep short)
    pub_summary = _extract_publishers_summary(pub_start_txt)
    pub_end_summary = _extract_publishers_summary(pub_end_txt)
    if pub_summary:
        agent_lines.append('- publishers_start=' + ' | '.join(pub_summary[:12]) + '\n')
    if pub_end_summary:
        agent_lines.append('- publishers_end=' + ' | '.join(pub_end_summary[:12]) + '\n')

    paths['agent_payload'].write_text(''.join(agent_lines), encoding='ascii', errors='replace')

    # Final terminal output: short.
    print(run_dir.as_posix())
    print(paths['summary'].as_posix())
    print(paths['agent_payload'].as_posix())
    print(f"cat {paths['agent_payload'].as_posix()}")
    return 0


if __name__ == '__main__':
    # Ensure Ctrl+C works even if ROS spin is blocked.
    signal.signal(signal.SIGINT, signal.default_int_handler)
    raise SystemExit(main())
