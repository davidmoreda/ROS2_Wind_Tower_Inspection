"""
Behavior Tree de mision — Wind Tower Inspection.

Este modulo decide QUE ruta o modo debe dispararse. La ejecucion fisica sigue
en MissionController/Nav2:

MissionRoot
├── BatteryCriticalMission
│   ├── If command == battery_critical
│   └── ChargeRoute
│       ├── If robot in maintenance: gate_left -> home_inspection -> charging_station
│       ├── If robot in tube: ramp_top -> home_inspection -> charging_station
│       └── Else: charging_station
├── ChargeMission
│   ├── If command == charging_station
│   └── ChargeRoute
│       ├── If robot in maintenance: gate_left -> home_inspection -> charging_station
│       ├── If robot in tube: ramp_top -> home_inspection -> charging_station
│       └── Else: charging_station
├── HomeMission
│   ├── If command == home_inspection
│   └── HomeRoute
│       ├── If robot in maintenance: gate_left -> home_inspection
│       ├── If robot in tube: ramp_top -> home_inspection
│       └── Else: home_inspection
├── MaintenanceMission
│   ├── If command == maintenance_station
│   └── MaintenanceRoute
│       ├── If robot in tube: ramp_top -> home_inspection -> gate_right -> maintenance_station
│       └── Else: gate_right -> maintenance_station
└── InspectionMission
    ├── If command == start_inspection
    └── StartMeteredInspection

Regla topologica importante:
si el robot esta dentro del tubo, vuelve por la rampa. El gate lateral no se
usa para salir del tubo hacia home o carga.
"""

import py_trees
from py_trees.common import Status

TUBE_CENTER_X = 0.245
TUBE_HALF_WIDTH = 1.25
TUBE_MIN_Y = 6.5
MAINTENANCE_MIN_Y = 45.0


class CommandIs(py_trees.behaviour.Behaviour):
    """SUCCESS si el comando activo coincide con el esperado."""

    def __init__(self, get_command, expected: str):
        super().__init__(name=f'CommandIs({expected})')
        self._get_command = get_command
        self._expected = expected

    def update(self) -> Status:
        command = self._get_command()
        self.feedback_message = f'command={command} expected={self._expected}'
        return Status.SUCCESS if command == self._expected else Status.FAILURE


class IsRobotInTubeArea(py_trees.behaviour.Behaviour):
    """SUCCESS si el robot esta en la calle axial central del tubo."""

    def __init__(self, get_robot_x, get_robot_y):
        super().__init__(name='IsRobotInTubeArea?')
        self._get_x = get_robot_x
        self._get_y = get_robot_y

    def update(self) -> Status:
        x = self._get_x()
        y = self._get_y()
        lateral_error = abs(x - TUBE_CENTER_X)
        in_tube = y >= TUBE_MIN_Y and lateral_error <= TUBE_HALF_WIDTH
        self.feedback_message = (
            f'robot=({x:.2f},{y:.2f}) '
            f'lateral_error={lateral_error:.2f} max={TUBE_HALF_WIDTH:.2f}'
        )
        return Status.SUCCESS if in_tube else Status.FAILURE


class IsRobotInMaintenanceArea(py_trees.behaviour.Behaviour):
    """SUCCESS si el robot esta en la zona norte de mantenimiento."""

    def __init__(self, get_robot_y):
        super().__init__(name='IsRobotInMaintenanceArea?')
        self._get_y = get_robot_y

    def update(self) -> Status:
        y = self._get_y()
        self.feedback_message = f'robot_y={y:.2f} min={MAINTENANCE_MIN_Y:.2f}'
        return Status.SUCCESS if y >= MAINTENANCE_MIN_Y else Status.FAILURE


class FireOnce(py_trees.behaviour.Behaviour):
    """
    Dispara una accion asíncrona una vez y devuelve SUCCESS.

    El BT decide y lanza; Nav2 y MissionController completan o cancelan el
    movimiento mediante callbacks fuera del arbol.
    """

    def __init__(self, name: str, action_fn):
        super().__init__(name=name)
        self._action = action_fn

    def update(self) -> Status:
        self._action()
        self.feedback_message = 'accion enviada'
        return Status.SUCCESS


def _wp_pose(make_pose_fn, wp: dict, default_yaw: float = 1.5708):
    return make_pose_fn(wp['x'], wp['y'], wp.get('yaw', default_yaw))


def _maintenance_right_gate_poses(make_pose_fn, maintenance: dict, gate_right: tuple):
    poses = [
        make_pose_fn(*gate_right),
    ]
    for via in maintenance.get('via_points') or []:
        poses.append(make_pose_fn(via['x'], via['y'], via.get('yaw', 1.5708)))
    poses.append(_wp_pose(make_pose_fn, maintenance))
    return poses


def _route_home_or_charge(
    *,
    name: str,
    get_robot_x,
    get_robot_y,
    direct_action_fn,
    tube_action_fn,
    maintenance_action_fn,
) -> py_trees.behaviour.Behaviour:
    route = py_trees.composites.Selector(name=name, memory=False)

    maintenance_sequence = py_trees.composites.Sequence(
        name=f'{name}ViaLeftGate',
        memory=False,
    )
    maintenance_sequence.add_children([
        IsRobotInMaintenanceArea(get_robot_y),
        FireOnce(f'{name}LeftGateAction', maintenance_action_fn),
    ])

    tube_sequence = py_trees.composites.Sequence(name=f'{name}ViaRamp', memory=False)
    tube_sequence.add_children([
        IsRobotInTubeArea(get_robot_x, get_robot_y),
        FireOnce(f'{name}RampAction', tube_action_fn),
    ])

    route.add_children([
        maintenance_sequence,
        tube_sequence,
        FireOnce(f'{name}DirectAction', direct_action_fn),
    ])
    return route


def _route_with_tube_fallback(
    *,
    name: str,
    get_robot_x,
    get_robot_y,
    direct_action_fn,
    tube_action_fn,
) -> py_trees.behaviour.Behaviour:
    route = py_trees.composites.Selector(name=name, memory=False)

    tube_sequence = py_trees.composites.Sequence(name=f'{name}ViaRamp', memory=False)
    tube_sequence.add_children([
        IsRobotInTubeArea(get_robot_x, get_robot_y),
        FireOnce(f'{name}RampAction', tube_action_fn),
    ])

    route.add_children([
        tube_sequence,
        FireOnce(f'{name}DirectAction', direct_action_fn),
    ])
    return route


def _command_sequence(command: str, get_command, child) -> py_trees.behaviour.Behaviour:
    seq = py_trees.composites.Sequence(name=f'{command}_mission', memory=False)
    seq.add_children([
        CommandIs(get_command, command),
        child,
    ])
    return seq


def build_mission_tree(
    command: str,
    get_robot_x,
    get_robot_y,
    send_through_fn,
    send_to_fn,
    make_pose_fn,
    waypoints: dict,
    gate_left: tuple,
    gate_right: tuple,
    post_gate: tuple,
    start_inspection_fn,
) -> py_trees.behaviour.Behaviour:
    """Construye el arbol completo para todos los comandos de la UI/API."""

    def get_command():
        return command

    home = waypoints.get('home_inspection', {'x': -0.026, 'y': 4.5, 'yaw': 1.5708})
    ramp = waypoints.get('ramp_top', {'x': 0.245, 'y': 6.901, 'yaw': 1.5708})
    charge = waypoints.get('charging_station')
    maintenance = waypoints.get('maintenance_station')

    def home_direct():
        send_to_fn(home)

    def home_via_ramp():
        send_through_fn([
            _wp_pose(make_pose_fn, ramp),
            _wp_pose(make_pose_fn, home),
        ])

    def home_from_maintenance():
        send_through_fn([
            make_pose_fn(*gate_left),
            _wp_pose(make_pose_fn, home),
        ])

    def charge_direct():
        if charge:
            send_to_fn(charge)

    def charge_via_ramp():
        if charge:
            send_through_fn([
                _wp_pose(make_pose_fn, ramp),
                _wp_pose(make_pose_fn, home),
                _wp_pose(make_pose_fn, charge),
            ])

    def charge_from_maintenance():
        if charge:
            send_through_fn([
                make_pose_fn(*gate_left),
                _wp_pose(make_pose_fn, home),
                _wp_pose(make_pose_fn, charge),
            ])

    def maintenance_via_right_gate():
        if not maintenance:
            return
        send_through_fn(_maintenance_right_gate_poses(
            make_pose_fn,
            maintenance,
            gate_right,
        ))

    def maintenance_from_tube():
        if not maintenance:
            return
        poses = [
            _wp_pose(make_pose_fn, ramp),
            _wp_pose(make_pose_fn, home),
        ]
        poses.extend(_maintenance_right_gate_poses(
            make_pose_fn,
            maintenance,
            gate_right,
        ))
        send_through_fn(poses)

    def make_charge_route(name='ChargeRoute'):
        return _route_home_or_charge(
            name=name,
            get_robot_x=get_robot_x,
            get_robot_y=get_robot_y,
            direct_action_fn=charge_direct,
            tube_action_fn=charge_via_ramp,
            maintenance_action_fn=charge_from_maintenance,
        )

    home_route = _route_home_or_charge(
        name='HomeRoute',
        get_robot_x=get_robot_x,
        get_robot_y=get_robot_y,
        direct_action_fn=home_direct,
        tube_action_fn=home_via_ramp,
        maintenance_action_fn=home_from_maintenance,
    )
    maintenance_route = _route_with_tube_fallback(
        name='MaintenanceRoute',
        get_robot_x=get_robot_x,
        get_robot_y=get_robot_y,
        direct_action_fn=maintenance_via_right_gate,
        tube_action_fn=maintenance_from_tube,
    )

    root = py_trees.composites.Selector(name='MissionRoot', memory=False)
    root.add_children([
        _command_sequence(
            'battery_critical',
            get_command,
            make_charge_route('BatteryCriticalChargeRoute'),
        ),
        _command_sequence('charging_station', get_command, make_charge_route()),
        _command_sequence('home_inspection', get_command, home_route),
        _command_sequence('maintenance_station', get_command, maintenance_route),
        _command_sequence(
            'start_inspection',
            get_command,
            FireOnce('StartMeteredInspection', start_inspection_fn),
        ),
    ])
    return root


def build_go_home_tree(
    get_robot_x,
    get_robot_y,
    send_through_fn,
    send_to_fn,
    make_pose_fn,
    waypoints: dict,
    gate_left: tuple,
    gate_right: tuple,
    post_gate: tuple,
) -> py_trees.behaviour.Behaviour:
    """Compatibilidad: construye solo la rama home usando el arbol completo."""
    return build_mission_tree(
        command='home_inspection',
        get_robot_x=get_robot_x,
        get_robot_y=get_robot_y,
        send_through_fn=send_through_fn,
        send_to_fn=send_to_fn,
        make_pose_fn=make_pose_fn,
        waypoints=waypoints,
        gate_left=gate_left,
        gate_right=gate_right,
        post_gate=post_gate,
        start_inspection_fn=lambda: None,
    )
