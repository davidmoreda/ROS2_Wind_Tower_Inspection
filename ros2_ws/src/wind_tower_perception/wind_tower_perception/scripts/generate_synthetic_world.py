"""Generate a Gazebo world with random circular defects on the tube wall.

Defects are spheres pegged to the *inner* surface of the wind tower tube,
positioned in absolute world coordinates so they do **not** rotate with the
turner during dataset capture (the turner is expected to stay at theta=0
while images are being collected — see ``synthetic_capture_node``).

Outputs:

* ``<output_world>`` — full SDF world, structurally identical to
  ``wind_tower_world.sdf`` but with ``<model name="defect_NNN">`` entries
  added inside the ``<world>`` block.
* ``<ground_truth_yaml>`` — list of every defect with its class id, world
  position, radius and (x_axial, theta_surface) cylindrical coordinates.
  This is the file ``synthetic_capture_node`` consumes for auto-labelling.

Usage::

    python -m wind_tower_perception.scripts.generate_synthetic_world \\
        --base-world  src/wind_tower_simulation/worlds/wind_tower_world.sdf \\
        --output-world ~/wind_tower_synthetic/wind_tower_world_synthetic.sdf \\
        --ground-truth ~/wind_tower_synthetic/defects_ground_truth.yaml \\
        --num-defects 80 --seed 42

The script does not require ROS to be running.
"""

import argparse
import math
import os
import random
import re
import sys
from dataclasses import dataclass, asdict
from typing import List

try:
    import yaml  # type: ignore
except ImportError:  # pragma: no cover — surface a clear error early
    print(
        '[generate_synthetic_world] PyYAML is required. '
        'Install with `pip install pyyaml`.',
        file=sys.stderr,
    )
    raise


DEFAULT_CLASSES = [
    # (class_id, class_name, color_rgba, radius_min_m, radius_max_m, weight)
    (0, 'rust',         '0.55 0.18 0.05 1', 0.04, 0.12, 0.5),
    (1, 'pitting',      '0.15 0.15 0.15 1', 0.02, 0.05, 0.3),
    (2, 'through_hole', '0.00 0.00 0.00 1', 0.04, 0.09, 0.2),
]


@dataclass
class Defect:
    defect_id: int
    class_id: int
    class_name: str
    x_axial_m: float
    theta_surface_deg: float
    radius_m: float
    world_x: float
    world_y: float
    world_z: float


def _sample_class(rng: random.Random):
    weights = [c[5] for c in DEFAULT_CLASSES]
    return rng.choices(DEFAULT_CLASSES, weights=weights, k=1)[0]


def _surface_point(
    *,
    x_axial_m: float,
    theta_surface_deg: float,
    radius_m: float,
    axis_center: tuple,
    cylinder_radius_m: float,
):
    """World position of a point that sits on the inner wall.

    Coordinates follow the project convention: tube axis along world Y,
    centred at ``axis_center`` (world frame), theta=0 at the bottom
    growing into the +world_x half-plane.
    """
    theta_rad = math.radians(theta_surface_deg)
    # Sit the defect ~1cm inside the wall so its visual stays inside the
    # cylinder geometry rather than clipping behind the STL mesh.
    effective_r = cylinder_radius_m - max(radius_m, 0.02)
    x = axis_center[0] + effective_r * math.sin(theta_rad)
    y = axis_center[1] + x_axial_m
    z = axis_center[2] - effective_r * math.cos(theta_rad)
    return x, y, z


def sample_defects(
    *,
    count: int,
    rng: random.Random,
    x_axial_range: tuple,
    theta_range_deg: tuple,
    axis_center: tuple,
    cylinder_radius_m: float,
    min_separation_m: float,
    min_separation_deg: float,
) -> List[Defect]:
    defects: List[Defect] = []
    max_tries = count * 50
    tries = 0
    while len(defects) < count and tries < max_tries:
        tries += 1
        cls = _sample_class(rng)
        radius = rng.uniform(cls[3], cls[4])
        x_axial = rng.uniform(*x_axial_range)
        theta = rng.uniform(*theta_range_deg)

        too_close = False
        for d in defects:
            dx = abs(d.x_axial_m - x_axial)
            dtheta = abs(
                ((d.theta_surface_deg - theta + 540.0) % 360.0) - 180.0
            )
            if dx < min_separation_m and dtheta < min_separation_deg:
                too_close = True
                break
        if too_close:
            continue

        wx, wy, wz = _surface_point(
            x_axial_m=x_axial,
            theta_surface_deg=theta,
            radius_m=radius,
            axis_center=axis_center,
            cylinder_radius_m=cylinder_radius_m,
        )
        defects.append(Defect(
            defect_id=len(defects),
            class_id=cls[0],
            class_name=cls[1],
            x_axial_m=x_axial,
            theta_surface_deg=theta,
            radius_m=radius,
            world_x=wx,
            world_y=wy,
            world_z=wz,
        ))
    if len(defects) < count:
        print(
            f'[generate_synthetic_world] warning: only placed '
            f'{len(defects)}/{count} defects under the configured spacing '
            'constraints. Consider lowering --min-sep-x or --min-sep-deg.'
        )
    return defects


def _defect_to_sdf(defect: Defect) -> str:
    name = f'defect_{defect.defect_id:04d}_{defect.class_name}'
    color = next(
        c[2] for c in DEFAULT_CLASSES if c[0] == defect.class_id
    )
    return f'''
    <model name="{name}">
      <static>true</static>
      <pose>{defect.world_x:.6f} {defect.world_y:.6f} {defect.world_z:.6f} 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <sphere>
              <radius>{defect.radius_m:.5f}</radius>
            </sphere>
          </geometry>
          <material>
            <ambient>{color}</ambient>
            <diffuse>{color}</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
          <cast_shadows>false</cast_shadows>
        </visual>
        <collision name="collision">
          <geometry>
            <sphere><radius>{defect.radius_m:.5f}</radius></sphere>
          </geometry>
          <surface>
            <contact>
              <collide_bitmask>0</collide_bitmask>
            </contact>
          </surface>
        </collision>
      </link>
    </model>'''


def inject_defects_into_world(base_world_xml: str, defects: List[Defect]) -> str:
    snippet = '\n    <!-- SYNTHETIC DEFECTS (auto-generated) -->'
    for d in defects:
        snippet += _defect_to_sdf(d)
    snippet += '\n    <!-- END SYNTHETIC DEFECTS -->\n'

    closing_world = re.search(r'</world>', base_world_xml)
    if not closing_world:
        raise ValueError('Base world SDF has no </world> closing tag.')
    insert_at = closing_world.start()
    return base_world_xml[:insert_at] + snippet + base_world_xml[insert_at:]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base-world', required=True,
                        help='Path to the original wind_tower_world.sdf')
    parser.add_argument('--output-world', required=True,
                        help='Path where the synthetic SDF will be written.')
    parser.add_argument('--ground-truth', required=True,
                        help='Path to the YAML ground-truth file.')
    parser.add_argument('--num-defects', type=int, default=80)
    parser.add_argument('--seed', type=int, default=42)

    parser.add_argument('--x-axial-min-m', type=float, default=-13.0,
                        help='Min axial coordinate (world Y, default -13).')
    parser.add_argument('--x-axial-max-m', type=float, default=13.0,
                        help='Max axial coordinate (world Y, default +13).')
    parser.add_argument('--theta-min-deg', type=float, default=0.0)
    parser.add_argument('--theta-max-deg', type=float, default=360.0)
    parser.add_argument('--min-sep-x-m', type=float, default=0.30)
    parser.add_argument('--min-sep-deg', type=float, default=4.0)

    parser.add_argument('--axis-cx-m', type=float, default=0.0)
    parser.add_argument('--axis-cy-m', type=float, default=0.0)
    parser.add_argument('--axis-cz-m', type=float, default=4.0)
    parser.add_argument('--cylinder-radius-m', type=float, default=3.925)

    args = parser.parse_args(argv)
    rng = random.Random(args.seed)

    with open(args.base_world, 'r', encoding='utf-8') as fh:
        base_xml = fh.read()

    defects = sample_defects(
        count=args.num_defects,
        rng=rng,
        x_axial_range=(args.x_axial_min_m, args.x_axial_max_m),
        theta_range_deg=(args.theta_min_deg, args.theta_max_deg),
        axis_center=(args.axis_cx_m, args.axis_cy_m, args.axis_cz_m),
        cylinder_radius_m=args.cylinder_radius_m,
        min_separation_m=args.min_sep_x_m,
        min_separation_deg=args.min_sep_deg,
    )

    out_xml = inject_defects_into_world(base_xml, defects)
    os.makedirs(os.path.dirname(os.path.expanduser(args.output_world)) or '.', exist_ok=True)
    with open(os.path.expanduser(args.output_world), 'w', encoding='utf-8') as fh:
        fh.write(out_xml)

    gt = {
        'meta': {
            'seed': args.seed,
            'num_defects': len(defects),
            'classes': [
                {'id': c[0], 'name': c[1]} for c in DEFAULT_CLASSES
            ],
            'axis_center_world': [
                args.axis_cx_m, args.axis_cy_m, args.axis_cz_m,
            ],
            'cylinder_radius_m': args.cylinder_radius_m,
            'tube_axis_direction': 'y',
        },
        'defects': [asdict(d) for d in defects],
    }
    os.makedirs(os.path.dirname(os.path.expanduser(args.ground_truth)) or '.', exist_ok=True)
    with open(os.path.expanduser(args.ground_truth), 'w', encoding='utf-8') as fh:
        yaml.safe_dump(gt, fh, sort_keys=False)

    print(
        f'[generate_synthetic_world] wrote SDF with {len(defects)} defects to '
        f'{args.output_world}'
    )
    print(
        f'[generate_synthetic_world] ground truth: {args.ground_truth}'
    )


if __name__ == '__main__':
    main()
