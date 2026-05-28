"""Generate a Gazebo world with synthetic defects + walking 'people'.

Defects:
  Reuses ``wind_tower_perception.scripts.generate_synthetic_world`` so the
  RGBA, radii and seed match the dataset the YOLO was trained on.

People:
  Two simple primitive-based models (torso cylinder + sphere head) driven
  by the ``gz::sim::systems::TrajectoryFollower`` plugin. Avoids the
  ``<actor><script><trajectory>`` path which is flaky on Gazebo Harmonic
  (actors stay at spawn pose while their walk animation plays in place).

Tube geometry in this world:
  Mesh has its long axis along local +Z (30 m). After roll=π/2 around
  world X, +Z local maps to -Y world, so the tube spans
  ``world_y ∈ [pose.y - 30, pose.y] = [-15, +15]``.
  Pose ``(-4, 15, 1)`` + 8 m diameter → world centre = ``(0, 0, 5)``.

Usage::

    python3 tools/make_defects_and_actors_world.py
"""
import os
import sys
import importlib.util
import random

REPO = os.path.expanduser('~/ROS2_Wind_Tower_Inspection')
BASE_WORLD   = os.path.join(REPO, 'ros2_ws/src/wind_tower_simulation/worlds/wind_tower_world.sdf')
OUTPUT_WORLD = os.path.join(REPO, 'ros2_ws/src/wind_tower_simulation/worlds/wind_tower_world_defects_actors.sdf')
GT_PATH      = os.path.expanduser('~/wind_tower_synthetic/defects_ground_truth.yaml')

# Tube centre in world frame. Y=0 because pose.y=15 + axis along -Y mapping
# means the tube extends from world_y=+15 to world_y=-15 → centre at y=0.
TUBE_CENTRE = (0.0, 0.0, 5.0)
TUBE_RADIUS = 3.925   # inner wall


# ──────────────────────────────────────────────────────────────────────
# Step 1 — Defects
# ──────────────────────────────────────────────────────────────────────

def generate_defects_world() -> str:
    script_path = os.path.join(
        REPO,
        'ros2_ws/src/wind_tower_perception/wind_tower_perception/scripts/generate_synthetic_world.py',
    )
    spec = importlib.util.spec_from_file_location('gen_world', script_path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)

    rng = random.Random(42)
    # Restrict axial range to [-13, +13] so defects sit strictly inside
    # the tube and not near its open ends (Y=±15).
    defects = mod.sample_defects(
        count=80,
        rng=rng,
        x_axial_range=(-13.0, 13.0),
        theta_range_deg=(0.0, 360.0),
        axis_center=TUBE_CENTRE,
        cylinder_radius_m=TUBE_RADIUS,
        min_separation_m=0.30,
        min_separation_deg=4.0,
    )

    with open(BASE_WORLD, 'r', encoding='utf-8') as fh:
        base_xml = fh.read()
    out_xml = mod.inject_defects_into_world(base_xml, defects)

    import yaml
    from dataclasses import asdict
    gt = {
        'meta': {
            'seed': 42,
            'num_defects': len(defects),
            'classes': [{'id': c[0], 'name': c[1]} for c in mod.DEFAULT_CLASSES],
            'axis_center_world': list(TUBE_CENTRE),
            'cylinder_radius_m': TUBE_RADIUS,
            'tube_axis_direction': 'y',
        },
        'defects': [asdict(d) for d in defects],
    }
    os.makedirs(os.path.dirname(GT_PATH), exist_ok=True)
    with open(GT_PATH, 'w', encoding='utf-8') as fh:
        yaml.safe_dump(gt, fh, sort_keys=False)
    print(f'[defects] {len(defects)} defectos. axis_center={TUBE_CENTRE}')
    print(f'          Y range: world_y ∈ [{TUBE_CENTRE[1]-13:.0f}, {TUBE_CENTRE[1]+13:.0f}]')
    print(f'          GT → {GT_PATH}')
    return out_xml


# ──────────────────────────────────────────────────────────────────────
# Step 2 — Simple human-ish models with TrajectoryFollower plugin.
#
# Model = base_link (cylinder torso) + head_link (sphere) fixed-jointed.
# Static=false (the plugin needs a dynamic model) but kinematic-only via
# the plugin so it doesn't fall or react to physics.
# ──────────────────────────────────────────────────────────────────────

def _person_model(
    name: str,
    initial_xy: tuple,
    initial_yaw: float,
    waypoints_xy: list,
    *,
    force: float,
    torque: float,
    linear_damping: float,
    angular_damping: float,
    shirt_color: str,
    pants_color: str,
) -> str:
    """Build a 'person' that walks upright without tipping over.

    Architecture: chain of joints prevents roll/pitch but allows XY
    translation and yaw rotation:

        world ─(fixed)→ anchor ─(prismatic X)→ x_slider
              ─(prismatic Y)→ y_slider ─(revolute Z)→ base_link

    TrajectoryFollower applies force to base_link's CG. The chain
    transmits horizontal forces to joint motion while absorbing any
    roll/pitch torque the offset between CG (z=0.85) and the joint
    creates. Friction-like damping comes from joint <damping>, not
    from collision with the ground (there is no collision: the body
    floats, held by the joints).

    Tuning: terminal velocity = force / linear_damping.
    """
    x0, y0 = initial_xy
    wp_xml = '\n'.join(
        f'          <waypoint>{x:.3f} {y:.3f}</waypoint>'
        for (x, y) in waypoints_xy
    )
    return f'''
    <!-- "Person" {name}: walks via joint chain so it can't fall over. -->
    <model name="{name}">
      <pose>{x0:.3f} {y0:.3f} 0 0 0 0</pose>

      <!-- Anchor link fixed to world. Almost massless. -->
      <link name="anchor">
        <inertial>
          <mass>0.001</mass>
          <inertia>
            <ixx>1e-4</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>1e-4</iyy><iyz>0</iyz>
            <izz>1e-4</izz>
          </inertia>
        </inertial>
      </link>
      <joint name="anchor_to_world" type="fixed">
        <parent>world</parent>
        <child>anchor</child>
      </joint>

      <!-- X-slider: allows X translation, with viscous damping. -->
      <link name="x_slider">
        <inertial>
          <mass>0.001</mass>
          <inertia>
            <ixx>1e-4</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>1e-4</iyy><iyz>0</iyz>
            <izz>1e-4</izz>
          </inertia>
        </inertial>
      </link>
      <joint name="j_x" type="prismatic">
        <parent>anchor</parent>
        <child>x_slider</child>
        <axis>
          <xyz>1 0 0</xyz>
          <limit><lower>-30</lower><upper>30</upper></limit>
          <dynamics><damping>{linear_damping:.1f}</damping></dynamics>
        </axis>
      </joint>

      <!-- Y-slider: allows Y translation. -->
      <link name="y_slider">
        <inertial>
          <mass>0.001</mass>
          <inertia>
            <ixx>1e-4</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>1e-4</iyy><iyz>0</iyz>
            <izz>1e-4</izz>
          </inertia>
        </inertial>
      </link>
      <joint name="j_y" type="prismatic">
        <parent>x_slider</parent>
        <child>y_slider</child>
        <axis>
          <xyz>0 1 0</xyz>
          <limit><lower>-30</lower><upper>30</upper></limit>
          <dynamics><damping>{linear_damping:.1f}</damping></dynamics>
        </axis>
      </joint>

      <!-- Body link (visuals only — no collision so no tipping from
           friction couple; joints fully constrain its pose). -->
      <link name="base_link">
        <pose>0 0 0.85 0 0 {initial_yaw:.4f}</pose>
        <visual name="torso">
          <pose>0 0 0.05 0 0 0</pose>
          <geometry><cylinder><radius>0.22</radius><length>1.2</length></cylinder></geometry>
          <material>
            <ambient>{shirt_color}</ambient>
            <diffuse>{shirt_color}</diffuse>
          </material>
        </visual>
        <visual name="legs">
          <pose>0 0 -0.5 0 0 0</pose>
          <geometry><cylinder><radius>0.18</radius><length>0.7</length></cylinder></geometry>
          <material>
            <ambient>{pants_color}</ambient>
            <diffuse>{pants_color}</diffuse>
          </material>
        </visual>
        <visual name="head">
          <pose>0 0 0.8 0 0 0</pose>
          <geometry><sphere><radius>0.13</radius></sphere></geometry>
          <material>
            <ambient>0.85 0.72 0.60 1</ambient>
            <diffuse>0.95 0.78 0.65 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>70.0</mass>
          <inertia>
            <ixx>10.0</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>10.0</iyy><iyz>0</iyz>
            <izz>2.5</izz>
          </inertia>
        </inertial>
      </link>

      <!-- Yaw rotation, with angular damping. -->
      <joint name="j_yaw" type="revolute">
        <parent>y_slider</parent>
        <child>base_link</child>
        <axis>
          <xyz>0 0 1</xyz>
          <limit><lower>-1e16</lower><upper>1e16</upper></limit>
          <dynamics><damping>{angular_damping:.1f}</damping></dynamics>
        </axis>
      </joint>

      <!-- TrajectoryFollower applies force at base_link CG; the joint
           chain redirects it into linear/angular motion. Terminal
           speed ≈ force / linear_damping. -->
      <plugin filename="gz-sim-trajectory-follower-system"
              name="gz::sim::systems::TrajectoryFollower">
        <link_name>base_link</link_name>
        <loop>true</loop>
        <force>{force:.1f}</force>
        <torque>{torque:.1f}</torque>
        <waypoints>
{wp_xml}
        </waypoints>
      </plugin>
    </model>'''


def build_people() -> str:
    # West corridor: X=-10 (safely between barrier_west at X=-5 and wall_west
    # at X=-15). Y ∈ [-17, +17] (3 m margin to separators at Y=±20).
    west_wp = [(-10.0, -17.0), (-10.0, +17.0)]
    east_wp = [(+10.0, +17.0), (+10.0, -17.0)]

    # Tuning (joint-chain mechanics, no ground friction):
    #   terminal velocity = force / linear_damping
    #   terminal angular velocity = torque / angular_damping
    # For walking pace ~0.5 m/s with force=275: damping=550.
    p1 = _person_model(
        'person_west',
        initial_xy=(-10.0, -17.0),
        initial_yaw=1.5708,    # facing +Y (north), toward first waypoint
        waypoints_xy=west_wp,
        force=275.0, torque=80.0,
        linear_damping=550.0,    # 275/550 = 0.50 m/s ≈ caminar normal
        angular_damping=50.0,    # 80/50  = 1.6 rad/s → gira en ~1 s
        shirt_color='0.10 0.40 0.15 1',  # green
        pants_color='0.08 0.12 0.35 1',  # blue jeans
    )
    p2 = _person_model(
        'person_east',
        initial_xy=(+10.0, +17.0),
        initial_yaw=-1.5708,   # facing -Y (south)
        waypoints_xy=east_wp,
        force=275.0, torque=80.0,
        linear_damping=800.0,    # 275/800 = 0.34 m/s ≈ caminar despacio
        angular_damping=60.0,    # 80/60  = 1.3 rad/s
        shirt_color='0.85 0.15 0.10 1',  # red
        pants_color='0.18 0.18 0.22 1',  # dark grey
    )
    return (
        '\n    <!-- ═════════ PEOPLE (side corridors only, no tube/no rooms) ═════════ -->'
        + p1 + p2
        + '\n    <!-- ═════════ END PEOPLE ═════════ -->\n'
    )


def main():
    print(f'[base] {BASE_WORLD}')
    out_xml = generate_defects_world()
    actor_xml = build_people()
    close = out_xml.rfind('</world>')
    if close < 0:
        sys.exit('SDF sin </world>, abortando.')
    final_xml = out_xml[:close] + actor_xml + out_xml[close:]
    with open(OUTPUT_WORLD, 'w', encoding='utf-8') as fh:
        fh.write(final_xml)
    print(f'[done] → {OUTPUT_WORLD}')
    print(f'       Tamaño: {os.path.getsize(OUTPUT_WORLD)/1024:.1f} KiB')


if __name__ == '__main__':
    main()
