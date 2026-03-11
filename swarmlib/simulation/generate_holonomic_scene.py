"""
Scene generator for holonomic robots with L-shaped forklift carriages.

Each robot is a box body with x-slide / y-slide / z-hinge joints (velocity
actuators in world frame).  An L-shaped carriage is rigidly attached on top:

  fork_wall  ← vertical backplate (payload presses against this)
  fork_base  ← horizontal plate   (payload rests on this)

MuJoCo <force> sensors sit on each carriage component so force on the
fork_base and fork_wall bodies can be read from data.sensordata.

Formation
---------
A list of (x_offset, y_offset, yaw) tuples, one per robot, measured from the
payload centre.  Default: all robots on the -x face of the payload (side-push
layout), which subsumes the old MPC scaling demo while supporting holonomic
drive.

Robots can also be arranged to surround the payload on multiple sides; set
yaw so each robot's +x axis points toward the payload centre.
"""

import math
from pathlib import Path
from typing import List, Optional, Tuple

# ---------------------------------------------------------------------------
# Robot geometry constants (metres, MuJoCo half-sizes for box geoms)
# ---------------------------------------------------------------------------
_RHX, _RHY, _RHZ = 0.10, 0.10, 0.05   # robot body half-sizes (0.2 × 0.2 × 0.1 m)

_FORK_LH = 0.075   # fork base half-length  (x in robot frame)
_FORK_WH = 0.10    # fork base half-width   (y — matches robot width)
_FORK_TH = 0.0075  # fork base half-thickness (z)

_WALL_DH = 0.010   # fork wall half-depth   (x)
_WALL_WH = 0.10    # fork wall half-width   (y — matches robot width)
_WALL_HH = 0.040   # fork wall half-height  (z)

# Positions of carriage child bodies relative to robot body frame origin
# (robot body centre is at z = _RHZ above the floor)
_FORK_BASE_POS = (_RHX + _FORK_LH, 0.0, _RHZ + _FORK_TH)  # (0.175, 0, 0.0575)
_FORK_WALL_POS = (_RHX + _WALL_DH, 0.0, _RHZ + _WALL_HH)  # (0.110, 0, 0.090)

# World-z of the top face of the fork base (payload rests just above this)
FORK_TOP_Z = _RHZ + _FORK_BASE_POS[2] + _FORK_TH  # 0.05 + 0.0575 + 0.0075 = 0.115


# ---------------------------------------------------------------------------
# Formation helpers
# ---------------------------------------------------------------------------

def side_push_formation(
    n: int,
    spacing: float = 0.35,
    payload_hx: float = 0.20,
) -> List[Tuple[float, float, float]]:
    """
    All robots lined up on the -x face of the payload, evenly spaced in y.
    Yaw = 0 so each robot's +x axis points toward the payload (inward = +x).
    standoff is chosen so the fork wall sits flush against the payload -x face.
    """
    standoff = payload_hx + _RHX  # robot body front at payload -x face
    out = []
    for i in range(n):
        y = 0.0 if n == 1 else (i / (n - 1) - 0.5) * (n - 1) * spacing
        out.append((-standoff, y, 0.0))
    return out


# ---------------------------------------------------------------------------
# XML fragment generators
# ---------------------------------------------------------------------------

_ROBOT_COLORS = [
    "0.20 0.45 0.70 1",  # blue
    "0.85 0.33 0.10 1",  # orange
    "0.17 0.63 0.17 1",  # green
    "0.84 0.15 0.16 1",  # red
    "0.58 0.40 0.74 1",  # purple
    "0.55 0.34 0.29 1",  # brown
    "0.89 0.47 0.76 1",  # pink
    "0.50 0.50 0.50 1",  # gray
]


def _robot_xml(robot_id: int, wx: float, wy: float, yaw: float) -> str:
    """XML for one holonomic robot + L-carriage at world position (wx, wy, yaw)."""
    bx, by, bz = _FORK_BASE_POS
    wx2, wy2, wz = _FORK_WALL_POS
    color = _ROBOT_COLORS[robot_id % len(_ROBOT_COLORS)]
    ri = robot_id
    return f'''\
    <!-- Robot {ri} -->
    <body name="robot_{ri}" pos="{wx:.4f} {wy:.4f} {_RHZ:.4f}" euler="0 0 {yaw:.6f}">
      <joint name="robot_{ri}_x"   type="slide" axis="1 0 0" limited="false" damping="1.0"/>
      <joint name="robot_{ri}_y"   type="slide" axis="0 1 0" limited="false" damping="1.0"/>
      <joint name="robot_{ri}_yaw" type="hinge" axis="0 0 1" limited="false" damping="10.0"/>
      <geom type="box" size="{_RHX} {_RHY} {_RHZ}" rgba="{color}"
            friction="0.8 0.005 0.0001"/>

      <!-- Fork base: payload rests on top; force sensor reads normal (z) load -->
      <body name="robot_{ri}_fork_base" pos="{bx:.4f} {by:.4f} {bz:.4f}">
        <geom name="robot_{ri}_fork_base_geom" type="box"
              size="{_FORK_LH:.4f} {_FORK_WH:.4f} {_FORK_TH:.4f}"
              rgba="0.75 0.75 0.75 1" friction="0.5 0.005 0.0001"/>
        <site name="robot_{ri}_base_site" pos="0 0 {_FORK_TH:.4f}"/>
      </body>

      <!-- Fork wall: payload presses against this; force sensor reads shear (x) load -->
      <body name="robot_{ri}_fork_wall" pos="{wx2:.4f} {wy2:.4f} {wz:.4f}">
        <geom name="robot_{ri}_fork_wall_geom" type="box"
              size="{_WALL_DH:.4f} {_WALL_WH:.4f} {_WALL_HH:.4f}"
              rgba="0.60 0.60 0.60 1" friction="0.5 0.005 0.0001"/>
        <site name="robot_{ri}_wall_site" pos="{_WALL_DH:.4f} 0 0"/>
      </body>
    </body>'''


def _actuator_xml(robot_id: int, max_vel: float = 2.0, kv: float = 50.0) -> str:
    """Velocity actuators on x-slide and y-slide joints."""
    ri = robot_id
    r = f"-{max_vel} {max_vel}"
    return f'''\
    <velocity name="robot_{ri}_vx" joint="robot_{ri}_x"
              kv="{kv}" ctrllimited="true" ctrlrange="{r}"/>
    <velocity name="robot_{ri}_vy" joint="robot_{ri}_y"
              kv="{kv}" ctrllimited="true" ctrlrange="{r}"/>'''


def _sensor_xml(n_robots: int) -> str:
    """Force sensors on fork_base and fork_wall bodies, one pair per robot.

    Sensor output is a 3-vector [fx, fy, fz] in the site's local frame:
      base_site: fz > 0 when payload presses down on the fork base (weight)
      wall_site: fx > 0 when payload presses against the fork wall (shear)
    """
    lines = []
    for i in range(n_robots):
        lines.append(
            f'    <force name="robot_{i}_base_force" site="robot_{i}_base_site"/>'
        )
        lines.append(
            f'    <force name="robot_{i}_wall_force" site="robot_{i}_wall_site"/>'
        )
    return '\n'.join(lines)


# ---------------------------------------------------------------------------
# Main generator
# ---------------------------------------------------------------------------

def generate_holonomic_scene(
    n_robots: int,
    formation: Optional[List[Tuple[float, float, float]]] = None,
    payload_pos: Tuple[float, float] = (0.0, 0.0),
    payload_size: Optional[Tuple[float, float, float]] = None,
    payload_mass: float = 10.0,
    goal: Tuple[float, float, float] = (5.0, 0.0, 0.0),
    output_path=None,
) -> str:
    """
    Generate a MuJoCo XML scene with holonomic robots and L-shaped carriages.

    Parameters
    ----------
    n_robots     : number of robots
    formation    : list of (x_off, y_off, yaw) per robot, relative to payload_pos.
                   Default: side_push_formation() — all robots on -x face, yaw=0.
    payload_pos  : (x, y) of payload centre in world frame.
    payload_size : (hx, hy, hz) half-sizes of payload box.  Auto-computed if None.
    payload_mass : kg
    goal         : (x, y, theta) goal pose for the payload.
    output_path  : if given, write XML to this path and create parent dirs.

    Returns
    -------
    xml_content : str
    """
    px, py = payload_pos
    default_phx = _RHX + 0.10  # 0.20 m — wider than robot front face

    if formation is None:
        formation = side_push_formation(n_robots, payload_hx=default_phx)
    if len(formation) != n_robots:
        raise ValueError(
            f"formation must have {n_robots} entries, got {len(formation)}"
        )

    # Auto-compute payload size to span the formation
    if payload_size is None:
        ys = [off[1] for off in formation]
        phx = default_phx
        phy = max(0.20, (max(ys) - min(ys)) / 2 + _FORK_WH + 0.05)
        phz = 0.10
        payload_size = (phx, phy, phz)

    phx, phy, phz = payload_size

    # Payload centre z: slightly above fork base top so it settles naturally
    payload_z = FORK_TOP_Z + phz + 0.005

    goal_x, goal_y, _ = goal

    # Moment of inertia for solid box
    m = payload_mass
    Ixx = (1 / 12) * m * ((2 * phy) ** 2 + (2 * phz) ** 2)
    Iyy = (1 / 12) * m * ((2 * phx) ** 2 + (2 * phz) ** 2)
    Izz = (1 / 12) * m * ((2 * phx) ** 2 + (2 * phy) ** 2)

    # Assemble XML fragments
    robot_bodies = '\n'.join(
        _robot_xml(i, px + ox, py + oy, yaw)
        for i, (ox, oy, yaw) in enumerate(formation)
    )
    actuators = '\n'.join(_actuator_xml(i) for i in range(n_robots))
    sensors = _sensor_xml(n_robots)

    scene_centre_x = (px + goal_x) / 2
    extent = max(abs(goal_x - px), 3.0)

    xml = f"""<mujoco model="holonomic_carriage">
  <compiler angle="radian" autolimits="true"/>

  <option integrator="Euler" timestep="0.005">
    <flag contact="enable" warmstart="enable"/>
  </option>

  <size njmax="1000" nconmax="500"/>

  <statistic center="{scene_centre_x:.2f} {py:.2f} 0.3" extent="{extent:.1f}"/>

  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="120" elevation="-20"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0"
             width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge"
             rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3" markrgb="0.8 0.8 0.8"
             width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true"
              texrepeat="5 5" reflectance="0.2"/>
  </asset>

  <default>
    <joint armature="0.01"/>
    <geom condim="4" solimp="0.99 0.99 0.001" solref="0.01 1"/>
  </default>

  <worldbody>
    <light pos="{scene_centre_x:.2f} 0 3" dir="0 0 -1" directional="true"/>
    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>

    <!-- Goal marker (visual only) -->
    <geom name="goal_marker" type="box"
          size="{phx:.3f} {phy:.3f} 0.005"
          pos="{goal_x:.3f} {goal_y:.3f} 0.005"
          rgba="0.2 0.8 0.2 0.3" contype="0" conaffinity="0"/>

{robot_bodies}

    <!-- Payload -->
    <body name="payload" pos="{px:.4f} {py:.4f} {payload_z:.4f}">
      <freejoint name="payload_joint"/>
      <inertial pos="0 0 0" mass="{m:.2f}"
                diaginertia="{Ixx:.6f} {Iyy:.6f} {Izz:.6f}"/>
      <geom name="payload_geom" type="box"
            size="{phx:.4f} {phy:.4f} {phz:.4f}"
            rgba="0.85 0.55 0.20 0.9" friction="0.5 0.005 0.0001"/>
    </body>
  </worldbody>

  <actuator>
{actuators}
  </actuator>

  <sensor>
{sensors}
  </sensor>
</mujoco>
"""

    if output_path is not None:
        p = Path(output_path)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(xml)

    return xml
