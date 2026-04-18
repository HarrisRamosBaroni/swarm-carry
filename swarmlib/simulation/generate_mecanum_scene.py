"""
Multi-robot scene generator using the Summit XL Steel mecanum model.

Each robot is a full Summit XL Steel (from models/holonomic_dp) with an
L-shaped forklift carriage attached to the top-front of the chassis.

The carriage sits on top of the robot (fork base at ~0.52m world height) so
that multiple robots can collectively support a payload from below.

Requires: models/holonomic_dp submodule initialised.
  git submodule update --init models/holonomic_dp
"""

import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import List, Optional, Tuple

# ---------------------------------------------------------------------------
# Paths to Summit XL Steel model assets
# ---------------------------------------------------------------------------
_SUMMIT_DIR = (
    Path(__file__).parent.parent.parent
    / "models" / "holonomic_dp" / "robots" / "summit_xl_description"
).resolve()
_URDF_XML    = _SUMMIT_DIR / "assets" / "summit_xls.urdf.xml"
_ACT_XML     = _SUMMIT_DIR / "assets" / "summit_xls_actuator.xml"
_MESH_DIR    = _SUMMIT_DIR / "meshes"

# ---------------------------------------------------------------------------
# Summit XL Steel geometry constants
# ---------------------------------------------------------------------------
WHEEL_RADIUS = 0.120    # m
LX = 0.2225             # half wheelbase (front/back)
LY = 0.2045             # half track width (left/right)

# Fork carriage dimensions (all in metres, MuJoCo half-sizes for box geoms)
# All positions are in the `base` body frame
_FORK_FRONT_X = 0.38    # x of robot front face in `base` frame
_FB_LH = 0.075          # fork base half-length (x)
_FB_WH = 0.20           # fork base half-width  (y)
_FB_TH = 0.010          # fork base half-thickness (z)
_FW_DH = 0.010          # fork wall half-depth   (x)
_FW_WH = 0.20           # fork wall half-width   (y)
_FW_HH = 0.040          # fork wall half-height  (z)

_FB_POS_X = _FORK_FRONT_X + _FB_LH   # 0.455 — fork base centre in base frame (x)
_FB_POS_Z = 0.392                      # just above chapas top plate at z=0.381
_FW_POS_X = _FORK_FRONT_X             # 0.38 — fork wall flush with front
_FW_POS_Z = _FB_POS_Z + _FW_HH        # 0.432

# World z of the fork base top face when robot stands on flat ground
# (base_footprint initial z = -0.009, base at +0.127, fork at +0.392, half-thick 0.010)
FORK_TOP_Z_WORLD = -0.009 + 0.127 + _FB_POS_Z + _FB_TH   # ≈ 0.520 m

# How far the fork wall outer face is forward of base_footprint origin in x
_FORK_WALL_REACH = _FORK_FRONT_X + 2 * _FW_DH  # 0.40 m

# Load-cell spring-damper parameters (both base and wall use the same values).
# Each fork plate/wall is attached to the robot chassis via a 1-DOF slide joint
# with a stiff linear spring + damper. We measure load as F = -(k·x + d·xdot):
# this breaks the rigid-rigid static indeterminacy in face-contact formations
# (see info/simulate_load_cells.md for motivation) and gives a physically
# meaningful per-robot reading.
#
# Tuning: with m=0.1 kg, k=2e4 N/m, d=100 N·s/m →
#   natural freq ω=447 rad/s (~71 Hz), slightly overdamped (ζ≈1.1),
#   settle <10 ms, static compression at 20 N load ≈1 mm.
LOAD_CELL_MASS      = 0.1       # kg — inertial mass of the sliding plate/wall
LOAD_CELL_STIFFNESS = 500.0     # N/m — soft spring; only breaks rigid-rigid indeterminacy,
                                 #   NOT used for force measurement (env reads cfrc_ext)
LOAD_CELL_DAMPING   = 40.0      # N·s/m — ζ≈1.15 relative to m_eff=0.6 kg
LOAD_CELL_RANGE     = 0.025     # m — half-range; equilibrium at ~12 mm, well within range
CONTACT_TIMECONST   = 0.01      # s — solref time constant for fork geoms


# ---------------------------------------------------------------------------
# Formation helpers
# ---------------------------------------------------------------------------

def mecanum_side_push_formation(
    n: int,
    spacing: float = 0.55,
    payload_hx: float = 0.30,
    with_carriage: bool = True,
) -> List[Tuple[float, float, float]]:
    """
    All robots on the -x face of the payload, evenly spaced in y, facing +x.
    spacing is wider than the box demo (Summit XL is ~0.55 m wide).

    with_carriage=True  — standoff positions the fork wall flush with the payload face.
    with_carriage=False — standoff positions the chassis front face flush with the
                          payload face (no carriage, direct body contact).
    """
    if with_carriage:
        standoff = payload_hx + _FORK_WALL_REACH
    else:
        # _FORK_FRONT_X is the x-position of the robot front face in the base frame.
        # Add 0.01 m clearance to avoid initial penetration.
        standoff = payload_hx + _FORK_FRONT_X + 0.01
    return [(-standoff, (i - (n - 1) / 2) * spacing, 0.0) for i in range(n)]


def face_contact_formation(
    n: int,
    payload_hx: float = 0.30,
    payload_hy: float = 0.30,
) -> List[Tuple[float, float, float]]:
    """
    Place *n* robots on distinct faces of a box payload, one robot per face,
    fork walls flush with each face.

    Face assignment (evenly spaced around the 4 faces):
      n=1: -x
      n=2: -x, +x  (opposing)
      n=3: -x, +y, -y
      n=4: -x, +y, +x, -y

    Parameters
    ----------
    payload_hx, payload_hy : payload box half-sizes (must match the scene).

    Returns
    -------
    [(x_off, y_off, yaw), ...] offsets from payload centre.
    """
    if n < 1 or n > 4:
        raise ValueError(f"face_contact_formation supports 1–4 robots, got {n}")

    # (dx, dy, yaw, standoff) per face.
    # dx/dy: unit direction from payload centre toward the robot.
    # yaw: robot heading so its front (+x body) faces the payload.
    _faces = [
        (-1,  0,  0.0,            payload_hx + _FORK_WALL_REACH),   # -x face
        ( 0,  1, -math.pi / 2,    payload_hy + _FORK_WALL_REACH),   # +y face
        ( 1,  0,  math.pi,        payload_hx + _FORK_WALL_REACH),   # +x face
        ( 0, -1,  math.pi / 2,    payload_hy + _FORK_WALL_REACH),   # -y face
    ]

    indices = [round(i * 4 / n) % 4 for i in range(n)]
    return [(dx * s, dy * s, yaw) for dx, dy, yaw, s in
            (_faces[fi] for fi in indices)]


# ---------------------------------------------------------------------------
# XML manipulation helpers
# ---------------------------------------------------------------------------

def _prefix_tree(elem: ET.Element, prefix: str) -> None:
    """Recursively prefix all name= and joint= attribute values."""
    if 'name' in elem.attrib:
        elem.attrib['name'] = prefix + elem.attrib['name']
    if 'joint' in elem.attrib:
        elem.attrib['joint'] = prefix + elem.attrib['joint']
    for child in elem:
        _prefix_tree(child, prefix)


def _make_fork_carriage(prefix: str, contact_timeconst: float = CONTACT_TIMECONST) -> List[ET.Element]:
    """
    Return [fork_base_body, fork_wall_body] ET elements for one robot.

    Each body is attached to the robot chassis via a 1-DOF slide joint with a
    stiff linear spring-damper, acting as a load cell:
      - fork_base: slides along the robot's +Z (vertical), measures normal load
      - fork_wall: slides along the robot's +X (forward),  measures push force
    """
    k = LOAD_CELL_STIFFNESS
    d = LOAD_CELL_DAMPING
    m = LOAD_CELL_MASS
    r = LOAD_CELL_RANGE
    I = 1.0e-4  # scalar inertia, same on each principal axis (small plate)

    fb = ET.Element('body', {
        'name': f'{prefix}fork_base',
        'pos':  f'{_FB_POS_X:.4f} 0 {_FB_POS_Z:.4f}',
    })
    ET.SubElement(fb, 'inertial', {
        'pos': '0 0 0',
        'mass': f'{m}',
        'diaginertia': f'{I} {I} {I}',
    })
    ET.SubElement(fb, 'joint', {
        'name': f'{prefix}fork_base_slide',
        'type': 'slide',
        'axis': '0 0 1',
        'stiffness': f'{k}',
        'damping':   f'{d}',
        'limited': 'true',
        'range': f'{-r} {r}',
    })
    ET.SubElement(fb, 'geom', {
        'type': 'box',
        'size': f'{_FB_LH:.4f} {_FB_WH:.4f} {_FB_TH:.4f}',
        'rgba': '0.75 0.75 0.75 1',
        'friction': '0.5 0.005 0.0001',
        'solref': f'{contact_timeconst} 1',
    })
    ET.SubElement(fb, 'site', {
        'name': f'{prefix}base_site',
        'pos':  f'0 0 {_FB_TH:.4f}',
    })

    fw = ET.Element('body', {
        'name': f'{prefix}fork_wall',
        'pos':  f'{_FW_POS_X:.4f} 0 {_FW_POS_Z:.4f}',
    })
    ET.SubElement(fw, 'inertial', {
        'pos': '0 0 0',
        'mass': f'{m}',
        'diaginertia': f'{I} {I} {I}',
    })
    ET.SubElement(fw, 'joint', {
        'name': f'{prefix}fork_wall_slide',
        'type': 'slide',
        'axis': '1 0 0',
        'stiffness': f'{k}',
        'damping':   f'{d}',
        'limited': 'true',
        'range': f'{-r} {r}',
    })
    ET.SubElement(fw, 'geom', {
        'type': 'box',
        'size': f'{_FW_DH:.4f} {_FW_WH:.4f} {_FW_HH:.4f}',
        'rgba': '0.60 0.60 0.60 1',
        'friction': '0.5 0.005 0.0001',
        'solref': f'{contact_timeconst} 1',
    })
    ET.SubElement(fw, 'site', {
        'name': f'{prefix}wall_site',
        'pos':  f'{_FW_DH:.4f} 0 0',
    })
    return [fb, fw]


def _robot_body_xml(robot_id: int, rx: float, ry: float, yaw: float,
                    with_carriage: bool = True,
                    contact_timeconst: float = CONTACT_TIMECONST) -> str:
    """
    Return the XML string for one prefixed Summit XL + optional carriage body,
    ready to paste inside <worldbody>.
    """
    prefix = f'robot_{robot_id}_'

    tree = ET.parse(_URDF_XML)
    root = tree.getroot()   # <mujocoinclude>

    # The single top-level child is base_footprint
    base_fp = root[0]

    # Apply name prefix to all bodies/joints in the tree
    _prefix_tree(base_fp, prefix)

    # Set initial position and orientation for this robot
    base_fp.attrib['pos']   = f'{rx:.4f} {ry:.4f} -0.009'
    base_fp.attrib['euler'] = f'0 0 {yaw:.6f}'

    if with_carriage:
        # Find prefixed `base` body and inject L-carriage
        base_body = None
        for child in base_fp:
            if child.tag == 'body' and child.attrib.get('name') == f'{prefix}base':
                base_body = child
                break
        if base_body is None:
            raise RuntimeError(f"Could not find '{prefix}base' body in URDF XML")
        for elem in _make_fork_carriage(prefix, contact_timeconst=contact_timeconst):
            base_body.append(elem)
    # TODO: with_carriage=False — optionally add a force site on the robot front face
    #       (a <site> on the chassis geom + <force> sensor) for contact force sensing
    #       without a physical carriage. Not needed for the simple push scenario.

    return ET.tostring(base_fp, encoding='unicode')


def _actuator_xml_for_robot(robot_id: int) -> str:
    """Return prefixed <motor ...> lines for one robot (no wrapping tag)."""
    prefix = f'robot_{robot_id}_'
    tree = ET.parse(_ACT_XML)
    root = tree.getroot()   # <mujocoinclude> → <actuator>
    actuator_elem = root.find('actuator')
    _prefix_tree(actuator_elem, prefix)
    # Return individual motor elements as text (skip the <actuator> wrapper)
    return '\n'.join(
        ET.tostring(m, encoding='unicode') for m in actuator_elem
    )


def _sensor_xml(n_robots: int) -> str:
    # Forces read directly from data.cfrc_ext in the env; no position sensors needed.
    return '    <!-- forces read via data.cfrc_ext; no sensors required -->'


def _no_sensor_xml() -> str:
    return '    <!-- no sensors — add carriage or front-face sites to enable force sensing -->'


# ---------------------------------------------------------------------------
# Main generator
# ---------------------------------------------------------------------------

def generate_mecanum_scene(
    n_robots: int,
    formation: Optional[List[Tuple[float, float, float]]] = None,
    payload_pos: Tuple[float, float] = (0.0, 0.0),
    payload_size: Optional[Tuple[float, float, float]] = None,
    payload_mass: float = 20.0,
    payload_density: Optional[float] = None,
    goal: Tuple[float, float, float] = (5.0, 0.0, 0.0),
    with_carriage: bool = True,
    contact_timeconst: float = CONTACT_TIMECONST,
    output_path=None,
) -> str:
    """
    Generate a MuJoCo scene with N Summit XL Steel robots.

    Parameters
    ----------
    n_robots        : number of robots
    formation       : [(x_off, y_off, yaw), ...] per robot, offsets from payload_pos.
                      Default: mecanum_side_push_formation(with_carriage=with_carriage).
    payload_pos     : (x, y) payload centre in world frame.
    payload_size    : (hx, hy, hz) half-sizes. Auto-computed from formation if None.
    payload_mass    : kg. Ignored if payload_density is set.
    payload_density : kg/m³. If given, mass = density × full box volume (overrides
                      payload_mass). Useful for lightweight payloads (e.g. 50 kg/m³).
    goal            : (x, y, theta) goal pose for the payload.
    with_carriage   : if True, attach L-shaped forklift carriages and force sensors.
                      if False, bare robots — payload sits on the ground and is pushed
                      by the chassis front face. No force sensors.
    output_path     : if given, write XML to this path.
    """
    if not _URDF_XML.exists():
        raise FileNotFoundError(
            f"Summit XL model not found: {_URDF_XML}\n"
            "Run: git submodule update --init models/holonomic_dp"
        )

    px, py = payload_pos
    default_phx = 0.30

    if formation is None:
        formation = mecanum_side_push_formation(
            n_robots, payload_hx=default_phx, with_carriage=with_carriage
        )
    if len(formation) != n_robots:
        raise ValueError(f"formation must have {n_robots} entries")

    if payload_size is None:
        ys = [off[1] for off in formation]
        phx = default_phx
        phy = max(0.30, (max(ys) - min(ys)) / 2 + _FB_WH + 0.10)
        phz = 0.12 if with_carriage else 0.20
        payload_size = (phx, phy, phz)

    phx, phy, phz = payload_size
    if with_carriage:
        payload_z = FORK_TOP_Z_WORLD + phz + 0.005
    else:
        payload_z = phz  # payload rests on the ground

    goal_x, goal_y, _ = goal
    if payload_density is not None:
        m = payload_density * (2 * phx) * (2 * phy) * (2 * phz)
    else:
        m = payload_mass
    Ixx = (1/12)*m*((2*phy)**2 + (2*phz)**2)
    Iyy = (1/12)*m*((2*phx)**2 + (2*phz)**2)
    Izz = (1/12)*m*((2*phx)**2 + (2*phy)**2)

    # --- Build XML fragments ---
    robot_bodies_xml = '\n'.join(
        _robot_body_xml(i, px + ox, py + oy, yaw,
                        with_carriage=with_carriage,
                        contact_timeconst=contact_timeconst)
        for i, (ox, oy, yaw) in enumerate(formation)
    )
    actuators_xml = '\n'.join(
        _actuator_xml_for_robot(i) for i in range(n_robots)
    )
    sensors_xml = _sensor_xml(n_robots) if with_carriage else _no_sensor_xml()

    scene_cx = (px + goal_x) / 2
    extent   = max(abs(goal_x - px), 4.0)

    xml = f"""<mujoco model="mecanum_carriage_swarm">
  <compiler angle="radian" autolimits="true" meshdir="{_MESH_DIR}"/>

  <option integrator="Euler" timestep="0.002">
    <flag contact="enable" warmstart="enable"/>
  </option>

  <size njmax="4000" nconmax="2000"/>

  <statistic center="{scene_cx:.2f} {py:.2f} 0.5" extent="{extent:.1f}"/>

  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="120" elevation="-20"/>
  </visual>

  <asset>
    <texture name="texplane" type="2d" builtin="checker"
             rgb1=".2 .3 .4" rgb2=".1 0.2 0.3" width="512" height="512"/>
    <material name="MatGnd" reflectance="0.5" texture="texplane"
              texrepeat="5 5" texuniform="true"/>
    <mesh name="summit_xls_chassis"          file="bases/xls/summit_xls_chassis.stl"/>
    <mesh name="summit_xls_chapas_inox_tapas" file="bases/xls/summit_xls_chapas_inox_tapas.stl"/>
    <mesh name="robotnik_logo_chasis"        file="bases/xls/robotnik_logo_chasis.stl"/>
    <mesh name="summit_xls_omni_wheel_1"     file="wheels/omni_wheel_1.stl"/>
    <mesh name="summit_xls_omni_wheel_2"     file="wheels/omni_wheel_2.stl"/>
    <mesh name="structure_hokuyo"            file="structures/structure_hokuyo.stl"/>
  </asset>

  <default>
    <joint armature="0.01"/>
    <geom condim="4" solimp="0.99 0.99 0.001" solref="0.01 1"/>
  </default>

  <worldbody>
    <light pos="{scene_cx:.2f} 0 4" dir="0 0 -1" directional="true"/>
    <geom name="floor" size="0 0 0.05" type="plane" material="MatGnd"/>

    <!-- Goal marker (visual only) -->
    <geom name="goal_marker" type="box"
          size="{phx:.3f} {phy:.3f} 0.005"
          pos="{goal_x:.3f} {goal_y:.3f} 0.005"
          rgba="0.2 0.8 0.2 0.3" contype="0" conaffinity="0"/>

{robot_bodies_xml}

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
{actuators_xml}
  </actuator>

  <sensor>
{sensors_xml}
  </sensor>
</mujoco>
"""

    if output_path is not None:
        p = Path(output_path)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(xml)

    return xml
