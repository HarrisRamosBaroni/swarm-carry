"""
MecanumTransportEnv: MuJoCo environment for Summit XL Steel robots.

Control interface: world-frame [vx, vy] per robot.
Internally converts to body-frame using each robot's current yaw, then applies
mecanum inverse kinematics to get wheel velocity targets, then PD control
to produce motor torques (same as mecanum_ros2_demo).

Observation dict
----------------
Always present:
  'payload'      : (6,)   [x, y, theta, vx, vy, omega]
  'robots'       : (n, 5) [x, y, theta, vx, vy] per robot (world frame)

Only when with_carriage=True:
  'base_forces'  : (n,) normal contact force between fork_base and payload (N, ≥0).
                   Sum equals payload weight; no taring required.
  'wall_forces'  : (n,) normal contact force between fork_wall and payload (N, ≥0).

Fork bodies have soft slide joints (spring k, damper d) that break rigid-rigid
static indeterminacy so each robot gets a unique, physically meaningful share of
the load.  Forces are read via mj_contactForce() — the constraint solver output —
rather than from spring displacement, so readings are independent of k and are
unaffected by joint range limits.

Fork geoms use contype=2 / conaffinity=1 so they collide with the payload
(contype=1) but not with each other, preventing spurious inter-robot contacts
when robots are close.
"""

from __future__ import annotations

import tempfile
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

try:
    import mujoco
except ImportError:
    raise ImportError("MuJoCo Python bindings required: pip install mujoco")

from swarmlib.simulation.generate_mecanum_scene import (
    generate_mecanum_scene,
    mecanum_side_push_formation,
    WHEEL_RADIUS, LX, LY, FORK_TOP_Z_WORLD,
    CONTACT_TIMECONST,
)

# Summit XL Steel mecanum inverse kinematics constants
_L = LX + LY                   # geometric factor for yaw contribution
_WHEEL_KV = 200.0               # PD gain (Nm per rad/s error)

# Actuator order in the generated XML: fr, fl, br, bl
_WHEEL_NAMES = [
    'front_right_wheel_rolling_joint',
    'front_left_wheel_rolling_joint',
    'back_right_wheel_rolling_joint',
    'back_left_wheel_rolling_joint',
]


def _body_to_wheel(vx: float, vy: float, omega: float = 0.0) -> np.ndarray:
    """
    Mecanum IK: body-frame [vx, vy, omega] → wheel angular velocities [fr,fl,br,bl].
    Convention: vx>0=forward, vy>0=left, omega>0=CCW (same as mecanum_ros2_demo).
    """
    r = WHEEL_RADIUS
    fl = ( vx - vy - _L * omega) / r
    fr = ( vx + vy + _L * omega) / r
    bl = ( vx + vy - _L * omega) / r
    br = ( vx - vy + _L * omega) / r
    return np.array([fr, fl, br, bl])   # actuator order: fr, fl, br, bl


class MecanumTransportEnv:
    """
    Simulation environment for Summit XL Steel mecanum robots with L-carriages.

    Parameters
    ----------
    n_robots     : number of robots
    scene_xml    : path to an existing MuJoCo XML; auto-generated if None
    formation    : [(x_off, y_off, yaw), ...] per robot relative to payload_pos.
                   Default: mecanum_side_push_formation()
    goal         : (x, y, theta) goal pose for the payload
    payload_pos  : (x, y) initial payload centre
    payload_size : (hx, hy, hz) half-sizes; auto if None
    payload_mass : kg
    wheel_kv     : PD gain for wheel velocity tracking (Nm per rad/s)
    dt_control   : control period (s); physics sub-stepped to match
    scenes_dir   : directory for auto-generated scenes (default: system temp)
    vel_feedback : enable per-robot PI velocity feedback (world frame).
                   Compensates for strafe inefficiency and other tracking errors.
    vel_fb_kp    : proportional gain for velocity feedback
    vel_fb_ki    : integral gain for velocity feedback
    vel_fb_integral_max : anti-windup clamp on integral magnitude (m/s)
    """

    def __init__(
        self,
        n_robots: int,
        scene_xml: Optional[str | Path] = None,
        formation: Optional[List[Tuple[float, float, float]]] = None,
        goal: Tuple[float, float, float] = (5.0, 0.0, 0.0),
        payload_pos: Tuple[float, float] = (0.0, 0.0),
        payload_size: Optional[Tuple[float, float, float]] = None,
        payload_mass: float = 20.0,
        payload_density: Optional[float] = None,
        with_carriage: bool = True,
        wheel_kv: float = _WHEEL_KV,
        dt_control: float = 0.05,
        scenes_dir: Optional[str | Path] = None,
        vel_feedback: bool = False,
        vel_fb_kp: float = 2.0,
        vel_fb_ki: float = 5.0,
        vel_fb_integral_max: float = 2.0,
        contact_timeconst: float = CONTACT_TIMECONST,
    ):
        self.n_robots = n_robots
        self._dt = dt_control
        self._goal = np.array(goal, dtype=float)
        self._wheel_kv = wheel_kv
        self._with_carriage = with_carriage
        self._payload_density = payload_density
        self._contact_timeconst = contact_timeconst

        # Per-robot velocity feedback (PI controller on world-frame vel error)
        self._vel_feedback = vel_feedback
        self._vel_fb_kp = vel_fb_kp
        self._vel_fb_ki = vel_fb_ki
        self._vel_fb_int_max = vel_fb_integral_max
        self._vel_integral = np.zeros((n_robots, 2))

        self._scenes_dir = (
            Path(scenes_dir) if scenes_dir is not None
            else Path(tempfile.gettempdir()) / "swarm_scenes"
        )

        if formation is None:
            phx = payload_size[0] if payload_size is not None else 0.30
            formation = mecanum_side_push_formation(
                n_robots, payload_hx=phx, with_carriage=with_carriage
            )
        self._formation = formation
        # Initial yaw per robot (used for world→body-frame conversion)
        self._yaw0 = np.array([f[2] for f in formation], dtype=float)

        if scene_xml is None:
            scene_path = self._auto_generate(payload_pos, payload_size, payload_mass, goal)
        else:
            scene_path = Path(scene_xml)

        self.model = mujoco.MjModel.from_xml_path(str(scene_path))
        self.data  = mujoco.MjData(self.model)
        self._n_substeps = max(1, round(dt_control / self.model.opt.timestep))

        self._setup_ids()

    # ------------------------------------------------------------------
    # Setup
    # ------------------------------------------------------------------

    def _auto_generate(self, payload_pos, payload_size, payload_mass, goal) -> Path:
        self._scenes_dir.mkdir(parents=True, exist_ok=True)
        suffix = "carriage" if self._with_carriage else "push"
        scene_path = self._scenes_dir / f"mecanum_scene_{suffix}_n{self.n_robots}.xml"
        generate_mecanum_scene(
            n_robots=self.n_robots,
            formation=self._formation,
            payload_pos=payload_pos,
            payload_size=payload_size,
            payload_mass=payload_mass,
            payload_density=self._payload_density,
            goal=goal,
            with_carriage=self._with_carriage,
            contact_timeconst=self._contact_timeconst,
            output_path=str(scene_path),
        )
        return scene_path

    def _setup_ids(self):
        """Cache MuJoCo IDs for fast lookup at each step."""
        self._payload_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "payload"
        )

        self._base_ids: List[int]              = []  # base_footprint body per robot
        self._wheel_act_ids: List[np.ndarray]  = []  # (4,) actuator ids per robot
        self._wheel_jnt_dofadr: List[np.ndarray] = []  # qvel addresses for 4 wheels
        # Load-cell: body IDs for fork_base and fork_wall per robot
        self._fork_base_body_ids: List[int]    = []
        self._fork_wall_body_ids: List[int]    = []

        for i in range(self.n_robots):
            # Base body (for state reading)
            bid = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_BODY, f"robot_{i}_base_footprint"
            )
            self._base_ids.append(bid)

            # Wheel actuator IDs
            act_ids = []
            for wname in _WHEEL_NAMES:
                aid = mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_ACTUATOR,
                    f"robot_{i}_{wname}"
                )
                act_ids.append(aid)
            self._wheel_act_ids.append(np.array(act_ids, dtype=int))

            # Wheel joint qvel DOF addresses (for reading current wheel velocities)
            dof_addrs = []
            for wname in _WHEEL_NAMES:
                jid = mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_JOINT,
                    f"robot_{i}_{wname}"
                )
                dof_addrs.append(int(self.model.jnt_dofadr[jid]))
            self._wheel_jnt_dofadr.append(np.array(dof_addrs, dtype=int))

            if self._with_carriage:
                fb_id = mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_BODY, f"robot_{i}_fork_base"
                )
                fw_id = mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_BODY, f"robot_{i}_fork_wall"
                )
                self._fork_base_body_ids.append(fb_id)
                self._fork_wall_body_ids.append(fw_id)

    # ------------------------------------------------------------------
    # Control helpers
    # ------------------------------------------------------------------

    def _apply_controls(self, controls: np.ndarray) -> None:
        """
        Convert world-frame [vx, vy] → body-frame → mecanum IK → wheel vel targets
        → PD torques → data.ctrl.
        """
        for i in range(self.n_robots):
            vx_w, vy_w = controls[i]
            yaw = self._yaw0[i]

            # World → robot body frame
            vx_b =  vx_w * np.cos(yaw) + vy_w * np.sin(yaw)
            vy_b = -vx_w * np.sin(yaw) + vy_w * np.cos(yaw)

            # Mecanum IK: body vel → target wheel angular velocities
            target = _body_to_wheel(vx_b, vy_b, omega=0.0)

            # Current wheel velocities (from qvel)
            dof = self._wheel_jnt_dofadr[i]
            current = np.array([self.data.qvel[d] for d in dof])

            # PD torque
            torque = self._wheel_kv * (target - current)

            for j, aid in enumerate(self._wheel_act_ids[i]):
                self.data.ctrl[aid] = torque[j]

    # ------------------------------------------------------------------
    # State readers
    # ------------------------------------------------------------------

    def _read_payload_state(self) -> np.ndarray:
        pid = self._payload_id
        pos = self.data.xpos[pid][:2]
        qw, qx, qy, qz = self.data.xquat[pid]
        theta = np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2))
        vel = self.data.cvel[pid]
        return np.array([pos[0], pos[1], theta, vel[3], vel[4], vel[2]])

    def _read_robot_states(self) -> np.ndarray:
        states = np.zeros((self.n_robots, 5))
        for i, bid in enumerate(self._base_ids):
            pos = self.data.xpos[bid][:2]
            qw, qx, qy, qz = self.data.xquat[bid]
            theta = np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2))
            vel = self.data.cvel[bid][3:5]
            states[i] = [pos[0], pos[1], theta, vel[0], vel[1]]
        return states

    def _read_carriage_forces(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Read fork forces by summing mj_contactForce() over all contacts between
        each fork body and the payload.  result[0] is the normal force (always ≥0).
        """
        base = np.zeros(self.n_robots)
        wall = np.zeros(self.n_robots)
        _result = np.zeros(6)
        for c_id in range(self.data.ncon):
            c = self.data.contact[c_id]
            b1 = int(self.model.geom_bodyid[c.geom1])
            b2 = int(self.model.geom_bodyid[c.geom2])
            if b1 != self._payload_id and b2 != self._payload_id:
                continue
            mujoco.mj_contactForce(self.model, self.data, c_id, _result)
            f = _result[0]
            for i in range(self.n_robots):
                if b1 == self._fork_base_body_ids[i] or b2 == self._fork_base_body_ids[i]:
                    base[i] += f
                    break
                if b1 == self._fork_wall_body_ids[i] or b2 == self._fork_wall_body_ids[i]:
                    wall[i] += f
                    break
        return base, wall

    def _obs(self) -> dict:
        obs = {
            'payload': self._read_payload_state(),
            'robots':  self._read_robot_states(),
        }
        if self._with_carriage:
            base_f, wall_f = self._read_carriage_forces()
            obs['base_forces'] = base_f
            obs['wall_forces'] = wall_f
        return obs

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def reset(self) -> dict:
        mujoco.mj_resetData(self.model, self.data)
        self._vel_integral = np.zeros((self.n_robots, 2))
        for _ in range(self._n_substeps):
            mujoco.mj_step(self.model, self.data)
        return self._obs()

    def step(self, controls: np.ndarray) -> dict:
        """
        Apply world-frame velocity commands and step physics.

        Parameters
        ----------
        controls : (n, 2) [vx, vy] in m/s, world frame, per robot.
        """
        controls = np.asarray(controls, dtype=float)
        if controls.shape != (self.n_robots, 2):
            raise ValueError(
                f"controls must be ({self.n_robots}, 2), got {controls.shape}"
            )

        if self._vel_feedback:
            # Read actual world-frame velocity per robot
            actual_vel = np.zeros((self.n_robots, 2))
            for i, bid in enumerate(self._base_ids):
                actual_vel[i] = self.data.cvel[bid][3:5]

            # PI correction on velocity error
            vel_error = controls - actual_vel
            self._vel_integral += vel_error * self._dt
            np.clip(self._vel_integral, -self._vel_fb_int_max,
                    self._vel_fb_int_max, out=self._vel_integral)
            controls = (controls
                        + self._vel_fb_kp * vel_error
                        + self._vel_fb_ki * self._vel_integral)

        for _ in range(self._n_substeps):
            self._apply_controls(controls)
            mujoco.mj_step(self.model, self.data)
        return self._obs()

    @property
    def time(self) -> float:
        return float(self.data.time)

    @property
    def goal(self) -> np.ndarray:
        return self._goal.copy()

    def close(self):
        pass
