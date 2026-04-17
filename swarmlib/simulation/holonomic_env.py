"""
HolonomicTransportEnv: MuJoCo environment for holonomic robots with L-carriages.

Robots accept world-frame [vx, vy] commands.  The env converts these to the
REST-frame joint velocities needed by the slide actuators (i.e. it accounts
for each robot's initial yaw from the formation so the controller always works
in world frame regardless of formation geometry).

Observation dict keys
---------------------
  'payload'      : (6,)   [x, y, theta, vx, vy, omega]
  'robots'       : (n, 4) [x, y, vx, vy] per robot
  'base_forces'  : (n,) scalar Fz per robot — vertical load on fork base (N)
  'wall_forces'  : (n,) scalar Fx per robot — horizontal contact force on fork wall (N)
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

from swarmlib.simulation.generate_holonomic_scene import (
    generate_holonomic_scene,
    side_push_formation,
    FORK_TOP_Z,
)


class HolonomicTransportEnv:
    """
    Simulation environment for holonomic robots with L-shaped forklift carriages.

    Parameters
    ----------
    n_robots     : number of robots
    scene_xml    : path to an existing MuJoCo XML; if None a scene is auto-generated
    formation    : list of (x_off, y_off, yaw) per robot relative to payload_pos.
                   Default: side_push_formation() — all robots on the -x face.
    goal         : (x, y, theta) goal pose for the payload
    payload_pos  : (x, y) initial payload centre in world frame
    payload_size : (hx, hy, hz) half-sizes of the payload box; auto if None
    payload_mass : kg
    dt_control   : control period (s); physics is sub-stepped to match
    scenes_dir   : directory for auto-generated scenes (default: system temp dir)
    """

    def __init__(
        self,
        n_robots: int,
        scene_xml: Optional[str | Path] = None,
        formation: Optional[List[Tuple[float, float, float]]] = None,
        goal: Tuple[float, float, float] = (5.0, 0.0, 0.0),
        payload_pos: Tuple[float, float] = (0.0, 0.0),
        payload_size: Optional[Tuple[float, float, float]] = None,
        payload_mass: float = 10.0,
        dt_control: float = 0.05,
        scenes_dir: Optional[str | Path] = None,
    ):
        self.n_robots = n_robots
        self._dt = dt_control
        self._goal = np.array(goal, dtype=float)

        self._scenes_dir = (
            Path(scenes_dir)
            if scenes_dir is not None
            else Path(tempfile.gettempdir()) / "swarm_scenes"
        )

        # Resolve formation (needed both for scene generation and axis conversion)
        if formation is None:
            phx = 0.20
            if payload_size is not None:
                phx = payload_size[0]
            formation = side_push_formation(n_robots, payload_hx=phx)
        self._formation = formation
        # Store REST-frame yaw per robot for world→body-frame conversion
        self._yaw0 = np.array([f[2] for f in formation], dtype=float)

        # Load (or generate) scene
        if scene_xml is None:
            scene_path = self._auto_generate(
                payload_pos, payload_size, payload_mass, goal
            )
        else:
            scene_path = Path(scene_xml)

        self.model = mujoco.MjModel.from_xml_path(str(scene_path))
        self.data = mujoco.MjData(self.model)

        # Sub-steps per control call
        self._n_substeps = max(1, round(dt_control / self.model.opt.timestep))

        self._setup_ids()

    # ------------------------------------------------------------------
    # Setup
    # ------------------------------------------------------------------

    def _auto_generate(self, payload_pos, payload_size, payload_mass, goal) -> Path:
        self._scenes_dir.mkdir(parents=True, exist_ok=True)
        scene_path = self._scenes_dir / f"holonomic_scene_n{self.n_robots}.xml"
        generate_holonomic_scene(
            n_robots=self.n_robots,
            formation=self._formation,
            payload_pos=payload_pos,
            payload_size=payload_size,
            payload_mass=payload_mass,
            goal=goal,
            output_path=str(scene_path),
        )
        return scene_path

    def _setup_ids(self):
        """Cache MuJoCo IDs for fast lookup."""
        self._payload_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "payload"
        )

        self._robot_ids: List[int] = []
        self._actuator_ids: List[Tuple[int, int]] = []   # (vx_id, vy_id) per robot
        self._base_sensor_adr: List[int] = []
        self._wall_sensor_adr: List[int] = []

        for i in range(self.n_robots):
            body_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_BODY, f"robot_{i}"
            )
            self._robot_ids.append(body_id)

            vx_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"robot_{i}_vx"
            )
            vy_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"robot_{i}_vy"
            )
            self._actuator_ids.append((vx_id, vy_id))

            for name, store in [
                (f"robot_{i}_base_force", self._base_sensor_adr),
                (f"robot_{i}_wall_force", self._wall_sensor_adr),
            ]:
                sid = mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_SENSOR, name
                )
                store.append(int(self.model.sensor_adr[sid]))

    # ------------------------------------------------------------------
    # State readers
    # ------------------------------------------------------------------

    def _read_payload_state(self) -> np.ndarray:
        """[x, y, theta, vx, vy, omega]"""
        pos = self.data.xpos[self._payload_id][:2]
        qw, qx, qy, qz = self.data.xquat[self._payload_id]
        theta = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy ** 2 + qz ** 2))
        vel = self.data.cvel[self._payload_id]  # [wx, wy, wz, vx, vy, vz]
        return np.array([pos[0], pos[1], theta, vel[3], vel[4], vel[2]])

    def _read_robot_states(self) -> np.ndarray:
        """(n, 4) [x, y, vx, vy] per robot."""
        states = np.zeros((self.n_robots, 4))
        for i, bid in enumerate(self._robot_ids):
            pos = self.data.xpos[bid][:2]
            vel = self.data.cvel[bid][3:5]
            states[i] = [pos[0], pos[1], vel[0], vel[1]]
        return states

    def _read_carriage_forces(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Returns (base_forces, wall_forces), each shape (n,).

        base_forces[i] is the Fz scalar — vertical load on robot i's fork base.
        wall_forces[i] is the Fx scalar — horizontal contact force on robot i's fork wall.
        """
        base = np.zeros(self.n_robots)
        wall = np.zeros(self.n_robots)
        sd = self.data.sensordata
        for i in range(self.n_robots):
            a = self._base_sensor_adr[i]
            base[i] = sd[a + 2]   # Fz — vertical load on fork base
            a = self._wall_sensor_adr[i]
            wall[i] = sd[a]        # Fx — horizontal contact force on fork wall
        return base, wall

    def _obs(self) -> dict:
        base_f, wall_f = self._read_carriage_forces()
        return {
            'payload':     self._read_payload_state(),
            'robots':      self._read_robot_states(),
            'base_forces': base_f,
            'wall_forces': wall_f,
        }

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def reset(self) -> dict:
        """Reset to initial state and return observation."""
        mujoco.mj_resetData(self.model, self.data)
        for _ in range(self._n_substeps):
            mujoco.mj_step(self.model, self.data)
        return self._obs()

    def step(self, controls: np.ndarray) -> dict:
        """
        Apply world-frame velocity commands and step physics.

        Parameters
        ----------
        controls : (n, 2) array  [vx, vy] in m/s, world frame, per robot.

        Returns
        -------
        obs dict with keys 'payload', 'robots', 'base_forces', 'wall_forces'
        """
        controls = np.asarray(controls, dtype=float)
        if controls.shape != (self.n_robots, 2):
            raise ValueError(
                f"controls must be ({self.n_robots}, 2), got {controls.shape}"
            )

        for i, (vx_id, vy_id) in enumerate(self._actuator_ids):
            vx, vy = controls[i]
            yaw = self._yaw0[i]
            # Rotate world-frame velocity into the robot's REST frame
            # (the slide joint axes are fixed in the body rest frame defined by euler)
            v_fwd = vx * np.cos(yaw) + vy * np.sin(yaw)
            v_lat = -vx * np.sin(yaw) + vy * np.cos(yaw)
            self.data.ctrl[vx_id] = v_fwd
            self.data.ctrl[vy_id] = v_lat

        for _ in range(self._n_substeps):
            mujoco.mj_step(self.model, self.data)

        return self._obs()

    # ------------------------------------------------------------------
    # Convenience properties
    # ------------------------------------------------------------------

    @property
    def payload_state(self) -> np.ndarray:
        return self._read_payload_state()

    @property
    def robot_states(self) -> np.ndarray:
        return self._read_robot_states()

    @property
    def goal(self) -> np.ndarray:
        return self._goal.copy()

    @property
    def time(self) -> float:
        return float(self.data.time)

    def close(self):
        pass
