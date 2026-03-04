"""
SwarmTransportEnv: unified MuJoCo environment for multi-robot payload transport.

Wraps raw MuJoCo model/data into a clean step/reset interface. Controllers
receive Cartesian [vx, vy] commands; the env handles diff-drive actuation
internally via cartesian_to_diff_drive().

Force sensing uses data.cfrc_ext (MuJoCo's summed external contact force
on each body in world frame, shape (nbody, 6) as [torque(3), force(3)]).
We extract [fx, fy, torque_z] for each robot body.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Optional

import numpy as np

try:
    import mujoco
except ImportError:
    raise ImportError("MuJoCo Python bindings required. Install with: pip install mujoco")

# Make scenarios importable when env.py is run directly
_pkg_root = Path(__file__).parent.parent
if str(_pkg_root) not in sys.path:
    sys.path.insert(0, str(_pkg_root))

from scenarios.generate_mpc_scene import generate_mpc_scene
from controllers.base_controller import cartesian_to_diff_drive


class SwarmTransportEnv:
    """
    Unified simulation environment for swarm payload transport.

    Exposes a reset/step interface. Controllers output Cartesian [vx, vy]
    per robot; this class converts to diff-drive wheel velocities and applies
    them to MuJoCo actuators.

    Parameters
    ----------
    n_robots : int
        Number of robots.
    scene_xml : str or Path, optional
        Path to an existing MuJoCo XML scene. If None, a scene is auto-generated
        using generate_mpc_scene().
    goal_pos : array-like, optional
        Goal position [x_goal, y_goal, theta_goal]. Defaults to [5, 0, 0].
    dt_control : float
        Control time step (seconds). The MuJoCo physics timestep is read from
        the XML; we step physics once per control call.
    push_distance : float
        Push distance used only when auto-generating a scene (scene_xml=None).
    """

    def __init__(
        self,
        n_robots: int,
        scene_xml: Optional[str | Path] = None,
        goal_pos=None,
        dt_control: float = 0.05,
        push_distance: float = 5.0,
    ):
        self.n_robots = n_robots
        self.dt = dt_control
        self._goal_pos = np.array(goal_pos if goal_pos is not None else [5.0, 0.0, 0.0],
                                  dtype=float)

        # Load (or generate) MuJoCo model
        if scene_xml is None:
            scene_path = self._auto_generate_scene(push_distance)
        else:
            scene_path = Path(scene_xml)

        self.model = mujoco.MjModel.from_xml_path(str(scene_path))
        self.data = mujoco.MjData(self.model)

        # Cache body / actuator IDs
        self._setup_ids()

    # ------------------------------------------------------------------
    # Setup helpers
    # ------------------------------------------------------------------

    def _auto_generate_scene(self, push_distance: float) -> Path:
        scene_dir = _pkg_root / "scenarios" / "scenes"
        scene_dir.mkdir(parents=True, exist_ok=True)
        scene_path = scene_dir / f"mpc_scene_n{self.n_robots}.xml"
        generate_mpc_scene(
            num_robots=self.n_robots,
            push_distance=push_distance,
            output_path=str(scene_path),
        )
        return scene_path

    def _setup_ids(self):
        """Cache MuJoCo body and actuator IDs for fast lookup."""
        self._payload_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "payload"
        )

        self._robot_body_ids = []
        self._robot_actuators = []  # list of (left_id, right_id)

        for i in range(self.n_robots):
            body_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_BODY, f"robot_{i}_base"
            )
            self._robot_body_ids.append(body_id)

            left_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"robot_{i}_left_actuator"
            )
            right_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"robot_{i}_right_actuator"
            )
            self._robot_actuators.append((left_id, right_id))

    # ------------------------------------------------------------------
    # State extraction helpers
    # ------------------------------------------------------------------

    def _read_payload_state(self) -> np.ndarray:
        """Return payload state [x, y, theta, vx, vy, omega]."""
        pos = self.data.xpos[self._payload_id][:2]
        quat = self.data.xquat[self._payload_id]  # [qw, qx, qy, qz]
        vel = self.data.cvel[self._payload_id]     # [wx, wy, wz, vx, vy, vz]

        qw, qx, qy, qz = quat
        theta = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy**2 + qz**2))
        vx, vy = vel[3], vel[4]
        omega = vel[2]
        return np.array([pos[0], pos[1], theta, vx, vy, omega])

    def _read_robot_states(self) -> np.ndarray:
        """Return robot states (n, 4) [x, y, vx, vy]."""
        states = np.zeros((self.n_robots, 4))
        for i, body_id in enumerate(self._robot_body_ids):
            pos = self.data.xpos[body_id][:2]
            vel = self.data.cvel[body_id][3:5]  # linear vx, vy
            states[i] = [pos[0], pos[1], vel[0], vel[1]]
        return states

    def _read_forces(self) -> np.ndarray:
        """
        Return per-robot external forces (n, 3) [fx, fy, torque_z] in world frame.

        Iterates over all active contacts. For each contact involving a robot body,
        mj_contactForce gives [f_normal, f_tang1, f_tang2, ...] in the contact frame.
        The contact frame matrix (stored row-major as contact axes in world frame)
        rotates the contact-frame force into world frame.

        Forces from ground contacts are excluded; robot-payload and robot-robot
        contact forces are included.
        """
        # Index 0 = world body; we want to exclude world-body contacts
        WORLD_BODY = 0

        # Build a set of robot body IDs for fast lookup
        robot_id_set = set(self._robot_body_ids)

        forces = np.zeros((self.n_robots, 3))
        contact_buf = np.zeros(6)  # [f1,f2,f3,t1,t2,t3] in contact frame

        for ci in range(self.data.ncon):
            contact = self.data.contact[ci]
            b1 = self.model.geom_bodyid[contact.geom1]
            b2 = self.model.geom_bodyid[contact.geom2]

            # Skip pure ground contacts (neither body is a robot)
            if b1 == WORLD_BODY and b2 not in robot_id_set:
                continue
            if b2 == WORLD_BODY and b1 not in robot_id_set:
                continue

            mujoco.mj_contactForce(self.model, self.data, ci, contact_buf)

            # Rotate force from contact frame to world frame
            # contact.frame shape (9,): row i = i-th contact axis in world frame
            frame = contact.frame.reshape(3, 3)
            force_world = frame.T @ contact_buf[:3]  # [fx, fy, fz] world

            # Accumulate: force on body of geom1
            if b1 in robot_id_set and b2 != WORLD_BODY:
                ri = self._robot_body_ids.index(b1)
                forces[ri, 0] += force_world[0]
                forces[ri, 1] += force_world[1]
                forces[ri, 2] += contact_buf[5]  # torque_z stays in contact frame

            # Newton's 3rd law: force on body of geom2 is the negative
            if b2 in robot_id_set and b1 != WORLD_BODY:
                ri = self._robot_body_ids.index(b2)
                forces[ri, 0] -= force_world[0]
                forces[ri, 1] -= force_world[1]
                forces[ri, 2] -= contact_buf[5]

        return forces

    def _obs(self) -> dict:
        return {
            'payload': self._read_payload_state(),
            'robots': self._read_robot_states(),
            'forces': self._read_forces(),
        }

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def reset(self) -> dict:
        """Reset simulation to initial state. Returns observation dict."""
        mujoco.mj_resetData(self.model, self.data)
        return self._obs()

    def step(self, controls: np.ndarray) -> dict:
        """
        Apply Cartesian velocity commands and step physics.

        Parameters
        ----------
        controls : np.ndarray, shape (n, 2)
            [vx, vy] in m/s (world frame) for each robot.

        Returns
        -------
        dict with keys 'payload', 'robots', 'forces'
        """
        controls = np.asarray(controls, dtype=float)
        if controls.shape != (self.n_robots, 2):
            raise ValueError(
                f"controls must have shape ({self.n_robots}, 2), got {controls.shape}"
            )

        # Convert Cartesian commands to wheel velocities and apply
        for i, (left_id, right_id) in enumerate(self._robot_actuators):
            # Robot heading from xmat (rotation matrix, row-major, 3x3)
            xmat = self.data.xmat[self._robot_body_ids[i]].reshape(3, 3)
            heading = np.arctan2(xmat[1, 0], xmat[0, 0])  # yaw from rotation matrix

            vx, vy = controls[i]
            v_left, v_right = cartesian_to_diff_drive(vx, vy, heading)
            self.data.ctrl[left_id] = v_left
            self.data.ctrl[right_id] = v_right

        mujoco.mj_step(self.model, self.data)
        return self._obs()

    # ------------------------------------------------------------------
    # Convenience properties
    # ------------------------------------------------------------------

    @property
    def payload_state(self) -> np.ndarray:
        """Current payload state (6,) [x, y, theta, vx, vy, omega]."""
        return self._read_payload_state()

    @property
    def robot_states(self) -> np.ndarray:
        """Current robot states (n, 4) [x, y, vx, vy]."""
        return self._read_robot_states()

    @property
    def forces(self) -> np.ndarray:
        """Current per-robot external forces (n, 3) [fx, fy, torque_z]."""
        return self._read_forces()

    @property
    def time(self) -> float:
        """Current simulation time in seconds."""
        return float(self.data.time)

    @property
    def goal_pos(self) -> np.ndarray:
        """Goal position (3,) [x_goal, y_goal, theta_goal]."""
        return self._goal_pos.copy()

    def close(self):
        """Release resources (no-op for headless MuJoCo, kept for API symmetry)."""
        pass
