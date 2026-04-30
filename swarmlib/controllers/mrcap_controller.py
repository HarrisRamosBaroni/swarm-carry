"""
MRCapController: centralized factor-graph controller for multi-robot payload transport.

Implements the core idea from:
  Jaafar et al., "MR.CAP: Multi-Robot Joint Control and Planning for Object Transport",
  IEEE Control Systems Letters, 2024.

Adapted for holonomic (mecanum) robots: the diff-drive arc motion model is replaced
with a world-frame Euler integrator, which is exact for holonomic drive.

Factor graph structure (receding horizon of N steps):
  Variables:
    C_j in R^3  -- centroid pose [x, y, theta]   for j = k .. k+N
    U_j in R^3  -- centroid control [vx, vy, omega]  for j = k .. k+N-1

  Factors:
    current-state anchor  -- tight prior on C_k (current centroid pose)
    reference priors      -- PriorFactor on C_j toward linear-interp reference
    control regulariser   -- PriorFactor on U_j toward zero
    motion model          -- C_{j+1} = C_j + dt * U_j   (linear; exact Jacobians)
    terminal anchor       -- tight prior on C_{k+N} at goal

Per-robot velocity (holonomic rigid body):
  v_robot_i = v_c + omega_c x r_i
  where r_i = [dx, dy] from centroid to robot i (fixed at reset).
"""


import time
from functools import partial
from typing import Dict, Any, List, Optional

import numpy as np

try:
    import gtsam
except ImportError:
    raise ImportError("gtsam required: pip install gtsam")

from .base_controller import BaseController
from .centroid_estimator import CentroidEstimator


# ---------------------------------------------------------------------------
# Reusable factor error functions
# ---------------------------------------------------------------------------

def _prior_error(
    measurement: np.ndarray,
    this: gtsam.CustomFactor,
    values: gtsam.Values,
    jacobians: Optional[List[np.ndarray]],
) -> np.ndarray:
    """Prior factor: error = variable - measurement."""
    v = values.atVector(this.keys()[0])
    error = v - measurement
    if jacobians is not None:
        jacobians[0] = np.eye(len(measurement))
    return error


def _motion_model_error(
    dt: float,
    this: gtsam.CustomFactor,
    values: gtsam.Values,
    jacobians: Optional[List[np.ndarray]],
) -> np.ndarray:
    """
    Euler motion model: C_{j+1} = C_j + dt * U_j
    Keys: [C_j, U_j, C_{j+1}]
    error = C_{j+1} - (C_j + dt * U_j)
    """
    Cj  = values.atVector(this.keys()[0])
    Uj  = values.atVector(this.keys()[1])
    Cj1 = values.atVector(this.keys()[2])

    error = Cj1 - (Cj + dt * Uj)

    if jacobians is not None:
        I3 = np.eye(3)
        jacobians[0] = -I3
        jacobians[1] = -dt * I3
        jacobians[2] =  I3

    return error


# ---------------------------------------------------------------------------
# Controller
# ---------------------------------------------------------------------------

class MRCapController(BaseController):
    """
    Centralized factor-graph controller (MR.CAP, holonomic adaptation).

    Parameters
    ----------
    num_robots : int
    formation  : list of (x_off, y_off, yaw) per robot relative to payload centre,
                 same format as MecanumTransportEnv's `formation` parameter.
                 If None, robots are placed on a ring of radius r_formation.
    config     : optional dict:
        horizon           int    receding horizon N (default 15)
        sigma_x           float  std-dev for reference trajectory priors (default 0.5)
        sigma_u           float  std-dev for control regularisation (default 0.3)
        sigma_anchor      float  std-dev for current-state / terminal anchors (default 0.01)
        sigma_mm          float  std-dev for motion model factor (default 1e-4)
        v_max             float  per-robot speed clamp m/s (default 1.0)
        omega_max         float  centroid angular velocity clamp rad/s (default 1.5)
        r_formation       float  ring radius used when formation=None (default 0.6)
        estimate_centroid bool   if True, infer centroid from robot states rather than
                                 using payload_state directly; payload_state is still
                                 used once on the first call after reset() to calibrate
                                 body-frame offsets (default False)
    """

    def __init__(
        self,
        num_robots: int,
        formation: Optional[List[tuple]] = None,
        config: Optional[Dict[str, Any]] = None,
    ):
        super().__init__(num_robots, config)

        cfg = self.config
        self._N          = int(cfg.get("horizon",       15))
        self._sigma_x    = float(cfg.get("sigma_x",     0.5))
        self._sigma_u    = float(cfg.get("sigma_u",     0.3))
        self._sigma_anc  = float(cfg.get("sigma_anchor", 0.01))
        self._sigma_mm   = float(cfg.get("sigma_mm",    1e-4))
        self._v_max      = float(cfg.get("v_max",       1.0))
        self._omega_max  = float(cfg.get("omega_max",   1.5))

        # Formation offsets r_i = [dx, dy] from centroid to robot i (world frame).
        # Rigid body: these stay constant throughout the run.
        if formation is not None:
            self._r = np.array([[f[0], f[1]] for f in formation], dtype=float)
        else:
            radius = float(cfg.get("r_formation", 0.6))
            angles = np.linspace(0, 2 * np.pi, num_robots, endpoint=False)
            self._r = np.column_stack([radius * np.cos(angles),
                                       radius * np.sin(angles)])

        # Noise models built once and reused
        self._noise_x   = gtsam.noiseModel.Diagonal.Sigmas(np.full(3, self._sigma_x))
        self._noise_u   = gtsam.noiseModel.Diagonal.Sigmas(np.full(3, self._sigma_u))
        self._noise_anc = gtsam.noiseModel.Diagonal.Sigmas(np.full(3, self._sigma_anc))
        self._noise_mm  = gtsam.noiseModel.Diagonal.Sigmas(np.full(3, self._sigma_mm))

        self._estimate_centroid = bool(cfg.get("estimate_centroid", False))
        self._estimator = CentroidEstimator() if self._estimate_centroid else None
        self._estimator_ready = False

    # ------------------------------------------------------------------
    # BaseController interface
    # ------------------------------------------------------------------

    def reset(self):
        self._last_solve_time = 0.0
        self._total_solves = 0
        if self._estimator is not None:
            self._estimator_ready = False

    def compute_control(
        self,
        payload_state: np.ndarray,
        robot_states: np.ndarray,
        goal_state: np.ndarray,
        dt: float,
        forces: np.ndarray = None,
    ) -> np.ndarray:
        if self._estimate_centroid:
            if not self._estimator_ready:
                self._estimator.reset(robot_states, payload_state)
                self._estimator_ready = True
            centroid = self._estimator.estimate(robot_states)[:3]
        else:
            centroid = payload_state[:3].copy()   # [x, y, theta]
        goal     = np.asarray(goal_state, dtype=float)[:3]

        t0 = time.perf_counter()
        U_c = self._solve_fg(centroid, goal, dt)
        self._set_solve_time(time.perf_counter() - t0)

        return self._robot_velocities(U_c)

    # ------------------------------------------------------------------
    # Factor graph solve
    # ------------------------------------------------------------------

    def _solve_fg(
        self, centroid: np.ndarray, goal: np.ndarray, dt: float
    ) -> np.ndarray:
        """Solve FG and return optimal centroid control [vx, vy, omega]."""
        N = self._N

        # Linear reference trajectory from current centroid to goal
        ref = np.array([centroid + (j / N) * (goal - centroid) for j in range(N + 1)])

        graph = gtsam.NonlinearFactorGraph()
        init  = gtsam.Values()

        def Ck(j): return gtsam.symbol('C', j)
        def Uk(j): return gtsam.symbol('U', j)

        for j in range(N):
            kC  = Ck(j)
            kU  = Uk(j)
            kC1 = Ck(j + 1)

            # State prior
            if j == 0:
                noise_c = self._noise_anc
            else:
                noise_c = self._noise_x

            graph.add(gtsam.CustomFactor(
                noise_c, [kC], partial(_prior_error, ref[j].copy())))
            init.insert(kC, ref[j].copy())

            # Control regularisation toward zero; warm-start with ref velocity
            u_warm = (ref[j + 1] - ref[j]) / dt if dt > 1e-9 else np.zeros(3)
            graph.add(gtsam.CustomFactor(
                self._noise_u, [kU], partial(_prior_error, np.zeros(3))))
            init.insert(kU, u_warm)

            # Motion model: C_{j+1} = C_j + dt * U_j
            graph.add(gtsam.CustomFactor(
                self._noise_mm, [kC, kU, kC1],
                partial(_motion_model_error, dt)))

        # Terminal anchor on C_N
        kCN = Ck(N)
        graph.add(gtsam.CustomFactor(
            self._noise_anc, [kCN], partial(_prior_error, goal.copy())))
        init.insert(kCN, ref[N].copy())

        params = gtsam.LevenbergMarquardtParams()
        params.setVerbosity("SILENT")
        result = gtsam.LevenbergMarquardtOptimizer(graph, init, params).optimize()

        U_opt = result.atVector(Uk(0))
        lo = np.array([-self._v_max, -self._v_max, -self._omega_max])
        hi = np.array([ self._v_max,  self._v_max,  self._omega_max])
        return np.clip(U_opt, lo, hi)

    # ------------------------------------------------------------------
    # Rigid-body velocity distribution
    # ------------------------------------------------------------------

    def _robot_velocities(self, U_c: np.ndarray) -> np.ndarray:
        """
        Derive per-robot [vx, vy] from centroid control U_c = [vx_c, vy_c, omega_c].

        Holonomic rigid-body kinematics:
          v_ix = vx_c - omega_c * r_iy
          v_iy = vy_c + omega_c * r_ix
        Then clamp individual robot speeds to v_max.
        """
        vx_c, vy_c, omega_c = U_c
        r = self._r                            # (n, 2)
        vx = vx_c - omega_c * r[:, 1]
        vy = vy_c + omega_c * r[:, 0]

        speeds = np.hypot(vx, vy)
        scale  = np.where(speeds > self._v_max, self._v_max / speeds, 1.0)
        return np.column_stack([vx * scale, vy * scale])
