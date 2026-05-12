"""
ContactHealthController: centralised FG controller for multi-robot payload
transport with force-sensed contact-health observation channels.

Variables and factor-graph structure are identical to MRCapController. Three
contact-health channels are layered on top, none of which expand the FG:

  (1) Force-weighted Procrustes anchor.
      The current-state anchor is pinned to a centroid pose estimated by
      *weighted* Procrustes over robot poses, with per-robot weights
        w_i = max( min(F_wall_i/F_wall*, 1) · min(F_base_i/F_base*, 1), ε ).
      Robots with degraded contact contribute near-ε weight; robots at or
      above nominal contact contribute weight 1. Reduces exactly to MR.CAP
      when all robots are healthy.

  (2) Contact-health-modulated control regulariser.
      σ_u^eff = σ_u^0 / (1 + α · max(F̄ − F_wall*, 0)),
      with F̄ = mean wall-squeeze. Above-target squeeze ⇒ tighter regulariser
      ⇒ commanded velocities pulled toward zero ⇒ formation slows down,
      relieving the collective drag.

  (3) Per-robot contact-pinch regulator (post-solve correction).
      v_i^cmd = v_i^rigid(U_k*) + β · (F_wall* − F_wall_i) · n̂_i
      where n̂_i = [cos θ_i, sin θ_i] is robot i's forward axis. A
      bidirectional P-controller on each robot's own wall force:
        F_i < F*  ⇒ correction +n̂_i  (push forward into payload — engage)
        F_i > F*  ⇒ correction −n̂_i  (back away — relieve pinch)
      Equilibrium at F_i ≈ F*. Formation-agnostic: each robot regulates
      its own contact; asymmetric formations (e.g. n=3 with one robot in
      the push direction) handle reaction-loading naturally.

See experiments/centralised_contact_health_fg/PROBLEM_STATEMENT.md.
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
from .mrcap_controller import _prior_error, _motion_model_error, _wrap_pi
from .weighted_centroid_estimator import WeightedCentroidEstimator


class ContactHealthController(BaseController):
    """
    Centralised FG controller with force-sensed contact-health channels.

    Parameters
    ----------
    num_robots : int
    formation  : list[(x_off, y_off, yaw)] per robot relative to payload centre.
                 If None, robots placed on a ring of radius r_formation.
    config     : dict, recognised keys (MR.CAP-shared and contact-health):
        # MR.CAP-shared
        horizon           int    receding horizon N (default 15)
        sigma_x           float  ref-trajectory prior σ (default 0.5)
        sigma_u           float  control regulariser σ_u^0 (default 0.3)
        sigma_anchor      float  anchor σ (default 0.01)
        sigma_mm          float  motion-model σ (default 1e-4)
        v_max             float  per-robot speed clamp m/s (default 1.0)
        omega_max         float  centroid ω clamp rad/s (default 1.5)
        r_formation       float  ring radius if formation=None (default 0.6)
        # contact-health
        F_wall_star       float  target wall-squeeze N (default 5.0)
        F_base_star       float  per-robot weight-share N (default: auto =
                                 payload_mass_nom·g/num_robots)
        payload_mass_nom  float  used for F_base_star auto-calc (default 2.0)
        alpha_sigma_u     float  σ_u modulation gain N^-1 (default 0.1)
        beta_recovery     float  recovery gain m·s^-1·N^-1 (default 0.005)
        weight_eps        float  floor on Procrustes weights (default 1e-3)
        # ablation switches
        use_weighted_anchor   bool (default True)
        use_modulated_sigma_u bool (default True)
        use_recovery_term     bool (default True)
    """

    def __init__(
        self,
        num_robots: int,
        formation: Optional[List[tuple]] = None,
        config: Optional[Dict[str, Any]] = None,
    ):
        super().__init__(num_robots, config)
        cfg = self.config

        # MR.CAP-shared parameters
        self._N          = int(cfg.get("horizon",       15))
        self._sigma_x    = float(cfg.get("sigma_x",     0.5))
        self._sigma_u0   = float(cfg.get("sigma_u",     0.3))
        self._sigma_anc  = float(cfg.get("sigma_anchor", 0.01))
        self._sigma_mm   = float(cfg.get("sigma_mm",    1e-4))
        self._v_max      = float(cfg.get("v_max",       1.0))
        self._omega_max  = float(cfg.get("omega_max",   1.5))

        # Contact-health parameters
        m_nom = float(cfg.get("payload_mass_nom", 2.0))
        g_    = 9.81
        self._F_wall_star = float(cfg.get("F_wall_star", 5.0))
        self._F_base_star = float(cfg.get("F_base_star",
                                           m_nom * g_ / max(num_robots, 1)))
        self._alpha       = float(cfg.get("alpha_sigma_u", 0.1))
        self._beta        = float(cfg.get("beta_recovery", 0.005))
        self._weight_eps  = float(cfg.get("weight_eps",    1e-3))
        # Per-robot position-lock P gain (kinematic counterpart to β):
        # v_i = v_i^rigid + K_p · (p_i^des − p_i^act), where p_i^des is the
        # robot's nominal slot in the *estimated* centroid frame. Pulls a
        # slipping bot back into formation geometrically; complements the
        # force-based wall recovery, which reattaches it physically.
        self._pos_kp      = float(cfg.get("pos_kp",        1.0))

        # Ablation switches
        self._use_weighted  = bool(cfg.get("use_weighted_anchor",    True))
        self._use_modulated = bool(cfg.get("use_modulated_sigma_u",  True))
        self._use_recovery  = bool(cfg.get("use_recovery_term",      True))
        self._use_pos_lock  = bool(cfg.get("use_pos_lock",           True))

        # Formation offsets (body frame)
        if formation is not None:
            self._r = np.array([[f[0], f[1]] for f in formation], dtype=float)
        else:
            radius = float(cfg.get("r_formation", 0.6))
            angles = np.linspace(0, 2 * np.pi, num_robots, endpoint=False)
            self._r = np.column_stack([radius * np.cos(angles),
                                       radius * np.sin(angles)])

        # Static noise models. σ_u is rebuilt per-step when modulated.
        self._noise_x   = gtsam.noiseModel.Diagonal.Sigmas(np.full(3, self._sigma_x))
        self._noise_anc = gtsam.noiseModel.Diagonal.Sigmas(np.full(3, self._sigma_anc))
        self._noise_mm  = gtsam.noiseModel.Diagonal.Sigmas(np.full(3, self._sigma_mm))

        self._estimator = WeightedCentroidEstimator(weight_eps=self._weight_eps)
        self._estimator_ready = False

        # Diagnostics — populated per step, readable for logging
        self._last_weights:     Optional[np.ndarray] = None
        self._last_F_bar:       Optional[float]      = None
        self._last_sigma_u_eff: Optional[float]      = None
        self._last_centroid:    Optional[np.ndarray] = None
        self._last_v_recovery:  Optional[np.ndarray] = None

    # ------------------------------------------------------------------
    # BaseController interface
    # ------------------------------------------------------------------

    def reset(self):
        self._last_solve_time = 0.0
        self._total_solves = 0
        self._estimator_ready = False
        self._last_weights = None
        self._last_F_bar = None
        self._last_sigma_u_eff = None
        self._last_centroid = None
        self._last_v_recovery = None

    def compute_control(
        self,
        payload_state: np.ndarray,
        robot_states: np.ndarray,
        goal_state: np.ndarray,
        dt: float,
        wall_forces: np.ndarray = None,
        base_forces: np.ndarray = None,
        forces: np.ndarray = None,
    ) -> np.ndarray:
        # BaseController-compat alias: if explicit wall_forces not provided,
        # interpret legacy `forces` kwarg as wall_forces.
        if wall_forces is None and forces is not None:
            wall_forces = forces

        # First-call calibration uses ground-truth payload pose to fix
        # body-frame robot offsets. After this, payload_state is unused
        # for pose; the centroid is estimated from robot poses + forces.
        if not self._estimator_ready:
            self._estimator.reset(robot_states, payload_state)
            self._estimator_ready = True

        # ---- Per-robot contact-health weights ----
        weights = self._compute_weights(wall_forces, base_forces)
        self._last_weights = weights

        # ---- Centroid pose estimate ----
        if self._use_weighted:
            centroid_full = self._estimator.estimate(robot_states, weights=weights)
        else:
            centroid_full = self._estimator.estimate(robot_states, weights=None)
        centroid = centroid_full[:3]
        self._last_centroid = centroid.copy()

        goal = np.asarray(goal_state, dtype=float)[:3]

        # ---- σ_u modulation ----
        sigma_u_eff = self._compute_sigma_u(wall_forces)
        self._last_sigma_u_eff = sigma_u_eff

        # ---- FG solve ----
        t0 = time.perf_counter()
        U_c = self._solve_fg(centroid, goal, dt, sigma_u_eff)
        self._set_solve_time(time.perf_counter() - t0)

        # ---- Rigid-body distribution + position lock + recovery ----
        v_rigid = self._robot_velocities(U_c, centroid[2])
        if self._use_pos_lock:
            v_rigid = self._apply_pos_lock(v_rigid, centroid, robot_states,
                                           weights=weights)
        if self._use_recovery and wall_forces is not None:
            v_out, v_corr = self._apply_recovery(v_rigid, robot_states, wall_forces)
            self._last_v_recovery = v_corr
            return v_out
        else:
            self._last_v_recovery = np.zeros_like(v_rigid)
            return v_rigid

    # ------------------------------------------------------------------
    # Contact-health helpers
    # ------------------------------------------------------------------

    def _compute_weights(
        self,
        wall_forces: Optional[np.ndarray],
        base_forces: Optional[np.ndarray],
    ) -> np.ndarray:
        n = self.num_robots
        if wall_forces is None and base_forces is None:
            return np.ones(n)

        w_wall = (np.ones(n) if wall_forces is None else
                  np.minimum(np.asarray(wall_forces, dtype=float) / self._F_wall_star, 1.0))
        w_base = (np.ones(n) if base_forces is None else
                  np.minimum(np.asarray(base_forces, dtype=float) / self._F_base_star, 1.0))
        # Both factors are already ≥ 0 from the load cells; product stays in [0,1].
        return np.maximum(w_wall * w_base, self._weight_eps)

    def _compute_sigma_u(self, wall_forces: Optional[np.ndarray]) -> float:
        if not self._use_modulated or wall_forces is None:
            self._last_F_bar = (None if wall_forces is None
                                 else float(np.mean(wall_forces)))
            return self._sigma_u0
        F_bar = float(np.mean(wall_forces))
        self._last_F_bar = F_bar
        excess = max(F_bar - self._F_wall_star, 0.0)
        return self._sigma_u0 / (1.0 + self._alpha * excess)

    def _apply_pos_lock(
        self,
        v_rigid: np.ndarray,
        centroid: np.ndarray,
        robot_states: np.ndarray,
        weights: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        """
        Contact-health-gated per-robot position-lock P controller in the
        *estimated* centroid frame.

          p_i^des = p_centroid + R(θ_centroid) · self._r[i]
          v_i     = v_i^rigid + (1 − w_i) · K_p · (p_i^des − p_i^act)

        Where w_i ∈ [ε, 1] is the contact-health weight (same one used by
        the weighted Procrustes anchor). Rationale:
          - Healthy robot (w_i ≈ 1): gate ≈ 0 → no pos-lock; force-recovery
            alone drives the robot to its physical force equilibrium, which
            may differ slightly from the geometric slot.
          - Lost-contact robot (w_i ≈ ε): gate ≈ 1 → pos-lock snaps the
            robot back toward its slot until it re-engages, after which the
            weight rises and pos-lock fades.
        Force is the trustworthy signal when contact exists; geometry is the
        fallback when force is uninformative.

        With weights=None (no force info), the gate is uniformly 1 and this
        reduces to the original ungated pos-lock.
        """
        theta = float(centroid[2])
        c, s = np.cos(theta), np.sin(theta)
        R = np.array([[c, -s], [s, c]])
        pos_des = centroid[:2] + self._r @ R.T              # (n, 2) world frame
        pos_err = pos_des - robot_states[:, :2].astype(float)

        if weights is None:
            gate = np.ones(self.num_robots)
        else:
            gate = 1.0 - np.asarray(weights, dtype=float)
        v_out = v_rigid + (gate[:, None] * self._pos_kp) * pos_err

        speeds = np.hypot(v_out[:, 0], v_out[:, 1])
        scale  = np.where(speeds > self._v_max, self._v_max / speeds, 1.0)
        return v_out * scale[:, None]

    def _apply_recovery(
        self,
        v_rigid: np.ndarray,
        robot_states: np.ndarray,
        wall_forces: np.ndarray,
    ) -> tuple:
        """
        Symmetric per-robot P-controller on wall force.

        v_i^cmd = v_i^rigid + β · (F_wall* − F_wall_i) · [cos θ_i, sin θ_i]

        Sign is automatic:
          F_i < F*  ⇒ +n̂_i  (push into payload)
          F_i > F*  ⇒ −n̂_i  (back away)
        Equilibrium at F_i ≈ F*. Returns (v_cmd_clamped, v_correction_raw).
        """
        thetas = robot_states[:, 2].astype(float)
        F = np.asarray(wall_forces, dtype=float)
        error = self._F_wall_star - F                          # signed
        nhat = np.column_stack([np.cos(thetas), np.sin(thetas)])
        v_corr = self._beta * error[:, None] * nhat
        v_out = v_rigid + v_corr

        speeds = np.hypot(v_out[:, 0], v_out[:, 1])
        scale = np.where(speeds > self._v_max, self._v_max / speeds, 1.0)
        return v_out * scale[:, None], v_corr

    # ------------------------------------------------------------------
    # Factor-graph solve (identical structure to MR.CAP; σ_u parametrised)
    # ------------------------------------------------------------------

    def _solve_fg(
        self,
        centroid: np.ndarray,
        goal: np.ndarray,
        dt: float,
        sigma_u_eff: float,
    ) -> np.ndarray:
        N = self._N
        # Wrap Δθ so the interpolation takes the short way around ±π.
        delta = goal - centroid
        delta[2] = _wrap_pi(delta[2])
        ref = np.array([centroid + (j / N) * delta for j in range(N + 1)])

        # σ_u may differ from step to step → rebuild on the fly. Other noise
        # models are static (built once in __init__).
        noise_u = gtsam.noiseModel.Diagonal.Sigmas(np.full(3, sigma_u_eff))

        graph = gtsam.NonlinearFactorGraph()
        init  = gtsam.Values()

        def Ck(j): return gtsam.symbol('C', j)
        def Uk(j): return gtsam.symbol('U', j)

        for j in range(N):
            kC, kU, kC1 = Ck(j), Uk(j), Ck(j + 1)

            noise_c = self._noise_anc if j == 0 else self._noise_x
            graph.add(gtsam.CustomFactor(
                noise_c, [kC], partial(_prior_error, ref[j].copy())))
            init.insert(kC, ref[j].copy())

            u_warm = (ref[j + 1] - ref[j]) / dt if dt > 1e-9 else np.zeros(3)
            graph.add(gtsam.CustomFactor(
                noise_u, [kU], partial(_prior_error, np.zeros(3))))
            init.insert(kU, u_warm)

            graph.add(gtsam.CustomFactor(
                self._noise_mm, [kC, kU, kC1],
                partial(_motion_model_error, dt)))

        kCN = Ck(N)
        graph.add(gtsam.CustomFactor(
            self._noise_anc, [kCN], partial(_prior_error, goal.copy())))
        init.insert(kCN, ref[N].copy())

        params = gtsam.LevenbergMarquardtParams()
        params.setVerbosity("SILENT")
        result = gtsam.LevenbergMarquardtOptimizer(graph, init, params).optimize()

        U_opt = result.atVector(Uk(0))
        speed = np.hypot(U_opt[0], U_opt[1])
        if speed > self._v_max:
            U_opt[0] *= self._v_max / speed
            U_opt[1] *= self._v_max / speed
        U_opt[2] = np.clip(U_opt[2], -self._omega_max, self._omega_max)
        return U_opt

    # ------------------------------------------------------------------
    # Rigid-body velocity distribution (identical to MR.CAP)
    # ------------------------------------------------------------------

    def _robot_velocities(self, U_c: np.ndarray, theta: float) -> np.ndarray:
        vx_c, vy_c, omega_c = U_c
        c, s = np.cos(theta), np.sin(theta)
        R = np.array([[c, -s], [s, c]])
        r = self._r @ R.T
        vx = vx_c - omega_c * r[:, 1]
        vy = vy_c + omega_c * r[:, 0]
        speeds = np.hypot(vx, vy)
        scale  = np.where(speeds > self._v_max, self._v_max / speeds, 1.0)
        return np.column_stack([vx * scale, vy * scale])

    # ------------------------------------------------------------------
    # Diagnostics
    # ------------------------------------------------------------------

    def get_diagnostics(self) -> Dict[str, Any]:
        """Per-step contact-health quantities for experiment logging."""
        return {
            "weights":     None if self._last_weights is None
                                else self._last_weights.copy(),
            "F_bar":       self._last_F_bar,
            "sigma_u_eff": self._last_sigma_u_eff,
            "centroid":    None if self._last_centroid is None
                                else self._last_centroid.copy(),
            "v_recovery":  None if self._last_v_recovery is None
                                else self._last_v_recovery.copy(),
            "F_wall_star": self._F_wall_star,
            "F_base_star": self._F_base_star,
        }
