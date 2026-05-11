"""
ContactHealthDistributedController: distributed contact-health FG controller via GBP.

Distributed adaptation of ContactHealthController, mirroring the DR.CAP →
MR.CAP relationship. Local FG layout, GBP loop, and per-robot message format
are identical to DRCapDistributedController, with two replacements per the
distributed-contact-health problem statement:

  (1) The xc[0] start-anchor target is the *force-weighted Procrustes*
      centroid estimate, computed locally from broadcast sufficient
      statistics (p_j, v_j, theta_j, r_j, w_j) over neighbours (and self).
      Reduces exactly to the unweighted DR.CAP estimator when w_i = 1 ∀i
      (Procrustes is scale-invariant in the weights).

  (2) The control regulariser uses the contact-health-modulated
      sigma_u_eff(F_bar) = sigma_u^0 / (1 + alpha * max(F_bar - F_wall*, 0)),
      with F_bar computed locally from broadcast F_wall_i.

Per-robot post-solve corrections (force-recovery + position lock) are local
and identical to ContactHealthController.

A single pre-GBP "stats round" per control step (one extra broadcast +
barrier on the existing CommunicationBackend) carries the contact-health
sufficient statistics. Subsequent GBP iterations carry the DR.CAP belief
payload unchanged. Under full topology this yields a centroid estimate and
F_bar identical to the centralised contact-health controller; under partial
topology each robot computes a neighbourhood-restricted estimate.

See experiments/distributed_contact_health_fg/PROBLEM_STATEMENT.md.
"""

import time
from dataclasses import dataclass
from typing import Dict, Any, List, Optional

import numpy as np

from .base_controller import BaseController
from ..communication.backend import (
    CommunicationBackend,
    GaussianMessage,
    SimulatedBackend,
    create_full_topology,
)
from .drcap_distributed_controller import LocalRobotGraph


# ---------------------------------------------------------------------------
# Pre-GBP contact-health sufficient statistics message
# ---------------------------------------------------------------------------

@dataclass
class ContactStatsMessage:
    """
    Per-robot contact-health sufficient statistics broadcast once per control
    step before the GBP rounds. Six scalars (p_xy, v_xy) + (r_body) + theta + w
    + F_wall — independent of the FG horizon and robot count.
    """
    p_xy:   np.ndarray   # (2,) own world position
    v_xy:   np.ndarray   # (2,) own world velocity
    theta:  float        # own yaw
    r_body: np.ndarray   # (2,) body-frame formation offset (calibrated once)
    w:      float        # contact-health weight in [eps, 1]
    F_wall: float        # wall-force reading (N), 0 if not provided

    def copy(self) -> "ContactStatsMessage":
        return ContactStatsMessage(
            p_xy=self.p_xy.copy(),
            v_xy=self.v_xy.copy(),
            theta=float(self.theta),
            r_body=self.r_body.copy(),
            w=float(self.w),
            F_wall=float(self.F_wall),
        )


# ---------------------------------------------------------------------------
# Local weighted Procrustes (operates on whatever subset of robots is known)
# ---------------------------------------------------------------------------

def _weighted_procrustes(
    p_all: np.ndarray,   # (m, 2) world positions
    r_all: np.ndarray,   # (m, 2) body-frame offsets
    v_all: np.ndarray,   # (m, 2) world velocities
    w_all: np.ndarray,   # (m,)   weights
) -> np.ndarray:
    """
    Weighted-Procrustes centroid pose + weighted-LS centroid velocity.
    Returns (6,) state [x, y, theta, vx, vy, omega]. Mirrors
    WeightedCentroidEstimator.estimate() but accepts an arbitrary subset
    of robots (the caller's local view).
    """
    w = np.asarray(w_all, dtype=float)
    w_sum = w.sum()
    if w_sum < 1e-12:
        w = np.full_like(w, 1.0 / len(w))
        w_sum = 1.0
    p_bar = (w[:, None] * p_all).sum(axis=0) / w_sum
    r_bar = (w[:, None] * r_all).sum(axis=0) / w_sum
    dp = p_all - p_bar
    dr = r_all - r_bar
    M = (w[:, None] * dp).T @ dr
    U, _, Vt = np.linalg.svd(M)
    R_est = U @ np.diag([1.0, np.linalg.det(U @ Vt)]) @ Vt
    theta_c = float(np.arctan2(R_est[1, 0], R_est[0, 0]))
    p_c = p_bar - R_est @ r_bar

    r_world = r_all @ R_est.T
    sqrt_w = np.sqrt(w)
    m = len(p_all)
    A = np.zeros((2 * m, 3))
    b = np.zeros(2 * m)
    for i in range(m):
        A[2 * i]     = sqrt_w[i] * np.array([1.0, 0.0, -r_world[i, 1]])
        A[2 * i + 1] = sqrt_w[i] * np.array([0.0, 1.0,  r_world[i, 0]])
        b[2 * i]     = sqrt_w[i] * v_all[i, 0]
        b[2 * i + 1] = sqrt_w[i] * v_all[i, 1]
    vel, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    return np.array([p_c[0], p_c[1], theta_c, vel[0], vel[1], vel[2]])


# ---------------------------------------------------------------------------
# Controller
# ---------------------------------------------------------------------------

class ContactHealthDistributedController(BaseController):
    """
    Distributed contact-health controller via GBP message-passing.

    Two operating modes (mirrors DRCapDistributedController):

      Simulation mode (my_id is None): owns one LocalRobotGraph per robot in
      a single process; drives the stats round and GBP loop by round-robining
      through all owned graphs over a SimulatedBackend (or async variant).
      compute_control takes (n, ≥5) robot_states and returns (n, 2) per-robot
      world-frame velocities.

      Deployment mode (my_id is int): owns only this robot's local graph;
      uses an externally-constructed single-agent backend
      (e.g. ZeroMQSingleAgentBackend) supplied via `backend`. compute_control
      takes (1, ≥5) of this robot's state and returns (1, 2) of its velocity.

    Parameters
    ----------
    num_robots : int
    formation  : list[(x_off, y_off, yaw)] per robot (centroid frame).
    backend    : CommunicationBackend.
                 Sim mode default: SimulatedBackend(num_robots, topology).
                 Deploy mode: required, must be a single-agent backend keyed
                 to my_id.
    my_id      : Optional[int]. None = sim mode; int = this robot's id in
                 deploy mode.
    topology   : Dict[int, list[int]]. Default: full mesh.
    config     : dict. DR.CAP-shared keys (horizon, sigma_x, sigma_u,
                 sigma_anchor, sigma_mm, sigma_r2r, sigma_pull_in,
                 sigma_consensus, gbp_max_iters, gbp_tol, v_max, omega_max,
                 r_formation) plus contact-health keys
                 (F_wall_star, F_base_star, payload_mass_nom, alpha_sigma_u,
                 beta_recovery, weight_eps, pos_kp,
                 use_weighted_anchor, use_modulated_sigma_u,
                 use_recovery_term, use_pos_lock).
    """

    def __init__(
        self,
        num_robots: int,
        formation: Optional[List[tuple]] = None,
        backend: Optional[CommunicationBackend] = None,
        my_id: Optional[int] = None,
        topology: Optional[Dict[int, List[int]]] = None,
        config: Optional[Dict[str, Any]] = None,
    ):
        super().__init__(num_robots, config)
        cfg = self.config
        self.my_id = my_id

        # Formation
        if formation is not None:
            self._r = np.array([[f[0], f[1]] for f in formation], dtype=float)
        else:
            radius = float(cfg.get("r_formation", 0.6))
            angles = np.linspace(0, 2 * np.pi, num_robots, endpoint=False)
            self._r = np.column_stack(
                [radius * np.cos(angles), radius * np.sin(angles)]
            )

        # Topology / backend
        if topology is None:
            topology = create_full_topology(num_robots)
        self._topology = topology
        if backend is None:
            if my_id is not None:
                raise ValueError(
                    "Deployment mode (my_id set) requires an externally "
                    "constructed backend (e.g. ZeroMQSingleAgentBackend)."
                )
            backend = SimulatedBackend(num_robots, topology)
        self.backend = backend

        # Contact-health parameters
        m_nom = float(cfg.get("payload_mass_nom", 2.0))
        g_ = 9.81
        self._F_wall_star = float(cfg.get("F_wall_star", 5.0))
        self._F_base_star = float(cfg.get("F_base_star",
                                           m_nom * g_ / max(num_robots, 1)))
        self._sigma_u0    = float(cfg.get("sigma_u",        0.3))
        self._alpha       = float(cfg.get("alpha_sigma_u",  0.1))
        self._beta        = float(cfg.get("beta_recovery",  0.005))
        self._weight_eps  = float(cfg.get("weight_eps",     1e-3))
        self._pos_kp      = float(cfg.get("pos_kp",         1.0))

        # Ablation switches
        self._use_weighted  = bool(cfg.get("use_weighted_anchor",   True))
        self._use_modulated = bool(cfg.get("use_modulated_sigma_u", True))
        self._use_recovery  = bool(cfg.get("use_recovery_term",     True))
        self._use_pos_lock  = bool(cfg.get("use_pos_lock",          True))

        # Per-robot local graphs (DR.CAP layout, sigma_u replaced per step).
        ids_to_build = [my_id] if my_id is not None else list(range(num_robots))
        self._owned_ids = ids_to_build
        self.local_graphs: Dict[int, LocalRobotGraph] = {
            i: LocalRobotGraph(
                robot_id=i,
                num_robots=num_robots,
                r_i=self._r[i],
                r_all=self._r,
                neighbors=topology[i],
                cfg=cfg,
            )
            for i in ids_to_build
        }

        self._max_iters = int(cfg.get("gbp_max_iters", 30))
        self._tol       = float(cfg.get("gbp_tol",     1e-3))
        self._v_max     = float(cfg.get("v_max",       1.0))
        self._omega_max = float(cfg.get("omega_max",   1.5))

        # Calibration: body-frame offsets from a known initial payload pose.
        self._calibrated = False
        self._r_body_all: Optional[np.ndarray] = None  # (n,2) sim mode
        self._r_body_self: Optional[np.ndarray] = None  # (2,) deploy mode

        # Per-step diagnostics keyed by owned id
        self._last_iters = 0
        self._last_centroid:    Dict[int, np.ndarray] = {}
        self._last_weights:     Dict[int, np.ndarray] = {}
        self._last_F_bar:       Dict[int, Optional[float]] = {}
        self._last_sigma_u_eff: Dict[int, float] = {}
        self._own_stats_this_step: Dict[int, ContactStatsMessage] = {}

    # --- BaseController interface -----------------------------------------

    def reset(self):
        self._last_solve_time = 0.0
        self._total_solves = 0
        self._last_iters = 0
        self._calibrated = False
        self._r_body_all = None
        self._r_body_self = None
        self._last_centroid.clear()
        self._last_weights.clear()
        self._last_F_bar.clear()
        self._last_sigma_u_eff.clear()
        self._own_stats_this_step.clear()

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
        # BaseController-compat alias
        if wall_forces is None and forces is not None:
            wall_forces = forces

        if not self._calibrated:
            self._calibrate(robot_states, payload_state)

        t0 = time.perf_counter()
        goal = np.asarray(goal_state, dtype=float)[:3]

        # ---- 1) Stats round ---------------------------------------------
        nbr_stats = self._stats_round(robot_states, wall_forces, base_forces)

        # ---- 2) Per-owned warm-start with contact-health anchor + sigma_u
        for owned_id in self._owned_ids:
            self._warm_start_with_contact_health(
                owned_id,
                own_stats=self._own_stats_this_step[owned_id],
                nbr_stats=nbr_stats[owned_id],
                goal=goal,
                dt=dt,
                forces_present=(wall_forces is not None),
            )

        # ---- 3) GBP iterations (DR.CAP loop, unchanged) -----------------
        iters_done = 0
        for it in range(self._max_iters):
            for i in self._owned_ids:
                msg = self.local_graphs[i].pack_outgoing_message()
                self.backend.broadcast(i, msg)

            self.backend.barrier()

            for i in self._owned_ids:
                inbox = self.backend.receive(i)
                for sender_id, msg in inbox:
                    # Ignore any leftover stats messages (shouldn't happen
                    # with synchronous backends, defensive for async).
                    if isinstance(msg, ContactStatsMessage):
                        continue
                    self.local_graphs[i].set_neighbor_message(sender_id, msg)

            max_change = 0.0
            for i in self._owned_ids:
                max_change = max(max_change, self.local_graphs[i].gbp_step())

            iters_done = it + 1
            if max_change < self._tol:
                break

        self._last_iters = iters_done

        # ---- 4) Per-owned U_c → per-owned commanded velocity ------------
        v_out_rows = []
        for owned_id in self._owned_ids:
            row = 0 if self.my_id is not None else owned_id

            U_c = self.local_graphs[owned_id].first_control()
            speed = np.hypot(U_c[0], U_c[1])
            if speed > self._v_max:
                U_c[0] *= self._v_max / speed
                U_c[1] *= self._v_max / speed
            U_c[2] = np.clip(U_c[2], -self._omega_max, self._omega_max)

            theta_i = float(robot_states[row, 2]) if robot_states.shape[1] >= 3 else 0.0
            p_i     = robot_states[row, :2].astype(float)
            centroid = self._last_centroid[owned_id]    # (3,) [x,y,theta]
            r_body_i = self._own_r_body(owned_id)
            own = self._own_stats_this_step[owned_id]

            v_cmd = self._rigid_velocity(U_c, centroid[2], r_body_i)

            if self._use_pos_lock:
                v_cmd = self._apply_pos_lock_self(
                    v_cmd, centroid, p_i, r_body_i, own.w
                )

            if self._use_recovery and wall_forces is not None:
                v_cmd = self._apply_recovery_self(v_cmd, theta_i, own.F_wall)

            spd = np.hypot(v_cmd[0], v_cmd[1])
            if spd > self._v_max:
                v_cmd = v_cmd * (self._v_max / spd)

            v_out_rows.append(v_cmd)

        self._set_solve_time(time.perf_counter() - t0)
        return np.array(v_out_rows)

    # --- Calibration -------------------------------------------------------

    def _calibrate(self, robot_states: np.ndarray, payload_state: np.ndarray):
        """
        One-shot body-frame offset calibration from a known initial payload
        pose. Mirrors WeightedCentroidEstimator.reset() in the centralised
        controller. After calibration, payload_state is no longer used.
        """
        p_c = payload_state[:2].astype(float)
        theta_c = float(payload_state[2])
        c, s = np.cos(theta_c), np.sin(theta_c)
        R_inv = np.array([[c, s], [-s, c]])  # world -> body
        if self.my_id is None:
            self._r_body_all = (robot_states[:, :2].astype(float) - p_c) @ R_inv.T
        else:
            p_self = robot_states[0, :2].astype(float)
            self._r_body_self = (p_self - p_c) @ R_inv.T
        self._calibrated = True

    def _own_r_body(self, owned_id: int) -> np.ndarray:
        if self.my_id is None:
            return self._r_body_all[owned_id]
        return self._r_body_self

    # --- Stats round -------------------------------------------------------

    def _compute_weight(self, F_wall_i: Optional[float],
                        F_base_i: Optional[float]) -> float:
        if F_wall_i is None and F_base_i is None:
            return 1.0
        w_wall = 1.0 if F_wall_i is None else min(F_wall_i / self._F_wall_star, 1.0)
        w_base = 1.0 if F_base_i is None else min(F_base_i / self._F_base_star, 1.0)
        return max(w_wall * w_base, self._weight_eps)

    def _stats_round(
        self,
        robot_states: np.ndarray,
        wall_forces: Optional[np.ndarray],
        base_forces: Optional[np.ndarray],
    ) -> Dict[int, Dict[int, ContactStatsMessage]]:
        """
        One pre-GBP round: each owned robot broadcasts ContactStatsMessage,
        barrier, then collects neighbour stats. Returns
        {owned_id: {neighbour_id: ContactStatsMessage}}.
        """
        wf = None if wall_forces is None else np.asarray(wall_forces, dtype=float)
        bf = None if base_forces is None else np.asarray(base_forces, dtype=float)

        # Build + send
        own_stats: Dict[int, ContactStatsMessage] = {}
        for owned_id in self._owned_ids:
            row = 0 if self.my_id is not None else owned_id
            # Force index: in deploy mode the forces array is full-sized
            # (entry per robot in the formation), addressed by owned_id.
            f_idx = owned_id

            p_xy   = robot_states[row, :2].astype(float).copy()
            v_xy   = (robot_states[row, 3:5].astype(float).copy()
                      if robot_states.shape[1] >= 5 else np.zeros(2))
            theta  = float(robot_states[row, 2]) if robot_states.shape[1] >= 3 else 0.0
            r_body = self._own_r_body(owned_id).copy()

            F_wall_i = None if wf is None else float(wf[f_idx])
            F_base_i = None if bf is None else float(bf[f_idx])
            w_i = self._compute_weight(F_wall_i, F_base_i)

            msg = ContactStatsMessage(
                p_xy=p_xy, v_xy=v_xy, theta=theta, r_body=r_body,
                w=w_i, F_wall=(0.0 if F_wall_i is None else F_wall_i),
            )
            own_stats[owned_id] = msg
            self.backend.broadcast(owned_id, msg)

        self.backend.barrier()

        # Receive
        inbox: Dict[int, Dict[int, ContactStatsMessage]] = {}
        for owned_id in self._owned_ids:
            received = self.backend.receive(owned_id)
            inbox[owned_id] = {
                sender: msg for (sender, msg) in received
                if isinstance(msg, ContactStatsMessage)
            }

        self._own_stats_this_step = own_stats
        return inbox

    # --- Per-robot warm-start --------------------------------------------

    def _warm_start_with_contact_health(
        self,
        owned_id: int,
        own_stats: ContactStatsMessage,
        nbr_stats: Dict[int, ContactStatsMessage],
        goal: np.ndarray,
        dt: float,
        forces_present: bool,
    ) -> None:
        all_msgs = [own_stats] + list(nbr_stats.values())
        p_arr = np.array([m.p_xy   for m in all_msgs])
        v_arr = np.array([m.v_xy   for m in all_msgs])
        r_arr = np.array([m.r_body for m in all_msgs])
        w_arr = np.array([m.w      for m in all_msgs])
        F_arr = np.array([m.F_wall for m in all_msgs])

        # Weighted-Procrustes centroid (or unweighted under ablation)
        if self._use_weighted:
            w_use = w_arr
        else:
            w_use = np.ones_like(w_arr)
        cent_full = _weighted_procrustes(p_arr, r_arr, v_arr, w_use)
        centroid = cent_full[:3]

        # sigma_u_eff
        if forces_present:
            F_bar = float(np.mean(F_arr))
        else:
            F_bar = None
        if self._use_modulated and F_bar is not None:
            excess = max(F_bar - self._F_wall_star, 0.0)
            sigma_u_eff = self._sigma_u0 / (1.0 + self._alpha * excess)
        else:
            sigma_u_eff = self._sigma_u0

        # Override the local graph's sigma_u for this step (consumed inside
        # warm_start -> _build_linear_part).
        graph = self.local_graphs[owned_id]
        graph.lam_u = 1.0 / (sigma_u_eff ** 2)

        # Robot pose 3-vector (x, y, theta) for the xi[0] anchor
        robot_pose = np.array([own_stats.p_xy[0], own_stats.p_xy[1], own_stats.theta])

        graph.warm_start(robot_pose, centroid, goal, dt)

        # Diagnostics
        self._last_centroid[owned_id]    = centroid.copy()
        self._last_weights[owned_id]     = w_arr.copy()
        self._last_F_bar[owned_id]       = F_bar
        self._last_sigma_u_eff[owned_id] = sigma_u_eff

    # --- Per-robot post-solve corrections --------------------------------

    @staticmethod
    def _rigid_velocity(U_c: np.ndarray, theta_c: float,
                        r_body_i: np.ndarray) -> np.ndarray:
        vx_c, vy_c, omega_c = U_c
        c, s = np.cos(theta_c), np.sin(theta_c)
        R = np.array([[c, -s], [s, c]])
        r_world = R @ r_body_i
        vx = vx_c - omega_c * r_world[1]
        vy = vy_c + omega_c * r_world[0]
        return np.array([vx, vy])

    def _apply_pos_lock_self(
        self,
        v_rigid_i: np.ndarray,
        centroid: np.ndarray,
        p_i: np.ndarray,
        r_body_i: np.ndarray,
        w_i: float,
    ) -> np.ndarray:
        theta_c = float(centroid[2])
        c, s = np.cos(theta_c), np.sin(theta_c)
        R = np.array([[c, -s], [s, c]])
        p_des = centroid[:2] + R @ r_body_i
        gate = 1.0 - float(w_i)
        return v_rigid_i + gate * self._pos_kp * (p_des - p_i)

    def _apply_recovery_self(
        self,
        v_i: np.ndarray,
        theta_i: float,
        F_wall_i: float,
    ) -> np.ndarray:
        nhat = np.array([np.cos(theta_i), np.sin(theta_i)])
        error = self._F_wall_star - float(F_wall_i)
        return v_i + self._beta * error * nhat

    # --- Diagnostics ------------------------------------------------------

    def get_gbp_iters(self) -> int:
        return self._last_iters

    def get_diagnostics(self) -> Dict[str, Any]:
        """Per-step contact-health quantities (representative owned robot)."""
        rep = self._owned_ids[0] if self._owned_ids else None
        return {
            "weights":     None if rep is None else self._last_weights.get(rep),
            "F_bar":       None if rep is None else self._last_F_bar.get(rep),
            "sigma_u_eff": None if rep is None else self._last_sigma_u_eff.get(rep),
            "centroid":    None if rep is None else self._last_centroid.get(rep),
            "F_wall_star": self._F_wall_star,
            "F_base_star": self._F_base_star,
            "gbp_iters":   self._last_iters,
        }

    def get_stats(self) -> Dict[str, Any]:
        s = super().get_stats()
        s["gbp_iters_last"] = self._last_iters
        s["comm_stats"] = self.backend.get_stats()
        return s
