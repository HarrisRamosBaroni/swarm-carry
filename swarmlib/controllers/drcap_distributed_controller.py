"""
DRCapDistributedController: distributed factor-graph controller using GBP.

Distributed adaptation of DR.CAP:
  Jaafar & Saeedi, "Distributed Velocity-based Global Formation Control and
  Planning for Object Transport", ICRA 2026 submission.

Unlike `drcap_centralised_controller.py` (GTSAM Levenberg-Marquardt over one
global factor graph), this controller maintains a per-robot local factor graph
and runs Gaussian Belief Propagation (GBP) with message passing over an
injected `CommunicationBackend` (SimulatedBackend for benchmarking,
ZeroMQSingleAgentBackend for real deployment, AsyncSimulatedBackend for
dropout/delay tests).

Holonomic adaptation (mecanum): the paper's non-holonomic motion model is
replaced by a linear Euler integrator. Only the R2R distance factor remains
nonlinear, and is re-linearised around the current mean estimate at every
GBP iteration (matches the iterated-GBP scheme the paper validates empirically).

Per-robot local variables (all 3D: [x, y, theta] or [vx, vy, omega]):
  xi_n   n=0..N   own pose trajectory
  xc_n   n=0..N   local copy of centroid pose trajectory  (estimated via pull-in)
  uc_n   n=0..N-1 local copy of shared centroid control

Per-robot local factors:
  anchor on xi[0]       tight prior at current robot pose
  anchor on xc[0]       tight prior at current centroid estimate
  reference priors      on xc[1..N-1] toward linear interp to goal
  terminal anchor       xc[N] = goal
  control regulariser   uc_n -> 0
  centroid motion       xc_{n+1} = xc_n + dt * uc_n
  robot motion          xi_{n+1} = xi_n + dt * (uc_n[:2] + omega x r_i, omega)
  pull-in (xy only)     xc_n - xi_n = 0            (soft; centroid != robot pos)
  R2R (relinearised)    ||xi_n - xj_n|| - L_ij     (uses latest neighbor mean)

Inter-robot messages (one GaussianMessage per GBP iteration per neighbor):
  Concatenated belief [own_traj_mean, own_centroid_mean] in canonical form.
  Receivers use the trajectory half to re-linearise their R2R factors and the
  centroid half as a consensus pull on their own centroid copy.

The centroid control is output from robot 0 (the "leader" carrying the live
centroid node, per the paper's failover scheme); in practice all robots'
local uc converge to nearly the same value after GBP converges.
"""


import time
from typing import Dict, Any, List, Optional

import numpy as np

from .base_controller import BaseController
from ..communication.backend import (
    CommunicationBackend,
    GaussianMessage,
    SimulatedBackend,
    create_full_topology,
    create_ring_topology,
)


# ---------------------------------------------------------------------------
# Local per-robot factor graph (canonical-form GBP, pure numpy)
# ---------------------------------------------------------------------------

class LocalRobotGraph:
    """
    One robot's local factor graph. Variables stacked into a single flat
    vector of length 9N + 6 in the order [xi_block, xc_block, uc_block].
    """

    def __init__(
        self,
        robot_id: int,
        num_robots: int,
        r_i: np.ndarray,
        r_all: np.ndarray,
        neighbors: List[int],
        cfg: Dict[str, Any],
    ):
        self.i = robot_id
        self.num_robots = num_robots
        self.r_i = np.asarray(r_i, dtype=float)
        self.r_all = np.asarray(r_all, dtype=float)
        self.neighbors = list(neighbors)

        self.N = int(cfg.get("horizon", 15))

        # Factor precisions (1/sigma^2)
        self.lam_anc = 1.0 / float(cfg.get("sigma_anchor", 0.01)) ** 2
        self.lam_x   = 1.0 / float(cfg.get("sigma_x",      0.5 )) ** 2
        self.lam_u   = 1.0 / float(cfg.get("sigma_u",      0.3 )) ** 2
        self.lam_mm  = 1.0 / float(cfg.get("sigma_mm",     1e-4)) ** 2
        self.lam_r2r = 1.0 / float(cfg.get("sigma_r2r",    0.05)) ** 2
        self.lam_pull = 1.0 / float(cfg.get("sigma_pull_in", 0.3)) ** 2
        self.lam_cons = 1.0 / float(cfg.get("sigma_consensus", 0.1)) ** 2

        # Reference inter-robot distances (from formation)
        self.L = {
            j: float(np.linalg.norm(self.r_i - self.r_all[j]))
            for j in self.neighbors
        }

        # Flat vector layout
        self.dim = 9 * self.N + 6
        self._xi_off = 0
        self._xc_off = 3 * (self.N + 1)
        self._uc_off = 6 * (self.N + 1)

        # Current mean estimate
        self.mu = np.zeros(self.dim)

        # Latest received neighbor means (unpacked from messages)
        self._nbr_traj_mean: Dict[int, np.ndarray] = {}   # j -> (N+1, 3)
        self._nbr_cent_mean: Dict[int, np.ndarray] = {}   # j -> (N+1, 3)

        # Precomputed linear information contributions, set in warm_start()
        self._H_lin: Optional[np.ndarray] = None
        self._b_lin: Optional[np.ndarray] = None

    # --- Index helpers ------------------------------------------------------

    def _xi(self, n: int) -> slice:
        return slice(self._xi_off + 3 * n, self._xi_off + 3 * (n + 1))

    def _xc(self, n: int) -> slice:
        return slice(self._xc_off + 3 * n, self._xc_off + 3 * (n + 1))

    def _uc(self, n: int) -> slice:
        return slice(self._uc_off + 3 * n, self._uc_off + 3 * (n + 1))

    def _xi_block(self) -> np.ndarray:
        s = self._xi_off
        return self.mu[s:s + 3 * (self.N + 1)].reshape(self.N + 1, 3)

    def _xc_block(self) -> np.ndarray:
        s = self._xc_off
        return self.mu[s:s + 3 * (self.N + 1)].reshape(self.N + 1, 3)

    def _uc_block(self) -> np.ndarray:
        s = self._uc_off
        return self.mu[s:s + 3 * self.N].reshape(self.N, 3)

    # --- Warm-start and linear factor assembly ------------------------------

    def warm_start(
        self,
        robot_pose: np.ndarray,
        centroid_pose: np.ndarray,
        goal: np.ndarray,
        dt: float,
    ) -> None:
        """Build a straight-line reference and precompute linear H_lin, b_lin."""
        self._robot_pose    = np.asarray(robot_pose,    dtype=float)
        self._centroid_pose = np.asarray(centroid_pose, dtype=float)
        self._goal          = np.asarray(goal,          dtype=float)
        self.dt = max(float(dt), 1e-9)

        N = self.N

        # Linear-interp centroid reference
        ref_c = np.array(
            [centroid_pose + (j / N) * (goal - centroid_pose) for j in range(N + 1)]
        )
        # Robot reference: centroid + xy formation offset
        ref_i = ref_c.copy()
        ref_i[:, :2] += self.r_i
        # Warm control: constant average velocity to goal
        u_warm = (goal - centroid_pose) / (N * self.dt)

        self.mu[self._xi_off:self._xi_off + 3 * (N + 1)] = ref_i.flatten()
        self.mu[self._xc_off:self._xc_off + 3 * (N + 1)] = ref_c.flatten()
        for n in range(N):
            self.mu[self._uc(n)] = u_warm

        self._H_lin, self._b_lin = self._build_linear_part(ref_c)

        # Clear stale neighbor messages from the previous control step
        self._nbr_traj_mean.clear()
        self._nbr_cent_mean.clear()

    def _build_linear_part(self, ref_c: np.ndarray):
        H = np.zeros((self.dim, self.dim))
        b = np.zeros(self.dim)
        N = self.N
        I3 = np.eye(3)

        # 1) xi[0] = robot_pose (tight)
        self._add_prior(H, b, self._xi(0), self._robot_pose, self.lam_anc * I3)
        # 2) xc[0] = centroid_pose (tight)
        self._add_prior(H, b, self._xc(0), self._centroid_pose, self.lam_anc * I3)
        # 3) xc[n] = ref_c[n] soft prior for 1..N-1
        for n in range(1, N):
            self._add_prior(H, b, self._xc(n), ref_c[n], self.lam_x * I3)
        # 4) xc[N] = goal (tight terminal)
        self._add_prior(H, b, self._xc(N), self._goal, self.lam_anc * I3)
        # 5) uc[n] -> 0 (control regulariser)
        for n in range(N):
            self._add_prior(H, b, self._uc(n), np.zeros(3), self.lam_u * I3)

        # 6) centroid motion model: xc[n+1] - xc[n] - dt * uc[n] = 0
        for n in range(N):
            self._add_linear(
                H, b,
                [self._xc(n), self._uc(n), self._xc(n + 1)],
                [-I3, -self.dt * I3, I3],
                np.zeros(3),
                self.lam_mm * I3,
            )

        # 7) robot motion model: xi[n+1] - xi[n] - dt * M * uc[n] = 0
        #    M maps uc to robot world-frame velocity via rigid body:
        #    vx = vx_c - omega * r_iy ;  vy = vy_c + omega * r_ix ;  wi = omega
        rix, riy = self.r_i[0], self.r_i[1]
        M = np.array([
            [1.0, 0.0, -riy],
            [0.0, 1.0,  rix],
            [0.0, 0.0,  1.0],
        ])
        for n in range(N):
            self._add_linear(
                H, b,
                [self._xi(n), self._uc(n), self._xi(n + 1)],
                [-I3, -self.dt * M, I3],
                np.zeros(3),
                self.lam_mm * I3,
            )

        # 8) pull-in (xy only): xi[n] - xc[n] = 0 with 0 precision on theta
        Lam_pull = np.diag([self.lam_pull, self.lam_pull, 0.0])
        for n in range(N + 1):
            self._add_linear(
                H, b,
                [self._xi(n), self._xc(n)],
                [np.eye(3), -np.eye(3)],
                np.zeros(3),
                Lam_pull,
            )

        return H, b

    # --- Factor assembly primitives -----------------------------------------

    @staticmethod
    def _add_prior(H, b, sl, target, Lam):
        """Prior factor: x = target, precision Lam."""
        H[sl, sl] += Lam
        b[sl] += Lam @ target

    @staticmethod
    def _add_linear(H, b, slices, As, c, Lam):
        """Factor: sum_k A_k x_k - c = 0, precision Lam."""
        LAs = [Lam @ A for A in As]
        for i, sli in enumerate(slices):
            b[sli] += As[i].T @ (Lam @ c)
            for j, slj in enumerate(slices):
                H[sli, slj] += As[i].T @ LAs[j]

    # --- Messaging ----------------------------------------------------------

    def pack_outgoing_message(self) -> GaussianMessage:
        """
        Broadcast-ready belief: concatenated (own_traj, own_centroid) means
        in canonical form. Uses a nominal consensus precision; receivers
        recover the mean by solving Lam @ mean = eta.
        """
        traj = self._xi_block().flatten()
        cent = self._xc_block().flatten()
        combined = np.concatenate([traj, cent])
        Lam = self.lam_cons * np.eye(len(combined))
        eta = Lam @ combined
        return GaussianMessage(eta=eta, lam=Lam)

    def set_neighbor_message(self, from_id: int, msg: GaussianMessage) -> None:
        """Unpack neighbor broadcast into trajectory and centroid means."""
        # Robust solve: msg.lam is diagonal-positive by construction
        mean = np.linalg.solve(msg.lam + 1e-9 * np.eye(msg.lam.shape[0]), msg.eta)
        split = 3 * (self.N + 1)
        self._nbr_traj_mean[from_id] = mean[:split].reshape(self.N + 1, 3)
        self._nbr_cent_mean[from_id] = mean[split:].reshape(self.N + 1, 3)

    # --- GBP iteration ------------------------------------------------------

    def gbp_step(self) -> float:
        """One GBP iteration: relinearise R2R, add centroid consensus, solve."""
        H = self._H_lin.copy()
        b = self._b_lin.copy()

        # (a) R2R linearisation using latest neighbor trajectory means
        xi_block = self._xi_block()
        for j in self.neighbors:
            if j not in self._nbr_traj_mean:
                continue
            nbr_traj = self._nbr_traj_mean[j]
            L_ij = self.L[j]
            for n in range(self.N + 1):
                xi_n = xi_block[n][:2]
                xj_n = nbr_traj[n][:2]
                diff = xi_n - xj_n
                d = float(np.linalg.norm(diff))
                if d < 1e-6:
                    continue
                # e(xi) = ||xi - xj|| - L_ij ~ J * (xi - xi0) + (d - L_ij)
                # J: (1, 3) with zero in theta column
                J = np.array([[diff[0] / d, diff[1] / d, 0.0]])
                residual = d - L_ij
                c_target = J @ np.array([xi_n[0], xi_n[1], 0.0]) - residual
                sl = self._xi(n)
                JtL = J.T * self.lam_r2r
                H[sl, sl] += JtL @ J
                b[sl] += (JtL @ c_target).flatten()

        # (b) Centroid consensus: each neighbor's centroid mean pulls ours
        Lam_cons = self.lam_cons * np.eye(3)
        for j in self.neighbors:
            if j not in self._nbr_cent_mean:
                continue
            nbr_cent = self._nbr_cent_mean[j]
            for n in range(self.N + 1):
                sl = self._xc(n)
                H[sl, sl] += Lam_cons
                b[sl] += Lam_cons @ nbr_cent[n]

        # (c) Linear solve
        try:
            mu_new = np.linalg.solve(H, b)
        except np.linalg.LinAlgError:
            mu_new = np.linalg.solve(H + 1e-6 * np.eye(self.dim), b)

        change = float(np.linalg.norm(mu_new - self.mu))
        self.mu = mu_new
        return change

    def first_control(self) -> np.ndarray:
        return self._uc_block()[0].copy()


# ---------------------------------------------------------------------------
# Controller
# ---------------------------------------------------------------------------

class DRCapDistributedController(BaseController):
    """
    Distributed DR.CAP via GBP message-passing.

    Operates in two modes:

      Simulation mode (my_id is None): holds one LocalRobotGraph per robot in
      a single process, drives the GBP loop by round-robining through all
      graphs, and uses a single shared in-process backend (SimulatedBackend
      or AsyncSimulatedBackend). `compute_control` takes (N,4) robot_states
      and returns (N,2) velocities. This is the sim scaling experiment path.

      Deployment mode (my_id is an int): holds only this robot's local graph,
      uses an externally-constructed single-agent backend
      (ZeroMQSingleAgentBackend, sync or async). `compute_control` takes (1,4)
      of this robot's state and returns (1,2) of this robot's velocity. This
      is the path used by real_robot/robot/agent_runner.py.

    Parameters
    ----------
    num_robots : int
        Total number of robots in the formation (both modes).
    formation  : list of (x_off, y_off, yaw) per robot. Same convention as
                 MRCap/DRCap centralised (world-frame xy offset from centroid).
    backend    : CommunicationBackend.
                 In sim mode defaults to SimulatedBackend(num_robots, topology).
                 In deployment mode this MUST be supplied (and its my_id must
                 match the controller's my_id).
    my_id      : Optional[int]. None = simulation mode; int = deployment mode
                 for that robot. Default None.
    topology   : dict agent_id -> [neighbor_ids]. Defaults to fully connected.
    config     :
        horizon, sigma_x, sigma_u, sigma_anchor, sigma_mm, sigma_r2r,
        sigma_pull_in, sigma_consensus, v_max, omega_max, r_formation,
        gbp_max_iters (default 30), gbp_tol (default 1e-3).
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

        # Formation: world-frame xy offset from centroid per robot
        if formation is not None:
            self._r = np.array([[f[0], f[1]] for f in formation], dtype=float)
        else:
            radius = float(cfg.get("r_formation", 0.6))
            angles = np.linspace(0, 2 * np.pi, num_robots, endpoint=False)
            self._r = np.column_stack(
                [radius * np.cos(angles), radius * np.sin(angles)]
            )

        # Topology: default to full (all robots neighbors in the formation)
        if topology is None:
            topology = create_full_topology(num_robots)
        self._topology = topology

        # Backend: default SimulatedBackend in sim mode; required in deploy mode.
        if backend is None:
            if my_id is not None:
                raise ValueError(
                    "Deployment mode (my_id set) requires an externally "
                    "constructed backend (e.g. ZeroMQSingleAgentBackend)."
                )
            backend = SimulatedBackend(num_robots, topology)
        self.backend = backend

        # Local graphs keyed by robot id. In sim: all N. In deploy: only my_id.
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
        self._tol       = float(cfg.get("gbp_tol",       1e-3))
        self._v_max     = float(cfg.get("v_max",         1.0))
        self._omega_max = float(cfg.get("omega_max",     1.5))

        self._last_iters = 0

    # --- BaseController interface ------------------------------------------

    def reset(self):
        self._last_solve_time = 0.0
        self._total_solves = 0
        self._last_iters = 0

    def compute_control(
        self,
        payload_state: np.ndarray,
        robot_states: np.ndarray,
        goal_state: np.ndarray,
        dt: float,
        forces: np.ndarray = None,
    ) -> np.ndarray:
        t0 = time.perf_counter()

        centroid_pose = payload_state[:3].copy()
        goal = np.asarray(goal_state, dtype=float)[:3]

        # Warm-start owned graphs. In deploy mode robot_states is (1,4) and
        # carries this robot's state at row 0. In sim mode it is (N,4) with
        # row i for robot i. Robot states carry (x, y, vx, vy) only, so we
        # use centroid theta as a proxy for robot theta.
        for i in self._owned_ids:
            row = 0 if self.my_id is not None else i
            robot_pose = np.array([
                robot_states[row, 0],
                robot_states[row, 1],
                centroid_pose[2],
            ])
            self.local_graphs[i].warm_start(robot_pose, centroid_pose, goal, dt)

        # GBP iterations
        iters_done = 0
        for it in range(self._max_iters):
            for i in self._owned_ids:
                msg = self.local_graphs[i].pack_outgoing_message()
                self.backend.broadcast(i, msg)

            self.backend.barrier()

            for i in self._owned_ids:
                inbox = self.backend.receive(i)
                for sender_id, msg in inbox:
                    self.local_graphs[i].set_neighbor_message(sender_id, msg)

            max_change = 0.0
            for i in self._owned_ids:
                max_change = max(max_change, self.local_graphs[i].gbp_step())

            iters_done = it + 1
            if max_change < self._tol:
                break

        self._last_iters = iters_done

        # Read centroid control from my own graph (deploy) or leader robot 0
        # (sim). All local copies converge to similar values once GBP settles.
        source_id = self.my_id if self.my_id is not None else 0
        U_c = self.local_graphs[source_id].first_control()
        lo = np.array([-self._v_max, -self._v_max, -self._omega_max])
        hi = np.array([ self._v_max,  self._v_max,  self._omega_max])
        U_c = np.clip(U_c, lo, hi)

        self._set_solve_time(time.perf_counter() - t0)
        return self._robot_velocities(U_c)

    # --- Rigid-body velocity distribution ----------------------------------

    def _robot_velocities(self, U_c: np.ndarray) -> np.ndarray:
        vx_c, vy_c, omega_c = U_c
        # In deploy mode, compute only my own row of the rigid-body map.
        r = self._r if self.my_id is None else self._r[[self.my_id]]
        vx = vx_c - omega_c * r[:, 1]
        vy = vy_c + omega_c * r[:, 0]
        speeds = np.hypot(vx, vy)
        scale = np.where(speeds > self._v_max, self._v_max / speeds, 1.0)
        return np.column_stack([vx * scale, vy * scale])

    # --- Diagnostics --------------------------------------------------------

    def get_gbp_iters(self) -> int:
        return self._last_iters

    def get_stats(self) -> Dict[str, Any]:
        s = super().get_stats()
        s["gbp_iters_last"] = self._last_iters
        s["comm_stats"] = self.backend.get_stats()
        return s
