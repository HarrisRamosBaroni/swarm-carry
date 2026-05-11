"""
WeightedCentroidEstimator: contact-force-weighted variant of CentroidEstimator.

Same orthogonal-Procrustes formulation as CentroidEstimator, but per-robot
contributions to the centroid pose and velocity estimates are scaled by
externally supplied weights w_i. Reduces *exactly* to the unweighted
estimator when w_i = const  (Procrustes is scale-invariant in the weights).

Intended use: in conjunction with a contact-health controller, weights are
derived from per-robot force sensors so that robots whose contact has
degraded (one or both load cells reading below their nominal target)
contribute proportionally less to the centroid estimate.

References
----------
- Horn 1987, "Closed-form solution of absolute orientation using unit
  quaternions" — the weighted-mean / weighted cross-covariance variant
  appears as a direct generalisation in standard rigid-registration texts.

See experiments/centralised_contact_health_fg/PROBLEM_STATEMENT.md.
"""

import numpy as np


class WeightedCentroidEstimator:
    """
    Estimate payload centroid pose and velocity from robot states with
    per-robot weights.

    Body-frame robot offsets are calibrated once via reset() against a known
    ground-truth payload pose, after which no payload sensor is required.

    Parameters
    ----------
    weight_eps : float
        Lower clamp on weights. Prevents rank deficiency if all robots
        simultaneously lose contact. Default 1e-3.
    """

    def __init__(self, weight_eps: float = 1e-3) -> None:
        self._r_body: np.ndarray | None = None    # (n, 2) body-frame offsets
        self._weight_eps = float(weight_eps)

    # ------------------------------------------------------------------

    def reset(self, robot_states: np.ndarray, payload_state: np.ndarray) -> None:
        """
        Calibrate body-frame robot offsets from a known payload pose.

        Parameters
        ----------
        robot_states  : (n, ≥2) world-frame robot positions in cols [0,1]
        payload_state : (≥3,) [x, y, θ, ...] ground-truth payload pose
        """
        p_c = payload_state[:2].astype(float)
        θ_c = float(payload_state[2])
        c, s = np.cos(θ_c), np.sin(θ_c)
        R_inv = np.array([[c, s], [-s, c]])                                  # world → body
        self._r_body = (robot_states[:, :2].astype(float) - p_c) @ R_inv.T  # (n, 2)

    # ------------------------------------------------------------------

    def estimate(
        self,
        robot_states: np.ndarray,
        weights: np.ndarray | None = None,
    ) -> np.ndarray:
        """
        Weighted-Procrustes centroid pose + weighted-LS centroid velocity.

        Parameters
        ----------
        robot_states : (n, ≥5)  columns [x, y, θ, vx, vy, ...]
        weights      : (n,) or None  per-robot weights; None ⇒ unweighted.

        Returns
        -------
        state : (6,)  [x, y, θ, vx, vy, ω]
        """
        if self._r_body is None:
            raise RuntimeError("WeightedCentroidEstimator.reset() must be called before estimate()")

        n = len(robot_states)
        p = robot_states[:, :2].astype(float)
        v = robot_states[:, 3:5].astype(float)
        r = self._r_body

        if weights is None:
            w = np.ones(n)
        else:
            w = np.maximum(np.asarray(weights, dtype=float), self._weight_eps)
        w_sum = w.sum()

        # ---- Weighted pose via orthogonal Procrustes ---------------------------
        p_bar = (w[:, None] * p).sum(axis=0) / w_sum
        r_bar = (w[:, None] * r).sum(axis=0) / w_sum
        dp = p - p_bar
        dr = r - r_bar
        M = (w[:, None] * dp).T @ dr               # (2, 2) weighted cross-cov

        U, _, Vt = np.linalg.svd(M)
        R_est = U @ np.diag([1.0, np.linalg.det(U @ Vt)]) @ Vt  # body → world
        θ_c   = np.arctan2(R_est[1, 0], R_est[0, 0])
        p_c   = p_bar - R_est @ r_bar

        # ---- Weighted velocity via rigid-body LS -------------------------------
        # For each i: vx_i = vx_c − ω_c r_iy_w  ;  vy_i = vy_c + ω_c r_ix_w
        # Apply √w_i to both sides for weighted least squares.
        r_world = r @ R_est.T                       # (n, 2)
        sqrt_w = np.sqrt(w)
        A = np.zeros((2 * n, 3))
        b = np.zeros(2 * n)
        for i in range(n):
            A[2*i]     = sqrt_w[i] * np.array([1.0, 0.0, -r_world[i, 1]])
            A[2*i + 1] = sqrt_w[i] * np.array([0.0, 1.0,  r_world[i, 0]])
            b[2*i]     = sqrt_w[i] * v[i, 0]
            b[2*i + 1] = sqrt_w[i] * v[i, 1]
        vel, _, _, _ = np.linalg.lstsq(A, b, rcond=None)

        return np.array([p_c[0], p_c[1], θ_c, vel[0], vel[1], vel[2]])
