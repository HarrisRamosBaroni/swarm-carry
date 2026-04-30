"""
CentroidEstimator: infer payload centroid pose and velocity from robot states alone.

Useful when the payload has no dedicated sensor (no mocap markers on the object),
but individual robot poses are available (e.g. mocap markers on each robot chassis).

Usage
-----
    est = CentroidEstimator()

    # Once, at run start — needs one ground-truth payload pose for body-frame calibration.
    est.reset(robot_states_init, payload_state_gt)

    # Every control step thereafter — no payload sensor required.
    centroid = est.estimate(robot_states)   # [x, y, θ, vx, vy, ω]

Pose estimation — orthogonal Procrustes (SVD)
---------------------------------------------
Given n robot positions p_i (world frame) and their fixed body-frame offsets r_i
(recorded at reset), we find the rotation R and translation t minimising

    Σ_i || p_i − (R r_i + t) ||²

Solution (Horn 1987 / Kabsch):
    p̄ = mean(p_i),  r̄ = mean(r_i)
    M = (p − p̄)ᵀ (r − r̄)          # (2×2 cross-covariance)
    U, S, Vᵀ = SVD(M)
    R_est = U diag(1, det(U Vᵀ)) Vᵀ  # det correction for proper rotation
    t_est = p̄ − R_est r̄

For symmetric formations (Σ r_i = 0, i.e. r̄ = 0) this reduces to
t_est = p̄ — centroid position is just the mean of robot positions.

Does not require robot heading measurements; heading is read from the
rotation matrix: θ_c = atan2(R_est[1,0], R_est[0,0]).

Velocity estimation — rigid-body least squares
----------------------------------------------
For each robot i:
    vx_i = vx_c − ω_c r_iy_world
    vy_i = vy_c + ω_c r_ix_world

Stacked for all n robots this is an over-determined (2n × 3) linear system
solved via least squares, giving [vx_c, vy_c, ω_c] directly.

Assumptions
-----------
- Formation is rigid throughout the run (body-frame offsets r_i are constant).
- At least 2 robots (1 robot leaves θ_c singular via Procrustes on xy alone).
- robot_states columns: [x, y, θ, vx, vy, ...] (θ unused; vx/vy used for velocity).
"""

import numpy as np


class CentroidEstimator:
    """
    Estimate payload centroid state from robot poses and velocities.

    Parameters are calibrated once from a ground-truth snapshot; no
    payload sensor is required after that.
    """

    def __init__(self) -> None:
        self._r_body: np.ndarray | None = None  # (n, 2) fixed body-frame offsets
        self._r_bar:  np.ndarray | None = None  # (2,)  mean body-frame offset

    # ------------------------------------------------------------------

    def reset(self, robot_states: np.ndarray, payload_state: np.ndarray) -> None:
        """
        Calibrate body-frame offsets from a known ground-truth snapshot.

        Parameters
        ----------
        robot_states  : (n, ≥2)  first two columns are [x, y] world-frame robot positions
        payload_state : (≥3,)    [x, y, θ, ...] ground-truth payload pose at this instant
        """
        p_c = payload_state[:2].astype(float)
        θ_c = float(payload_state[2])
        c, s = np.cos(θ_c), np.sin(θ_c)
        R_inv = np.array([[c, s], [-s, c]])                                  # world → body
        self._r_body = (robot_states[:, :2].astype(float) - p_c) @ R_inv.T  # (n, 2)
        self._r_bar  = self._r_body.mean(axis=0)                             # (2,)

    # ------------------------------------------------------------------

    def estimate(self, robot_states: np.ndarray) -> np.ndarray:
        """
        Estimate centroid state from current robot states.

        Parameters
        ----------
        robot_states : (n, ≥5)  columns [x, y, θ, vx, vy, ...]

        Returns
        -------
        state : (6,)  [x, y, θ, vx, vy, ω]

        Raises
        ------
        RuntimeError  if reset() has not been called yet
        """
        if self._r_body is None:
            raise RuntimeError("CentroidEstimator.reset() must be called before estimate()")

        p = robot_states[:, :2].astype(float)   # (n, 2) positions
        v = robot_states[:, 3:5].astype(float)  # (n, 2) world-frame velocities
        r = self._r_body                          # (n, 2) body-frame offsets

        # ---- Pose via orthogonal Procrustes ------------------------------------
        p_bar = p.mean(axis=0)
        M     = (p - p_bar).T @ (r - self._r_bar)   # (2, 2) cross-covariance
        U, _, Vt = np.linalg.svd(M)
        R_est = U @ np.diag([1.0, np.linalg.det(U @ Vt)]) @ Vt  # body → world
        θ_c   = np.arctan2(R_est[1, 0], R_est[0, 0])
        p_c   = p_bar - R_est @ self._r_bar

        # ---- Velocity via rigid-body least squares ----------------------------
        r_world = r @ R_est.T     # (n, 2) current world-frame offsets
        n = len(robot_states)
        A = np.zeros((2 * n, 3))
        for i in range(n):
            A[2*i]   = [1.0, 0.0, -r_world[i, 1]]
            A[2*i+1] = [0.0, 1.0,  r_world[i, 0]]
        vel, _, _, _ = np.linalg.lstsq(A, v.ravel(), rcond=None)  # [vx_c, vy_c, ω_c]

        return np.array([p_c[0], p_c[1], θ_c, vel[0], vel[1], vel[2]])
