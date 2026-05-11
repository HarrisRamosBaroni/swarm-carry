"""
Collision-avoiding batch trajectory planner for multi-robot reset.

Formulates multi-robot planning as a GTSAM factor graph (Patwardhan et al. 2023,
"Distributing Collaborative Multi-Robot Planning with Gaussian Belief Propagation").
Since the laptop holds all poses centrally, we skip GBP and run LM directly.

Factor graph per plan:
  Variables : x_k^i = [px, py, vx, vy]  for robot i at timestep k
  Factors   :
    pose prior (start) -- tight prior anchoring x_0^i to current pose, v=0
    pose prior (goal)  -- soft prior anchoring x_{K-1}^i to target pose, v=0
    dynamics           -- GP noise-on-acceleration model between consecutive states
    inter-robot        -- hinge repulsion for k in [1, K-2] (skip endpoints
                          since robots may start/end in close formation)
"""

from functools import partial
from typing import List, Optional

import numpy as np

try:
    import gtsam
except ImportError:
    raise ImportError("gtsam required: pip install gtsam")

# r* = 2 * (ROBOT_RADIUS + MARGIN) = 2 * (0.2 + 0.1)
_R_STAR_DEFAULT = 0.6


# ---------------------------------------------------------------------------
# Factor error functions
# ---------------------------------------------------------------------------

def _prior_error(
    z: np.ndarray,
    this: gtsam.CustomFactor,
    values: gtsam.Values,
    jacobians: Optional[List[np.ndarray]],
) -> np.ndarray:
    x = values.atVector(this.keys()[0])
    e = x - z
    if jacobians is not None:
        jacobians[0] = np.eye(len(z))
    return e


def _dynamics_error(
    Phi: np.ndarray,
    this: gtsam.CustomFactor,
    values: gtsam.Values,
    jacobians: Optional[List[np.ndarray]],
) -> np.ndarray:
    """GP noise-on-acceleration: Phi @ x_k - x_{k+1} should be ~0."""
    xk  = values.atVector(this.keys()[0])
    xk1 = values.atVector(this.keys()[1])
    e = Phi @ xk - xk1
    if jacobians is not None:
        jacobians[0] =  Phi
        jacobians[1] = -np.eye(4)
    return e


def _interrobot_error(
    r_star: float,
    this: gtsam.CustomFactor,
    values: gtsam.Values,
    jacobians: Optional[List[np.ndarray]],
) -> np.ndarray:
    """Hinge repulsion: error = max(0, 1 - d/r*)."""
    xa = values.atVector(this.keys()[0])
    xb = values.atVector(this.keys()[1])
    dp = xa[:2] - xb[:2]
    d  = np.linalg.norm(dp)
    if jacobians is not None:
        jacobians[0] = np.zeros((1, 4))
        jacobians[1] = np.zeros((1, 4))
    if d >= r_star or d < 1e-9:
        return np.zeros(1)
    e = np.array([1.0 - d / r_star])
    if jacobians is not None:
        g = -dp / (r_star * d)
        jacobians[0][0, :2] =  g
        jacobians[1][0, :2] = -g
    return e


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def plan_reset(
    current_poses: np.ndarray,
    target_poses: np.ndarray,
    dt: float = 0.5,
    K: int = 50,
    sigma_d: float = 0.5,
    sigma_start: float = 1e-3,
    sigma_goal: float = 0.03,
    sigma_v0: float = 0.01,
    sigma_r: float = 0.03,
    r_star: float = _R_STAR_DEFAULT,
) -> np.ndarray:
    """
    Plan collision-avoiding reset trajectories.

    Parameters
    ----------
    current_poses : (N, 3)  [x, y, theta] — current robot poses
    target_poses  : (N, 3)  [x, y, theta] — snapshot target poses
    dt            : timestep (s)
    K             : number of timesteps in plan
    sigma_d       : GP process noise — controls trajectory smoothness/aggressiveness
    sigma_start   : position std at k=0 (tight anchor)
    sigma_goal    : position std at k=K-1 (softer — robot may not reach exactly)
    sigma_v0      : velocity std at k=0 and k=K-1 (near-zero velocity at endpoints)
    sigma_r       : inter-robot repulsion noise (smaller = stronger avoidance)
    r_star        : minimum allowed inter-robot distance (m)

    Returns
    -------
    waypoints : (N, K, 2)  [x, y] at each planned timestep
    """
    N = len(current_poses)
    assert len(target_poses) == N

    # State transition: x_{k+1} = Phi @ x_k  (noise-on-acceleration GP)
    Phi = np.array([
        [1, 0, dt, 0],
        [0, 1, 0,  dt],
        [0, 0, 1,  0],
        [0, 0, 0,  1],
    ], dtype=float)

    # GP process noise covariance (eq. 16 in paper, 2D)
    Q = sigma_d**2 * np.array([
        [dt**3/3, 0,       dt**2/2, 0      ],
        [0,       dt**3/3, 0,       dt**2/2],
        [dt**2/2, 0,       dt,      0      ],
        [0,       dt**2/2, 0,       dt     ],
    ])
    noise_dyn   = gtsam.noiseModel.Gaussian.Covariance(Q)
    noise_start = gtsam.noiseModel.Diagonal.Sigmas(
        np.array([sigma_start, sigma_start, sigma_v0, sigma_v0]))
    noise_goal  = gtsam.noiseModel.Diagonal.Sigmas(
        np.array([sigma_goal,  sigma_goal,  sigma_v0, sigma_v0]))
    noise_r     = gtsam.noiseModel.Isotropic.Sigma(1, sigma_r)

    def key(i, k):
        return gtsam.symbol(chr(ord('a') + i), k)

    graph = gtsam.NonlinearFactorGraph()
    init  = gtsam.Values()

    # Initial guess: linearly interpolate positions, finite-diff velocities
    for i in range(N):
        p0 = current_poses[i, :2]
        pT = target_poses[i, :2]
        positions = np.stack([p0 + (k / (K - 1)) * (pT - p0) for k in range(K)])
        for k in range(K):
            if k == 0 or k == K - 1:
                v = np.zeros(2)
            else:
                v = (positions[k + 1] - positions[k - 1]) / (2 * dt)
            init.insert(key(i, k), np.concatenate([positions[k], v]))

    # Pose priors
    for i in range(N):
        x0 = np.array([current_poses[i, 0], current_poses[i, 1], 0.0, 0.0])
        xT = np.array([target_poses[i, 0],  target_poses[i, 1],  0.0, 0.0])
        graph.add(gtsam.CustomFactor(noise_start, [key(i, 0)],     partial(_prior_error, x0)))
        graph.add(gtsam.CustomFactor(noise_goal,  [key(i, K - 1)], partial(_prior_error, xT)))

    # Dynamics factors
    for i in range(N):
        for k in range(K - 1):
            graph.add(gtsam.CustomFactor(
                noise_dyn, [key(i, k), key(i, k + 1)],
                partial(_dynamics_error, Phi)))

    # Inter-robot repulsion factors (skip k=0 and k=K-1: robots may start/end in close formation)
    for k in range(1, K - 1):
        for i in range(N):
            for j in range(i + 1, N):
                graph.add(gtsam.CustomFactor(
                    noise_r, [key(i, k), key(j, k)],
                    partial(_interrobot_error, r_star)))

    params = gtsam.LevenbergMarquardtParams()
    params.setMaxIterations(100)
    result = gtsam.LevenbergMarquardtOptimizer(graph, init, params).optimize()

    waypoints = np.zeros((N, K, 2))
    for i in range(N):
        for k in range(K):
            waypoints[i, k] = result.atVector(key(i, k))[:2]
    return waypoints
