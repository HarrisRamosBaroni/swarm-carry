"""
Centralized MPC controller for multi-robot payload transport.

Implements the formulation from .plan/multi_robot_transport_mpc.md using CasADi.
"""

import numpy as np
import time
from typing import Dict, Any, Optional

try:
    import casadi as ca
except ImportError:
    raise ImportError(
        "CasADi is required for MPC controller. "
        "Install with: pip install casadi"
    )

from .base_controller import BaseController


class CentralizedMPC(BaseController):
    """
    Centralized MPC controller using kinematic velocity control.

    Formulation:
    - State: X = [x_p, y_p, theta_p, x_1, y_1, ..., x_n, y_n]
    - Control: U = [vx_1, vy_1, ..., vx_n, vy_n]
    - Payload velocity is average of robot velocities (uniform pushing)
    """

    def __init__(
        self,
        num_robots: int,
        config: Optional[Dict[str, Any]] = None
    ):
        """
        Initialize centralized MPC controller.

        Config options:
            horizon: Prediction horizon (default: 20)
            dt: Time step (default: 0.05)
            Q_pos: Weight for payload position error (default: 10.0)
            Q_theta: Weight for payload orientation error (default: 1.0)
            R: Weight for control effort (default: 0.1)
            v_max: Maximum robot velocity (default: 1.0 m/s)
            wheel_base: Robot wheel base (default: 0.287 m for TurtleBot3)
            solver: Solver to use (default: 'ipopt')
        """
        super().__init__(num_robots, config)

        # MPC parameters
        self.T = self.config.get('horizon', 20)
        self.dt = self.config.get('dt', 0.05)
        self.Q_pos = self.config.get('Q_pos', 10.0)
        self.Q_theta = self.config.get('Q_theta', 1.0)
        self.R = self.config.get('R', 0.1)
        self.v_max = self.config.get('v_max', 1.0)
        self.wheel_base = self.config.get('wheel_base', 0.287)
        self.solver_name = self.config.get('solver', 'ipopt')

        # Build MPC problem
        self._build_mpc_problem()

        # Solve time tracking
        self._solve_times = []

    def _build_mpc_problem(self):
        """Build the CasADi optimization problem."""
        n = self.num_robots
        T = self.T

        # Decision variables: controls over horizon
        # U[t] = [vx_1, vy_1, ..., vx_n, vy_n] for each t in [0, T-1]
        self.U_var = ca.MX.sym('U', 2*n, T)

        # Parameters: initial state and goal
        # X_0 = [x_p, y_p, theta_p, x_1, y_1, ..., x_n, y_n]
        self.X_0 = ca.MX.sym('X_0', 3 + 2*n)
        # Goal = [x_goal, y_goal, theta_goal]
        self.X_goal = ca.MX.sym('X_goal', 3)

        # State trajectory
        X = self.X_0
        cost = 0

        for t in range(T):
            U_t = self.U_var[:, t]

            # Extract current states
            x_p = X[0]
            y_p = X[1]
            theta_p = X[2]
            # robot_positions = X[3:]  # [x_1, y_1, ..., x_n, y_n]

            # Compute payload velocity (average of robot velocities for uniform push)
            vx_p = ca.sum1(U_t[0::2]) / n  # Average of x-velocities
            vy_p = ca.sum1(U_t[1::2]) / n  # Average of y-velocities

            # For straight pushing along one face with no rotation requirement,
            # angular velocity is zero
            omega_p = 0.0

            # Update payload state
            x_p_next = x_p + vx_p * self.dt
            y_p_next = y_p + vy_p * self.dt
            theta_p_next = theta_p + omega_p * self.dt

            # Update robot states (robots move according to their commanded velocities)
            robot_states_next = []
            for i in range(n):
                x_i = X[3 + 2*i]
                y_i = X[3 + 2*i + 1]
                vx_i = U_t[2*i]
                vy_i = U_t[2*i + 1]

                x_i_next = x_i + vx_i * self.dt
                y_i_next = y_i + vy_i * self.dt

                robot_states_next.extend([x_i_next, y_i_next])

            # Construct next state
            X = ca.vertcat(x_p_next, y_p_next, theta_p_next, *robot_states_next)

            # Cost function
            # Payload tracking error
            pos_error = ca.sumsqr(X[0:2] - self.X_goal[0:2])
            theta_error = (X[2] - self.X_goal[2])**2

            # Control effort
            control_effort = ca.sumsqr(U_t)

            # Add to total cost
            cost += self.Q_pos * pos_error + self.Q_theta * theta_error + self.R * control_effort

        # Terminal cost (higher weight on final state)
        pos_error_final = ca.sumsqr(X[0:2] - self.X_goal[0:2])
        theta_error_final = (X[2] - self.X_goal[2])**2
        cost += 10 * self.Q_pos * pos_error_final + 10 * self.Q_theta * theta_error_final

        # Constraints
        g = []  # Constraint expressions
        lbg = []  # Lower bounds
        ubg = []  # Upper bounds

        # Velocity limits for each robot at each time step
        for t in range(T):
            for i in range(n):
                vx = self.U_var[2*i, t]
                vy = self.U_var[2*i+1, t]
                # Velocity magnitude constraint
                g.append(vx**2 + vy**2)
                lbg.append(0)
                ubg.append(self.v_max**2)

        # NLP problem
        nlp = {
            'x': ca.reshape(self.U_var, -1, 1),  # Flatten decision variables
            'f': cost,
            'g': ca.vertcat(*g) if g else ca.MX(),
            'p': ca.vertcat(self.X_0, self.X_goal)  # Parameters
        }

        # Solver options
        opts = {
            'ipopt.print_level': 0,
            'print_time': 0,
            'ipopt.max_iter': 100,
            'ipopt.warm_start_init_point': 'yes',
            'ipopt.tol': 1e-3,
        }

        # Create solver
        self.solver = ca.nlpsol('solver', self.solver_name, nlp, opts)

        # Store bounds for solver
        self.lbg = lbg
        self.ubg = ubg

        print(f"MPC Problem Built:")
        print(f"  - Horizon: {self.T} steps ({self.T * self.dt:.2f}s)")
        print(f"  - Decision variables: {2*n*T}")
        print(f"  - Constraints: {len(lbg)}")
        print(f"  - Solver: {self.solver_name}")

    def compute_control(
        self,
        payload_state: np.ndarray,
        robot_states: np.ndarray,
        goal_state: np.ndarray,
        dt: float,
        forces: np.ndarray = None,
    ) -> np.ndarray:
        """
        Solve MPC problem and return control commands.

        Args:
            payload_state: (6,) [x, y, theta, vx, vy, omega]
            robot_states: (n, 4) array of [x, y, vx, vy]
            goal_state: (3,) [x_goal, y_goal, theta_goal]
            dt: Time step (not used, we use self.dt from config)
            forces: (n, 3) external forces (unused by MPC, accepted for interface compat)

        Returns:
            controls: (n, 2) array of [vx, vy] in m/s (Cartesian, world frame)
        """
        n = self.num_robots

        # Construct initial state vector
        x_0 = np.concatenate([
            payload_state[:3],  # [x_p, y_p, theta_p]
            robot_states[:, :2].flatten()  # [x_1, y_1, ..., x_n, y_n]
        ])

        # Parameters
        p = np.concatenate([x_0, goal_state])

        # Initial guess (warm start with zeros or previous solution)
        if not hasattr(self, '_last_solution'):
            u_init = np.zeros(2 * n * self.T)
        else:
            # Warm start: shift previous solution and append zeros
            u_init = np.roll(self._last_solution, -2*n)
            u_init[-2*n:] = 0

        # Solve
        start_time = time.time()
        try:
            sol = self.solver(
                x0=u_init,
                lbx=-np.inf,
                ubx=np.inf,
                lbg=self.lbg,
                ubg=self.ubg,
                p=p
            )
            solve_time = time.time() - start_time
            self._set_solve_time(solve_time)
            self._solve_times.append(solve_time)

            # Extract solution — first step's [vx, vy] for each robot
            u_opt = sol['x'].full().flatten()
            self._last_solution = u_opt
            u_0 = u_opt[:2*n]

            # Return as (n, 2) Cartesian [vx, vy] — environment handles actuation
            controls = u_0.reshape(n, 2)
            return controls

        except Exception as e:
            print(f"MPC solve failed: {e}")
            self._set_solve_time(time.time() - start_time)
            return np.zeros((n, 2))

    def reset(self):
        """Reset controller state."""
        if hasattr(self, '_last_solution'):
            delattr(self, '_last_solution')
        self._solve_times = []

    def get_stats(self) -> Dict[str, Any]:
        """Get controller statistics."""
        stats = super().get_stats()
        if self._solve_times:
            stats['avg_solve_time'] = np.mean(self._solve_times)
            stats['max_solve_time'] = np.max(self._solve_times)
            stats['min_solve_time'] = np.min(self._solve_times)
            stats['std_solve_time'] = np.std(self._solve_times)
        return stats
