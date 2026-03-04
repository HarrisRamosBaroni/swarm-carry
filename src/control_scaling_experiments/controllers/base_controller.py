"""
Base controller interface for multi-robot payload transport.

All controllers must implement this interface to be compatible with
the experiment runner and scaling analysis framework.
"""

from abc import ABC, abstractmethod
import numpy as np
from typing import Dict, Any, Optional


def cartesian_to_diff_drive(
    vx: float,
    vy: float,
    heading_rad: float,
    wheel_base: float = 0.287,
    wheel_radius: float = 0.033,
) -> tuple:
    """
    Project a holonomic [vx, vy] command onto a differential-drive robot.

    The robot can only move along its heading axis, so lateral velocity is
    dropped and only the forward component is used. The returned wheel speeds
    are in rad/s.

    Args:
        vx: Desired x-velocity in world frame (m/s)
        vy: Desired y-velocity in world frame (m/s)
        heading_rad: Robot heading (yaw) in radians
        wheel_base: Distance between wheels in metres (default: TurtleBot3 0.287 m)
        wheel_radius: Wheel radius in metres (default: TurtleBot3 0.033 m)

    Returns:
        (v_left, v_right): Left and right wheel angular velocities in rad/s
    """
    # Project world-frame velocity onto robot heading to get forward speed
    v_linear = vx * np.cos(heading_rad) + vy * np.sin(heading_rad)
    omega = 0.0  # No angular velocity command from Cartesian interface
    v_left = (v_linear - omega * wheel_base / 2.0) / wheel_radius
    v_right = (v_linear + omega * wheel_base / 2.0) / wheel_radius
    return v_left, v_right


class BaseController(ABC):
    """Abstract base class for multi-robot transport controllers."""

    def __init__(self, num_robots: int, config: Optional[Dict[str, Any]] = None):
        """
        Initialize the controller.

        Args:
            num_robots: Number of robots in the system
            config: Optional configuration dictionary
        """
        self.num_robots = num_robots
        self.config = config or {}
        self._last_solve_time = 0.0
        self._total_solves = 0

    @abstractmethod
    def compute_control(
        self,
        payload_state: np.ndarray,
        robot_states: np.ndarray,
        goal_state: np.ndarray,
        dt: float,
        forces: np.ndarray = None,
    ) -> np.ndarray:
        """
        Compute control commands for all robots.

        Args:
            payload_state: (6,) [x, y, theta, vx, vy, omega]
            robot_states: (n, 4) array of [x, y, vx, vy] for each robot
            goal_state: (3,) [x_goal, y_goal, theta_goal]
            dt: Time step (seconds)
            forces: (n, 3) [fx, fy, torque_z] external forces per robot, or None

        Returns:
            controls: (n, 2) array of [vx, vy] in m/s (Cartesian, world frame)
        """
        pass

    @abstractmethod
    def reset(self):
        """Reset controller state (e.g., for new experiment run)."""
        pass

    def get_solve_time(self) -> float:
        """
        Get the time taken for the last control computation.

        Returns:
            solve_time: Time in seconds
        """
        return self._last_solve_time

    def get_stats(self) -> Dict[str, Any]:
        """
        Get controller statistics.

        Returns:
            stats: Dictionary with controller-specific statistics
        """
        return {
            'total_solves': self._total_solves,
            'last_solve_time': self._last_solve_time,
            'avg_solve_time': self._last_solve_time,  # Override in subclass for running average
        }

    def _set_solve_time(self, time: float):
        """Internal method to record solve time."""
        self._last_solve_time = time
        self._total_solves += 1
