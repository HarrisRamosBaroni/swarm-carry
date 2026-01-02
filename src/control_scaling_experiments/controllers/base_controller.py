"""
Base controller interface for multi-robot payload transport.

All controllers must implement this interface to be compatible with
the experiment runner and scaling analysis framework.
"""

from abc import ABC, abstractmethod
import numpy as np
from typing import Dict, Any, Optional


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
        dt: float
    ) -> np.ndarray:
        """
        Compute control commands for all robots.

        Args:
            payload_state: [x, y, theta, vx, vy, omega] - payload pose and velocity
            robot_states: (n, 4) array of [x, y, vx, vy] for each robot
            goal_state: [x_goal, y_goal, theta_goal] - target payload pose
            dt: Time step (seconds)

        Returns:
            controls: (n, 2) array of [left_wheel_vel, right_wheel_vel] for each robot
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
