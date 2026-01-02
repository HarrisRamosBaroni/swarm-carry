#!/usr/bin/env python3
"""
Run MPC scaling experiments with configurable parameters.

This script runs experiments with varying numbers of robots, measures
performance metrics, and logs results to JSON for analysis.
"""

import argparse
import json
import time
import numpy as np
from pathlib import Path
from datetime import datetime
from typing import List, Dict, Any

try:
    import mujoco
    import mujoco.viewer
except ImportError:
    print("Error: MuJoCo Python bindings not found.")
    print("Install with: pip install mujoco")
    exit(1)

import sys
sys.path.insert(0, str(Path(__file__).parent.parent))

from controllers import CentralizedMPC
from scenarios.generate_mpc_scene import generate_mpc_scene


class MPCExperimentRunner:
    """Runs MPC scaling experiments in MuJoCo simulation."""

    def __init__(
        self,
        num_robots: int,
        push_distance: float = 10.0,
        controller_config: Dict[str, Any] = None,
        use_viewer: bool = False
    ):
        """
        Initialize experiment runner.

        Args:
            num_robots: Number of robots
            push_distance: Distance to push payload
            controller_config: Configuration for MPC controller
            use_viewer: Whether to show visualization
        """
        self.num_robots = num_robots
        self.push_distance = push_distance
        self.use_viewer = use_viewer
        self.controller_config = controller_config or {}

        # Generate scene
        self.scene_path = self._generate_scene()

        # Load MuJoCo model
        self.model = mujoco.MjModel.from_xml_path(str(self.scene_path))
        self.data = mujoco.MjData(self.model)

        # Initialize controller
        self.controller = CentralizedMPC(num_robots, controller_config)

        # Get body/actuator IDs
        self._setup_ids()

        # Goal state
        self.goal_state = np.array([push_distance, 0.0, 0.0])

        print(f"Experiment initialized: {num_robots} robots, {push_distance}m push")

    def _generate_scene(self) -> Path:
        """Generate scene file for this experiment."""
        scene_dir = Path(__file__).parent.parent / "scenarios" / "scenes"
        scene_dir.mkdir(parents=True, exist_ok=True)
        scene_path = scene_dir / f"mpc_scene_n{self.num_robots}.xml"

        generate_mpc_scene(
            num_robots=self.num_robots,
            push_distance=self.push_distance,
            output_path=str(scene_path)
        )
        return scene_path

    def _setup_ids(self):
        """Get MuJoCo body and actuator IDs."""
        # Payload
        self.payload_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "payload"
        )

        # Robot actuators
        self.robot_actuators = []
        for i in range(self.num_robots):
            left_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"robot_{i}_left_actuator"
            )
            right_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"robot_{i}_right_actuator"
            )
            self.robot_actuators.append((left_id, right_id))

        # Robot bodies
        self.robot_body_ids = []
        for i in range(self.num_robots):
            body_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_BODY, f"robot_{i}_base"
            )
            self.robot_body_ids.append(body_id)

    def get_payload_state(self) -> np.ndarray:
        """Get payload state [x, y, theta, vx, vy, omega]."""
        pos = self.data.xpos[self.payload_id][:2]  # x, y
        quat = self.data.xquat[self.payload_id]
        vel = self.data.cvel[self.payload_id]

        # Convert quaternion to euler angle (yaw)
        # For small rotations around z-axis: theta ≈ 2 * quat[3] if quat[0] > 0
        qw, qx, qy, qz = quat
        theta = np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2))

        vx, vy = vel[3:5]  # Linear velocity
        omega = vel[2]  # Angular velocity around z

        return np.array([pos[0], pos[1], theta, vx, vy, omega])

    def get_robot_states(self) -> np.ndarray:
        """Get robot states (n, 4) [x, y, vx, vy]."""
        states = np.zeros((self.num_robots, 4))
        for i, body_id in enumerate(self.robot_body_ids):
            pos = self.data.xpos[body_id][:2]
            vel = self.data.cvel[body_id][3:5]
            states[i] = [pos[0], pos[1], vel[0], vel[1]]
        return states

    def set_robot_controls(self, controls: np.ndarray):
        """Set robot wheel velocities."""
        for i, (left_id, right_id) in enumerate(self.robot_actuators):
            self.data.ctrl[left_id] = controls[i, 0]
            self.data.ctrl[right_id] = controls[i, 1]

    def run_experiment(
        self,
        max_time: float = 60.0,
        success_threshold: float = 0.5
    ) -> Dict[str, Any]:
        """
        Run a single experiment.

        Args:
            max_time: Maximum simulation time
            success_threshold: Distance threshold for success (meters)

        Returns:
            results: Dictionary with experiment results
        """
        print(f"Running experiment (n={self.num_robots})...")

        # Reset
        mujoco.mj_resetData(self.model, self.data)
        self.controller.reset()

        # Data collection
        payload_trajectory = []
        solve_times = []
        contact_counts = []

        start_wall_time = time.time()
        converged = False
        convergence_time = None

        if self.use_viewer:
            # Run with viewer
            with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
                viewer.cam.distance = max(8.0, self.push_distance / 2)
                viewer.cam.azimuth = 135
                viewer.cam.elevation = -25

                step_count = 0
                while viewer.is_running() and self.data.time < max_time:
                    # Get states
                    payload_state = self.get_payload_state()
                    robot_states = self.get_robot_states()

                    # Compute control
                    controls = self.controller.compute_control(
                        payload_state, robot_states, self.goal_state, self.model.opt.timestep
                    )
                    self.set_robot_controls(controls)

                    # Step simulation
                    mujoco.mj_step(self.model, self.data)
                    step_count += 1

                    # Log data
                    payload_trajectory.append(payload_state[:3].copy())
                    solve_times.append(self.controller.get_solve_time())
                    contact_counts.append(self.data.ncon)

                    # Check convergence
                    distance_to_goal = np.linalg.norm(payload_state[:2] - self.goal_state[:2])
                    if not converged and distance_to_goal < success_threshold:
                        converged = True
                        convergence_time = self.data.time

                    # Sync viewer
                    if step_count % 10 == 0:
                        viewer.sync()

                    # Progress reporting
                    if step_count % 200 == 0:
                        print(f"  t={self.data.time:.1f}s | d={distance_to_goal:.2f}m | "
                              f"solve_time={solve_times[-1]*1000:.1f}ms")

        else:
            # Run headless (faster)
            while self.data.time < max_time:
                # Get states
                payload_state = self.get_payload_state()
                robot_states = self.get_robot_states()

                # Compute control
                controls = self.controller.compute_control(
                    payload_state, robot_states, self.goal_state, self.model.opt.timestep
                )
                self.set_robot_controls(controls)

                # Step simulation
                mujoco.mj_step(self.model, self.data)

                # Log data
                payload_trajectory.append(payload_state[:3].copy())
                solve_times.append(self.controller.get_solve_time())
                contact_counts.append(self.data.ncon)

                # Check convergence
                distance_to_goal = np.linalg.norm(payload_state[:2] - self.goal_state[:2])
                if not converged and distance_to_goal < success_threshold:
                    converged = True
                    convergence_time = self.data.time

                # Early termination if converged
                if converged and self.data.time > convergence_time + 5.0:
                    break

        wall_time = time.time() - start_wall_time

        # Final distance
        final_payload_state = self.get_payload_state()
        final_distance = np.linalg.norm(final_payload_state[:2] - self.goal_state[:2])

        # Compile results
        results = {
            'n_agents': self.num_robots,
            'converged': converged,
            'convergence_time': convergence_time if converged else None,
            'final_distance_to_goal': float(final_distance),
            'simulation_time': float(self.data.time),
            'wall_time': float(wall_time),
            'solve_times': [float(t) for t in solve_times],
            'mean_solve_time': float(np.mean(solve_times)),
            'max_solve_time': float(np.max(solve_times)),
            'min_solve_time': float(np.min(solve_times)),
            'std_solve_time': float(np.std(solve_times)),
            'payload_trajectory': [[float(x) for x in state] for state in payload_trajectory],
            'mean_contacts': float(np.mean(contact_counts)),
            'max_contacts': int(np.max(contact_counts)),
            'controller_stats': self.controller.get_stats(),
        }

        print(f"  ✓ Completed: converged={converged}, "
              f"final_dist={final_distance:.2f}m, "
              f"mean_solve={results['mean_solve_time']*1000:.1f}ms")

        return results


def run_batch_experiments(
    n_values: List[int],
    push_distance: float = 10.0,
    controller_config: Dict[str, Any] = None,
    max_time: float = 60.0,
    use_viewer: bool = False,
    output_dir: Path = None
) -> Dict[str, Any]:
    """
    Run experiments for multiple n values.

    Args:
        n_values: List of robot counts to test
        push_distance: Push distance for all experiments
        controller_config: MPC controller configuration
        max_time: Maximum simulation time per experiment
        use_viewer: Show visualization
        output_dir: Directory to save logs

    Returns:
        batch_results: Dictionary with all results
    """
    results = {
        'metadata': {
            'date': datetime.now().isoformat(),
            'push_distance': push_distance,
            'max_time': max_time,
            'controller_config': controller_config or {},
            'n_values': n_values,
        },
        'runs': []
    }

    for n in n_values:
        print(f"\n{'='*60}")
        print(f"Experiment: n={n} robots")
        print(f"{'='*60}")

        runner = MPCExperimentRunner(
            num_robots=n,
            push_distance=push_distance,
            controller_config=controller_config,
            use_viewer=use_viewer
        )

        run_result = runner.run_experiment(max_time=max_time)
        results['runs'].append(run_result)

    # Save results
    if output_dir:
        output_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"mpc_scaling_{timestamp}.json"
        filepath = output_dir / filename

        with open(filepath, 'w') as f:
            json.dump(results, f, indent=2)

        print(f"\n{'='*60}")
        print(f"Results saved: {filepath}")
        print(f"{'='*60}")

        # Also save as 'latest.json' for easy access
        latest_path = output_dir / "latest.json"
        with open(latest_path, 'w') as f:
            json.dump(results, f, indent=2)

    return results


def main():
    parser = argparse.ArgumentParser(
        description='Run MPC scaling experiments',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )

    parser.add_argument('-n', '--n-values', type=str, default='2,4,8,16',
                       help='Comma-separated list of robot counts (default: 2,4,8,16)')
    parser.add_argument('-d', '--distance', type=float, default=10.0,
                       help='Push distance in meters (default: 10.0)')
    parser.add_argument('-t', '--max-time', type=float, default=60.0,
                       help='Maximum simulation time per run (default: 60.0)')
    parser.add_argument('--horizon', type=int, default=20,
                       help='MPC horizon (default: 20)')
    parser.add_argument('--dt', type=float, default=0.05,
                       help='MPC time step (default: 0.05)')
    parser.add_argument('--solver', type=str, default='ipopt',
                       help='MPC solver (default: ipopt)')
    parser.add_argument('--viewer', action='store_true',
                       help='Show visualization')
    parser.add_argument('-o', '--output', type=str, default=None,
                       help='Output directory for logs')

    args = parser.parse_args()

    # Parse n values
    n_values = [int(x.strip()) for x in args.n_values.split(',')]

    # Controller config
    controller_config = {
        'horizon': args.horizon,
        'dt': args.dt,
        'solver': args.solver,
    }

    # Output directory
    if args.output:
        output_dir = Path(args.output)
    else:
        output_dir = Path(__file__).parent.parent / "analysis" / "logs"

    # Run experiments
    run_batch_experiments(
        n_values=n_values,
        push_distance=args.distance,
        controller_config=controller_config,
        max_time=args.max_time,
        use_viewer=args.viewer,
        output_dir=output_dir
    )


if __name__ == '__main__':
    main()
