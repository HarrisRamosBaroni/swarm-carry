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

import sys
sys.path.insert(0, str(Path(__file__).parent.parent))

from controllers import CentralizedMPC
from simulation import SwarmTransportEnv


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
            use_viewer: Whether to show visualization (currently unused in env loop)
        """
        self.num_robots = num_robots
        self.push_distance = push_distance
        self.use_viewer = use_viewer
        self.controller_config = controller_config or {}

        # Create environment (auto-generates scene if needed)
        self.env = SwarmTransportEnv(
            n_robots=num_robots,
            goal_pos=[push_distance, 0.0, 0.0],
            push_distance=push_distance,
        )

        # Initialize controller
        self.controller = CentralizedMPC(num_robots, controller_config)

        print(f"Experiment initialized: {num_robots} robots, {push_distance}m push")

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

        obs = self.env.reset()
        self.controller.reset()

        payload_trajectory = []
        solve_times = []
        contact_counts = []

        start_wall_time = time.time()
        converged = False
        convergence_time = None

        while self.env.time < max_time:
            controls = self.controller.compute_control(
                obs['payload'],
                obs['robots'],
                self.env.goal_pos,
                self.env.dt,
                obs['forces'],
            )
            obs = self.env.step(controls)

            payload_trajectory.append(obs['payload'][:3].copy())
            solve_times.append(self.controller.get_solve_time())
            contact_counts.append(self.env.data.ncon)

            distance_to_goal = np.linalg.norm(obs['payload'][:2] - self.env.goal_pos[:2])
            if not converged and distance_to_goal < success_threshold:
                converged = True
                convergence_time = self.env.time

            if converged and self.env.time > convergence_time + 5.0:
                break

        wall_time = time.time() - start_wall_time
        final_distance = np.linalg.norm(obs['payload'][:2] - self.env.goal_pos[:2])

        results = {
            'n_agents': self.num_robots,
            'converged': converged,
            'convergence_time': convergence_time if converged else None,
            'final_distance_to_goal': float(final_distance),
            'simulation_time': float(self.env.time),
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
        use_viewer: Show visualization (currently unused)
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

    if output_dir:
        output_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"mpc_scaling_{timestamp}.json"
        filepath = output_dir / filename

        with open(filepath, 'w') as f:
            json.dump(results, f, indent=2)

        latest_path = output_dir / "latest.json"
        with open(latest_path, 'w') as f:
            json.dump(results, f, indent=2)

        print(f"\n{'='*60}")
        print(f"Results saved: {filepath}")
        print(f"{'='*60}")

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

    n_values = [int(x.strip()) for x in args.n_values.split(',')]

    controller_config = {
        'horizon': args.horizon,
        'dt': args.dt,
        'solver': args.solver,
    }

    if args.output:
        output_dir = Path(args.output)
    else:
        output_dir = Path(__file__).parent.parent / "analysis" / "logs"

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
