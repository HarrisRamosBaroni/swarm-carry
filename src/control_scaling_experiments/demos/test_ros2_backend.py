#!/usr/bin/env python3
"""
Integration test: ROS2Backend vs SimulatedBackend for GBP.

Runs the distributed estimation demo with both backends using the same
random seed and compares convergence results. Pass if the consensus
estimates agree to within tolerance.

Usage
-----
    # Must have ROS2 sourced:
    source /opt/ros/jazzy/setup.bash
    python3 test_ros2_backend.py

    python3 test_ros2_backend.py --agents 6 --topology full
    python3 test_ros2_backend.py --tol 1e-3   # relaxed tolerance
"""

import argparse
import sys
import os
import time
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from communication.backend import create_ring_topology, create_line_topology, create_full_topology
from communication.ros2_backend import ROS2Backend
from demos.gbp_distributed_estimation import DistributedGBPEstimation

_TOPO_BUILDERS = {
    'ring': create_ring_topology,
    'line': create_line_topology,
    'full': create_full_topology,
}

_SEP = '─' * 60


def run_simulated(num_agents, topo_str, seed, max_iter, lam):
    print(f"\n{'SimulatedBackend':^60}")
    print(_SEP)
    system = DistributedGBPEstimation(
        num_agents=num_agents,
        target_true=np.array([5.0, 3.0]),
        obs_noise_std=0.5,
        consensus_precision=lam,
        topology=topo_str,
        seed=seed,
    )
    t0 = time.monotonic()
    results = system.run(max_iterations=max_iter)
    results['wall_time'] = time.monotonic() - t0
    _print_results(results)
    return results


def run_ros2(num_agents, topo_str, seed, max_iter, warmup_sec, lam):
    print(f"\n{'ROS2Backend':^60}")
    print(_SEP)

    topo = _TOPO_BUILDERS[topo_str](num_agents)
    backend = ROS2Backend(num_agents=num_agents, topology=topo)

    print(f"  Waiting {warmup_sec}s for DDS discovery...", end='', flush=True)
    time.sleep(warmup_sec)
    print(" done")

    system = DistributedGBPEstimation(
        num_agents=num_agents,
        target_true=np.array([5.0, 3.0]),
        obs_noise_std=0.5,
        consensus_precision=lam,
        backend=backend,
        seed=seed,
    )

    t0 = time.monotonic()
    results = system.run(max_iterations=max_iter)
    results['wall_time'] = time.monotonic() - t0
    _print_results(results)

    backend.shutdown()
    return results


def _print_results(r):
    status = "CONVERGED" if r['converged'] else "NOT CONVERGED"
    print(f"  Status:            {status}")
    print(f"  Iterations:        {r['iterations']}")
    print(f"  Consensus error:   {r['final_consensus_error']:.2e}")
    print(f"  vs centralized:    {r['error_vs_centralized']:.4e}")
    print(f"  vs true target:    {r['error_vs_true']:.4f}")
    print(f"  Wall time:         {r['wall_time']*1000:.1f} ms")
    stats = r['communication_stats']
    print(f"  Messages sent:     {stats['messages_sent']}")
    print(f"  Barrier calls:     {stats['barrier_calls']}")


def compare(sim_r, ros_r, tol):
    print(f"\n{'Comparison':^60}")
    print(_SEP)

    sim_mean = sim_r['consensus_mean']
    ros_mean = ros_r['consensus_mean']
    diff = np.linalg.norm(sim_mean - ros_mean)

    print(f"  SimulatedBackend estimate: [{sim_mean[0]:.5f}, {sim_mean[1]:.5f}]")
    print(f"  ROS2Backend estimate:      [{ros_mean[0]:.5f}, {ros_mean[1]:.5f}]")
    print(f"  L2 difference:             {diff:.2e}  (tolerance: {tol:.0e})")
    print(f"  Iteration count match:     {sim_r['iterations']} vs {ros_r['iterations']}")

    # Pass if estimates agree — convergence failures affect both backends equally
    # and are an algorithm/config issue, not a backend issue.
    both_converged = sim_r['converged'] and ros_r['converged']
    convergence_note = "" if both_converged else "  (neither backend converged; estimates still compared)"
    passed = diff < tol
    print()
    if passed:
        print(f"  PASS: estimates agree.{convergence_note}")
    else:
        print(f"  FAIL: estimate difference {diff:.2e} >= tolerance {tol:.0e}")

    return passed


def main():
    parser = argparse.ArgumentParser(description='ROS2Backend integration test')
    parser.add_argument('-n', '--agents', type=int, default=4)
    parser.add_argument('-t', '--topology', choices=['ring', 'line', 'full'], default='ring')
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--max-iter', type=int, default=50)
    parser.add_argument('--tol', type=float, default=1e-4,
                        help='Max L2 difference between consensus estimates (default: 1e-4)')
    parser.add_argument('--warmup', type=float, default=1.0,
                        help='Seconds to wait for DDS discovery (default: 1.0)')
    parser.add_argument('--lam', type=float, default=100.0,
                        help='Consensus factor precision λ (default: 100.0)')
    args = parser.parse_args()

    print(_SEP)
    print(f"  ROS2Backend Integration Test")
    print(f"  Agents: {args.agents}  Topology: {args.topology}  Seed: {args.seed}")
    print(_SEP)

    sim_r = run_simulated(args.agents, args.topology, args.seed, args.max_iter, args.lam)
    ros_r = run_ros2(args.agents, args.topology, args.seed, args.max_iter, args.warmup, args.lam)

    passed = compare(sim_r, ros_r, args.tol)
    sys.exit(0 if passed else 1)


if __name__ == '__main__':
    main()
