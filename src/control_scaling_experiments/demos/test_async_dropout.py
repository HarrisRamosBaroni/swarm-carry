#!/usr/bin/env python3
"""
Dropout robustness experiment for GBP with AsyncSimulatedBackend.

Reproduces the DR.CAP-style dropout sweep: runs GBP at increasing dropout
rates and measures how convergence and estimation accuracy degrade.

Failure definition (matching DR.CAP):
    A run "fails" if its final error_vs_true exceeds the 90th percentile
    of errors observed at 0% dropout. This is a task-relative threshold,
    not an absolute one.

Usage
-----
    python3 test_async_dropout.py
    python3 test_async_dropout.py -n 8 -t line --lam 1000 --trials 50
    python3 test_async_dropout.py --no-plot   # print table only
"""

import argparse
import sys
import os
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from communication.backend import AsyncSimulatedBackend, create_line_topology, create_ring_topology, create_full_topology
from demos.gbp_distributed_estimation import DistributedGBPEstimation

_TOPO_BUILDERS = {
    'ring': create_ring_topology,
    'line': create_line_topology,
    'full': create_full_topology,
}

_DROPOUT_RATES = [0.0, 0.3, 0.5, 0.7, 0.9]


def run_trials(num_agents, topo_str, lam, max_iter, dropout_rate, num_trials, base_seed):
    """Run num_trials GBP experiments at a given dropout rate."""
    topo = _TOPO_BUILDERS[topo_str](num_agents)
    consensus_errors = []
    iters = []
    converged_count = 0

    for trial in range(num_trials):
        backend = AsyncSimulatedBackend(
            num_agents=num_agents,
            topology=topo,
            dropout_rate=dropout_rate,
            mean_delay_steps=0,
            seed=base_seed + trial * 100,   # separate seed per trial
        )
        system = DistributedGBPEstimation(
            num_agents=num_agents,
            target_true=np.array([5.0, 3.0]),
            obs_noise_std=0.5,
            consensus_precision=lam,
            backend=backend,
            seed=base_seed + trial,         # separate observation seed per trial
        )
        result = system.run(max_iterations=max_iter)
        consensus_errors.append(result['final_consensus_error'])
        iters.append(result['iterations'])
        if result['converged']:
            converged_count += 1

    return {
        'dropout_rate': dropout_rate,
        'consensus_errors': np.array(consensus_errors),
        'iters': np.array(iters),
        'convergence_rate': converged_count / num_trials,
        'mean_iter': np.mean(iters),
        'std_iter': np.std(iters),
        'mean_consensus_error': np.mean(consensus_errors),
    }


def compute_failure_rate(results_by_rate, threshold):
    """Add failure_rate field to each result dict (in-place)."""
    for r in results_by_rate:
        r['failure_rate'] = float(np.mean(r['consensus_errors'] > threshold))


def print_table(results_by_rate, threshold):
    header = (
        f"{'Dropout':>8}  {'Trials':>6}  {'Conv%':>6}  "
        f"{'Iters(mean±std)':>18}  {'Consensus err':>14}  {'Fail%':>6}"
    )
    sep = '─' * len(header)
    print(sep)
    print(header)
    print(sep)
    for r in results_by_rate:
        print(
            f"{r['dropout_rate']:>7.0%}  "
            f"{len(r['consensus_errors']):>6}  "
            f"{r['convergence_rate']:>5.0%}  "
            f"  {r['mean_iter']:>6.1f} ± {r['std_iter']:<6.1f}  "
            f"{r['mean_consensus_error']:>14.2e}  "
            f"{r['failure_rate']:>5.0%}"
        )
    print(sep)
    print(f"  Failure threshold (90th pct consensus err @ 0% dropout): {threshold:.2e}")


def plot_results(results_by_rate, threshold, save_path=None):
    import matplotlib.pyplot as plt

    rates = [r['dropout_rate'] for r in results_by_rate]
    mean_iters = [r['mean_iter'] for r in results_by_rate]
    std_iters = [r['std_iter'] for r in results_by_rate]
    fail_rates = [r['failure_rate'] * 100 for r in results_by_rate]
    conv_rates = [(1 - r['failure_rate']) * 100 for r in results_by_rate]

    fig, axes = plt.subplots(1, 2, figsize=(11, 4))

    # Plot 1: iterations vs dropout
    ax = axes[0]
    ax.errorbar(
        [r * 100 for r in rates], mean_iters, yerr=std_iters,
        fmt='o-', capsize=4, linewidth=2, color='steelblue', label='Mean ± std'
    )
    ax.set_xlabel('Dropout rate (%)')
    ax.set_ylabel('Iterations to converge (or max)')
    ax.set_title('GBP Convergence Speed vs Dropout')
    ax.grid(True, alpha=0.3)
    ax.legend()

    # Plot 2: failure rate vs dropout
    ax = axes[1]
    ax.bar([r * 100 for r in rates], fail_rates, width=7, color='salmon', edgecolor='black', label='Failure rate')
    ax.plot([r * 100 for r in rates], fail_rates, 'o-', color='darkred', linewidth=2)
    ax.set_xlabel('Dropout rate (%)')
    ax.set_ylabel('Failure rate (%)')
    ax.set_title(f'Failure Rate vs Dropout\n(threshold = 90th pct @ 0%: {threshold:.3f})')
    ax.set_ylim(0, 105)
    ax.grid(True, alpha=0.3, axis='y')
    ax.legend()

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Figure saved to: {save_path}")
    plt.show()


def main():
    parser = argparse.ArgumentParser(description='GBP dropout robustness experiment')
    parser.add_argument('-n', '--agents', type=int, default=6)
    parser.add_argument('-t', '--topology', choices=['ring', 'line', 'full'], default='line')
    parser.add_argument('--lam', type=float, default=100000.0,
                        help='Consensus precision λ (default: 100000)')
    parser.add_argument('--max-iter', type=int, default=200)
    parser.add_argument('--trials', type=int, default=30,
                        help='Trials per dropout rate (default: 30)')
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--no-plot', action='store_true')
    parser.add_argument('--save', type=str, default=None)
    args = parser.parse_args()

    print(f"\nGBP Dropout Robustness Experiment")
    print(f"  Agents: {args.agents}  Topology: {args.topology}  λ: {args.lam}")
    print(f"  Max iter: {args.max_iter}  Trials/rate: {args.trials}  Seed: {args.seed}\n")

    results_by_rate = []
    for rate in _DROPOUT_RATES:
        print(f"  Running dropout={rate:.0%}...", end='', flush=True)
        r = run_trials(
            num_agents=args.agents,
            topo_str=args.topology,
            lam=args.lam,
            max_iter=args.max_iter,
            dropout_rate=rate,
            num_trials=args.trials,
            base_seed=args.seed,
        )
        results_by_rate.append(r)
        print(f" done  (mean iters: {r['mean_iter']:.1f}, conv: {r['convergence_rate']:.0%})")

    # Failure threshold: 90th percentile of 0% dropout CONSENSUS errors
    # (consensus error measures algorithm quality independent of observation noise)
    baseline = results_by_rate[0]
    threshold = float(np.percentile(baseline['consensus_errors'], 90))

    compute_failure_rate(results_by_rate, threshold)

    print()
    print_table(results_by_rate, threshold)

    if not args.no_plot:
        save_path = args.save
        if save_path is None:
            fig_dir = os.path.join(
                os.path.dirname(os.path.dirname(__file__)), 'analysis', 'figures'
            )
            os.makedirs(fig_dir, exist_ok=True)
            save_path = os.path.join(fig_dir, 'async_dropout.pdf')
        plot_results(results_by_rate, threshold, save_path)


if __name__ == '__main__':
    main()
