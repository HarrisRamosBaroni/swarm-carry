#!/usr/bin/env python3
"""
Generate publication-ready plots from MPC scaling experiment logs.

Reads JSON log files and produces figures for:
- Solve time vs n (primary scaling law)
- Success metrics vs n
- Example trajectories
- Contact analysis
"""

import argparse
import json
import numpy as np
from pathlib import Path
from typing import List, Dict, Any
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit


def load_log(log_path: Path) -> Dict[str, Any]:
    """Load experiment log from JSON file."""
    with open(log_path, 'r') as f:
        return json.load(f)


def power_law(x, alpha, beta):
    """Power law function: tau = alpha * n^beta"""
    return alpha * np.power(x, beta)


def plot_solve_time_scaling(
    logs: List[Dict[str, Any]],
    labels: List[str],
    output_path: Path
):
    """
    Plot solve time vs number of robots with power law fit.

    Primary figure for demonstrating computational scaling.
    """
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    for log, label in zip(logs, labels):
        runs = log['runs']
        n_values = [run['n_agents'] for run in runs]
        mean_times = [run['mean_solve_time'] for run in runs]
        std_times = [run['std_solve_time'] for run in runs]

        # Plot on linear scale
        ax1.errorbar(n_values, mean_times, yerr=std_times,
                    marker='o', capsize=5, label=label, linewidth=2, markersize=8)

        # Plot on log-log scale
        ax2.loglog(n_values, mean_times, marker='o', label=label,
                  linewidth=2, markersize=8)

        # Fit power law
        try:
            popt, _ = curve_fit(power_law, n_values, mean_times, p0=[1e-3, 2.0])
            alpha, beta = popt

            # Generate fit line
            n_fit = np.linspace(min(n_values), max(n_values), 100)
            time_fit = power_law(n_fit, alpha, beta)

            ax1.plot(n_fit, time_fit, '--', alpha=0.7,
                    label=f'{label} fit: τ = {alpha:.2e} n^{beta:.2f}')
            ax2.loglog(n_fit, time_fit, '--', alpha=0.7)

            print(f"{label}: α={alpha:.4e}, β={beta:.3f}")
        except Exception as e:
            print(f"Could not fit power law for {label}: {e}")

    # Linear scale plot
    ax1.set_xlabel('Number of Robots (n)', fontsize=12)
    ax1.set_ylabel('Mean Solve Time (s)', fontsize=12)
    ax1.set_title('MPC Solve Time vs Team Size', fontsize=14, fontweight='bold')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # Log-log plot
    ax2.set_xlabel('Number of Robots (n)', fontsize=12)
    ax2.set_ylabel('Mean Solve Time (s)', fontsize=12)
    ax2.set_title('Scaling Law (Log-Log)', fontsize=14, fontweight='bold')
    ax2.legend()
    ax2.grid(True, alpha=0.3, which='both')

    plt.tight_layout()
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"Saved: {output_path}")
    plt.close()


def plot_performance_metrics(
    logs: List[Dict[str, Any]],
    labels: List[str],
    output_path: Path
):
    """
    Plot performance metrics: convergence time and final distance vs n.
    """
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    for log, label in zip(logs, labels):
        runs = log['runs']
        n_values = [run['n_agents'] for run in runs]
        convergence_times = [
            run['convergence_time'] if run['converged'] else run['simulation_time']
            for run in runs
        ]
        final_distances = [run['final_distance_to_goal'] for run in runs]
        converged = [run['converged'] for run in runs]

        # Convergence time
        colors = ['green' if c else 'red' for c in converged]
        ax1.scatter(n_values, convergence_times, c=colors, s=100, alpha=0.7, label=label)
        ax1.plot(n_values, convergence_times, '--', alpha=0.5)

        # Final distance to goal
        ax2.scatter(n_values, final_distances, c=colors, s=100, alpha=0.7, label=label)
        ax2.plot(n_values, final_distances, '--', alpha=0.5)

    ax1.set_xlabel('Number of Robots (n)', fontsize=12)
    ax1.set_ylabel('Time to Goal (s)', fontsize=12)
    ax1.set_title('Convergence Time vs Team Size', fontsize=14, fontweight='bold')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    ax2.set_xlabel('Number of Robots (n)', fontsize=12)
    ax2.set_ylabel('Final Distance to Goal (m)', fontsize=12)
    ax2.set_title('Task Completion Accuracy', fontsize=14, fontweight='bold')
    ax2.axhline(0.5, color='green', linestyle='--', alpha=0.5, label='Success threshold')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"Saved: {output_path}")
    plt.close()


def plot_trajectories(
    log: Dict[str, Any],
    n_values_to_plot: List[int],
    output_path: Path
):
    """
    Plot example payload trajectories for different team sizes.
    """
    fig, ax = plt.subplots(figsize=(10, 6))

    runs = log['runs']
    goal_x = log['metadata']['push_distance']

    for run in runs:
        n = run['n_agents']
        if n not in n_values_to_plot:
            continue

        trajectory = np.array(run['payload_trajectory'])
        x_traj = trajectory[:, 0]
        y_traj = trajectory[:, 1]

        ax.plot(x_traj, y_traj, linewidth=2, label=f'n={n}', alpha=0.8)
        ax.scatter(x_traj[0], y_traj[0], marker='o', s=100, zorder=5)  # Start
        ax.scatter(x_traj[-1], y_traj[-1], marker='s', s=100, zorder=5)  # End

    # Goal region
    ax.axvline(goal_x, color='green', linestyle='--', linewidth=2, alpha=0.5, label='Goal')
    ax.axvspan(goal_x - 0.5, goal_x + 0.5, alpha=0.1, color='green')

    ax.set_xlabel('X Position (m)', fontsize=12)
    ax.set_ylabel('Y Position (m)', fontsize=12)
    ax.set_title('Payload Trajectories', fontsize=14, fontweight='bold')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.axis('equal')

    plt.tight_layout()
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"Saved: {output_path}")
    plt.close()


def plot_contact_analysis(
    log: Dict[str, Any],
    output_path: Path
):
    """
    Plot contact statistics vs number of robots.
    """
    fig, ax = plt.subplots(figsize=(8, 6))

    runs = log['runs']
    n_values = [run['n_agents'] for run in runs]
    mean_contacts = [run['mean_contacts'] for run in runs]
    max_contacts = [run['max_contacts'] for run in runs]

    ax.plot(n_values, mean_contacts, marker='o', linewidth=2, markersize=8,
            label='Mean contacts', color='blue')
    ax.plot(n_values, max_contacts, marker='s', linewidth=2, markersize=8,
            label='Max contacts', color='red')

    ax.set_xlabel('Number of Robots (n)', fontsize=12)
    ax.set_ylabel('Contact Count', fontsize=12)
    ax.set_title('Contact Dynamics vs Team Size', fontsize=14, fontweight='bold')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"Saved: {output_path}")
    plt.close()


def plot_solve_time_distribution(
    log: Dict[str, Any],
    n_value: int,
    output_path: Path
):
    """
    Plot distribution of solve times for a specific n.
    Useful for understanding variance and outliers.
    """
    runs = log['runs']
    run = next((r for r in runs if r['n_agents'] == n_value), None)

    if run is None:
        print(f"Warning: No data for n={n_value}")
        return

    solve_times = np.array(run['solve_times']) * 1000  # Convert to ms

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    # Histogram
    ax1.hist(solve_times, bins=50, edgecolor='black', alpha=0.7, color='steelblue')
    ax1.axvline(np.mean(solve_times), color='red', linestyle='--',
                linewidth=2, label=f'Mean: {np.mean(solve_times):.2f} ms')
    ax1.axvline(np.median(solve_times), color='green', linestyle='--',
                linewidth=2, label=f'Median: {np.median(solve_times):.2f} ms')
    ax1.set_xlabel('Solve Time (ms)', fontsize=12)
    ax1.set_ylabel('Frequency', fontsize=12)
    ax1.set_title(f'Solve Time Distribution (n={n_value})', fontsize=14, fontweight='bold')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # Time series
    ax2.plot(solve_times, linewidth=1, alpha=0.7, color='steelblue')
    ax2.axhline(np.mean(solve_times), color='red', linestyle='--',
                linewidth=2, label=f'Mean: {np.mean(solve_times):.2f} ms')
    ax2.set_xlabel('MPC Iteration', fontsize=12)
    ax2.set_ylabel('Solve Time (ms)', fontsize=12)
    ax2.set_title(f'Solve Time Over Simulation (n={n_value})', fontsize=14, fontweight='bold')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"Saved: {output_path}")
    plt.close()


def generate_summary_report(
    logs: List[Dict[str, Any]],
    labels: List[str],
    output_path: Path
):
    """
    Generate a text summary report with key statistics.
    """
    with open(output_path, 'w') as f:
        f.write("MPC SCALING EXPERIMENT SUMMARY\n")
        f.write("=" * 70 + "\n\n")

        for log, label in zip(logs, labels):
            f.write(f"Experiment: {label}\n")
            f.write("-" * 70 + "\n")

            metadata = log['metadata']
            f.write(f"Date: {metadata['date']}\n")
            f.write(f"Push distance: {metadata['push_distance']} m\n")
            f.write(f"Controller config: {metadata['controller_config']}\n\n")

            f.write(f"{'n':<6} {'Mean (ms)':<12} {'Std (ms)':<12} {'Max (ms)':<12} "
                   f"{'Converged':<12} {'Conv. Time (s)':<15}\n")
            f.write("-" * 70 + "\n")

            for run in log['runs']:
                n = run['n_agents']
                mean = run['mean_solve_time'] * 1000
                std = run['std_solve_time'] * 1000
                max_t = run['max_solve_time'] * 1000
                conv = 'Yes' if run['converged'] else 'No'
                conv_time = f"{run['convergence_time']:.2f}" if run['converged'] else "N/A"

                f.write(f"{n:<6} {mean:<12.2f} {std:<12.2f} {max_t:<12.2f} "
                       f"{conv:<12} {conv_time:<15}\n")

            f.write("\n" + "=" * 70 + "\n\n")

    print(f"Saved: {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description='Generate plots from MPC scaling experiment logs'
    )

    parser.add_argument('--log', type=str, default='latest.json',
                       help='Log file or comma-separated list (default: latest.json)')
    parser.add_argument('--labels', type=str, default=None,
                       help='Comma-separated labels for multiple logs')
    parser.add_argument('--output-dir', type=str, default=None,
                       help='Output directory for figures')
    parser.add_argument('--trajectories', type=str, default='4,16,64',
                       help='n values to plot trajectories for (default: 4,16,64)')
    parser.add_argument('--distribution', type=int, default=None,
                       help='n value to plot solve time distribution for')

    args = parser.parse_args()

    # Paths
    script_dir = Path(__file__).parent
    log_dir = script_dir / "logs"

    if args.output_dir:
        output_dir = Path(args.output_dir)
    else:
        output_dir = script_dir.parent / "figures"
    output_dir.mkdir(parents=True, exist_ok=True)

    # Load logs
    log_files = [log_dir / f.strip() for f in args.log.split(',')]
    logs = [load_log(log_file) for log_file in log_files]

    if args.labels:
        labels = [l.strip() for l in args.labels.split(',')]
    else:
        labels = [log_file.stem for log_file in log_files]

    print(f"Loaded {len(logs)} log file(s)")

    # Generate plots
    print("\nGenerating plots...")

    # 1. Scaling law (primary figure)
    plot_solve_time_scaling(logs, labels, output_dir / 'solve_time_vs_n.pdf')

    # 2. Performance metrics
    plot_performance_metrics(logs, labels, output_dir / 'performance_metrics.pdf')

    # 3. Trajectories (use first log)
    trajectory_n_values = [int(x.strip()) for x in args.trajectories.split(',')]
    plot_trajectories(logs[0], trajectory_n_values, output_dir / 'trajectories.pdf')

    # 4. Contact analysis (use first log)
    plot_contact_analysis(logs[0], output_dir / 'contacts_vs_n.pdf')

    # 5. Solve time distribution (if specified)
    if args.distribution is not None:
        plot_solve_time_distribution(
            logs[0], args.distribution, output_dir / f'solve_time_dist_n{args.distribution}.pdf'
        )

    # 6. Summary report
    generate_summary_report(logs, labels, output_dir / 'summary.txt')

    print(f"\nAll figures saved to: {output_dir}")


if __name__ == '__main__':
    main()
