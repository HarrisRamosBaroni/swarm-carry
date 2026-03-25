#!/usr/bin/env python3
"""
Plot MR.CAP FG experiment results from results.json.

Produces three figures saved to figures/:
  1. trajectory.png  — payload XY path per n_robots vs straight-line reference
  2. solve_times.png — FG solve time per step for each n_robots
  3. summary.png     — bar charts: mean solve time and final error vs n_robots

Usage
-----
  python plot_results.py                    # reads results.json in same directory
  python plot_results.py --results my.json
"""

import argparse
import json
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

COLORS = ["#2196F3", "#F44336", "#4CAF50", "#FF9800", "#9C27B0"]


def load(path: Path) -> dict:
    with open(path) as f:
        return json.load(f)


def plot_trajectories(results: list, out: Path):
    fig, ax = plt.subplots(figsize=(7, 5))

    for i, r in enumerate(results):
        traj = np.array(r["trajectory"])
        goal = np.array(r["goal"])
        n    = r["n_robots"]
        c    = COLORS[i % len(COLORS)]

        # Reference: straight line from start to goal
        start = traj[0, :2]
        ref   = np.array([start, goal[:2]])
        ax.plot(ref[:, 0], ref[:, 1], "--", color=c, alpha=0.4, linewidth=1.2)

        ax.plot(traj[:, 0], traj[:, 1], color=c, linewidth=1.8,
                label=f"n={n}  (err={r['final_error_m']:.2f} m)")
        ax.plot(*traj[0, :2],  "o", color=c, markersize=6)
        ax.plot(*traj[-1, :2], "s", color=c, markersize=6)

    ax.plot(*np.array(results[0]["goal"])[:2], "k*", markersize=12, label="goal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Payload XY trajectory\n(dashed = straight-line reference)")
    ax.legend(fontsize=9)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f"  saved {out}")


def plot_solve_times(results: list, out: Path):
    fig, ax = plt.subplots(figsize=(8, 4))

    for i, r in enumerate(results):
        st   = np.array(r["solve_times_ms"])
        n    = r["n_robots"]
        c    = COLORS[i % len(COLORS)]
        ax.plot(st, color=c, alpha=0.8, linewidth=0.9, label=f"n={n}")
        ax.axhline(np.mean(st), color=c, linestyle="--", linewidth=1.0, alpha=0.6)

    ax.set_xlabel("Control step")
    ax.set_ylabel("FG solve time (ms)")
    ax.set_title("Factor-graph solve time per step\n(dashed = mean per run)")
    ax.legend(fontsize=9)
    ax.yaxis.set_minor_locator(ticker.AutoMinorLocator())
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f"  saved {out}")


def plot_summary(results: list, out: Path):
    ns          = [r["n_robots"]           for r in results]
    solve_means = [r["solve_time_mean_ms"] for r in results]
    solve_stds  = [r["solve_time_std_ms"]  for r in results]
    errors      = [r["final_error_m"]      for r in results]
    deviations  = [r["mean_deviation_m"]   for r in results]

    fig, axes = plt.subplots(1, 3, figsize=(12, 4))

    # --- solve time ---
    ax = axes[0]
    bars = ax.bar(ns, solve_means, yerr=solve_stds, capsize=4,
                  color=COLORS[:len(ns)], alpha=0.85, width=0.5)
    ax.set_xlabel("Number of robots")
    ax.set_ylabel("Mean FG solve time (ms)")
    ax.set_title("Solve time vs n\n(O(1) expected)")
    ax.set_xticks(ns)
    ax.grid(axis="y", alpha=0.3)
    for bar, v in zip(bars, solve_means):
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.05,
                f"{v:.2f}", ha="center", va="bottom", fontsize=8)

    # --- final position error ---
    ax = axes[1]
    bars = ax.bar(ns, errors, color=COLORS[:len(ns)], alpha=0.85, width=0.5)
    ax.set_xlabel("Number of robots")
    ax.set_ylabel("Final position error (m)")
    ax.set_title("Final error vs n")
    ax.set_xticks(ns)
    ax.grid(axis="y", alpha=0.3)
    for bar, v in zip(bars, errors):
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.002,
                f"{v:.3f}", ha="center", va="bottom", fontsize=8)

    # --- mean trajectory deviation ---
    ax = axes[2]
    bars = ax.bar(ns, deviations, color=COLORS[:len(ns)], alpha=0.85, width=0.5)
    ax.set_xlabel("Number of robots")
    ax.set_ylabel("Mean deviation from ref (m)")
    ax.set_title("Trajectory deviation vs n")
    ax.set_xticks(ns)
    ax.grid(axis="y", alpha=0.3)
    for bar, v in zip(bars, deviations):
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.001,
                f"{v:.3f}", ha="center", va="bottom", fontsize=8)

    fig.suptitle("MR.CAP FG Controller — Scaling Summary", fontsize=12, y=1.02)
    fig.tight_layout()
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  saved {out}")


def main():
    parser = argparse.ArgumentParser(description="Plot MR.CAP FG experiment results")
    parser.add_argument("--results", default=None,
                        help="Path to results.json (default: same dir as this script)")
    args = parser.parse_args()

    script_dir   = Path(__file__).parent
    results_path = Path(args.results) if args.results else script_dir / "results.json"

    if not results_path.exists():
        print(f"results.json not found at {results_path}")
        print("Run run_experiment.py first to generate it.")
        return

    data    = load(results_path)
    results = data["results"]

    figures_dir = script_dir / "figures"
    figures_dir.mkdir(exist_ok=True)

    print(f"Plotting {len(results)} runs from {results_path} ...")
    plot_trajectories(results, figures_dir / "trajectory.png")
    plot_solve_times(results,  figures_dir / "solve_times.png")
    plot_summary(results,      figures_dir / "summary.png")
    print("Done.")


if __name__ == "__main__":
    main()
