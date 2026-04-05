#!/usr/bin/env python3
"""
DR.CAP factor-graph controller (centralised mode) — scaling experiment.

Runs DRCapController in MecanumTransportEnv for n_robots in [2, 3, 4], #only 2 right now
measuring FG solve time per step and payload trajectory error.
Results are saved to results.json alongside a printed summary.

Usage
-----
  python run_experiment.py                        # default: n=2,3,4  dist=5m
  python run_experiment.py --n-values 2,4 --distance 3.0
  python run_experiment.py --n-values 3 --max-time 30
  python run_experiment.py --n-values 3 --vis      # watch one run in MuJoCo viewer
"""

import argparse
import json
import time
from datetime import datetime
from pathlib import Path

import numpy as np

from swarmlib.simulation.mecanum_env import MecanumTransportEnv
from swarmlib.simulation.generate_mecanum_scene import mecanum_side_push_formation
from swarmlib.controllers import DRCapController #TODO does that work ?


# ---------------------------------------------------------------------------
# Formation helper: surround the payload on all sides
# ---------------------------------------------------------------------------

def surround_formation(n: int, radius: float = 0.8) -> list:
    """
    Place n robots evenly around the payload at given radius.
    Each robot's yaw points inward toward the payload centre.
    Returns a list of (x_off, y_off, yaw) tuples.
    """
    angles = np.linspace(0, 2 * np.pi, n, endpoint=False)
    return [
        (-radius * np.cos(a), -radius * np.sin(a), a + np.pi)
        for a in angles
    ]


# ---------------------------------------------------------------------------
# Single-run experiment
# ---------------------------------------------------------------------------

def run_single(
    n_robots: int,
    distance: float,
    max_time: float,
    success_threshold: float,
    horizon: int,
    v_max: float,
    visualise: bool = False,
    sim_speed: float = 1.0,
) -> dict:
    goal = (distance, 0.0, 0.0)
    formation = surround_formation(n_robots)

    env = MecanumTransportEnv(
        n_robots=n_robots,
        formation=formation,
        goal=goal,
        payload_pos=(0.0, 0.0),
        payload_mass=10.0,
        with_carriage=True,
        dt_control=0.05,
    )

    # Formation offsets passed to controller must match env formation
    controller = DRCapController(
        num_robots=n_robots,
        formation=formation,
        config={"horizon": horizon, "v_max": v_max, "sigma_x": 0.5,
                "sigma_u": 0.3, "sigma_anchor": 0.01},
    )

    #TODO add developped controller

    obs = env.reset()
    controller.reset()

    # Optional live viewer
    viewer = None
    if visualise:
        import mujoco.viewer as mjv
        viewer = mjv.launch_passive(env.model, env.data)
        input("  Viewer open — adjust camera, then press Enter to start...")

    goal_arr = np.array(goal)
    payload_trajectory = []
    solve_times = []

    wall_start = time.perf_counter()
    success = False

    while env.time < max_time:
        if viewer is not None and not viewer.is_running():
            break
        payload = obs["payload"]
        robots  = obs["robots"]
        forces  = obs.get("wall_forces")

        payload_trajectory.append(payload[:3].tolist())

        controls = controller.compute_control(
            payload_state=payload,
            robot_states=robots,
            goal_state=goal_arr,
            dt=0.05,
            forces=forces,
        )
        solve_times.append(controller.get_solve_time())

        obs = env.step(controls)

        if viewer is not None:
            viewer.sync()
            time.sleep(0.05 / sim_speed)

        dist_to_goal = float(np.linalg.norm(payload[:2] - goal_arr[:2]))
        if dist_to_goal < success_threshold:
            success = True
            break

    wall_elapsed = time.perf_counter() - wall_start
    if viewer is not None:
        viewer.close()
    traj = np.array(payload_trajectory)
    final_error = float(np.linalg.norm(traj[-1, :2] - goal_arr[:2]))

    # Trajectory deviation from straight line start→goal
    start_pos = traj[0, :2]
    end_pos   = goal_arr[:2]
    leg       = end_pos - start_pos
    leg_len   = np.linalg.norm(leg)
    if leg_len > 1e-6:
        t_proj   = ((traj[:, :2] - start_pos) @ leg) / leg_len
        proj_pts = start_pos + np.outer(np.clip(t_proj, 0, leg_len), leg / leg_len)
        deviation = float(np.mean(np.linalg.norm(traj[:, :2] - proj_pts, axis=1)))
    else:
        deviation = 0.0

    env.close()

    return {
        "n_robots":           n_robots,
        "success":            success,
        "sim_time":           float(env.time),
        "wall_time_s":        wall_elapsed,
        "final_error_m":      final_error,
        "mean_deviation_m":   deviation,
        "solve_time_mean_ms": float(np.mean(solve_times) * 1e3),
        "solve_time_std_ms":  float(np.std(solve_times)  * 1e3),
        "solve_time_max_ms":  float(np.max(solve_times)  * 1e3),
        "n_steps":            len(solve_times),
        # Full time-series for plotting
        "trajectory":         traj.tolist(),
        "solve_times_ms":     (np.array(solve_times) * 1e3).tolist(),
        "goal":               goal_arr.tolist(),
    }


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="MR.CAP FG scaling experiment")
    parser.add_argument("--n-values",  default="2,3,4",
                        help="Comma-separated robot counts (default: 2,3,4)")
    parser.add_argument("--distance",  type=float, default=5.0,
                        help="Transport distance in metres (default: 5.0)")
    parser.add_argument("--max-time",  type=float, default=60.0,
                        help="Max simulation time per run in seconds (default: 60)")
    parser.add_argument("--horizon",   type=int,   default=15,
                        help="FG horizon N (default: 15)")
    parser.add_argument("--v-max",     type=float, default=1.0,
                        help="Per-robot speed limit m/s (default: 1.0)")
    parser.add_argument("--threshold", type=float, default=0.3,
                        help="Success distance threshold m (default: 0.3)")
    parser.add_argument("--vis", action="store_true",
                        help="Open MuJoCo viewer for the first run only")
    parser.add_argument("--sim-speed", type=float, default=0.5,
                        help="Playback speed multiplier when --vis is set (default: 0.5 = half real-time)")
    args = parser.parse_args()

    n_values = [int(x.strip()) for x in args.n_values.split(",")]

    print(f"\n{'='*60}")
    print(f"MR.CAP Factor-Graph Scaling Experiment")
    print(f"  robots:   {n_values}")
    print(f"  distance: {args.distance} m")
    print(f"  horizon:  {args.horizon}")
    print(f"  v_max:    {args.v_max} m/s")
    print(f"{'='*60}\n")

    all_results = []

    for idx, n in enumerate(n_values):
        print(f"Running n={n} ...", flush=True)
        result = run_single(
            n_robots=n,
            distance=args.distance,
            max_time=args.max_time,
            success_threshold=args.threshold,
            horizon=args.horizon,
            v_max=args.v_max,
            visualise=args.vis and idx == 0,
            sim_speed=args.sim_speed,
        )
        all_results.append(result)

        status = "SUCCESS" if result["success"] else "TIMEOUT"
        print(
            f"  [{status}]  final_error={result['final_error_m']:.3f} m"
            f"  deviation={result['mean_deviation_m']:.3f} m"
            f"  solve={result['solve_time_mean_ms']:.1f} ± {result['solve_time_std_ms']:.1f} ms"
            f"  steps={result['n_steps']}"
        )

    # Summary table
    print(f"\n{'='*60}")
    print(f"{'n':>4}  {'status':>8}  {'final_err(m)':>12}  {'deviation(m)':>12}  {'solve_mean(ms)':>14}  {'solve_max(ms)':>13}")
    print(f"{'-'*4}  {'-'*8}  {'-'*12}  {'-'*12}  {'-'*14}  {'-'*13}")
    for r in all_results:
        status = "OK" if r["success"] else "TIMEOUT"
        print(
            f"{r['n_robots']:>4}  {status:>8}  "
            f"{r['final_error_m']:>12.3f}  {r['mean_deviation_m']:>12.3f}  "
            f"{r['solve_time_mean_ms']:>14.2f}  {r['solve_time_max_ms']:>13.2f}"
        )
    print(f"{'='*60}\n")

    # Save JSON
    out = {
        "experiment":  "mrcap_fg_scaling",
        "timestamp":   datetime.now().isoformat(),
        "params": {
            "distance_m":  args.distance,
            "max_time_s":  args.max_time,
            "horizon":     args.horizon,
            "v_max":       args.v_max,
            "threshold_m": args.threshold,
        },
        "results": all_results,
    }
    out_path = Path(__file__).parent / "results.json"
    out_path.write_text(json.dumps(out, indent=2))
    print(f"Results saved to {out_path}")


if __name__ == "__main__":
    main()
