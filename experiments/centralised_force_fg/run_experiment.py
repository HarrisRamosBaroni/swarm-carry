#!/usr/bin/env python3
"""
MR.CAP factor-graph controller — scaling experiment.

Runs MRCapController in MecanumTransportEnv for n_robots in [2, 3, 4],
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
from swarmlib.simulation.generate_mecanum_scene import face_contact_formation
from swarmlib.controllers import ForceCentralisedController


# ---------------------------------------------------------------------------
# Payload geometry (box half-sizes) — shared between formation and env
# ---------------------------------------------------------------------------
PAYLOAD_HX = 0.450   # m
PAYLOAD_HY = 0.450   # m
PAYLOAD_HZ = 0.12   # m


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
    payload_mass: float,
    visualise: bool = False,
    sim_speed: float = 1.0,
) -> dict:
    goal = (distance, 0.0, 0.0)
    payload_size = (PAYLOAD_HX, PAYLOAD_HY, PAYLOAD_HZ)
    formation = face_contact_formation(n_robots,
                                       payload_hx=PAYLOAD_HX,
                                       payload_hy=PAYLOAD_HY)

    env = MecanumTransportEnv(
        n_robots=n_robots,
        formation=formation,
        goal=goal,
        payload_pos=(0.0, 0.0),
        payload_mass=payload_mass,
        payload_size=payload_size,
        vel_feedback=True,
        with_carriage=True,
        dt_control=0.05,
    )

    # Formation offsets passed to controller must match env formation
    controller = ForceCentralisedController(
        num_robots=n_robots,
        formation=formation,
        config={"horizon": horizon, "v_max": v_max, "sigma_x": 0.5,
                "sigma_u": 0.3, "sigma_anchor": 0.01},
    )

    mass_estimate = None

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
    torque_log = []        # per step: (n_robots, 4) wheel torques
    TORQUE_LIMIT = 10.0    # Nm — matches ctrlrange in actuator XML

    # Flat list of wheel actuator ids in robot order: shape (n_robots, 4)
    wheel_act_ids = env._wheel_act_ids   # list of length n_robots, each (4,) array

    wall_start = time.perf_counter()
    success = False

    while env.time < max_time:
        if viewer is not None and not viewer.is_running():
            break
        payload = obs["payload"]
        robots  = obs["robots"]
        forces  = obs.get("wall_forces")

        payload_trajectory.append(payload[:3].tolist())

        controls, mass_estimate = controller.compute_control(
            payload_state=payload,
            robot_states=robots,
            goal_state=goal_arr,
            dt=0.05,
            forces=forces,
            mass_estimate=mass_estimate
        )
        solve_times.append(controller.get_solve_time())

        obs = env.step(controls)

        # Record wheel torques from last substep (data.ctrl holds PD output)
        step_torques = np.array([[env.data.ctrl[aid] for aid in ids]
                                 for ids in wheel_act_ids])   # (n_robots, 4)
        torque_log.append(step_torques)

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

    torques = np.array(torque_log)   # (n_steps, n_robots, 4)
    saturated = np.abs(torques) >= TORQUE_LIMIT   # bool mask
    sat_frac = float(saturated.mean())            # fraction of wheel-steps at limit
    peak_torque = float(np.abs(torques).max())

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
        "sat_frac":           sat_frac,           # fraction of (step, robot, wheel) at torque limit
        "peak_torque_Nm":     peak_torque,
        # Full time-series for plotting
        "trajectory":         traj.tolist(),
        "solve_times_ms":     (np.array(solve_times) * 1e3).tolist(),
        "payload_mass_kg":    payload_mass,
        "goal":               goal_arr.tolist(),
        "torques_Nm":         torques.tolist(),   # (n_steps, n_robots, 4)
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
    parser.add_argument("--v-max",        type=float, default=0.25,
                        help="Per-robot speed limit m/s (default: 0.25 — keeps strafing robots below torque saturation)")
    parser.add_argument("--payload-mass", type=float, default=2.0,
                        help="Payload mass in kg (default: 2.0)")
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
    print(f"  robots:       {n_values}")
    print(f"  distance:     {args.distance} m")
    print(f"  horizon:      {args.horizon}")
    print(f"  v_max:        {args.v_max} m/s")
    print(f"  payload_mass: {args.payload_mass} kg")
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
            payload_mass=args.payload_mass,
            visualise=args.vis and idx == 0,
            sim_speed=args.sim_speed,
        )
        all_results.append(result)

        status = "SUCCESS" if result["success"] else "TIMEOUT"
        print(
            f"  [{status}]  payload={result['payload_mass_kg']:.1f} kg"
            f"  final_error={result['final_error_m']:.3f} m"
            f"  deviation={result['mean_deviation_m']:.3f} m"
            f"  solve={result['solve_time_mean_ms']:.1f} ± {result['solve_time_std_ms']:.1f} ms"
            f"  sat={result['sat_frac']*100:.1f}%  peak={result['peak_torque_Nm']:.1f} Nm"
            f"  steps={result['n_steps']}"
        )

    # Summary table
    print(f"\n{'='*75}")
    print(f"{'n':>4}  {'status':>8}  {'final_err(m)':>12}  {'deviation(m)':>12}  {'solve_mean(ms)':>14}  {'sat%':>6}  {'peak(Nm)':>9}")
    print(f"{'-'*4}  {'-'*8}  {'-'*12}  {'-'*12}  {'-'*14}  {'-'*6}  {'-'*9}")
    for r in all_results:
        status = "OK" if r["success"] else "TIMEOUT"
        print(
            f"{r['n_robots']:>4}  {status:>8}  "
            f"{r['final_error_m']:>12.3f}  {r['mean_deviation_m']:>12.3f}  "
            f"{r['solve_time_mean_ms']:>14.2f}  "
            f"{r['sat_frac']*100:>6.1f}  {r['peak_torque_Nm']:>9.2f}"
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
