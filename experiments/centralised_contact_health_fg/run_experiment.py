#!/usr/bin/env python3
"""
Contact-health factor-graph controller — ablation experiment.

Mirrors experiments/mrcap_fg/run_experiment.py. Adds:
  - ablation switch (--ablation baseline|weighted|fg|full)
  - per-step logging of wall_forces, base_forces, and contact-health
    diagnostics (weights, F_bar, sigma_u_eff, recovery correction).

Ablation conditions (see PROBLEM_STATEMENT.md):
  baseline : MRCapController, equal-weight centroid estimator.
  weighted : ContactHealthController, weighted anchor only.
  fg       : ContactHealthController, weighted anchor + σ_u modulation.
  full     : ContactHealthController, all three changes active.

Usage
-----
  python run_experiment.py                                  # full, n=3, dist=5m
  python run_experiment.py --ablation baseline --n-values 3
  python run_experiment.py --ablation full --n-values 3,4 --vis
  python run_experiment.py --ablation full --slip-robot 0 --slip-time 5.0
"""

import argparse
import json
import time
from datetime import datetime
from pathlib import Path

import numpy as np

from swarmlib.simulation.mecanum_env import MecanumTransportEnv
from swarmlib.simulation.generate_mecanum_scene import face_contact_formation
from swarmlib.controllers import MRCapController, ContactHealthController


# ---------------------------------------------------------------------------
# Payload geometry — shared between formation and env
# ---------------------------------------------------------------------------
PAYLOAD_HX = 0.450
PAYLOAD_HY = 0.450
PAYLOAD_HZ = 0.12


# ---------------------------------------------------------------------------
ABLATION_FLAGS = {
    # ablation_name → (use_weighted, use_modulated_sigma_u, use_recovery)
    "baseline": None,                       # special: use MRCapController
    "weighted": (True,  False, False),
    "fg":       (True,  True,  False),
    "full":     (True,  True,  True),
}


def make_controller(ablation: str, n_robots: int, formation, horizon: int,
                    v_max: float, payload_mass_nom: float,
                    F_wall_star: float, alpha: float, beta: float):
    if ablation == "baseline":
        return MRCapController(
            num_robots=n_robots,
            formation=formation,
            config={
                "horizon": horizon, "v_max": v_max,
                "sigma_x": 0.5, "sigma_u": 0.3, "sigma_anchor": 0.01,
                "estimate_centroid": False,  # MR.CAP uses ground-truth payload pose
            },
        )

    flags = ABLATION_FLAGS[ablation]
    return ContactHealthController(
        num_robots=n_robots,
        formation=formation,
        config={
            "horizon":  horizon, "v_max": v_max,
            "sigma_x":  0.5, "sigma_u": 0.3, "sigma_anchor": 0.01,
            "F_wall_star":      F_wall_star,
            "payload_mass_nom": payload_mass_nom,
            "alpha_sigma_u":    alpha,
            "beta_recovery":    beta,
            "use_weighted_anchor":    flags[0],
            "use_modulated_sigma_u":  flags[1],
            "use_recovery_term":      flags[2],
        },
    )


# ---------------------------------------------------------------------------
# Single-run experiment
# ---------------------------------------------------------------------------

def run_single(
    n_robots: int,
    goal: tuple,
    max_time: float,
    success_threshold: float,
    horizon: int,
    v_max: float,
    payload_mass: float,
    ablation: str,
    F_wall_star: float,
    alpha: float,
    beta: float,
    slip_robot: int,
    slip_time: float,
    slip_duration: float,
    slip_friction_scale: float,
    visualise: bool = False,
    sim_speed: float = 1.0,
) -> dict:
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

    controller = make_controller(
        ablation=ablation, n_robots=n_robots, formation=formation,
        horizon=horizon, v_max=v_max,
        payload_mass_nom=payload_mass,
        F_wall_star=F_wall_star, alpha=alpha, beta=beta,
    )

    obs = env.reset()
    controller.reset()

    # ---- Cache nominal wheel friction for slip injection ----
    # Sim-injected slip: scale geom friction of one robot's wheel geoms
    # between t = slip_time and t = slip_time + slip_duration.
    nominal_friction = None
    wheel_geom_ids = []
    if slip_robot >= 0:
        for gid in range(env.model.ngeom):
            name = env.model.geom(gid).name or ""
            if name.startswith(f"robot_{slip_robot}_wheel"):
                wheel_geom_ids.append(gid)
        if wheel_geom_ids:
            nominal_friction = env.model.geom_friction[wheel_geom_ids].copy()

    # ---- Viewer ----
    viewer = None
    if visualise:
        import mujoco.viewer as mjv
        viewer = mjv.launch_passive(env.model, env.data)
        input("  Viewer open — adjust camera, then press Enter to start...")

    goal_arr = np.array(goal)

    # Per-step logs (lists of lists for JSON serialisability)
    payload_trajectory = []
    centroid_estimate_log = []
    solve_times = []
    wall_forces_log = []
    base_forces_log = []
    weights_log = []
    F_bar_log = []
    sigma_u_log = []
    v_recovery_log = []
    torque_log = []

    TORQUE_LIMIT = 10.0
    wheel_act_ids = env._wheel_act_ids

    wall_start = time.perf_counter()
    success = False
    slip_active = False

    while env.time < max_time:
        if viewer is not None and not viewer.is_running():
            break

        # ---- Slip injection ----
        if (slip_robot >= 0 and nominal_friction is not None
                and not slip_active and env.time >= slip_time):
            scaled = nominal_friction.copy()
            scaled[:, 0] *= slip_friction_scale   # slide friction
            env.model.geom_friction[wheel_geom_ids] = scaled
            slip_active = True
        if (slip_active and nominal_friction is not None
                and env.time >= slip_time + slip_duration):
            env.model.geom_friction[wheel_geom_ids] = nominal_friction
            slip_active = False

        payload     = obs["payload"]
        robots      = obs["robots"]
        wall_forces = obs.get("wall_forces")
        base_forces = obs.get("base_forces")

        payload_trajectory.append(payload[:3].tolist())
        wall_forces_log.append(
            None if wall_forces is None else np.asarray(wall_forces).tolist())
        base_forces_log.append(
            None if base_forces is None else np.asarray(base_forces).tolist())

        # Controller call. MR.CAP takes `forces` (treated as wall_forces);
        # ContactHealthController takes both kwargs.
        if isinstance(controller, ContactHealthController):
            controls = controller.compute_control(
                payload_state=payload, robot_states=robots,
                goal_state=goal_arr, dt=0.05,
                wall_forces=wall_forces, base_forces=base_forces,
            )
            diag = controller.get_diagnostics()
            weights_log.append(None if diag["weights"] is None
                               else diag["weights"].tolist())
            F_bar_log.append(diag["F_bar"])
            sigma_u_log.append(diag["sigma_u_eff"])
            v_recovery_log.append(None if diag["v_recovery"] is None
                                  else diag["v_recovery"].tolist())
            centroid_estimate_log.append(None if diag["centroid"] is None
                                         else diag["centroid"].tolist())
        else:
            controls = controller.compute_control(
                payload_state=payload, robot_states=robots,
                goal_state=goal_arr, dt=0.05, forces=wall_forces,
            )
            weights_log.append(None)
            F_bar_log.append(None if wall_forces is None
                             else float(np.mean(wall_forces)))
            sigma_u_log.append(None)
            v_recovery_log.append(None)
            centroid_estimate_log.append(None)

        solve_times.append(controller.get_solve_time())

        obs = env.step(controls)

        step_torques = np.array([[env.data.ctrl[aid] for aid in ids]
                                 for ids in wheel_act_ids])
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

    # Trajectory deviation from straight line start → goal
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

    torques = np.array(torque_log)
    saturated = np.abs(torques) >= TORQUE_LIMIT
    sat_frac = float(saturated.mean()) if torques.size > 0 else 0.0
    peak_torque = float(np.abs(torques).max()) if torques.size > 0 else 0.0

    # Squeeze drift summary (H1)
    wf_array = np.array([wf for wf in wall_forces_log if wf is not None])
    if wf_array.size > 0:
        F_bar_series = wf_array.mean(axis=1)
        F_bar_excess_max = float(np.max(F_bar_series - F_wall_star))
        F_bar_std        = float(np.std(F_bar_series))
    else:
        F_bar_excess_max = None
        F_bar_std        = None

    env.close()

    return {
        "n_robots":           n_robots,
        "ablation":           ablation,
        "success":            success,
        "sim_time":           float(env.time),
        "wall_time_s":        wall_elapsed,
        "final_error_m":      final_error,
        "mean_deviation_m":   deviation,
        "solve_time_mean_ms": float(np.mean(solve_times) * 1e3),
        "solve_time_std_ms":  float(np.std(solve_times)  * 1e3),
        "solve_time_max_ms":  float(np.max(solve_times)  * 1e3),
        "n_steps":            len(solve_times),
        "sat_frac":           sat_frac,
        "peak_torque_Nm":     peak_torque,
        "F_wall_star":        F_wall_star,
        "F_bar_excess_max_N": F_bar_excess_max,
        "F_bar_std_N":        F_bar_std,
        "payload_mass_kg":    payload_mass,
        "goal":               goal_arr.tolist(),
        "slip_robot":         slip_robot,
        "slip_time":          slip_time,
        "slip_duration":      slip_duration,
        "slip_friction_scale": slip_friction_scale,
        # Time series
        "trajectory":           traj.tolist(),
        "centroid_estimate":    centroid_estimate_log,
        "solve_times_ms":       (np.array(solve_times) * 1e3).tolist(),
        "wall_forces":          wall_forces_log,
        "base_forces":          base_forces_log,
        "weights":              weights_log,
        "F_bar":                F_bar_log,
        "sigma_u_eff":          sigma_u_log,
        "v_recovery":           v_recovery_log,
        "torques_Nm":           torques.tolist(),
    }


# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="Contact-health FG ablation experiment")
    parser.add_argument("--ablation", default="full",
                        choices=list(ABLATION_FLAGS.keys()),
                        help="Ablation condition (default: full)")
    parser.add_argument("--n-values", default="3",
                        help="Comma-separated robot counts (default: 3)")
    parser.add_argument("--distance", type=float, default=5.0,
                        help="Transport distance along +x m (default: 5.0). "
                             "Ignored if --goal is given.")
    parser.add_argument("--goal", default=None,
                        help="Goal pose as 'x,y,theta' (m,m,rad). "
                             "Overrides --distance.")
    parser.add_argument("--max-time", type=float, default=60.0,
                        help="Max sim time per run s (default: 60)")
    parser.add_argument("--horizon",  type=int,   default=15)
    parser.add_argument("--v-max",    type=float, default=0.25)
    parser.add_argument("--payload-mass", type=float, default=2.0)
    parser.add_argument("--threshold",    type=float, default=0.3)
    parser.add_argument("--F-wall-star",  type=float, default=5.0,
                        help="Target wall-squeeze N (default: 5.0)")
    parser.add_argument("--alpha",        type=float, default=0.1,
                        help="σ_u modulation gain 1/N (default: 0.1)")
    parser.add_argument("--beta",         type=float, default=0.005,
                        help="Recovery gain m/(s·N) (default: 0.005)")
    parser.add_argument("--slip-robot",   type=int,   default=-1,
                        help="Robot index to inject slip on; -1 disables (default: -1)")
    parser.add_argument("--slip-time",    type=float, default=5.0)
    parser.add_argument("--slip-duration", type=float, default=2.0)
    parser.add_argument("--slip-friction-scale", type=float, default=0.1,
                        help="Multiplier on slide friction during slip (default: 0.1)")
    parser.add_argument("--vis", action="store_true",
                        help="Open viewer for first run")
    parser.add_argument("--sim-speed", type=float, default=1.0)
    parser.add_argument("--output", default="results.json",
                        help="Output JSON filename (default: results.json)")
    args = parser.parse_args()

    n_values = [int(x.strip()) for x in args.n_values.split(",")]

    if args.goal is not None:
        parts = [float(x.strip()) for x in args.goal.split(",")]
        if len(parts) != 3:
            raise SystemExit("--goal must be 'x,y,theta' (3 floats)")
        goal_tuple = tuple(parts)
    else:
        goal_tuple = (args.distance, 0.0, 0.0)

    print(f"\n{'='*60}")
    print(f"Contact-Health FG Ablation Experiment")
    print(f"  ablation:     {args.ablation}")
    print(f"  robots:       {n_values}")
    print(f"  goal (x,y,θ): ({goal_tuple[0]:.3f}, {goal_tuple[1]:.3f}, {goal_tuple[2]:.3f})")
    print(f"  F_wall*:      {args.F_wall_star} N")
    print(f"  α, β:         {args.alpha}, {args.beta}")
    print(f"  slip_robot:   {args.slip_robot}"
          + ("" if args.slip_robot < 0
             else f" (t={args.slip_time}s, dur={args.slip_duration}s, "
                  f"scale={args.slip_friction_scale})"))
    print(f"{'='*60}\n")

    all_results = []
    for idx, n in enumerate(n_values):
        print(f"Running n={n} ablation={args.ablation} ...", flush=True)
        result = run_single(
            n_robots=n,
            goal=goal_tuple,
            max_time=args.max_time,
            success_threshold=args.threshold,
            horizon=args.horizon,
            v_max=args.v_max,
            payload_mass=args.payload_mass,
            ablation=args.ablation,
            F_wall_star=args.F_wall_star,
            alpha=args.alpha,
            beta=args.beta,
            slip_robot=args.slip_robot,
            slip_time=args.slip_time,
            slip_duration=args.slip_duration,
            slip_friction_scale=args.slip_friction_scale,
            visualise=args.vis and idx == 0,
            sim_speed=args.sim_speed,
        )
        all_results.append(result)

        status = "SUCCESS" if result["success"] else "TIMEOUT"
        excess = (f"{result['F_bar_excess_max_N']:.2f}"
                  if result['F_bar_excess_max_N'] is not None else "n/a")
        print(
            f"  [{status}]  final_err={result['final_error_m']:.3f} m"
            f"  dev={result['mean_deviation_m']:.3f} m"
            f"  solve={result['solve_time_mean_ms']:.1f} ms"
            f"  sat={result['sat_frac']*100:.1f}%"
            f"  F̄-F*max={excess} N"
            f"  steps={result['n_steps']}"
        )

    print(f"\n{'='*60}")

    out = {
        "timestamp": datetime.now().isoformat(),
        "args":      vars(args),
        "results":   all_results,
    }
    output_path = Path(__file__).parent / args.output
    with open(output_path, "w") as f:
        json.dump(out, f, indent=2)
    print(f"Results saved to {output_path}")


if __name__ == "__main__":
    main()
