"""
Decentralised force-feedback factor-graph-based controller 

Mirrors experiments/mrcap_fg/run_experiment.py but with the decentralised 
force-feedback controller driven by a swappable communication backend.

  python run_experiment.py                          # default: n=2,3,4  sim backend
  python run_experiment.py --n-values 2,4 --distance 3.0
  python run_experiment.py --backend async --dropout 0.3
  python run_experiment.py --n-values 3 --vis       # MuJoCo viewer for first run
"""

import argparse
import json
import time
from datetime import datetime
from pathlib import Path

import numpy as np

from swarmlib.simulation.mecanum_env import MecanumTransportEnv
from swarmlib.simulation.generate_mecanum_scene import face_contact_formation
from swarmlib.controllers import ForceDistributedController
from swarmlib.communication.backend import (
    SimulatedBackend,
    AsyncSimulatedBackend,
    create_full_topology,
    create_ring_topology,
)


# ---------------------------------------------------------------------------
# Payload geometry (same as mrcap_fg)
# ---------------------------------------------------------------------------
PAYLOAD_HX = 0.450
PAYLOAD_HY = 0.450
PAYLOAD_HZ = 0.12


# ---------------------------------------------------------------------------
# Backend factory
# ---------------------------------------------------------------------------

def build_backend(kind: str, n: int, topology_kind: str, dropout: float, seed: int):
    if topology_kind == "full":
        topo = create_full_topology(n)
    elif topology_kind == "ring":
        topo = create_ring_topology(n)
    else:
        raise ValueError(f"Unknown topology: {topology_kind}")

    if kind == "simulated":
        return SimulatedBackend(n, topo)
    if kind == "async":
        return AsyncSimulatedBackend(
            num_agents=n, topology=topo,
            dropout_rate=dropout, mean_delay_steps=0, seed=seed,
        )
    raise ValueError(f"Unknown backend kind: {kind}")


# ---------------------------------------------------------------------------
# Single run
# ---------------------------------------------------------------------------

def run_single(
    n_robots: int,
    distance_x: float,
    distance_y: float,
    max_time: float,
    success_threshold: float,
    horizon: int,
    v_max: float,
    payload_mass: float,
    backend_kind: str,
    topology_kind: str,
    dropout: float,
    gbp_max_iters: int,
    visualise: bool = False,
    sim_speed: float = 1.0,
) -> dict:
    goal = (distance_x, distance_y, 0.0)
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

    backend = build_backend(backend_kind, n_robots, topology_kind, dropout, seed=42)

    controller = ForceDistributedController(
        num_robots=n_robots,
        formation=formation,
        backend=backend,
        config={
            "horizon": horizon,
            "v_max": v_max,
            "sigma_x": 0.5,
            "sigma_u": 0.3,
            "sigma_anchor": 0.01,
            "sigma_r2r": 0.05,
            "sigma_pull_in": 0.3,
            "sigma_consensus": 0.1,
            "gbp_max_iters": gbp_max_iters,
            "gbp_tol": 1e-3,
        },
    )

    obs = env.reset()
    controller.reset()

    viewer = None
    if visualise:
        import mujoco.viewer as mjv
        viewer = mjv.launch_passive(env.model, env.data)
        input("  Viewer open — adjust camera, then press Enter to start...")

    goal_arr = np.array(goal)
    payload_trajectory = []
    solve_times = []
    gbp_iters_log = []
    torque_log = []
    TORQUE_LIMIT = 10.0

    wheel_act_ids = env._wheel_act_ids

    wall_start = time.perf_counter()
    success = False

    while env.time < max_time:
        if viewer is not None and not viewer.is_running():
            break
        payload = obs["payload"]
        robots  = obs["robots"]

        #force is 2xN vector of forces (where N=num of robots), top half is 
        # vertical forces, bottom half is horizontal forces
        forces  = np.array([obs.get("base_forces"), obs.get("wall_forces")])

        #print('!!forces passed to controller', forces)

        payload_trajectory.append(payload[:3].tolist())

        controls = controller.compute_control(
            payload_state=payload,
            robot_states=robots,
            goal_state=goal_arr,
            dt=0.05,
            forces=forces,
        )
        solve_times.append(controller.get_solve_time())
        gbp_iters_log.append(controller.get_gbp_iters())

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

    # Straight-line deviation
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
    sat_frac = float(saturated.mean())
    peak_torque = float(np.abs(torques).max())

    comm_stats = backend.get_stats()

    env.close()

    return {
        "n_robots":           n_robots,
        "backend":            backend_kind,
        "topology":           topology_kind,
        "dropout":            dropout,
        "success":            success,
        "sim_time":           float(env.time),
        "wall_time_s":        wall_elapsed,
        "final_error_m":      final_error,
        "mean_deviation_m":   deviation,
        "solve_time_mean_ms": float(np.mean(solve_times) * 1e3),
        "solve_time_std_ms":  float(np.std(solve_times)  * 1e3),
        "solve_time_max_ms":  float(np.max(solve_times)  * 1e3),
        "gbp_iters_mean":     float(np.mean(gbp_iters_log)),
        "gbp_iters_max":      int(np.max(gbp_iters_log)),
        "messages_sent":      int(comm_stats.get("messages_sent", 0)),
        "messages_dropped":   int(comm_stats.get("messages_dropped", 0)),
        "n_steps":            len(solve_times),
        "sat_frac":           sat_frac,
        "peak_torque_Nm":     peak_torque,
        "trajectory":         traj.tolist(),
        "solve_times_ms":     (np.array(solve_times) * 1e3).tolist(),
        "gbp_iters":          list(map(int, gbp_iters_log)),
        "payload_mass_kg":    payload_mass,
        "goal":               goal_arr.tolist(),
        "torques_Nm":         torques.tolist(),
    }


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="DR.CAP distributed FG experiment")
    parser.add_argument("--n-values", default="2,3,4",
                        help="Comma-separated robot counts (default: 2,3,4)")
    parser.add_argument("--distance_x", type=float, default=5.0)
    parser.add_argument("--distance_y", type=float, default=5.0)
    parser.add_argument("--max-time", type=float, default=60.0)
    parser.add_argument("--horizon", type=int, default=15)
    parser.add_argument("--v-max", type=float, default=0.25)
    parser.add_argument("--payload-mass", type=float, default=2.0)
    parser.add_argument("--threshold", type=float, default=0.3)
    parser.add_argument("--backend", choices=["simulated", "async"],
                        default="simulated",
                        help="Communication backend (default: simulated)")
    parser.add_argument("--topology", choices=["full", "ring"], default="full",
                        help="Neighbor topology (default: full)")
    parser.add_argument("--dropout", type=float, default=0.0,
                        help="Message dropout rate for async backend (default: 0.0)")
    parser.add_argument("--gbp-max-iters", type=int, default=30,
                        help="Max GBP iterations per control step (default: 30)")
    parser.add_argument("--vis", action="store_true")
    parser.add_argument("--sim-speed", type=float, default=0.5)
    args = parser.parse_args()

    n_values = [int(x.strip()) for x in args.n_values.split(",")]

    print(f"\n{'='*60}")
    print(f"DR.CAP Distributed Factor-Graph Experiment")
    print(f"  robots:       {n_values}")
    print(f"  distance_x:     {args.distance_x} m")
    print(f"  distance_y:     {args.distance_y} m")
    print(f"  horizon:      {args.horizon}")
    print(f"  v_max:        {args.v_max} m/s")
    print(f"  backend:      {args.backend}")
    print(f"  topology:     {args.topology}")
    print(f"  dropout:      {args.dropout}")
    print(f"  gbp_max_iter: {args.gbp_max_iters}")
    print(f"{'='*60}\n")

    all_results = []

    for idx, n in enumerate(n_values):
        print(f"Running n={n} ...", flush=True)
        result = run_single(
            n_robots=n,
            distance_x=args.distance_x,
            distance_y=args.distance_y,
            max_time=args.max_time,
            success_threshold=args.threshold,
            horizon=args.horizon,
            v_max=args.v_max,
            payload_mass=args.payload_mass,
            backend_kind=args.backend,
            topology_kind=args.topology,
            dropout=args.dropout,
            gbp_max_iters=args.gbp_max_iters,
            visualise=args.vis and idx == 0,
            sim_speed=args.sim_speed,
        )
        all_results.append(result)

        status = "SUCCESS" if result["success"] else "TIMEOUT"
        print(
            f"  [{status}]  final_error={result['final_error_m']:.3f} m"
            f"  deviation={result['mean_deviation_m']:.3f} m"
            f"  solve={result['solve_time_mean_ms']:.1f}±{result['solve_time_std_ms']:.1f} ms"
            f"  gbp_iters_mean={result['gbp_iters_mean']:.1f}"
            f"  msgs={result['messages_sent']}"
            f"  (dropped={result['messages_dropped']})"
        )

    # Summary table
    print(f"\n{'='*90}")
    print(f"{'n':>4}  {'status':>8}  {'final_err(m)':>12}  {'dev(m)':>8}  "
          f"{'solve(ms)':>10}  {'gbp_it':>7}  {'msgs':>7}  {'drop':>5}")
    print(f"{'-'*4}  {'-'*8}  {'-'*12}  {'-'*8}  {'-'*10}  {'-'*7}  {'-'*7}  {'-'*5}")
    for r in all_results:
        status = "OK" if r["success"] else "TIMEOUT"
        print(
            f"{r['n_robots']:>4}  {status:>8}  "
            f"{r['final_error_m']:>12.3f}  {r['mean_deviation_m']:>8.3f}  "
            f"{r['solve_time_mean_ms']:>10.2f}  "
            f"{r['gbp_iters_mean']:>7.1f}  "
            f"{r['messages_sent']:>7}  {r['messages_dropped']:>5}"
        )
    print(f"{'='*90}\n")

    out = {
        "experiment":  "drcap_distributed_scaling",
        "timestamp":   datetime.now().isoformat(),
        "params": {
            "distance_m":   args.distance,
            "max_time_s":   args.max_time,
            "horizon":      args.horizon,
            "v_max":        args.v_max,
            "threshold_m":  args.threshold,
            "backend":      args.backend,
            "topology":     args.topology,
            "dropout":      args.dropout,
            "gbp_max_iters": args.gbp_max_iters,
        },
        "results": all_results,
    }
    out_path = Path(__file__).parent / "results.json"
    out_path.write_text(json.dumps(out, indent=2))
    print(f"Results saved to {out_path}")


if __name__ == "__main__":
    main()
