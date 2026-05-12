"""
Run a simulation of the different controllers and collect metrics to determine 
which one is better

Metrics:
- time to reach goal (0.3m distance to goal threshold)
- messages (GBP)
- solve time (ms)
- success (boolean)

Controllers tested:
- Force decentralised
- Force centralised
- DrCAP (=> centralised/decentralised ?)
- MRCAP
- Forceless centralised

Distance:
- 5m
- 10m
- 20m

Robots:
- 2
- 4
- 6
- 8

Horizon (N):
- 5
- 15
- 30

Dropout msgs ?
GBP max iter ?
"""


import argparse
import json
import time
from datetime import datetime
from pathlib import Path

import matplotlib.pyplot as plt
from collections import defaultdict

import numpy as np

from swarmlib.simulation.mecanum_env import MecanumTransportEnv
from swarmlib.simulation.generate_mecanum_scene import face_contact_formation
from swarmlib.simulation.generate_mecanum_scene import face_contact_formation_many_robots
from swarmlib.controllers import (
    MRCapController,
    # DRCapController,
    DRCapDistributedController,
    ForceCentralisedControllerCVel,
    ForceDistributedController,
    ForcelessCentralisedControllerCVel,

)
from swarmlib.communication.backend import (
    SimulatedBackend,
    AsyncSimulatedBackend,
    create_full_topology,
    create_ring_topology,
)


# values
ControllersList = [MRCapController, 
                   DRCapDistributedController, 
                   ForceCentralisedControllerCVel,
                   ForceDistributedController, 
                   ForcelessCentralisedControllerCVel]

DecentralisedControllers = [DRCapDistributedController, ForceDistributedController]

distances_to_goal = [i * 3 for i in range(1,11)]

numbers_of_robots = [i*2 for i in range(1,11)] #TODO look into getting more robots in formation, currently max of 4
# numbers_of_robots = [20] #TEMP


horizons = [i * 3 for i in range(1,11)]

#overwrite to simpler vals for testing
# distances_to_goal = [5, 10]
# horizons = [10, 15]
# ControllersList = [MRCapController, DRCapDistributedController]


#defaults
default_distance = 5.0
default_robot_num = 2
default_horizon = 15



# ---------------------------------------------------------------------------
# Payload geometry (same as mrcap_fg)
# ---------------------------------------------------------------------------
PAYLOAD_HX = 0.450
PAYLOAD_HY = 0.450
PAYLOAD_HZ = 0.12

# PAYLOAD_HX = 0.850
# PAYLOAD_HY = 0.850
# PAYLOAD_HZ = 0.12


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
    distance: float,
    max_time: float,
    success_threshold: float,
    horizon: int,
    v_max: float,
    payload_mass: float,
    backend_kind: str,
    topology_kind: str,
    dropout: float,
    gbp_max_iters: int,
    ChosenController,
    visualise: bool = False,
    sim_speed: float = 1.0,
    
) -> dict:
    goal = (distance, 0.0, 0.0)
    payload_size = (PAYLOAD_HX, PAYLOAD_HY, PAYLOAD_HZ)
    # formation = face_contact_formation(n_robots,
    #                                    payload_hx=PAYLOAD_HX,
    #                                    payload_hy=PAYLOAD_HY)
    formation, recommended_payload_xy_size = face_contact_formation_many_robots(n_robots,
                                       payload_hx=PAYLOAD_HX,
                                       payload_hy=PAYLOAD_HY)
    
    if payload_size[0:2] != recommended_payload_xy_size:
        print(f"Warning: payload size {payload_size[0:2]} might be too small \
for {n_robots} robots formation, using size {recommended_payload_xy_size} instead ")
        payload_size = (recommended_payload_xy_size[0], 
                        recommended_payload_xy_size[1], 
                        PAYLOAD_HZ)
    

    

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

    if ChosenController not in DecentralisedControllers:
        if ChosenController is MRCapController:
            controller = ChosenController(
                num_robots=n_robots,
                formation=formation,
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
        elif ChosenController is ForceCentralisedControllerCVel:
            controller = ChosenController(
                num_robots=n_robots,
                formation=formation,
                config={
                    "horizon": horizon,
                    "v_max": v_max,
                    "sigma_x": 0.5,
                    "sigma_u": 3, # ForcelessCentralised and ForceCentralised use value of 3 for sigma_u
                    "sigma_anchor": 0.01,
                    "sigma_r2r": 0.05,
                    "sigma_pull_in": 0.3,
                    "sigma_consensus": 0.1,
                    "gbp_max_iters": gbp_max_iters,
                    "gbp_tol": 1e-3,
                },
            )
        elif ChosenController is ForcelessCentralisedControllerCVel:
            controller = ChosenController(
                num_robots=n_robots,
                formation=formation,
                config={
                    "horizon": horizon,
                    "v_max": v_max,
                    "sigma_x": 0.5,
                    "sigma_u": 3, # ForcelessCentralised and ForceCentralised use value of 3 for sigma_u
                    "sigma_anchor": 0.01
                },
            )

    else:
        if ChosenController is ForceDistributedController:
            controller = ChosenController(
            num_robots=n_robots,
            formation=formation,
            backend=backend,
            config={
                "horizon": horizon,
                "v_max": v_max,
                "sigma_x": 0.5,
                "sigma_u": 3, #force distributed needs higher val
                "sigma_anchor": 0.01,
                "sigma_r2r": 0.05,
                "sigma_pull_in": 0.3,
                "sigma_consensus": 0.1,
                "gbp_max_iters": gbp_max_iters,
                "gbp_tol": 1e-3,
            },
        )
        else:
            controller = ChosenController(
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

    mass_estimate = None
    centroid_velocity_estimtate = None

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

        # print("Controller:", controller)
        # print("Config:", controller.config)

        #force is 2xN vector of forces (where N=num of robots), top half is 
        # vertical forces, bottom half is horizontal forces

        #print('!!forces passed to controller', forces)

        payload_trajectory.append(payload[:3].tolist())

        if ChosenController is ForceCentralisedControllerCVel: #these two have a different way to pass forces for whatever reason

            wall_forces = obs.get("wall_forces")
            base_forces = obs.get("base_forces")
            # print('wall_forces:', wall_forces)
            # print('base_forces:', base_forces)

            controls, mass_estimate, centroid_velocity_estimtate = controller.compute_control(
                payload_state=payload,
                robot_states=robots,
                goal_state=goal_arr,
                dt=0.05,
                wall_forces=wall_forces,
                base_forces=base_forces,
                mass_estimate=mass_estimate,
                centroid_velocity_estimate=centroid_velocity_estimtate
            )
            # print('controls generated:', controls)
            # print('mass_estimate:', mass_estimate)
            # print('centroid_velocity_estimtate:', centroid_velocity_estimtate)

        elif ChosenController is ForcelessCentralisedControllerCVel:
            
            controls= controller.compute_control(
                payload_state=payload,
                robot_states=robots,
                goal_state=goal_arr,
                dt=0.05,
            )
        else:
            forces  = np.array([obs.get("base_forces"), obs.get("wall_forces")])

            controls = controller.compute_control(
                payload_state=payload,
                robot_states=robots,
                goal_state=goal_arr,
                dt=0.05,
                forces=forces,
            )


        solve_times.append(controller.get_solve_time())
        if ChosenController in DecentralisedControllers:
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
        "gbp_iters_max":      int(np.max(gbp_iters_log)) if ChosenController in DecentralisedControllers else np.nan,
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



def plot_results(results):
    import matplotlib.pyplot as plt
    import numpy as np
    from collections import defaultdict

    def plot_metric_vs_x(x_key, y_key, y_label):
        plt.figure()

        controllers = sorted(set(r["controller"] for r in results))

        for ctrl in controllers:
            ctrl_data = [r for r in results if r["controller"] == ctrl]

            x_vals = sorted(set(r[x_key] for r in ctrl_data))

            y_vals = []
            for x in x_vals:
                subset = [r[y_key] for r in ctrl_data if r[x_key] == x]

                # Handle success separately (bool → float)
                if y_key == "success":
                    subset = [float(s) for s in subset]

                y_vals.append(np.mean(subset) if subset else np.nan)

            plt.plot(x_vals, y_vals, marker='o', label=ctrl)

        plt.xlabel(x_key.replace("_", " ").title())
        plt.ylabel(y_label)
        plt.title(f"{y_label} vs {x_key.replace('_',' ')}")
        plt.legend()
        plt.grid()

    # -------------------------
    # SUCCESS
    # -------------------------
    plot_metric_vs_x("n_robots", "success", "Success rate")
    plot_metric_vs_x("distance", "success", "Success rate")
    plot_metric_vs_x("horizon", "success", "Success rate")

    # -------------------------
    # SOLVE TIME
    # -------------------------
    plot_metric_vs_x("n_robots", "solve_time_mean", "Solve time (ms)")
    plot_metric_vs_x("distance", "solve_time_mean", "Solve time (ms)")
    plot_metric_vs_x("horizon", "solve_time_mean", "Solve time (ms)")

    # -------------------------
    # FINAL ERROR
    # -------------------------
    plot_metric_vs_x("n_robots", "final_error", "Final error (m)")
    plot_metric_vs_x("distance", "final_error", "Final error (m)")
    plot_metric_vs_x("horizon", "final_error", "Final error (m)")

    # -------------------------
    # TIME TO COMPLETE
    # -------------------------
    plot_metric_vs_x("n_robots", "time", "Time to complete (s)")
    plot_metric_vs_x("distance", "time", "Time to complete (s)")
    plot_metric_vs_x("horizon", "time", "Time to complete (s)")

    plt.show()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="DR.CAP distributed FG experiment")
    parser.add_argument("--n-values", default="2,3,4",
                        help="Comma-separated robot counts (default: 2,3,4)")
    parser.add_argument("--distances", type=float, default=distances_to_goal)
    parser.add_argument("--max-time", type=float, default=300.0)
    parser.add_argument("--horizons", type=int, default=horizons)
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

    # print(f"\n{'='*60}")
    # print(f"DR.CAP Distributed Factor-Graph Experiment")
    # print(f"  robots:       {n_values}")
    # print(f"  distance:     {args.distance} m")
    # print(f"  horizon:      {args.horizon}")
    # print(f"  v_max:        {args.v_max} m/s")
    # print(f"  backend:      {args.backend}")
    # print(f"  topology:     {args.topology}")
    # print(f"  dropout:      {args.dropout}")
    # print(f"  gbp_max_iter: {args.gbp_max_iters}")
    # print(f"{'='*60}\n")

    all_results = []
    all_results_plotting = []
    robot_num_resuts_plotting = []
    distance_resuts_plotting = []
    horizon_resuts_plotting = []


    #TEST NUMBER OF ROBOTS
    for idx, n in enumerate(numbers_of_robots):
        for controller in ControllersList:
            print(f"Running n={n} with controller:{controller} ...", flush=True)
            result = run_single(
                n_robots=n,
                distance=default_distance,
                max_time=args.max_time,
                success_threshold=args.threshold,
                horizon=default_horizon,
                v_max=args.v_max,
                payload_mass=args.payload_mass,
                backend_kind=args.backend,
                topology_kind=args.topology,
                dropout=args.dropout,
                gbp_max_iters=args.gbp_max_iters,
                visualise=args.vis and idx == 0,
                sim_speed=args.sim_speed,
                ChosenController=controller
            )
            all_results.append(result)
            all_results_plotting.append({
                "controller": controller.__name__,
                "n_robots": n,
                "distance": default_distance,
                "horizon": default_horizon,

                "time": result["sim_time"],
                "success": result["success"],
                "final_error": result["final_error_m"],
                "deviation": result["mean_deviation_m"],
                "solve_time_mean": result["solve_time_mean_ms"],
                "solve_time_std": result["solve_time_std_ms"],

                # optional if available
                "gbp_iters": result.get("gbp_iters_mean"),
                "messages": result.get("msgs"),
            })

            status = "SUCCESS" if result["success"] else "TIMEOUT"
            print(
                f"  [{status}]  final_error={result['final_error_m']:.3f} m"
                f"  deviation={result['mean_deviation_m']:.3f} m"
                f"  solve={result['solve_time_mean_ms']:.1f}±{result['solve_time_std_ms']:.1f} ms"
                f"  gbp_iters_mean={result['gbp_iters_mean']:.1f}"
                f"  msgs={result['messages_sent']}"
                f"  (dropped={result['messages_dropped']})"
            )


    #TEST DISTANCES TO GOAL
    for dist in distances_to_goal:
        for controller in ControllersList:
            print(f"Running distance={dist} with controller:{controller} ...", flush=True)
            result = run_single(
                n_robots=default_robot_num,
                distance=dist,
                max_time=args.max_time,
                success_threshold=args.threshold,
                horizon=default_horizon,
                v_max=args.v_max,
                payload_mass=args.payload_mass,
                backend_kind=args.backend,
                topology_kind=args.topology,
                dropout=args.dropout,
                gbp_max_iters=args.gbp_max_iters,
                visualise=args.vis and idx == 0,
                sim_speed=args.sim_speed,
                ChosenController=controller
            )
            all_results.append(result)
            all_results_plotting.append({
                "controller": controller.__name__,
                "n_robots": default_robot_num,
                "distance": dist,
                "horizon": default_horizon,

                "time": result["sim_time"],
                "success": result["success"],
                "final_error": result["final_error_m"],
                "deviation": result["mean_deviation_m"],
                "solve_time_mean": result["solve_time_mean_ms"],
                "solve_time_std": result["solve_time_std_ms"],

                # optional if available
                "gbp_iters": result.get("gbp_iters_mean"),
                "messages": result.get("msgs"),
            })

            status = "SUCCESS" if result["success"] else "TIMEOUT"
            print(
                f"  [{status}]  final_error={result['final_error_m']:.3f} m"
                f"  deviation={result['mean_deviation_m']:.3f} m"
                f"  solve={result['solve_time_mean_ms']:.1f}±{result['solve_time_std_ms']:.1f} ms"
                f"  gbp_iters_mean={result['gbp_iters_mean']:.1f}"
                f"  msgs={result['messages_sent']}"
                f"  (dropped={result['messages_dropped']})"
            )

    #TEST HORIZONS
    for horizon in horizons:
        for controller in ControllersList:
            print(f"Running horizon={horizon} with controller:{controller} ...", flush=True)
            result = run_single(
                n_robots=default_robot_num,
                distance=default_distance,
                max_time=args.max_time,
                success_threshold=args.threshold,
                horizon=horizon,
                v_max=args.v_max,
                payload_mass=args.payload_mass,
                backend_kind=args.backend,
                topology_kind=args.topology,
                dropout=args.dropout,
                gbp_max_iters=args.gbp_max_iters,
                visualise=args.vis and idx == 0,
                sim_speed=args.sim_speed,
                ChosenController=controller
            )
            all_results.append(result)
            all_results_plotting.append({
                "controller": controller.__name__,
                "n_robots": default_robot_num,
                "distance": default_distance,
                "horizon": horizon,

                "time": result["sim_time"],
                "success": result["success"],
                "final_error": result["final_error_m"],
                "deviation": result["mean_deviation_m"],
                "solve_time_mean": result["solve_time_mean_ms"],
                "solve_time_std": result["solve_time_std_ms"],

                # optional if available
                "gbp_iters": result.get("gbp_iters_mean"),
                "messages": result.get("msgs"),
            })

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

    # out = {
    #     "experiment":  "drcap_distributed_scaling",
    #     "timestamp":   datetime.now().isoformat(),
    #     "params": {
    #         "distance_m":   args.distance,
    #         "max_time_s":   args.max_time,
    #         "horizon":      args.horizon,
    #         "v_max":        args.v_max,
    #         "threshold_m":  args.threshold,
    #         "backend":      args.backend,
    #         "topology":     args.topology,
    #         "dropout":      args.dropout,
    #         "gbp_max_iters": args.gbp_max_iters,
    #     },
    #     "results": all_results,
    # }
    # out_path = Path(__file__).parent / "results.json"
    # out_path.write_text(json.dumps(out, indent=2))
    # print(f"Results saved to {out_path}")

    plot_results(all_results_plotting)

if __name__ == "__main__":
    main()

