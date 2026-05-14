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
    ContactHealthController,
    ContactHealthDistributedController
)
from swarmlib.communication.backend import (
    SimulatedBackend,
    AsyncSimulatedBackend,
    create_full_topology,
    create_ring_topology,
)


# values
ControllersList = [
                   ContactHealthDistributedController,
                   ContactHealthController,
                   MRCapController, 
                   DRCapDistributedController, 
                   ForceCentralisedControllerCVel,
                   ForceDistributedController, 
                   ForcelessCentralisedControllerCVel]

DecentralisedControllers = [DRCapDistributedController, ForceDistributedController, ContactHealthDistributedController]

distances_to_goal = [i * 3 for i in range(1,11)] #3-30

numbers_of_robots = [i*2 for i in range(1,11)] #2-20

horizons = [i * 3 for i in range(1,11)] #3-30

gbp_dropout_rate = [i * 0.1 for i in range(10)] #0-90 %
gbp_num_robots = [2,3,4]



##overwrite to simpler vals for testing

# gbp_dropout_rate = [0.0, 0.5]
# gbp_num_robots = [2]
# distances_to_goal = [5, 10]
# horizons = [10, 15]
# ControllersList = [MRCapController, DRCapDistributedController]


#defaults
default_distance = 5.0
default_robot_num = 2
default_horizon = 15
default_threshold = 0.1
default_dropout = 0.0


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
    wall_time_limit: float = None,
) -> dict:
    goal = (np.sqrt(distance**2/2) , np.sqrt(distance**2/2) , 0.0) #move in diagonal, to a goal <distance>m away
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
        if ChosenController in [MRCapController, ContactHealthController]:
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
        elif ChosenController is ContactHealthDistributedController:
            #         cfg = {
            #     "horizon":  horizon, "v_max": v_max,
            #     "sigma_x":  0.5, "sigma_u": 0.3, "sigma_anchor": 0.01,
            #     "F_wall_star":      F_wall_star,
            #     "payload_mass_nom": payload_mass_nom,
            #     "alpha_sigma_u":    alpha,
            #     "beta_recovery":    beta,
            #     "use_weighted_anchor":    flags[0],
            #     "use_modulated_sigma_u":  flags[1],
            #     "use_recovery_term":      flags[2],
            # }
            # if controller_kind == "distributed":
            #     cfg["gbp_max_iters"] = gbp_max_iters
            #     cfg["gbp_tol"]       = gbp_tol
            #     return ContactHealt
            controller = ChosenController(
                num_robots=n_robots,
                formation=formation,
                # backend=backend,
                config={
                    "horizon":  horizon, "v_max": v_max,
                    "sigma_x":  0.5, "sigma_u": 0.3, "sigma_anchor": 0.01,
                    "F_wall_star":      10.0,
                    "payload_mass_nom": payload_mass,
                    "alpha_sigma_u":    0.1,
                    "beta_recovery":    0.005,
                    "use_weighted_anchor":    True,
                    "use_modulated_sigma_u":  True,
                    "use_recovery_term":      True,

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
    formation_arr = np.array([(f[0], f[1]) for f in formation])  # (n, 2) body-frame offsets
    payload_trajectory = []
    solve_times = []
    gbp_iters_log = []
    torque_log = []
    formation_error_log = []   # mean distance from nominal slot per step
    wall_force_log = []        # (n,) per step
    base_force_log = []        # (n,) per step
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

        # Formation error: distance of each robot from nominal slot in payload frame
        cx, cy, ctheta = payload[:3]
        cos_t, sin_t = np.cos(ctheta), np.sin(ctheta)
        R_pay = np.array([[cos_t, -sin_t], [sin_t, cos_t]])
        expected_pos = (R_pay @ formation_arr.T).T + np.array([cx, cy])
        actual_pos = robots[:, :2]
        formation_error_log.append(float(np.mean(np.linalg.norm(actual_pos - expected_pos, axis=1))))

        # Contact force logging
        wf = obs.get("wall_forces")
        bf = obs.get("base_forces")
        if wf is not None:
            wall_force_log.append(wf.tolist() if hasattr(wf, "tolist") else list(wf))
        if bf is not None:
            base_force_log.append(bf.tolist() if hasattr(bf, "tolist") else list(bf))

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

        elif ChosenController in [ContactHealthController, ContactHealthDistributedController]:

            wall_forces = obs.get("wall_forces")
            base_forces = obs.get("base_forces")
            # print('wall_forces:', wall_forces)
            # print('base_forces:', base_forces)

            controls = controller.compute_control(
                payload_state=payload,
                robot_states=robots,
                goal_state=goal_arr,
                dt=0.05,
                wall_forces=wall_forces,
                base_forces=base_forces,
            )


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

        if wall_time_limit is not None and (time.perf_counter() - wall_start) > wall_time_limit:
            print(f"  [WALL TIMEOUT] {wall_time_limit:.0f}s wall-time limit reached", flush=True)
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

    # Path length and ratio
    steps_xy = traj[:, :2]
    path_length = float(np.sum(np.linalg.norm(np.diff(steps_xy, axis=0), axis=1)))
    straight_dist = float(np.linalg.norm(goal_arr[:2] - steps_xy[0]))
    path_length_ratio = path_length / straight_dist if straight_dist > 1e-6 else 1.0

    # Formation error aggregation
    fe_arr = np.array(formation_error_log) if formation_error_log else np.array([0.0])

    # Contact force aggregation
    if wall_force_log:
        wf_arr = np.array(wall_force_log)   # (T, n)
        wall_force_mean   = float(np.mean(wf_arr))
        wall_force_std    = float(np.std(wf_arr))
        wall_force_min    = float(np.min(wf_arr))
        # imbalance: std across robots per step, then time-averaged
        wall_force_imbalance = float(np.mean(np.std(wf_arr, axis=1)))
    else:
        wf_arr = None
        wall_force_mean = wall_force_std = wall_force_min = wall_force_imbalance = float("nan")

    if base_force_log:
        bf_arr = np.array(base_force_log)
        base_force_mean = float(np.mean(bf_arr))
        base_force_std  = float(np.std(bf_arr))
    else:
        bf_arr = None
        base_force_mean = base_force_std = float("nan")

    comm_stats = backend.get_stats()

    env.close()

    return {
        "n_robots":               n_robots,
        "backend":                backend_kind,
        "topology":               topology_kind,
        "dropout":                dropout,
        "success":                success,
        "sim_time":               float(env.time),
        "wall_time_s":            wall_elapsed,
        "final_error_m":          final_error,
        "mean_deviation_m":       deviation,
        "path_length_m":          path_length,
        "path_length_ratio":      path_length_ratio,
        "formation_error_mean_m": float(np.mean(fe_arr)),
        "formation_error_max_m":  float(np.max(fe_arr)),
        "wall_force_mean_N":      wall_force_mean,
        "wall_force_std_N":       wall_force_std,
        "wall_force_min_N":       wall_force_min,
        "wall_force_imbalance_N": wall_force_imbalance,
        "base_force_mean_N":      base_force_mean,
        "base_force_std_N":       base_force_std,
        "solve_time_mean_ms":     float(np.mean(solve_times) * 1e3),
        "solve_time_std_ms":      float(np.std(solve_times)  * 1e3),
        "solve_time_max_ms":      float(np.max(solve_times)  * 1e3),
        "gbp_iters_mean":         float(np.mean(gbp_iters_log)) if gbp_iters_log else float("nan"),
        "gbp_iters_max":          int(np.max(gbp_iters_log)) if ChosenController in DecentralisedControllers and gbp_iters_log else None,
        "messages_sent":          int(comm_stats.get("messages_sent", 0)),
        "messages_dropped":       int(comm_stats.get("messages_dropped", 0)),
        "n_steps":                len(solve_times),
        "sat_frac":               sat_frac,
        "peak_torque_Nm":         peak_torque,
        "trajectory":             traj.tolist(),
        "solve_times_ms":         (np.array(solve_times) * 1e3).tolist(),
        "gbp_iters":              list(map(int, gbp_iters_log)),
        "payload_mass_kg":        payload_mass,
        "goal":                   goal_arr.tolist(),
        "torques_Nm":             torques.tolist(),
        "wall_forces_ts":         wf_arr.tolist() if wf_arr is not None else [],
        "base_forces_ts":         bf_arr.tolist() if bf_arr is not None else [],
        "formation_errors_ts":    fe_arr.tolist(),
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


import matplotlib.pyplot as plt
from collections import defaultdict

def plot_time_vs_dropout(plotting_array):
    # Structure:
    # data[n_robots][controller][dropout] -> list of times
    data = defaultdict(lambda: defaultdict(lambda: defaultdict(list)))

    # Fill structure
    for entry in plotting_array:
        n = entry["n_robots"]
        controller = entry["controller"]
        dropout = entry["dropout"]
        time = entry["time"]

        data[n][controller][dropout].append(time)

    # Create one figure per n_robots
    for n_robots, controllers in data.items():
        plt.figure(figsize=(6, 5))

        for controller, dropout_dict in controllers.items():
            # Sort dropout values
            dropouts = sorted(dropout_dict.keys())

            # Average time per dropout
            times = [
                sum(dropout_dict[d]) / len(dropout_dict[d])
                for d in dropouts
            ]

            plt.plot(dropouts, times, marker='o', label=controller)

        plt.title(f"{n_robots} Robots")
        plt.xlabel("Dropout")
        plt.ylabel("Time")
        plt.grid(True)
        plt.legend()

        plt.tight_layout()
        plt.show()
    
# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="DR.CAP distributed FG experiment")
    # PARAMETERS WRITEN IN TOP OF PYTHON FILE

    # parser.add_argument("--n-values", default="2,3,4",
    #                     help="Comma-separated robot counts (default: 2,3,4)")
    # parser.add_argument("--distances", type=float, default=distances_to_goal)
    parser.add_argument("--max-time", type=float, default=300.0) #5min
    # parser.add_argument("--horizons", type=int, default=horizons)
    parser.add_argument("--v-max", type=float, default=0.25)
    parser.add_argument("--payload-mass", type=float, default=2.0)
    # parser.add_argument("--threshold", type=float, default=0.3)
    parser.add_argument("--backend", choices=["simulated", "async"],
                        default="async",
                        help="Communication backend (default: simulated)")
    parser.add_argument("--topology", choices=["full", "ring"], default="full",
                        help="Neighbor topology (default: full)")
    # parser.add_argument("--dropout", type=float, default=0.0,
    #                     help="Message dropout rate for async backend (default: 0.0)")
    parser.add_argument("--gbp-max-iters", type=int, default=30,
                        help="Max GBP iterations per control step (default: 30)")
    parser.add_argument("--vis", action="store_true")
    parser.add_argument("--sim-speed", type=float, default=0.5)
    args = parser.parse_args()

    # n_values = [int(x.strip()) for x in args.n_values.split(",")]

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
    gbp_results_plotting = []


    #TEST NUMBER OF ROBOTS
    for idx, n in enumerate(numbers_of_robots):
        for controller in ControllersList:
            print(f"Running n={n} with controller:{controller} ...", flush=True)
            result = run_single(
                n_robots=n,
                distance=default_distance,
                max_time=args.max_time,
                success_threshold=default_threshold,
                horizon=default_horizon,
                v_max=args.v_max,
                payload_mass=args.payload_mass,
                backend_kind=args.backend,
                topology_kind=args.topology,
                dropout=default_dropout,
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
                success_threshold=default_threshold,
                horizon=default_horizon,
                v_max=args.v_max,
                payload_mass=args.payload_mass,
                backend_kind=args.backend,
                topology_kind=args.topology,
                dropout=default_dropout,
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
                success_threshold=default_threshold,
                horizon=horizon,
                v_max=args.v_max,
                payload_mass=args.payload_mass,
                backend_kind=args.backend,
                topology_kind=args.topology,
                dropout=default_dropout,
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

    #TEST GBP DROPOUT
    for robot_num in gbp_num_robots:
        for dropout_rate in gbp_dropout_rate:
            for controller in DecentralisedControllers:
                print(f"Running dropout rate={dropout_rate} with controller:{controller} ...", flush=True)
                result = run_single(
                    n_robots=robot_num,
                    distance=default_distance,
                    max_time=args.max_time,
                    success_threshold=default_threshold,
                    horizon=default_horizon,
                    v_max=args.v_max,
                    payload_mass=args.payload_mass,
                    backend_kind=args.backend,
                    topology_kind=args.topology,
                    dropout=dropout_rate,
                    gbp_max_iters=args.gbp_max_iters,
                    visualise=args.vis,
                    sim_speed=args.sim_speed,
                    ChosenController=controller
                )
                all_results.append(result)
                
                gbp_results_plotting.append({
                    "controller": controller.__name__,
                    "n_robots": default_robot_num,
                    "distance": default_distance,
                    "horizon": default_horizon,

                    "time": result["sim_time"],
                    "success": result["success"],
                    "final_error": result["final_error_m"],
                    "deviation": result["mean_deviation_m"],
                    "solve_time_mean": result["solve_time_mean_ms"],
                    "solve_time_std": result["solve_time_std_ms"],

                    # should all be there now that we only test gbp dropout
                    "gbp_iters": result.get("gbp_iters_mean"),
                    "messages": result.get("msgs"),
                    "dropout": result.get("dropout")
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

    plot_time_vs_dropout(gbp_results_plotting)

    # plot_results(all_results_plotting)

    out_path = Path(__file__).parent / f"results_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    out_path.write_text(json.dumps({
        "timestamp": datetime.now().isoformat(),
        "results":   all_results,
    }, indent=2))
    print(f"\nResults saved → {out_path}")


if __name__ == "__main__":
    main()

