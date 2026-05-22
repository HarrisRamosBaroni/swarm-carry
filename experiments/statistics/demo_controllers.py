"""
Interactive demo: run controllers one-by-one with the MuJoCo viewer.

On the first run the viewer opens and waits for you to position the camera
before pressing Enter to start. That camera pose is then restored
automatically for every subsequent run.

Between runs the script pauses so you can start/stop your screen recorder.

Usage:
    python3 demo_controllers.py
    python3 demo_controllers.py --n-robots 4 --distance 3.0 --sim-speed 2.0
    python3 demo_controllers.py --controllers MRCapController,ForceDistributedController
    python3 demo_controllers.py --list-controllers
"""

import argparse
import sys
from pathlib import Path

from run_report_experiments import REPORT_CONTROLLERS, LABEL, DEFAULT_HORIZON, DEFAULT_DROPOUT
from run_tests import run_single, DecentralisedControllers

# ---------------------------------------------------------------------------
# Defaults
# ---------------------------------------------------------------------------

DEFAULT_N        = 4
DEFAULT_DISTANCE = 5.0
DEFAULT_V_MAX    = 0.25
DEFAULT_MAX_TIME = 120.0
DEFAULT_MASS_PER = 1.0
DEFAULT_SIM_SPD  = 1.0
DEFAULT_HORIZON  = DEFAULT_HORIZON
DEFAULT_DROPOUT  = DEFAULT_DROPOUT


def _parse_args():
    p = argparse.ArgumentParser(
        description="Run all (or selected) controllers with the MuJoCo viewer.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--n-robots",        type=int,   default=DEFAULT_N,
                   help=f"Number of robots (default: {DEFAULT_N})")
    p.add_argument("--distance",        type=float, default=DEFAULT_DISTANCE,
                   help=f"Diagonal distance to goal in metres (default: {DEFAULT_DISTANCE}). "
                        "Ignored if --goal is given.")
    p.add_argument("--goal",            default=None,
                   help="Goal pose as 'x,y,theta' (m, m, rad). Overrides --distance.")
    p.add_argument("--v-max",           type=float, default=DEFAULT_V_MAX,
                   help=f"Max robot velocity m/s (default: {DEFAULT_V_MAX})")
    p.add_argument("--max-time",        type=float, default=DEFAULT_MAX_TIME,
                   help=f"Max simulation time in seconds (default: {DEFAULT_MAX_TIME})")
    p.add_argument("--horizon",         type=int,   default=DEFAULT_HORIZON,
                   help=f"MPC horizon steps (default: {DEFAULT_HORIZON})")
    p.add_argument("--dropout",         type=float, default=DEFAULT_DROPOUT,
                   help=f"Communication dropout rate 0–1 (default: {DEFAULT_DROPOUT})")
    p.add_argument("--mass-per-robot",  type=float, default=DEFAULT_MASS_PER,
                   help=f"Payload mass per robot kg (default: {DEFAULT_MASS_PER})")
    p.add_argument("--sim-speed",       type=float, default=DEFAULT_SIM_SPD,
                   help=f"Simulation speed multiplier (default: {DEFAULT_SIM_SPD}, 2.0 = twice real-time)")
    p.add_argument("--backend",         default="async", choices=["simulated", "async"])
    p.add_argument("--topology",        default="full",  choices=["full", "ring"])
    p.add_argument("--gbp-max-iters",   type=int,   default=30)
    p.add_argument("--controllers",     default=None,
                   help="Comma-separated controller class names to run (default: all). "
                        "Use --list-controllers to see available names.")
    p.add_argument("--list-controllers", action="store_true",
                   help="Print available controller names and exit.")
    p.add_argument("--no-pause",        action="store_true",
                   help="Skip the between-run 'press Enter' pause (runs back-to-back).")
    return p.parse_args()


def main():
    args = _parse_args()

    _all = {c.__name__: c for c in REPORT_CONTROLLERS}

    if args.list_controllers:
        print("Available controllers:")
        for name, label in LABEL.items():
            print(f"  {name:45s}  ({label})")
        return

    if args.controllers:
        names = [n.strip() for n in args.controllers.split(",")]
        bad   = [n for n in names if n not in _all]
        if bad:
            raise SystemExit(f"Unknown controller(s): {bad}\nKnown: {list(_all)}")
        controllers = [_all[n] for n in names]
    else:
        controllers = list(REPORT_CONTROLLERS)

    if args.goal is not None:
        parts = [float(x.strip()) for x in args.goal.split(",")]
        if len(parts) != 3:
            raise SystemExit("--goal must be 'x,y,theta' (3 floats, e.g. '3.0,1.0,0.0')")
        goal_tuple = tuple(parts)
    else:
        import math
        d = args.distance
        goal_tuple = (math.sqrt(d**2 / 2), math.sqrt(d**2 / 2), 0.0)

    n           = args.n_robots
    mass_total  = n * args.mass_per_robot

    print()
    print("=" * 60)
    print("  CONTROLLER DEMO")
    print("=" * 60)
    print(f"  robots       : {n}")
    print(f"  goal (x,y,θ) : ({goal_tuple[0]:.3f}, {goal_tuple[1]:.3f}, {goal_tuple[2]:.3f})")
    print(f"  v_max        : {args.v_max} m/s")
    print(f"  payload mass : {mass_total:.1f} kg  ({args.mass_per_robot} kg/robot)")
    print(f"  sim speed    : {args.sim_speed}x")
    print(f"  max sim time : {args.max_time} s")
    print(f"  controllers  : {[c.__name__ for c in controllers]}")
    print("=" * 60)
    print()

    camera_pose = None

    for i, ctrl_cls in enumerate(controllers):
        label = LABEL.get(ctrl_cls.__name__, ctrl_cls.__name__)
        print(f"[{i+1}/{len(controllers)}]  {label}")

        if camera_pose is None:
            print("  First run — the viewer will open.")
            print("  Position the camera to your liking, then press Enter to start.")
        else:
            if not args.no_pause:
                input(f"  Ready for next controller ({label}). Press Enter when recording ...")
            print("  Camera pose restored from first run.")

        result = run_single(
            n_robots=n,
            distance=args.distance,
            max_time=args.max_time,
            success_threshold=0.15,
            horizon=args.horizon,
            v_max=args.v_max,
            payload_mass=mass_total,
            backend_kind=args.backend,
            topology_kind=args.topology,
            dropout=args.dropout,
            gbp_max_iters=args.gbp_max_iters,
            ChosenController=ctrl_cls,
            visualise=True,
            sim_speed=args.sim_speed,
            camera_pose=camera_pose,
            goal=goal_tuple,
        )

        # Latch camera pose from the first run
        if camera_pose is None and result.get("camera_pose"):
            camera_pose = result["camera_pose"]
            print(f"  Camera pose saved: az={camera_pose['azimuth']:.1f}  "
                  f"el={camera_pose['elevation']:.1f}  dist={camera_pose['distance']:.2f}")

        status = "SUCCESS" if result["success"] else "timed out / did not reach goal"
        print(f"  → {status}  "
              f"fe={result['formation_error_mean_m']:.3f} m  "
              f"solve={result['solve_time_mean_ms']:.1f} ms")
        print()

    print("=" * 60)
    print("  All controllers done.")
    print("=" * 60)


if __name__ == "__main__":
    main()
