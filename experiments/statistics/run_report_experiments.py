"""
Focused experiment suite for the DOT Report 2 findings.

Runs a tractable subset of conditions and saves a JSON that
generate_findings.py can load to produce figures and LaTeX tables.

Estimated runtime: 30-90 min depending on hardware.

Usage:
    python run_report_experiments.py [--max-time 120] [--v-max 0.25]
"""

import argparse
import json
import time
from datetime import datetime
from pathlib import Path

import numpy as np

from run_tests import run_single, DecentralisedControllers
from swarmlib.controllers import (
    MRCapController,
    DRCapDistributedController,
    ForceCentralisedControllerCVel,
    ForcelessCentralisedControllerCVel,
    ContactHealthController,
    ContactHealthDistributedController,
)

# ---------------------------------------------------------------------------
# Experiment matrix
# ---------------------------------------------------------------------------

REPORT_CONTROLLERS = [
    MRCapController,
    ContactHealthController,
    ForcelessCentralisedControllerCVel,
    DRCapDistributedController,
    ContactHealthDistributedController,
]

LABEL = {
    "MRCapController":                    "MR.CAP",
    "ContactHealthController":            "ContactHealth (central.)",
    "ForcelessCentralisedControllerCVel": "Forceless (central.)",
    "DRCapDistributedController":         "DR.CAP (distr.)",
    "ContactHealthDistributedController": "ContactHealth (distr.)",
}

N_ROBOTS_SWEEP    = [2, 4, 6, 8, 10]
DEFAULT_N         = 4
DEFAULT_DISTANCE  = 5.0
DEFAULT_HORIZON   = 15
DEFAULT_THRESHOLD = 0.1
DEFAULT_DROPOUT   = 0.0
F_WALL_STAR       = 10.0   # target wall force (N) — must match ContactHealth config

# Dropout sweep — decentralised controllers only
DROPOUT_SWEEP  = [0.0, 0.15, 0.3, 0.45, 0.6, 0.75, 0.9]
DROPOUT_N      = [2, 4]

FINDINGS_DIR = Path(__file__).parents[2] / "final_report" / "0225_DOT_Report_2" / "findings"


def _resolve_findings_dir():
    """Try project-relative path first, fall back to absolute."""
    candidates = [
        FINDINGS_DIR,
        Path("/home/harris/Documents/y3/dot/final_report/0225_DOT_Report_2/findings"),
    ]
    for p in candidates:
        if p.parent.exists():
            p.mkdir(parents=True, exist_ok=True)
            return p
    raise RuntimeError("Could not locate findings/ directory")


def run_one(label, ctrl_cls, n, dist, horizon, dropout, max_time, v_max, mass_per_robot,
            backend, topology, gbp_max_iters, wall_time_limit=None):
    mass = n * mass_per_robot
    print(f"  {label}  n={n}  dist={dist}m  dropout={dropout:.0%} ...", flush=True)
    t0 = time.perf_counter()
    result = run_single(
        n_robots=n,
        distance=dist,
        max_time=max_time,
        success_threshold=DEFAULT_THRESHOLD,
        horizon=horizon,
        v_max=v_max,
        payload_mass=mass,
        backend_kind=backend,
        topology_kind=topology,
        dropout=dropout,
        gbp_max_iters=gbp_max_iters,
        ChosenController=ctrl_cls,
        visualise=False,
        wall_time_limit=wall_time_limit,
    )
    elapsed = time.perf_counter() - t0
    status = "OK" if result["success"] else "TIMEOUT"
    print(
        f"    [{status}] mass={mass:.1f}kg  fe={result['formation_error_mean_m']:.3f}m  "
        f"wf={result['wall_force_mean_N']:.1f}N  "
        f"dev={result['mean_deviation_m']:.3f}m  "
        f"solve={result['solve_time_mean_ms']:.1f}ms  "
        f"(wall {elapsed:.0f}s)",
        flush=True,
    )
    result["controller"] = ctrl_cls.__name__
    result["controller_label"] = LABEL.get(ctrl_cls.__name__, ctrl_cls.__name__)
    result["mass_per_robot_kg"] = mass_per_robot
    result["experiment"] = ""  # filled by caller
    result["distance"] = dist
    result["horizon"] = horizon
    result["F_wall_star"] = F_WALL_STAR
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--max-time",      type=float, default=120.0)
    parser.add_argument("--v-max",         type=float, default=0.25)
    parser.add_argument("--mass-per-robot", type=float, default=1.0,
                        help="Payload mass per robot (kg). Total mass = n × this. Default: 1.0 kg/robot")
    parser.add_argument("--backend",       default="async", choices=["simulated", "async"])
    parser.add_argument("--topology",      default="full",  choices=["full", "ring"])
    parser.add_argument("--gbp-max-iters", type=int,   default=30)
    parser.add_argument("--skip-dropout",  action="store_true",
                        help="Skip the dropout sweep (saves ~40 min)")
    parser.add_argument("--wall-time-limit", type=float, default=300.0,
                        help="Max wall-clock seconds per run before aborting (default: 300)")
    parser.add_argument("--fast",          action="store_true",
                        help="Smoke-test mode: 2 controllers × 2 robot counts, 20 s timeout, no dropout")
    args = parser.parse_args()

    if args.fast:
        controllers     = [MRCapController, ContactHealthController]
        n_robots_list   = [2, 4]
        max_time        = 20.0
        wall_time_limit = 60.0
        skip_dropout    = True
        print("*** FAST MODE: 4 runs, 20 s sim / 60 s wall timeout — smoke test only ***")
    else:
        controllers     = REPORT_CONTROLLERS
        n_robots_list   = N_ROBOTS_SWEEP
        max_time        = args.max_time
        wall_time_limit = args.wall_time_limit
        skip_dropout    = args.skip_dropout

    findings = _resolve_findings_dir()
    all_results = []

    # ------------------------------------------------------------------
    # 1. Scalability sweep — all controllers × n_robots
    # ------------------------------------------------------------------
    print("\n=== SCALABILITY SWEEP (n_robots) ===")
    for ctrl_cls in controllers:
        for n in n_robots_list:
            r = run_one(
                ctrl_cls.__name__, ctrl_cls, n,
                DEFAULT_DISTANCE, DEFAULT_HORIZON, DEFAULT_DROPOUT,
                max_time, args.v_max, args.mass_per_robot,
                args.backend, args.topology, args.gbp_max_iters,
                wall_time_limit=wall_time_limit,
            )
            r["experiment"] = "scalability"
            all_results.append(r)

    # ------------------------------------------------------------------
    # 2. Dropout robustness — decentralised controllers only
    # ------------------------------------------------------------------
    if not skip_dropout:
        print("\n=== DROPOUT SWEEP ===")
        decentralised = [c for c in controllers if c in DecentralisedControllers]
        for ctrl_cls in decentralised:
            for n in DROPOUT_N:
                for do in DROPOUT_SWEEP:
                    r = run_one(
                        ctrl_cls.__name__, ctrl_cls, n,
                        DEFAULT_DISTANCE, DEFAULT_HORIZON, do,
                        max_time, args.v_max, args.mass_per_robot,
                        "async", args.topology, args.gbp_max_iters,
                        wall_time_limit=wall_time_limit,
                    )
                    r["experiment"] = "dropout"
                    all_results.append(r)

    # ------------------------------------------------------------------
    # Save
    # ------------------------------------------------------------------
    out_path = findings / "data" / f"report_results_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    out_path.write_text(json.dumps({
        "timestamp":   datetime.now().isoformat(),
        "F_wall_star": F_WALL_STAR,
        "results":     all_results,
    }, indent=2))
    print(f"\nSaved {len(all_results)} runs → {out_path}")
    print("Next step: python generate_findings.py --results", out_path)


if __name__ == "__main__":
    main()
