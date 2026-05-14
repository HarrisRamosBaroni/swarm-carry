"""
Analyse a directory of (trimmed) JSONL recordings and produce:

  - Per-recording trajectory PDF in <out_dir>/figures/
  - Aggregate JSON of all metrics in <out_dir>/real_results_<timestamp>.json
  - Terminal summary table

Metrics computed per recording
  path_length_ratio       actual payload path / straight-line start→goal
  formation_error_mean_m  mean deviation from frozen-at-start nominal offsets
  wall_force_mean_N       mean total force across all robots and time
  wall_force_imbalance_N  std of per-robot time-averaged forces
  success                 payload within goal tolerance at recording end

Usage:
  python3 -m real_robot.laptop.analyse_recordings real_robot/recordings/
  python3 -m real_robot.laptop.analyse_recordings real_robot/recordings/ --out findings/real/
  python3 -m real_robot.laptop.analyse_recordings recording.jsonl
"""

import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np

from real_robot.laptop.review import (
    _load, _color_for, _label_for, _collect_paths,
    PAYLOAD_ID, Recording,
)

plt.rcParams.update({
    "font.size": 10,
    "axes.titlesize": 10,
    "axes.labelsize": 10,
    "legend.fontsize": 8,
    "figure.dpi": 150,
    "savefig.bbox": "tight",
})

DEFAULT_GOAL_TOL = 0.20  # metres, used when not stored in recording


# ---------------------------------------------------------------------------
# Metrics
# ---------------------------------------------------------------------------

def _payload_trajectory(rec: Recording):
    """Return (ts, xs, ys) arrays for the payload, NaNs removed."""
    if PAYLOAD_ID not in rec.poses:
        return np.array([]), np.array([]), np.array([])
    ts, xs, ys, _ = rec.poses[PAYLOAD_ID]
    valid = ~(np.isnan(xs) | np.isnan(ys))
    return ts[valid], xs[valid], ys[valid]


def _goal_tol(rec: Recording) -> float:
    tol = DEFAULT_GOAL_TOL
    for _, topic, data in rec.rows:
        if topic == "goal" and isinstance(data, dict) and "tol" in data:
            tol = float(data["tol"])
    return tol


def compute_metrics(rec: Recording) -> dict:
    result: dict = {
        "recording": rec.path.name,
        "controller": rec.meta.get("controller", "unknown"),
        "mode": rec.meta.get("mode", "unknown"),
        "n_robots": sum(1 for rid in rec.poses if rid != PAYLOAD_ID),
        "experiment": "real",
        "duration_s": rec.duration,
    }

    ts_p, xs_p, ys_p = _payload_trajectory(rec)
    goal_tol = _goal_tol(rec)
    last_goal = rec.goals[-1] if rec.goals else None  # (t, x, y, theta)

    # ---- path length ratio ----
    if len(xs_p) >= 2:
        path_len = float(np.hypot(np.diff(xs_p), np.diff(ys_p)).sum())
        gx = last_goal[1] if last_goal and last_goal[1] is not None else xs_p[-1]
        gy = last_goal[2] if last_goal and last_goal[2] is not None else ys_p[-1]
        straight = float(np.hypot(xs_p[0] - gx, ys_p[0] - gy))
        result["path_length_ratio"] = path_len / straight if straight > 0.01 else float("nan")
    else:
        path_len = float("nan")
        result["path_length_ratio"] = float("nan")

    # ---- success ----
    if len(xs_p) >= 1 and last_goal and last_goal[1] is not None:
        dist_to_goal = float(np.hypot(xs_p[-1] - last_goal[1], ys_p[-1] - last_goal[2]))
        result["success"] = dist_to_goal <= goal_tol
        result["dist_to_goal_m"] = dist_to_goal
    else:
        result["success"] = False
        result["dist_to_goal_m"] = float("nan")

    result["goal"] = ([last_goal[1], last_goal[2], last_goal[3]] if last_goal
                      else [float(xs_p[-1]) if len(xs_p) else 0.0,
                            float(ys_p[-1]) if len(ys_p) else 0.0, 0.0])
    result["goal_tol_m"] = goal_tol

    # downsampled trajectory for visual figures
    if len(xs_p) >= 2:
        step = max(1, len(xs_p) // 300)
        result["trajectory"] = [[float(x), float(y)]
                                 for x, y in zip(xs_p[::step], ys_p[::step])]
    else:
        result["trajectory"] = []

    # ---- formation error ----
    robot_ids = [rid for rid in sorted(rec.poses) if rid != PAYLOAD_ID]
    nominal_offsets: dict = {}

    if len(ts_p) > 0 and robot_ids:
        t_start = float(ts_p[0])
        px0 = float(np.interp(t_start, ts_p, xs_p))
        py0 = float(np.interp(t_start, ts_p, ys_p))
        for rid in robot_ids:
            r_ts, r_xs, r_ys, _ = rec.poses[rid]
            if len(r_ts) == 0:
                continue
            t_ref = max(t_start, float(r_ts[0]))
            rx0 = float(np.interp(t_ref, ts_p, xs_p))  # payload at t_ref
            ry0 = float(np.interp(t_ref, ts_p, ys_p))
            rxi = float(np.interp(t_ref, r_ts, r_xs))
            ryi = float(np.interp(t_ref, r_ts, r_ys))
            nominal_offsets[rid] = (rxi - rx0, ryi - ry0)

    fe_ts: list[float] = []
    if nominal_offsets and len(ts_p) > 0:
        step = max(1, len(ts_p) // 500)
        for t, px, py in zip(ts_p[::step], xs_p[::step], ys_p[::step]):
            errs = []
            for rid, (dx_nom, dy_nom) in nominal_offsets.items():
                r_ts, r_xs, r_ys, _ = rec.poses[rid]
                if len(r_ts) == 0 or t < r_ts[0] or t > r_ts[-1]:
                    continue
                rx = float(np.interp(t, r_ts, r_xs))
                ry = float(np.interp(t, r_ts, r_ys))
                errs.append(float(np.hypot((rx - px) - dx_nom, (ry - py) - dy_nom)))
            if errs:
                fe_ts.append(float(np.mean(errs)))

    result["formation_errors_ts"] = fe_ts
    result["formation_error_mean_m"] = float(np.mean(fe_ts)) if fe_ts else float("nan")

    # ---- wall forces ----
    robot_ids_f = [rid for rid in sorted(rec.forces) if rid != PAYLOAD_ID]
    per_robot_mean: list[float] = []
    per_robot_series: dict = {}

    for rid in robot_ids_f:
        f_ts, f_totals, _ = rec.forces[rid]
        if len(f_ts) == 0:
            continue
        per_robot_mean.append(float(np.mean(np.abs(f_totals))))
        step = max(1, len(f_ts) // 500)
        per_robot_series[rid] = (f_ts[::step], np.abs(f_totals[::step]))

    result["wall_force_mean_N"] = float(np.mean(per_robot_mean)) if per_robot_mean else float("nan")
    result["wall_force_imbalance_N"] = (float(np.std(per_robot_mean))
                                        if len(per_robot_mean) > 1 else float("nan"))

    # common time grid for wall_forces_ts (rows = time, cols = robots)
    if per_robot_series:
        rids_f = sorted(per_robot_series)
        t_grid = per_robot_series[rids_f[0]][0]
        wf_ts = []
        for t in t_grid:
            row = []
            for rid in rids_f:
                f_t, f_v = per_robot_series[rid]
                row.append(float(np.interp(t, f_t, f_v))
                           if f_t[0] <= t <= f_t[-1] else float("nan"))
            wf_ts.append(row)
        result["wall_forces_ts"] = wf_ts
    else:
        result["wall_forces_ts"] = []

    return result


# ---------------------------------------------------------------------------
# Figures
# ---------------------------------------------------------------------------

def plot_trajectory(rec: Recording, metrics: dict, out_path: Path):
    """XY trajectory for one recording — payload + robots, goal ring."""
    ts_p, xs_p, ys_p = _payload_trajectory(rec)
    goal = metrics["goal"]
    tol = metrics["goal_tol_m"]
    t0 = rec.t0

    fig, ax = plt.subplots(figsize=(5.5, 5.0))
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")

    from matplotlib.lines import Line2D
    legend_handles = []

    # robots
    for rid in sorted(rec.poses):
        if rid == PAYLOAD_ID:
            continue
        r_ts, r_xs, r_ys, _ = rec.poses[rid]
        valid = ~(np.isnan(r_xs) | np.isnan(r_ys))
        if not valid.any():
            continue
        color = _color_for(rid)
        ax.plot(r_xs[valid], r_ys[valid], "-", color=color, alpha=0.35, linewidth=0.9)
        legend_handles.append(Line2D([0], [0], color=color, linewidth=1.5,
                                     label=_label_for(rid)))

    # payload
    if len(xs_p) >= 2:
        norm = plt.Normalize(ts_p[0] - t0, ts_p[-1] - t0)
        for i in range(len(xs_p) - 1):
            ax.plot(xs_p[i:i+2], ys_p[i:i+2], "-",
                    color=cm.plasma(norm(ts_p[i] - t0)), linewidth=1.8)
        ax.plot(xs_p[0], ys_p[0], "o", color="black", ms=8, zorder=5,
                label="payload start")
        ax.plot(xs_p[-1], ys_p[-1], "s", color="black", ms=8, zorder=5,
                label="payload end")
        legend_handles += [
            Line2D([0], [0], color=cm.plasma(0.1), linewidth=2, label="payload (early)"),
            Line2D([0], [0], color=cm.plasma(0.9), linewidth=2, label="payload (late)"),
            Line2D([0], [0], marker="o", color="black", linewidth=0, ms=7, label="start"),
            Line2D([0], [0], marker="s", color="black", linewidth=0, ms=7, label="end"),
        ]

    # goal
    if goal[0] is not None:
        ax.plot(goal[0], goal[1], "g*", ms=14, mec="black", zorder=6)
        ring = plt.Circle((goal[0], goal[1]), tol, fill=False,
                          linestyle="--", edgecolor="green", linewidth=1.2, alpha=0.7)
        ax.add_patch(ring)
        legend_handles.append(
            Line2D([0], [0], marker="*", color="green", mec="black",
                   linewidth=0, ms=10, label=f"goal (tol={tol:.2f} m)")
        )

    name = rec.meta.get("name") or rec.path.stem
    ctrl = metrics["controller"]
    ratio = metrics["path_length_ratio"]
    fe = metrics["formation_error_mean_m"]
    ax.set_title(
        f"{name}  |  {ctrl}\n"
        f"path ratio={ratio:.2f}  fe={fe:.3f} m  "
        f"{'✓' if metrics['success'] else '✗'}",
        fontsize=9,
    )
    ax.legend(handles=legend_handles, loc="best", fontsize=7)

    fig.savefig(out_path)
    plt.close(fig)
    print(f"  → {out_path.name}")


# ---------------------------------------------------------------------------
# Summary table
# ---------------------------------------------------------------------------

def _fmt(v, fmt=".2f"):
    if v is None or (isinstance(v, float) and (np.isnan(v) or np.isinf(v))):
        return "—"
    return format(v, fmt)


def print_summary(all_metrics: list[dict]):
    cols = [
        ("recording",              "Recording",       "<40"),
        ("controller",             "Controller",      "<35"),
        ("n_robots",               "N",               ">3"),
        ("duration_s",             "Dur(s)",          ">7"),
        ("path_length_ratio",      "PathRatio",       ">10"),
        ("formation_error_mean_m", "FormErr(m)",      ">10"),
        ("wall_force_mean_N",      "Force(N)",        ">9"),
        ("wall_force_imbalance_N", "Imbal(N)",        ">9"),
        ("success",                "OK",              ">4"),
    ]
    header = "  ".join(format(h, spec) for _, h, spec in cols)
    print("\n" + header)
    print("-" * len(header))
    for m in all_metrics:
        def fv(key, fmt=".2f"):
            v = m.get(key)
            if isinstance(v, bool):
                return "✓" if v else "✗"
            return _fmt(v, fmt)
        row = "  ".join([
            format(m.get("recording", "")[:38],  "<40"),
            format(m.get("controller", "")[:33], "<35"),
            format(str(m.get("n_robots", "?")),  ">3"),
            format(fv("duration_s", ".1f"),       ">7"),
            format(fv("path_length_ratio"),       ">10"),
            format(fv("formation_error_mean_m", ".3f"), ">10"),
            format(fv("wall_force_mean_N", ".1f"), ">9"),
            format(fv("wall_force_imbalance_N", ".2f"), ">9"),
            format(fv("success"),                 ">4"),
        ])
        print(row)
    print()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Analyse real-robot JSONL recordings and produce metrics + figures."
    )
    parser.add_argument("target",
                        help="Directory of .jsonl recordings, or a single .jsonl file.")
    parser.add_argument("--out", default=None,
                        help="Output directory (default: <target>/analysis/ or ./analysis/)")
    args = parser.parse_args()

    target = Path(args.target)
    if not target.exists():
        print(f"[analyse] not found: {target}")
        sys.exit(1)

    paths = _collect_paths(target)
    if not paths:
        print(f"[analyse] no .jsonl recordings found in {target}")
        sys.exit(1)
    print(f"[analyse] {len(paths)} recordings found")

    out_dir = Path(args.out) if args.out else (target if target.is_dir() else target.parent) / "analysis"
    fig_dir = out_dir / "figures"
    fig_dir.mkdir(parents=True, exist_ok=True)

    all_metrics: list[dict] = []

    for path in paths:
        print(f"\nLoading {path.name} …")
        try:
            rec = _load(path)
        except Exception as e:
            print(f"  [skip] failed to load: {e}")
            continue

        m = compute_metrics(rec)
        all_metrics.append(m)

        traj_path = fig_dir / (path.stem + "_trajectory.pdf")
        try:
            plot_trajectory(rec, m, traj_path)
        except Exception as e:
            print(f"  [warn] trajectory plot failed: {e}")

    if not all_metrics:
        print("[analyse] no results — nothing written")
        sys.exit(1)

    # strip timeseries from per-row display but keep in JSON
    print_summary(all_metrics)

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    json_path = out_dir / f"real_results_{stamp}.json"

    # make JSON serialisable (convert nan → null)
    def _clean(obj):
        if isinstance(obj, float) and (np.isnan(obj) or np.isinf(obj)):
            return None
        if isinstance(obj, dict):
            return {k: _clean(v) for k, v in obj.items()}
        if isinstance(obj, list):
            return [_clean(v) for v in obj]
        return obj

    payload = _clean({"results": all_metrics, "generated_at": stamp})
    json_path.write_text(json.dumps(payload, indent=2))
    print(f"[analyse] metrics → {json_path}")
    print(f"[analyse] figures → {fig_dir}/")


if __name__ == "__main__":
    main()
