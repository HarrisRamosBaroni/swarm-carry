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

    # ---- per-segment stats (one segment per goal) ----
    # Segment i: robot moves toward goals[i].
    #   starts at goals[i].t (clamped to payload range), or ts_p[0] for i==0
    #   ends   at goals[i+1].t (clamped)              , or ts_p[-1] for last
    def _px_at(t):
        """Payload (x,y) interpolated at absolute time t."""
        if len(ts_p) == 0:
            return float("nan"), float("nan")
        t = float(np.clip(t, ts_p[0], ts_p[-1]))
        return float(np.interp(t, ts_p, xs_p)), float(np.interp(t, ts_p, ys_p))

    def _path_len_between(t_a, t_b):
        """Cumulative payload path length in [t_a, t_b]."""
        if len(ts_p) < 2:
            return float("nan")
        mask = (ts_p >= t_a) & (ts_p <= t_b)
        xs_s, ys_s = xs_p[mask], ys_p[mask]
        if len(xs_s) < 2:
            return 0.0
        return float(np.hypot(np.diff(xs_s), np.diff(ys_s)).sum())

    segments: list[dict] = []
    goals = rec.goals  # list of (t, x, y, theta)

    if goals and len(ts_p) >= 2:
        for i, (g_t, gx, gy, gth) in enumerate(goals):
            if gx is None or gy is None:
                continue
            t_seg_start = float(np.clip(
                ts_p[0] if i == 0 else goals[i - 1][0],
                ts_p[0], ts_p[-1],
            ))
            t_seg_end = float(np.clip(
                ts_p[-1] if i == len(goals) - 1 else goals[i + 1][0],
                ts_p[0], ts_p[-1],
            ))
            if t_seg_end <= t_seg_start:
                continue

            px_start, py_start = _px_at(t_seg_start)
            px_end,   py_end   = _px_at(t_seg_end)
            straight = float(np.hypot(px_start - gx, py_start - gy))
            path_len_seg = _path_len_between(t_seg_start, t_seg_end)
            dist_end = float(np.hypot(px_end - gx, py_end - gy))

            segments.append({
                "goal_index":      i,
                "goal_x":          float(gx),
                "goal_y":          float(gy),
                "t_start":         t_seg_start - rec.t0,
                "t_end":           t_seg_end   - rec.t0,
                "duration_s":      t_seg_end - t_seg_start,
                "path_len_m":      path_len_seg,
                "straight_line_m": straight,
                "path_ratio":      path_len_seg / straight if straight > 0.01 else float("nan"),
                "dist_to_goal_m":  dist_end,
                "success":         dist_end <= goal_tol,
            })

    result["segments"] = segments
    result["n_goals"] = len(goals)

    # overall path ratio: total path / piecewise-ideal (sum of per-segment straight lines)
    total_path = _path_len_between(ts_p[0], ts_p[-1]) if len(ts_p) >= 2 else float("nan")
    total_straight = sum(s["straight_line_m"] for s in segments) if segments else float("nan")
    result["path_length_ratio"] = (total_path / total_straight
                                   if segments and total_straight > 0.01
                                   else float("nan"))

    # success = all segments reached their goal
    result["success"] = bool(segments and all(s["success"] for s in segments))
    last_seg = segments[-1] if segments else None
    result["dist_to_goal_m"] = last_seg["dist_to_goal_m"] if last_seg else float("nan")

    # backwards-compat fields used by plot
    last_goal = goals[-1] if goals else None
    result["goal"] = ([float(last_goal[1]), float(last_goal[2]), float(last_goal[3])]
                      if last_goal and last_goal[1] is not None
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

    # robots — solid color line + viridis scatter for time, faded
    for rid in sorted(rec.poses):
        if rid == PAYLOAD_ID:
            continue
        r_ts, r_xs, r_ys, _ = rec.poses[rid]
        valid = ~(np.isnan(r_xs) | np.isnan(r_ys))
        if not valid.any():
            continue
        r_ts_v = r_ts[valid];  r_xs_v = r_xs[valid];  r_ys_v = r_ys[valid]
        rc = _color_for(rid)
        ax.plot(r_xs_v, r_ys_v, "-", color=rc, alpha=0.5, linewidth=1.3)
        ax.scatter(r_xs_v, r_ys_v, c=r_ts_v - t0, cmap="viridis", s=4, zorder=3)
        ax.plot(r_xs_v[0],  r_ys_v[0],  "o", color=rc, ms=5,
                mec="black", mew=0.6, alpha=0.85, zorder=4)
        ax.plot(r_xs_v[-1], r_ys_v[-1], "s", color=rc, ms=5,
                mec="black", mew=0.6, alpha=0.85, zorder=4)
        legend_handles.append(
            Line2D([0], [0], color=rc, linewidth=1.3, alpha=0.6,
                   label=_label_for(rid))
        )

    # payload — solid color line + viridis scatter for time, thicker
    if len(xs_p) >= 2:
        pc = _color_for(PAYLOAD_ID)
        ax.plot(xs_p, ys_p, "-", color=pc, alpha=0.5, linewidth=2.2)
        ax.scatter(xs_p, ys_p, c=ts_p - t0, cmap="viridis", s=6, zorder=4)
        ax.plot(xs_p[0],  ys_p[0],  "o", color=pc, ms=9, mec="black", mew=0.8, zorder=5)
        ax.plot(xs_p[-1], ys_p[-1], "s", color=pc, ms=9, mec="black", mew=0.8, zorder=5)
        legend_handles += [
            Line2D([0], [0], color="black", linewidth=2.2, label="payload"),
            Line2D([0], [0], marker="o", color="black", linewidth=0, ms=7, label="start"),
            Line2D([0], [0], marker="s", color="black", linewidth=0, ms=7, label="end"),
        ]

    # goals — all waypoints, numbered, faded except last
    all_goals = rec.goals
    goal_added_to_legend = False
    for i, (g_t, gx, gy, gth) in enumerate(all_goals):
        if gx is None or gy is None:
            continue
        is_last = (i == len(all_goals) - 1)
        alpha_pt   = 0.95 if is_last else 0.45
        alpha_ring = 0.65 if is_last else 0.20
        ax.plot(gx, gy, "g*", ms=14 if is_last else 10,
                mec="black", alpha=alpha_pt, zorder=6)
        if len(all_goals) > 1:
            ax.annotate(f"G{i}", (gx, gy),
                        textcoords="offset points", xytext=(6, 4),
                        fontsize=7, alpha=alpha_pt, color="darkgreen")
        ring = plt.Circle((gx, gy), tol, fill=False, linestyle="--",
                          edgecolor="green", linewidth=1.2, alpha=alpha_ring)
        ax.add_patch(ring)
        if not goal_added_to_legend:
            legend_handles.append(
                Line2D([0], [0], marker="*", color="green", mec="black",
                       linewidth=0, ms=10, label=f"goal (tol={tol:.2f} m)")
            )
            goal_added_to_legend = True

    name = rec.meta.get("name") or rec.path.stem
    ctrl = metrics["controller"]
    ratio = metrics["path_length_ratio"]
    segs = metrics.get("segments", [])
    if len(segs) > 1:
        seg_ratios = "/".join(
            f"{s['path_ratio']:.2f}" if not (isinstance(s['path_ratio'], float) and np.isnan(s['path_ratio'])) else "—"
            for s in segs
        )
        ratio_str = f"path ratio={ratio:.2f} (per goal: {seg_ratios})"
    else:
        ratio_str = f"path ratio={ratio:.2f}"
    ax.set_title(
        f"{name}  |  {ctrl}\n"
        f"{ratio_str}  {'✓' if metrics['success'] else '✗'}",
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
        segs = m.get("segments", [])
        if len(segs) > 1:
            for s in segs:
                ok = "✓" if s["success"] else "✗"
                pr = _fmt(s["path_ratio"]) if not (isinstance(s["path_ratio"], float) and np.isnan(s["path_ratio"])) else "—"
                print(f"    G{s['goal_index']}  dur={s['duration_s']:.1f}s  "
                      f"path_ratio={pr}  dist_end={s['dist_to_goal_m']:.3f}m  {ok}")
    print()


# ---------------------------------------------------------------------------
# LaTeX table
# ---------------------------------------------------------------------------

def _tex_fmt(v, fmt=".2f"):
    if v is None or (isinstance(v, float) and (np.isnan(v) or np.isinf(v))):
        return r"—"
    return format(v, fmt)


def save_latex_table(all_metrics: list[dict], out_path: Path):
    """Write a booktabs LaTeX table, one row per recording."""
    header = "\n".join([
        r"\begin{table}[t]",
        r"\centering",
        r"\caption{Real-robot experiment results. "
        r"Formation error: mean deviation of each robot from its initial offset relative to the payload centroid. "
        r"Wall force: mean total contact force per robot over the run. "
        r"Imbalance: standard deviation of per-robot time-averaged forces. "
        r"Path ratio: total payload path\,/\,sum of per-waypoint straight-line distances, "
        r"where each straight-line distance is measured from the payload position at the start of that waypoint segment to the waypoint goal; "
        r"values above 1.0 indicate the payload took a longer-than-straight path (e.g.\ overshoot or curvature). "
        r"$\dagger$~multi-waypoint run; per-waypoint ratios in parentheses.}",
        r"\label{tab:real_results}",
        r"\begin{tabular}{llcccccccc}",
        r"\toprule",
        r"Recording & Controller & $n$ & Dur.\,(s) & Path ratio "
        r"& Form.\,err.\,(m) & Force\,(N) & Imbal.\,(N) & Success \\",
        r"\midrule",
    ])

    body_lines = []
    for m in all_metrics:
        name = m.get("recording", "")
        # strip timestamp prefix and .jsonl suffix for brevity
        stem = name.replace(".jsonl", "")
        parts = stem.split("_", 1)
        short_name = parts[1] if len(parts) == 2 else stem
        short_name = short_name.replace("_", r"\_")

        ctrl_raw = m.get("controller", "unknown")
        ctrl = ctrl_raw.replace("_", r"\_")

        n = str(m.get("n_robots", "?"))
        dur = _tex_fmt(m.get("duration_s"), ".1f")

        segs = m.get("segments", [])
        multi = len(segs) > 1
        overall_ratio = _tex_fmt(m.get("path_length_ratio"))
        if multi:
            per_seg = "/".join(
                _tex_fmt(s["path_ratio"]) if not (isinstance(s["path_ratio"], float) and np.isnan(s["path_ratio"])) else r"—"
                for s in segs
            )
            ratio_cell = rf"{overall_ratio}$^\dagger$ ({per_seg})"
        else:
            ratio_cell = overall_ratio

        fe   = _tex_fmt(m.get("formation_error_mean_m"), ".3f")
        wf   = _tex_fmt(m.get("wall_force_mean_N"), ".1f")
        imb  = _tex_fmt(m.get("wall_force_imbalance_N"), ".2f")
        ok   = r"\checkmark" if m.get("success") else r"$\times$"

        body_lines.append(
            rf"{short_name} & {ctrl} & {n} & {dur} & {ratio_cell} & {fe} & {wf} & {imb} & {ok} \\"
        )

    footer = "\n".join([
        r"\bottomrule",
        r"\end{tabular}",
        r"\end{table}",
    ])

    tex = header + "\n" + "\n".join(body_lines) + "\n" + footer + "\n"
    out_path.write_text(tex)
    print(f"[analyse] table   → {out_path}")


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

    tex_path = out_dir / f"real_results_{stamp}.tex"
    save_latex_table(all_metrics, tex_path)


if __name__ == "__main__":
    main()
