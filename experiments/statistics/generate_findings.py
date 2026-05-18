"""
Generate report figures and LaTeX table fragments from a findings JSON.

Usage:
    python generate_findings.py --results findings/data/report_results_TIMESTAMP.json

Outputs (all in findings/figures/ and findings/tables/):

  figures/
    01_solve_time_vs_nrobots.pdf
    02_formation_error_vs_nrobots.pdf
    03_wall_force_vs_nrobots.pdf
    04_success_rate_vs_nrobots.pdf
    05_path_ratio_vs_nrobots.pdf
    06_trajectory_comparison.pdf
    07_wall_force_timeseries.pdf      (MR.CAP vs ContactHealth, n=DEFAULT_N)
    08_formation_error_timeseries.pdf (same)
    09_success_vs_dropout.pdf         (decentralised controllers)
    10_contact_imbalance_vs_nrobots.pdf

  tables/
    tab_summary.tex       (one row per controller, default-param run)
    tab_scalability.tex   (solve time × n_robots)
"""

import argparse
import json
import math
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import medfilt

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

FINDINGS_ROOT = Path("/home/harris/Documents/y3/dot/final_report/0225_DOT_Report_2/findings")
FIG_DIR   = FINDINGS_ROOT / "figures"
TABLE_DIR = FINDINGS_ROOT / "tables"
NOTE_DIR  = FINDINGS_ROOT / "notes"

# Colour / marker scheme — consistent across all figures
DECENTRALISED = {
    "DRCapDistributedController",
    "ForceDistributedController",
    "ContactHealthDistributedController",
}

STYLE = {
    "MRCapController":                    {"color": "#1f77b4", "marker": "o",  "ls": "-"},
    "ContactHealthController":            {"color": "#d62728", "marker": "s",  "ls": "-"},
    "ForcelessCentralisedControllerCVel": {"color": "#2ca02c", "marker": "^",  "ls": "--"},
    "DRCapDistributedController":         {"color": "#ff7f0e", "marker": "D",  "ls": "-."},
    "ContactHealthDistributedController": {"color": "#9467bd", "marker": "v",  "ls": ":"},
    "ForceCentralisedControllerCVel":     {"color": "#8c564b", "marker": "x",  "ls": "--"},
    "ForceDistributedController":         {"color": "#e377c2", "marker": "+",  "ls": ":"},
}

LABEL = {
    "MRCapController":                    "MR.CAP",
    "ContactHealthController":            "ContactHealth",
    "ForcelessCentralisedControllerCVel": "Forceless",
    "DRCapDistributedController":         "DR.CAP (distr.)",
    "ContactHealthDistributedController": "CH-Distr.",
    "ForceCentralisedControllerCVel":     "Force (centr.)",
    "ForceDistributedController":         "Force (distr.)",
}

plt.rcParams.update({
    "font.size": 10,
    "axes.titlesize": 10,
    "axes.labelsize": 10,
    "legend.fontsize": 8,
    "figure.dpi": 150,
    "savefig.bbox": "tight",
})


def load(path: str) -> list:
    data = json.loads(Path(path).read_text())
    results = data.get("results", data)  # handle both formats
    return results, data.get("F_wall_star", 10.0)


def controllers_in(results):
    return list(dict.fromkeys(r["controller"] for r in results))


def style(ctrl):
    return STYLE.get(ctrl, {"color": "grey", "marker": ".", "ls": "-"})


def label(ctrl):
    return LABEL.get(ctrl, ctrl)


def scalability_subset(results):
    return [r for r in results if r.get("experiment", "scalability") == "scalability"]


def mean_metric_vs_n(results, metric, ctrl):
    rows = [r for r in results if r["controller"] == ctrl]
    ns = sorted(set(r["n_robots"] for r in rows))
    vals = []
    for n in ns:
        subset = [r[metric] for r in rows if r["n_robots"] == n]
        vals.append(float(np.nanmean(subset)) if subset else float("nan"))
    return np.array(ns), np.array(vals)


def save_fig(fig, name):
    path = FIG_DIR / name
    fig.savefig(path)
    plt.close(fig)
    print(f"  → {path.name}")


def save_note_placeholder(name, content=""):
    path = NOTE_DIR / (name.replace(".pdf", "") + ".md")
    if not path.exists():
        path.write_text(f"# Notes: {name}\n\n{content}\n")


# ---------------------------------------------------------------------------
# Figure generators
# ---------------------------------------------------------------------------

def fig_metric_vs_n(results, metric, ylabel, title, fname, *, hline=None, hline_label=None,
                    scalability_only=True, yscale="linear"):
    subset = scalability_subset(results) if scalability_only else results
    ctrls = controllers_in(subset)
    fig, ax = plt.subplots(figsize=(5.5, 3.8))
    for ctrl in ctrls:
        ns, vals = mean_metric_vs_n(subset, metric, ctrl)
        st = style(ctrl)
        ax.plot(ns, vals, color=st["color"], marker=st["marker"],
                linestyle=st["ls"], label=label(ctrl), linewidth=1.5)
    if hline is not None:
        ax.axhline(hline, color="black", linestyle=":", linewidth=1, label=hline_label)
    all_ns = sorted(set(r["n_robots"] for r in subset))
    ax.set_xticks(all_ns)
    ax.set_xlabel("Number of robots $n$")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.set_yscale(yscale)
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3, which="both" if yscale == "log" else "major")
    save_fig(fig, fname)
    save_note_placeholder(fname)


def fig_solve_time(results):
    fig_metric_vs_n(
        results, "solve_time_mean_ms", "Solve time (ms)",
        "Solve time vs. team size", "01_solve_time_vs_nrobots.pdf",
    )
    fig_metric_vs_n(
        results, "solve_time_mean_ms", "Solve time (ms, log scale)",
        "Solve time vs. team size (log scale)", "01b_solve_time_vs_nrobots_log.pdf",
        yscale="log",
    )


def fig_solve_time_distributed_bounds(results):
    """Solve time with parallel bounds for distributed controllers.

    Centralised controllers: direct measurement (solid line).
    Distributed controllers: shaded region between solve_time/n (parallel lower
    bound, assumes perfect load balance and zero comms overhead) and solve_time
    (sequential upper bound, single-machine cost).  The lower-bound line is
    labelled; the upper-bound edge of the shading is unlabelled to avoid clutter.
    """
    subset = scalability_subset(results)
    ctrls = controllers_in(subset)
    fig, ax = plt.subplots(figsize=(5.5, 3.8))

    for ctrl in ctrls:
        ns, vals = mean_metric_vs_n(subset, "solve_time_mean_ms", ctrl)
        st = style(ctrl)
        if ctrl in DECENTRALISED:
            lb = vals / ns          # lower bound: per-robot parallel time
            ub = vals               # upper bound: sequential single-machine time
            ax.fill_between(ns, lb, ub, color=st["color"], alpha=0.15)
            ax.plot(ns, lb, color=st["color"], marker=st["marker"],
                    linestyle=st["ls"], linewidth=1.5,
                    label=f"{label(ctrl)} (parallel lb)")
            ax.plot(ns, ub, color=st["color"], marker=st["marker"],
                    linestyle=":", linewidth=0.8, alpha=0.5,
                    label=f"{label(ctrl)} (sequential)")
        else:
            ax.plot(ns, vals, color=st["color"], marker=st["marker"],
                    linestyle=st["ls"], linewidth=1.5, label=label(ctrl))

    all_ns = sorted(set(r["n_robots"] for r in subset))
    ax.set_xticks(all_ns)
    ax.set_xlabel("Number of robots $n$")
    ax.set_ylabel("Solve time (ms, log scale)")
    ax.set_title("Solve time vs. team size\n"
                 "(distributed: shaded = parallel lb $\\ldots$ sequential ub)")
    ax.set_yscale("log")
    ax.legend(loc="upper left", fontsize=7)
    ax.grid(True, alpha=0.3, which="both")
    save_fig(fig, "01c_solve_time_distributed_bounds.pdf")
    save_note_placeholder(
        "01c_solve_time_distributed_bounds.pdf",
        "Parallel lower bound assumes perfect load balance and zero comms overhead. "
        "Upper bound is sequential single-machine cost (loose at large n). "
        "True distributed wall time lies within the shaded region.",
    )


def fig_formation_error(results):
    fig_metric_vs_n(
        results, "formation_error_mean_m", "Formation error (m)",
        "Formation error vs. team size", "02_formation_error_vs_nrobots.pdf",
    )


def fig_wall_force(results, F_wall_star):
    fig_metric_vs_n(
        results, "wall_force_mean_N", "Mean wall force (N)",
        "Wall contact force vs. team size", "03_wall_force_vs_nrobots.pdf",
        hline=F_wall_star, hline_label=f"$F_{{wall}}^*={F_wall_star:.0f}$ N",
    )


def fig_success_rate(results):
    """Meaningful only when each condition has multiple repeated runs.
    With single runs per condition this degenerates to binary dots — prefer tab_summary."""
    subset = scalability_subset(results)
    ctrls = controllers_in(subset)
    all_ns = sorted(set(r["n_robots"] for r in subset))

    # Check if we have repeated runs — warn if not
    runs_per_condition = max(
        len([r for r in subset if r["controller"] == c and r["n_robots"] == n])
        for c in ctrls for n in all_ns
        if any(r["controller"] == c and r["n_robots"] == n for r in subset)
    )

    fig, ax = plt.subplots(figsize=(5.5, 3.8))
    for ctrl in ctrls:
        rows = [r for r in subset if r["controller"] == ctrl]
        ns_u = sorted(set(r["n_robots"] for r in rows))
        vals = [float(np.mean([float(r["success"]) for r in rows if r["n_robots"] == n]))
                for n in ns_u]
        st = style(ctrl)
        ax.plot(ns_u, vals, color=st["color"], marker=st["marker"],
                linestyle=st["ls"], label=label(ctrl), linewidth=1.5)

    ax.set_xticks(all_ns)
    ax.set_yticks([0.0, 0.5, 1.0])
    ax.set_xlabel("Number of robots $n$")
    ax.set_ylabel("Success rate")
    ax.set_ylim(-0.05, 1.05)
    title = "Success rate vs. team size"
    if runs_per_condition == 1:
        title += "\n(single run — binary; use table instead)"
    ax.set_title(title, fontsize=9)
    ax.legend(loc="lower left")
    ax.grid(True, alpha=0.3)
    save_fig(fig, "04_success_rate_vs_nrobots.pdf")
    save_note_placeholder("04_success_rate_vs_nrobots.pdf",
        "NOTE: With single runs per condition this is binary (0 or 1). "
        "Only include in report if repeated runs were performed. "
        "Otherwise defer to tab_summary.tex.")


def fig_path_ratio(results):
    fig_metric_vs_n(
        results, "path_length_ratio", "Path length ratio (actual / straight-line)",
        "Trajectory efficiency vs. team size", "05_path_ratio_vs_nrobots.pdf",
        hline=1.0, hline_label="Ideal (1.0)",
    )


def fig_contact_imbalance(results, F_wall_star):
    fig_metric_vs_n(
        results, "wall_force_imbalance_N", "Contact imbalance (N, $\\sigma$ across robots)",
        "Per-robot wall-force imbalance vs. team size", "10_contact_imbalance_vs_nrobots.pdf",
    )


def fig_trajectory_comparison(results, default_n=4):
    subset = [r for r in scalability_subset(results)
              if r["n_robots"] == default_n and r.get("trajectory")]
    if not subset:
        print(f"  [skip] no trajectory data for n={default_n}")
        return

    ctrls = controllers_in(subset)
    fig, ax = plt.subplots(figsize=(5.5, 4.5))

    goal = None
    for ctrl in ctrls:
        rows = [r for r in subset if r["controller"] == ctrl]
        if not rows:
            continue
        r = rows[0]
        traj = np.array(r["trajectory"])
        goal = np.array(r["goal"][:2])
        st = style(ctrl)
        ax.plot(traj[:, 0], traj[:, 1], color=st["color"],
                linestyle=st["ls"], linewidth=1.5, label=label(ctrl))
        ax.plot(traj[0, 0], traj[0, 1], "o", color=st["color"], markersize=4)

    if goal is not None:
        ax.plot(*goal, "k*", markersize=10, label="Goal", zorder=5)

    ax.set_xlabel("$x$ (m)")
    ax.set_ylabel("$y$ (m)")
    ax.set_title(f"Payload trajectories ($n={default_n}$, $d=5$ m)")
    ax.legend(loc="best")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    save_fig(fig, "06_trajectory_comparison.pdf")
    save_note_placeholder("06_trajectory_comparison.pdf")


def _centralised_ts_runs(results, default_n):
    """Return list of result dicts for centralised controllers at default_n, scalability exp."""
    subset = [r for r in scalability_subset(results)
              if r["n_robots"] == default_n and r["controller"] not in DECENTRALISED]
    # one entry per controller (first run if repeated)
    seen = {}
    for r in subset:
        seen.setdefault(r["controller"], r)
    return list(seen.values())


def fig_wall_force_timeseries(results, F_wall_star, default_n=4):
    """07 — per-robot wall force + mean, one subplot per centralised controller."""
    runs = _centralised_ts_runs(results, default_n)
    if not runs:
        print("  [skip] no centralised runs for wall-force timeseries")
        return

    dt = 0.05
    n_ctrl = len(runs)
    fig, axes = plt.subplots(n_ctrl, 1, figsize=(6, 2.8 * n_ctrl), sharex=True)
    if n_ctrl == 1:
        axes = [axes]

    for ax, r in zip(axes, runs):
        ctrl = r["controller"]
        wf = r.get("wall_forces_ts")
        if not wf:
            ax.text(0.5, 0.5, "No force data", transform=ax.transAxes, ha="center")
            ax.set_title(label(ctrl))
            continue
        wf_arr = np.array(wf)   # (T, n_robots)
        t = np.arange(wf_arr.shape[0]) * dt
        for i in range(wf_arr.shape[1]):
            ax.plot(t, wf_arr[:, i], alpha=0.45, linewidth=0.7)
        ax.plot(t, wf_arr.mean(axis=1), "k-", linewidth=1.5, label="Mean")
        ax.axhline(F_wall_star, color="red", linestyle="--", linewidth=1,
                   label=f"$F_{{wall}}^*$")
        ax.set_ylabel("Wall force (N)")
        ax.set_title(label(ctrl))
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel("Time (s)")
    fig.suptitle(f"Per-robot wall contact force ($n={default_n}$)", y=1.01)
    fig.tight_layout()
    save_fig(fig, "07_wall_force_timeseries.pdf")
    save_note_placeholder("07_wall_force_timeseries.pdf",
        "Note which controller maintains forces near F_wall* vs drifts below it.")


def fig_wall_force_timeseries_mean(results, F_wall_star, default_n=4):
    """07b — mean wall force only, all centralised controllers on one axes."""
    runs = _centralised_ts_runs(results, default_n)
    if not runs:
        print("  [skip] no centralised runs for 07b")
        return

    dt = 0.05
    fig, ax = plt.subplots(figsize=(6, 3.5))

    for r in runs:
        ctrl = r["controller"]
        wf = r.get("wall_forces_ts")
        if not wf:
            continue
        wf_arr = np.array(wf)
        t = np.arange(wf_arr.shape[0]) * dt
        st = style(ctrl)
        ax.plot(t, wf_arr.mean(axis=1), color=st["color"], linestyle=st["ls"],
                linewidth=1.5, label=label(ctrl))

    ax.axhline(F_wall_star, color="black", linestyle=":", linewidth=1,
               label=f"$F_{{wall}}^*={F_wall_star:.0f}$ N")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Mean wall force (N)")
    ax.set_title(f"Mean wall contact force — centralised controllers ($n={default_n}$)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    save_fig(fig, "07b_wall_force_mean_timeseries.pdf")
    save_note_placeholder("07b_wall_force_mean_timeseries.pdf",
        "Cleaner comparison of mean tracking behaviour without per-robot noise.")


def fig_wall_force_timeseries_smooth(results, F_wall_star, default_n=4, kernel=11):
    """07c — median-filtered (per robot, then averaged) wall force comparison.

    Filter is applied to each robot's signal individually before averaging so that
    per-robot transient spikes are suppressed independently.
    """
    runs = _centralised_ts_runs(results, default_n)
    if not runs:
        print("  [skip] no centralised runs for 07c")
        return

    dt = 0.05
    fig, ax = plt.subplots(figsize=(6, 3.5))

    for r in runs:
        ctrl = r["controller"]
        wf = r.get("wall_forces_ts")
        if not wf:
            continue
        wf_arr = np.array(wf)   # (T, n_robots)
        # median filter each robot's signal, then average
        filtered = np.stack([medfilt(wf_arr[:, i], kernel_size=kernel)
                             for i in range(wf_arr.shape[1])], axis=1)
        t = np.arange(wf_arr.shape[0]) * dt
        st = style(ctrl)
        ax.plot(t, filtered.mean(axis=1), color=st["color"], linestyle=st["ls"],
                linewidth=1.5, label=label(ctrl))

    ax.axhline(F_wall_star, color="black", linestyle=":", linewidth=1,
               label=f"$F_{{wall}}^*={F_wall_star:.0f}$ N")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Wall force (N, smoothed)")
    ax.set_title(f"Wall force — median-filtered mean ($k={kernel}$, $n={default_n}$)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    save_fig(fig, "07c_wall_force_smooth_timeseries.pdf")
    save_note_placeholder("07c_wall_force_smooth_timeseries.pdf",
        f"Median filter kernel={kernel} steps applied per robot before averaging. "
        "Removes transient contact spikes; trend is interpretable.")


def fig_formation_error_timeseries(results, default_n=4):
    """08 — formation error over time for all centralised controllers."""
    runs = _centralised_ts_runs(results, default_n)
    if not runs:
        print("  [skip] no centralised runs for formation-error timeseries")
        return

    dt = 0.05
    fig, ax = plt.subplots(figsize=(6, 3.5))

    for r in runs:
        ctrl = r["controller"]
        fe = r.get("formation_errors_ts")
        if not fe:
            continue
        fe_arr = np.array(fe)
        t = np.arange(len(fe_arr)) * dt
        st = style(ctrl)
        ax.plot(t, fe_arr, color=st["color"], linestyle=st["ls"],
                linewidth=1.5, label=label(ctrl))

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Formation error (m)")
    ax.set_title(f"Formation tracking error — centralised controllers ($n={default_n}$)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    save_fig(fig, "08_formation_error_timeseries.pdf")
    save_note_placeholder("08_formation_error_timeseries.pdf",
        "Compare drift rate and steady-state error across all centralised controllers.")


def fig_dropout(results):
    dropout_runs = [r for r in results if r.get("experiment") == "dropout"]
    if not dropout_runs:
        print("  [skip] no dropout experiment data")
        return

    decentralised = list(dict.fromkeys(r["controller"] for r in dropout_runs))
    ns = sorted(set(r["n_robots"] for r in dropout_runs))

    fig, axes = plt.subplots(1, len(ns), figsize=(4 * len(ns), 3.8), sharey=True)
    if len(ns) == 1:
        axes = [axes]

    for ax, n in zip(axes, ns):
        for ctrl in decentralised:
            rows = [r for r in dropout_runs if r["controller"] == ctrl and r["n_robots"] == n]
            dos = sorted(set(r["dropout"] for r in rows))
            suc = [float(np.mean([float(r["success"]) for r in rows if r["dropout"] == d]))
                   for d in dos]
            st = style(ctrl)
            ax.plot(dos, suc, color=st["color"], marker=st["marker"],
                    linestyle=st["ls"], label=label(ctrl), linewidth=1.5)
        ax.set_xlabel("Dropout rate")
        ax.set_title(f"$n={n}$")
        ax.set_ylim(-0.05, 1.05)
        ax.grid(True, alpha=0.3)
        if ax is axes[0]:
            ax.set_ylabel("Success rate")
        ax.legend(fontsize=7)

    fig.suptitle("Success rate vs. message dropout (decentralised controllers)")
    fig.tight_layout()
    save_fig(fig, "09_success_vs_dropout.pdf")
    save_note_placeholder("09_success_vs_dropout.pdf",
        "Note at what dropout threshold each controller degrades.")


# ---------------------------------------------------------------------------
# LaTeX table generators
# ---------------------------------------------------------------------------

def _nanfmt(v, fmt=".2f"):
    if v is None or (isinstance(v, float) and math.isnan(v)):
        return "—"
    return format(v, fmt)


def tab_summary(results, F_wall_star, default_n=4):
    """One-row-per-controller summary at default_n robots."""
    subset = [r for r in scalability_subset(results) if r["n_robots"] == default_n]
    ctrls = controllers_in(results)

    rows = []
    for ctrl in ctrls:
        runs = [r for r in subset if r["controller"] == ctrl]
        if not runs:
            continue
        def m(key):
            vals = [r[key] for r in runs if r.get(key) is not None]
            return float(np.nanmean(vals)) if vals else float("nan")
        def s(key):
            vals = [r[key] for r in runs if r.get(key) is not None]
            return float(np.nanstd(vals)) if len(vals) > 1 else float("nan")

        suc = np.mean([float(r["success"]) for r in runs])
        rows.append({
            "ctrl": label(ctrl),
            "solve_mean": m("solve_time_mean_ms"),
            "solve_std":  s("solve_time_mean_ms"),
            "fe_mean":    m("formation_error_mean_m"),
            "wf_mean":    m("wall_force_mean_N"),
            "wf_imb":     m("wall_force_imbalance_N"),
            "path_ratio": m("path_length_ratio"),
            "success":    suc,
        })

    header = (
        r"\begin{table}[t]" + "\n"
        r"\centering" + "\n"
        r"\caption{Controller comparison at $n=" + str(default_n) + r"$ robots, "
        r"$d=5\,$m, horizon $N=15$. "
        r"Formation error: mean distance of robots from nominal slot. "
        r"Wall force: mean contact force (target $F_{wall}^*=" + f"{F_wall_star:.0f}" + r"\,$N). "
        r"Imbalance: $\sigma$ across robots, time-averaged. "
        r"Path ratio: actual length / straight-line distance.}" + "\n"
        r"\label{tab:summary}" + "\n"
        r"\begin{tabular}{lcccccc}" + "\n"
        r"\toprule" + "\n"
        r"Controller & Solve (ms) & Form.\ err.\ (m) & "
        r"Wall $F$ (N) & Imbalance (N) & Path ratio & Success \\" + "\n"
        r"\midrule"
    )
    body_lines = []
    for row in rows:
        solve = f"${_nanfmt(row['solve_mean'])} \\pm {_nanfmt(row['solve_std'])}$"
        fe    = _nanfmt(row["fe_mean"], ".3f")
        wf    = _nanfmt(row["wf_mean"], ".1f")
        imb   = _nanfmt(row["wf_imb"], ".2f")
        pr    = _nanfmt(row["path_ratio"], ".3f")
        suc   = f"{row['success']:.0%}"
        body_lines.append(f"{row['ctrl']} & {solve} & {fe} & {wf} & {imb} & {pr} & {suc} \\\\")

    footer = (
        r"\bottomrule" + "\n"
        r"\end{tabular}" + "\n"
        r"\end{table}"
    )

    tex = header + "\n" + "\n".join(body_lines) + "\n" + footer
    out = TABLE_DIR / "tab_summary.tex"
    out.write_text(tex)
    print(f"  → {out.name}")


def tab_scalability(results):
    """Solve time × n_robots table."""
    subset = scalability_subset(results)
    ctrls = controllers_in(subset)
    ns    = sorted(set(r["n_robots"] for r in subset))

    header = (
        r"\begin{table}[t]" + "\n"
        r"\centering" + "\n"
        r"\caption{Mean solve time (ms) per control step as a function of team size $n$. "
        r"All controllers use horizon $N=15$.}" + "\n"
        r"\label{tab:scalability}" + "\n"
        r"\begin{tabular}{l" + "c" * len(ns) + "}\n"
        r"\toprule" + "\n"
        "Controller & " + " & ".join(f"$n={n}$" for n in ns) + r" \\" + "\n"
        r"\midrule"
    )
    body_lines = []
    for ctrl in ctrls:
        cells = []
        for n in ns:
            runs = [r for r in subset if r["controller"] == ctrl and r["n_robots"] == n]
            if not runs:
                cells.append("—")
            else:
                mn = float(np.nanmean([r["solve_time_mean_ms"] for r in runs]))
                cells.append(f"{mn:.1f}")
        body_lines.append(f"{label(ctrl)} & " + " & ".join(cells) + r" \\")

    footer = (
        r"\bottomrule" + "\n"
        r"\end{tabular}" + "\n"
        r"\end{table}"
    )

    tex = header + "\n" + "\n".join(body_lines) + "\n" + footer
    out = TABLE_DIR / "tab_scalability.tex"
    out.write_text(tex)
    print(f"  → {out.name}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--results", required=True,
                        help="Path to JSON produced by run_report_experiments.py")
    parser.add_argument("--default-n", type=int, default=4,
                        help="Default robot count for per-controller time-series figures")
    args = parser.parse_args()

    FIG_DIR.mkdir(parents=True, exist_ok=True)
    TABLE_DIR.mkdir(parents=True, exist_ok=True)
    NOTE_DIR.mkdir(parents=True, exist_ok=True)

    results, F_wall_star = load(args.results)
    print(f"Loaded {len(results)} runs from {args.results}")
    print(f"F_wall_star = {F_wall_star} N\n")

    print("Generating figures...")
    fig_solve_time(results)
    fig_solve_time_distributed_bounds(results)
    fig_formation_error(results)
    fig_wall_force(results, F_wall_star)
    fig_success_rate(results)
    fig_path_ratio(results)
    fig_contact_imbalance(results, F_wall_star)
    fig_trajectory_comparison(results, args.default_n)
    fig_wall_force_timeseries(results, F_wall_star, args.default_n)
    fig_wall_force_timeseries_mean(results, F_wall_star, args.default_n)
    fig_wall_force_timeseries_smooth(results, F_wall_star, args.default_n)
    fig_formation_error_timeseries(results, args.default_n)
    fig_dropout(results)

    print("\nGenerating LaTeX tables...")
    tab_summary(results, F_wall_star, args.default_n)
    tab_scalability(results)

    print(f"\nDone. Output in:\n  {FIG_DIR}\n  {TABLE_DIR}")
    print("\nNext steps:")
    print("  1. Review figures visually, add notes in findings/notes/*.md")
    print("  2. Include tables in report with \\input{findings/tables/tab_summary.tex}")


if __name__ == "__main__":
    main()
