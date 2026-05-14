"""
Sanity-check the formation geometry for each n_robots value.

Prints a table of key numbers and saves a top-down figure per n showing
robot footprints, payload, and fork geometry.

Usage:
    python check_formation.py [--n-list 2,4,6,8,10,12] [--save]
"""

import argparse
import math
import importlib.util
import sys
from pathlib import Path

import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np

# Import directly from the module file to avoid the MuJoCo-dependent __init__.py
_scene_path = Path(__file__).parents[2] / "swarmlib" / "simulation" / "generate_mecanum_scene.py"
_spec = importlib.util.spec_from_file_location("generate_mecanum_scene", _scene_path)
_scene = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_scene)

face_contact_formation_many_robots = _scene.face_contact_formation_many_robots
_FORK_WALL_REACH = _scene._FORK_WALL_REACH
_FW_WH           = _scene._FW_WH
_FB_LH           = _scene._FB_LH
LX               = _scene.LX
LY               = _scene.LY

# Robot footprint for robot-to-robot clearance checking.
# The fork points INWARD toward the payload so we only use chassis dimensions here.
# _FORK_WALL_REACH (0.39 m) is reserved for drawing and fork-coverage checks below.
ROBOT_HALF_X = LX   # chassis half-length front/back from axle centre (0.2225 m)
ROBOT_HALF_Y = LY   # chassis half-width left/right                   (0.2045 m)

DEFAULT_PAYLOAD_HX = 0.45
DEFAULT_PAYLOAD_HY = 0.45

CLEARANCE_WARN = 0.05  # warn if adjacent robots closer than this (m)


def formation_for_n(n, hx=DEFAULT_PAYLOAD_HX, hy=DEFAULT_PAYLOAD_HY):
    formation, (rec_hx, rec_hy) = face_contact_formation_many_robots(
        n, payload_hx=hx, payload_hy=hy
    )
    # Use recommended size if payload grew
    used_hx = max(hx, rec_hx)
    used_hy = max(hy, rec_hy)
    return formation, used_hx, used_hy


def robot_corners(cx, cy, yaw, half_x, half_y):
    """Return (4,2) array of robot bounding-box corners in world frame."""
    corners_local = np.array([
        [ half_x,  half_y],
        [ half_x, -half_y],
        [-half_x, -half_y],
        [-half_x,  half_y],
    ])
    c, s = math.cos(yaw), math.sin(yaw)
    R = np.array([[c, -s], [s, c]])
    return (R @ corners_local.T).T + np.array([cx, cy])


def min_inter_robot_gap(formation):
    """Minimum centre-to-centre distance between any two robots."""
    positions = np.array([(f[0], f[1]) for f in formation])
    n = len(positions)
    if n < 2:
        return float("inf")
    dists = []
    for i in range(n):
        for j in range(i + 1, n):
            dists.append(np.linalg.norm(positions[i] - positions[j]))
    return min(dists)


def robot_body_extent(formation):
    """Rough min gap between robot bounding boxes (not just centres)."""
    positions = np.array([(f[0], f[1]) for f in formation])
    yaws = [f[2] for f in formation]
    n = len(formation)
    if n < 2:
        return float("inf")
    gaps = []
    for i in range(n):
        ci = positions[i]
        for j in range(i + 1, n):
            cj = positions[j]
            centre_dist = np.linalg.norm(ci - cj)
            # conservative: subtract both half-extents in the direction of separation
            d = (cj - ci) / (centre_dist + 1e-9)
            # project robot half-extents onto separation direction
            yi, yj = yaws[i], yaws[j]
            Ri = np.array([[math.cos(yi), -math.sin(yi)], [math.sin(yi), math.cos(yi)]])
            Rj = np.array([[math.cos(yj), -math.sin(yj)], [math.sin(yj), math.cos(yj)]])
            # support function: max projection of corners onto d
            corners_i = np.array([[ ROBOT_HALF_X,  ROBOT_HALF_Y],
                                   [ ROBOT_HALF_X, -ROBOT_HALF_Y],
                                   [-ROBOT_HALF_X,  ROBOT_HALF_Y],
                                   [-ROBOT_HALF_X, -ROBOT_HALF_Y]])
            reach_i = float(np.max((Ri @ corners_i.T).T @ d))
            reach_j = float(np.max((Rj @ corners_i.T).T @ (-d)))
            gap = centre_dist - reach_i - reach_j
            gaps.append(gap)
    return min(gaps)


def fork_coverage(formation, used_hx, used_hy):
    """
    For each face, check how well the fork walls span the payload face.
    Fork wall half-width = _FW_WH = 0.20 m.
    Returns (min_coverage_fraction, min_gap_between_adjacent_forks_m).
    Coverage fraction = (n_robots_on_face * 2*_FW_WH) / face_length.
    Fork gap = spacing_between_robot_centres - 2*_FW_WH  (same-face adjacent robots).
    """
    # Group robots by face (by their dominant offset direction)
    face_robots = {"-x": [], "+x": [], "+y": [], "-y": []}
    for ox, oy, yaw in formation:
        if abs(ox) > abs(oy):
            face_robots["-x" if ox < 0 else "+x"].append((ox, oy))
        else:
            face_robots["+y" if oy > 0 else "-y"].append((ox, oy))

    face_lengths = {"-x": 2*used_hy, "+x": 2*used_hy, "+y": 2*used_hx, "-y": 2*used_hx}
    coverages, fork_gaps = [], []

    for face, robots in face_robots.items():
        if not robots:
            continue
        n_f = len(robots)
        face_len = face_lengths[face]
        total_fork_width = n_f * 2 * _FW_WH
        coverages.append(total_fork_width / face_len)

        if n_f >= 2:
            # Sort by lateral position and check adjacent spacing
            lateral = sorted(r[1] if face in ("-x", "+x") else r[0] for r in robots)
            for a, b in zip(lateral, lateral[1:]):
                fork_gaps.append((b - a) - 2 * _FW_WH)

    return (
        min(coverages) if coverages else 1.0,
        min(fork_gaps)  if fork_gaps  else float("inf"),
    )


def analyse(n, hx=DEFAULT_PAYLOAD_HX, hy=DEFAULT_PAYLOAD_HY):
    formation, used_hx, used_hy = formation_for_n(n, hx, hy)
    payload_grew = (used_hx > hx or used_hy > hy)
    min_centre_dist = min_inter_robot_gap(formation)
    min_bbox_gap    = robot_body_extent(formation)
    min_coverage, min_fork_gap = fork_coverage(formation, used_hx, used_hy)
    ok = min_bbox_gap > CLEARANCE_WARN and min_fork_gap > -0.01
    return {
        "n": n,
        "payload_hx": used_hx,
        "payload_hy": used_hy,
        "payload_grew": payload_grew,
        "min_centre_dist_m": min_centre_dist,
        "min_bbox_gap_m": min_bbox_gap,
        "min_coverage": min_coverage,
        "min_fork_gap_m": min_fork_gap,
        "ok": ok,
        "formation": formation,
    }


def print_table(analyses):
    print(f"\n{'n':>3}  {'payload (m)':>14}  {'grew':>5}  "
          f"{'chassis gap':>11}  {'fork coverage':>13}  {'same-face fork gap':>18}  {'status':>12}")
    print("-" * 90)
    for a in analyses:
        grew    = "YES" if a["payload_grew"] else "no"
        status  = "OK" if a["ok"] else "PROBLEM"
        fc      = f"{a['min_coverage']:.0%}"
        fg      = f"{a['min_fork_gap_m']:.3f}m" if a["min_fork_gap_m"] < 1e5 else "  (1 per face)"
        print(
            f"{a['n']:>3}  "
            f"{a['payload_hx']:.3f} × {a['payload_hy']:.3f}  "
            f"{grew:>5}  "
            f"{a['min_bbox_gap_m']:>11.3f}m  "
            f"{fc:>13}  "
            f"{fg:>18}  "
            f"{status:>12}"
        )
    print()
    print("  chassis gap   : clearance between robot bodies (forks point inward, excluded)")
    print("  fork coverage : fraction of payload face covered by fork walls (want ≥ 1.0)")
    print("  fork gap      : gap between adjacent fork walls on same face (want > 0)")
    print()


def draw_formation(ax, formation, used_hx, used_hy, n):
    # Payload
    payload = mpatches.Rectangle(
        (-used_hx, -used_hy), 2 * used_hx, 2 * used_hy,
        linewidth=1.5, edgecolor="steelblue", facecolor="lightblue", alpha=0.5,
        label="Payload",
    )
    ax.add_patch(payload)

    for i, (ox, oy, yaw) in enumerate(formation):
        corners = robot_corners(ox, oy, yaw, ROBOT_HALF_X, ROBOT_HALF_Y)
        poly = mpatches.Polygon(corners, closed=True,
                                linewidth=1, edgecolor="darkorange",
                                facecolor="wheat", alpha=0.7)
        ax.add_patch(poly)

        # Fork wall indicator (thick line at front of robot)
        c, s = math.cos(yaw), math.sin(yaw)
        fw_centre = np.array([ox + c * _FORK_WALL_REACH,
                               oy + s * _FORK_WALL_REACH])
        perp = np.array([-s, c])
        fw_l = fw_centre - perp * _FW_WH
        fw_r = fw_centre + perp * _FW_WH
        ax.plot([fw_l[0], fw_r[0]], [fw_l[1], fw_r[1]],
                color="red", linewidth=2, zorder=3)

        # Robot label
        ax.text(ox, oy, str(i), ha="center", va="center", fontsize=7, fontweight="bold")

    # Axes formatting
    margin = 1.0
    all_x = [f[0] for f in formation]
    all_y = [f[1] for f in formation]
    ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
    ax.set_ylim(min(all_y) - margin, max(all_y) + margin)
    ax.set_aspect("equal")
    ax.set_title(f"n={n}  payload {2*used_hx:.2f}×{2*used_hy:.2f} m", fontsize=9)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.grid(True, alpha=0.3)

    # Legend proxy
    ax.plot([], [], color="darkorange", linewidth=2, label="Robot bbox")
    ax.plot([], [], color="red",        linewidth=2, label="Fork wall")
    ax.legend(fontsize=7, loc="upper right")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--n-list", default="2,4,6,8,10,12",
                        help="Comma-separated robot counts to check")
    parser.add_argument("--save", action="store_true",
                        help="Save figure to findings/figures/ instead of showing")
    args = parser.parse_args()

    ns = [int(x.strip()) for x in args.n_list.split(",")]

    analyses = [analyse(n) for n in ns]
    print_table(analyses)

    # Warn on problematic cases
    bad = [a for a in analyses if not a["ok"]]
    if bad:
        print(f"WARNING: {len(bad)} configuration(s) have tight clearance (<{CLEARANCE_WARN}m):")
        for a in bad:
            print(f"  n={a['n']}  bbox gap={a['min_bbox_gap_m']:.3f}m")
    else:
        print("All configurations have adequate clearance.")

    # Figure
    ncols = min(3, len(ns))
    nrows = math.ceil(len(ns) / ncols)
    fig, axes = plt.subplots(nrows, ncols, figsize=(5 * ncols, 4.5 * nrows))
    axes_flat = np.array(axes).flatten() if len(ns) > 1 else [axes]

    for ax, a in zip(axes_flat, analyses):
        draw_formation(ax, a["formation"], a["payload_hx"], a["payload_hy"], a["n"])

    for ax in axes_flat[len(ns):]:
        ax.set_visible(False)

    fig.suptitle("Formation geometry check — top-down view\n"
                 "Blue: payload  |  Orange: robot bbox  |  Red: fork wall",
                 fontsize=10)
    fig.tight_layout()

    if args.save:
        out = Path("/home/harris/Documents/y3/dot/final_report/0225_DOT_Report_2/findings/figures/formation_geometry_check.pdf")
        fig.savefig(out, bbox_inches="tight")
        print(f"Saved → {out}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
