"""
Live viewer — subscribes to mocap pose messages and draws robots, payload, and goal.

python real_robot/laptop/live_viewer.py \
    --config real_robot/config/network.yaml \
    --n-robots 2 \
    --goal 5.0 0.0 0.0

Run this alongside central_runner.py — it only reads, never writes.
Pass --goal to draw the target marker and distance readout.
"""
import argparse
import threading
import time

import yaml
import zmq
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.animation import FuncAnimation

PAYLOAD_ID = -1
ROBOT_COLORS = ["tab:blue", "tab:orange", "tab:green", "tab:red",
                "tab:purple", "tab:brown", "tab:pink", "tab:gray"]
ARROW_LEN = 0.15   # metres
ROBOT_RADIUS = 0.2


class State:
    def __init__(self, n: int):
        self.lock = threading.Lock()
        self.robot_pose = np.full((n, 3), np.nan)   # x, y, theta
        self.payload_pose = np.full(3, np.nan)       # x, y, theta
        self.n = n


def _listener(state: State, cfg: dict, n: int):
    ctx = zmq.Context.instance()
    sub = ctx.socket(zmq.SUB)
    for r in cfg["robots"][:n]:
        sub.connect(f"tcp://{r['ip']}:{r['pub_port']}")
    sub.connect(f"tcp://{cfg['laptop']['ip']}:{cfg['laptop']['mocap_pub_port']}")
    sub.setsockopt_string(zmq.SUBSCRIBE, "pose")

    import msgpack

    while True:
        try:
            _, raw = sub.recv_multipart()
            d = msgpack.unpackb(raw, raw=False)
        except Exception:
            continue
        if d.get("t") != "pose":
            continue
        rid = d.get("id", 0)
        x, y, theta = d["x"], d["y"], d["theta"]
        with state.lock:
            if rid == PAYLOAD_ID:
                state.payload_pose[:] = [x, y, theta]
            elif 0 <= rid < n:
                state.robot_pose[rid] = [x, y, theta]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="real_robot/config/network.yaml")
    parser.add_argument("--n-robots", type=int, default=2)
    parser.add_argument("--goal", type=float, nargs=3, default=None,
                        metavar=("X", "Y", "THETA"),
                        help="Goal pose to draw (world frame)")
    parser.add_argument("--goal-tol", type=float, default=0.15,
                        help="Tolerance radius to draw around goal (metres)")
    parser.add_argument("--history", type=int, default=500,
                        help="Number of payload centroid history points to keep")
    args = parser.parse_args()

    with open(args.config) as f:
        cfg = yaml.safe_load(f)

    n = args.n_robots
    goal = np.array(args.goal) if args.goal is not None else None
    state = State(n)

    t = threading.Thread(target=_listener, args=(state, cfg, n), daemon=True)
    t.start()

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_aspect("equal")
    ax.set_title("Live viewer")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.grid(True, linestyle="--", alpha=0.4)

    # Static goal elements
    if goal is not None:
        ax.plot(goal[0], goal[1], marker="*", markersize=18,
                color="gold", zorder=5, label=f"goal ({goal[0]:.2f}, {goal[1]:.2f})")
        tol_circle = plt.Circle((goal[0], goal[1]), args.goal_tol,
                                color="gold", fill=False, linestyle="--", linewidth=1.2,
                                label=f"tol {args.goal_tol*100:.0f} cm")
        ax.add_patch(tol_circle)

    # Dynamic artists — robots
    robot_circles = []
    robot_arrows = []
    robot_labels = []
    for i in range(n):
        col = ROBOT_COLORS[i % len(ROBOT_COLORS)]
        circ = plt.Circle((0, 0), ROBOT_RADIUS, color=col, alpha=0.5, zorder=3)
        ax.add_patch(circ)
        arr = ax.annotate("", xy=(0, 0), xytext=(0, 0),
                          arrowprops=dict(arrowstyle="->", color=col, lw=2), zorder=4)
        lbl = ax.text(0, 0, f"r{i}", fontsize=8, ha="center", va="center",
                      color="white", fontweight="bold", zorder=5)
        robot_circles.append(circ)
        robot_arrows.append(arr)
        robot_labels.append(lbl)

    # Payload centroid
    payload_dot, = ax.plot([], [], marker="s", markersize=12,
                           color="black", zorder=4, label="payload centroid")
    payload_arrow = ax.annotate("", xy=(0, 0), xytext=(0, 0),
                                arrowprops=dict(arrowstyle="->", color="black", lw=2), zorder=4)

    # Payload trail
    trail_x, trail_y = [], []
    trail_line, = ax.plot([], [], color="black", alpha=0.3, linewidth=1, zorder=2)

    # Distance readout
    dist_text = ax.text(0.02, 0.97, "", transform=ax.transAxes,
                        fontsize=11, va="top", ha="left",
                        bbox=dict(boxstyle="round,pad=0.3", fc="white", alpha=0.8))

    # Auto-range padding
    _PAD = 0.5
    ax.set_xlim(-2, 7)
    ax.set_ylim(-3, 3)
    ax.legend(loc="lower right", fontsize=8)

    def _update(_frame):
        with state.lock:
            rp = state.robot_pose.copy()
            pp = state.payload_pose.copy()

        all_x, all_y = [], []

        for i in range(n):
            x, y, theta = rp[i]
            if np.isnan(x):
                robot_circles[i].set_visible(False)
                robot_arrows[i].set_visible(False)
                robot_labels[i].set_visible(False)
                continue
            robot_circles[i].set_visible(True)
            robot_arrows[i].set_visible(True)
            robot_labels[i].set_visible(True)
            robot_circles[i].center = (x, y)
            robot_labels[i].set_position((x, y))
            dx, dy = np.cos(theta) * ARROW_LEN, np.sin(theta) * ARROW_LEN
            robot_arrows[i].set_position((x, y))
            robot_arrows[i].xy = (x + dx, y + dy)
            robot_arrows[i].xytext = (x, y)
            all_x.append(x); all_y.append(y)

        if not np.isnan(pp[0]):
            payload_dot.set_data([pp[0]], [pp[1]])
            payload_dot.set_visible(True)
            dx, dy = np.cos(pp[2]) * ARROW_LEN, np.sin(pp[2]) * ARROW_LEN
            payload_arrow.set_position((pp[0], pp[1]))
            payload_arrow.xy = (pp[0] + dx, pp[1] + dy)
            payload_arrow.xytext = (pp[0], pp[1])
            payload_arrow.set_visible(True)
            trail_x.append(pp[0]); trail_y.append(pp[1])
            if len(trail_x) > args.history:
                trail_x.pop(0); trail_y.pop(0)
            trail_line.set_data(trail_x, trail_y)
            all_x.append(pp[0]); all_y.append(pp[1])
        else:
            payload_dot.set_visible(False)
            payload_arrow.set_visible(False)

        # Distance readout
        if goal is not None and not np.isnan(pp[0]):
            d = float(np.linalg.norm(pp[:2] - goal[:2]))
            reached = d < args.goal_tol
            dist_text.set_text(f"dist to goal: {d*100:.1f} cm"
                               + (" ✓ REACHED" if reached else ""))
            dist_text.get_bbox_patch().set_facecolor("lightgreen" if reached else "white")
        elif goal is not None:
            dist_text.set_text("dist to goal: —")
        else:
            dist_text.set_text("")

        # Auto-expand axes
        if all_x:
            if goal is not None:
                all_x.append(goal[0]); all_y.append(goal[1])
            xmin, xmax = min(all_x) - _PAD, max(all_x) + _PAD
            ymin, ymax = min(all_y) - _PAD, max(all_y) + _PAD
            cur_xl = ax.get_xlim(); cur_yl = ax.get_ylim()
            ax.set_xlim(min(cur_xl[0], xmin), max(cur_xl[1], xmax))
            ax.set_ylim(min(cur_yl[0], ymin), max(cur_yl[1], ymax))

        return (robot_circles + robot_arrows + robot_labels +
                [payload_dot, payload_arrow, trail_line, dist_text])

    _anim = FuncAnimation(fig, _update, interval=50, blit=False)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
