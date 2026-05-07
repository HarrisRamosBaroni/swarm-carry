"""
Control panel — live pose viewer with interactive goal placement and ZMQ publishing.

Left-click on the map to place the goal XY. Use sliders to fine-tune X, Y, θ,
and tolerance. Hit "Send Goal" to publish the goal over ZMQ — central_runner and
agent_runners pick it up without restarting.

"Stop Robots" broadcasts an emergency-stop (estop) over ZMQ; all runners send
zero velocities and exit immediately.

python real_robot/laptop/control_panel.py \
    --config real_robot/config/network.yaml \
    --n-robots 2
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
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider, Button

from real_robot.transport.messages import goal_msg, estop_msg

PAYLOAD_ID = -1
ROBOT_COLORS = ["tab:blue", "tab:orange", "tab:green", "tab:red",
                "tab:purple", "tab:brown", "tab:pink", "tab:gray"]
ARROW_LEN = 0.15
PAYLOAD_ARROW_LEN = 0.45
ROBOT_RADIUS = 0.2
TRAIL_LEN = 500


class _PoseState:
    def __init__(self, n: int):
        self.lock = threading.Lock()
        self.robot_pose = np.full((n, 3), np.nan)
        self.payload_pose = np.full(3, np.nan)
        self.n = n


def _pose_listener(state: _PoseState, cfg: dict, n: int):
    import msgpack
    ctx = zmq.Context.instance()
    sub = ctx.socket(zmq.SUB)
    for r in cfg["robots"][:n]:
        sub.connect(f"tcp://{r['ip']}:{r['pub_port']}")
    sub.connect(f"tcp://{cfg['laptop']['ip']}:{cfg['laptop']['mocap_pub_port']}")
    sub.setsockopt_string(zmq.SUBSCRIBE, "pose")
    while True:
        try:
            _, raw = sub.recv_multipart()
            d = msgpack.unpackb(raw, raw=False)
        except Exception:
            continue
        if d.get("t") != "pose":
            continue
        rid = d.get("id", 0)
        with state.lock:
            if rid == PAYLOAD_ID:
                state.payload_pose[:] = [d["x"], d["y"], d["theta"]]
            elif 0 <= rid < n:
                state.robot_pose[rid] = [d["x"], d["y"], d["theta"]]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="real_robot/config/network.yaml")
    parser.add_argument("--n-robots", type=int, default=2)
    parser.add_argument("--goal-tol", type=float, default=0.15)
    args = parser.parse_args()

    with open(args.config) as f:
        cfg = yaml.safe_load(f)

    n = args.n_robots
    state = _PoseState(n)
    threading.Thread(target=_pose_listener, args=(state, cfg, n), daemon=True).start()

    ctx = zmq.Context.instance()
    pub = ctx.socket(zmq.PUB)
    pub.bind(f"tcp://*:{cfg['laptop']['goal_pub_port']}")
    time.sleep(0.1)

    # -------------------------------------------------------------------------
    # Layout: map (top 65%), sliders (next 25%), buttons (bottom 8%)
    # -------------------------------------------------------------------------
    fig = plt.figure(figsize=(10, 9))
    fig.suptitle("Goal Setter", fontsize=13, fontweight="bold")

    ax = fig.add_axes([0.05, 0.33, 0.90, 0.62])
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.set_xlim(-2, 7)
    ax.set_ylim(-3, 3)

    ax_sx  = fig.add_axes([0.15, 0.25, 0.70, 0.025])
    ax_sy  = fig.add_axes([0.15, 0.20, 0.70, 0.025])
    ax_sth = fig.add_axes([0.15, 0.15, 0.54, 0.025])
    ax_st  = fig.add_axes([0.15, 0.10, 0.70, 0.025])

    sl_x   = Slider(ax_sx,  "Goal X (m)",    -5.0, 10.0, valinit=0.0, color="gold")
    sl_y   = Slider(ax_sy,  "Goal Y (m)",    -5.0,  5.0, valinit=0.0, color="gold")
    sl_th  = Slider(ax_sth, "Goal θ (rad)", -3.14,  3.14, valinit=0.0, color="gold")
    sl_tol = Slider(ax_st,  "Tolerance (m)",  0.01,  1.0, valinit=args.goal_tol, color="lightblue")

    ax_btn_send  = fig.add_axes([0.58, 0.025, 0.28, 0.05])
    ax_btn_stop  = fig.add_axes([0.15, 0.025, 0.28, 0.05])
    ax_btn_match = fig.add_axes([0.71, 0.135, 0.16, 0.038])
    btn_send  = Button(ax_btn_send,  "Send Goal",   color="lightgreen", hovercolor="#00cc44")
    btn_stop  = Button(ax_btn_stop,  "Stop Robots", color="salmon",     hovercolor="#cc2200")
    btn_match = Button(ax_btn_match, "Match θ",     color="lightyellow", hovercolor="#ffee55")

    # -------------------------------------------------------------------------
    # Map artists
    # -------------------------------------------------------------------------
    robot_circles, robot_arrows, robot_labels = [], [], []
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

    payload_dot, = ax.plot([], [], marker="s", markersize=12, color="black",
                           zorder=4, label="payload")
    payload_arrow = ax.annotate("", xy=(0, 0), xytext=(0, 0),
                                arrowprops=dict(arrowstyle="->", color="black", lw=2), zorder=4)
    trail_x, trail_y = [], []
    trail_line, = ax.plot([], [], color="black", alpha=0.25, linewidth=1, zorder=2)

    goal_star, = ax.plot([0.0], [0.0], marker="*", markersize=20, color="gold",
                         zorder=6, label="goal (not sent)", linestyle="None")
    goal_heading = ax.annotate("", xy=(0, 0), xytext=(0, 0),
                               arrowprops=dict(arrowstyle="->", color="goldenrod", lw=2), zorder=6)
    tol_circle = plt.Circle((0.0, 0.0), args.goal_tol, color="gold",
                             fill=False, linestyle="--", linewidth=1.4, zorder=5)
    ax.add_patch(tol_circle)

    status_text = ax.text(0.02, 0.97, "click map or use sliders — then Send Goal",
                          transform=ax.transAxes, fontsize=10, va="top", ha="left",
                          bbox=dict(boxstyle="round,pad=0.3", fc="white", alpha=0.85))
    sent_text = ax.text(0.98, 0.97, "last sent: —",
                        transform=ax.transAxes, fontsize=9, va="top", ha="right",
                        color="gray",
                        bbox=dict(boxstyle="round,pad=0.2", fc="white", alpha=0.75))
    payload_theta_text = ax.text(0.50, 0.97, "payload θ: —",
                                 transform=ax.transAxes, fontsize=10, va="top", ha="center",
                                 color="black", fontweight="bold",
                                 bbox=dict(boxstyle="round,pad=0.3", fc="lightyellow", alpha=0.9))

    ax.legend(loc="lower right", fontsize=8)

    # -------------------------------------------------------------------------
    # Shared goal state
    # -------------------------------------------------------------------------
    _goal = {"x": 0.0, "y": 0.0, "theta": 0.0, "tol": args.goal_tol, "sent": False}
    _block_slider = [False]  # prevent re-entrant slider callbacks

    def _refresh_marker():
        gx, gy, gth = _goal["x"], _goal["y"], _goal["theta"]
        goal_star.set_data([gx], [gy])
        dx = np.cos(gth) * ARROW_LEN * 2
        dy = np.sin(gth) * ARROW_LEN * 2
        goal_heading.set_position((gx, gy))
        goal_heading.xy = (gx + dx, gy + dy)
        goal_heading.xytext = (gx, gy)
        tol_circle.center = (gx, gy)
        tol_circle.set_radius(_goal["tol"])
        label = "goal (sent)" if _goal["sent"] else "goal (pending)"
        goal_star.set_label(label)
        ax.legend(loc="lower right", fontsize=8)

    def _on_map_click(event):
        if event.inaxes is not ax or event.button != 1:
            return
        _goal["x"] = event.xdata
        _goal["y"] = event.ydata
        _goal["sent"] = False
        _block_slider[0] = True
        sl_x.set_val(_goal["x"])
        sl_y.set_val(_goal["y"])
        _block_slider[0] = False
        _refresh_marker()
        fig.canvas.draw_idle()

    def _on_slider(_val):
        if _block_slider[0]:
            return
        _goal["x"]     = sl_x.val
        _goal["y"]     = sl_y.val
        _goal["theta"] = sl_th.val
        _goal["tol"]   = sl_tol.val
        _goal["sent"]  = False
        _refresh_marker()
        fig.canvas.draw_idle()

    sl_x.on_changed(_on_slider)
    sl_y.on_changed(_on_slider)
    sl_th.on_changed(_on_slider)
    sl_tol.on_changed(_on_slider)
    fig.canvas.mpl_connect("button_press_event", _on_map_click)

    def _send(_event=None):
        gx, gy, gth, gtol = _goal["x"], _goal["y"], _goal["theta"], _goal["tol"]
        pub.send_multipart([b"goal", goal_msg(gx, gy, gth, gtol)])
        _goal["sent"] = True
        sent_text.set_text(f"last sent: ({gx:.2f}, {gy:.2f}, {gth:.2f} rad) tol={gtol:.2f} m")
        print(f"[control_panel] sent x={gx:.3f} y={gy:.3f} theta={gth:.3f} tol={gtol:.3f}")
        _refresh_marker()
        fig.canvas.draw_idle()

    def _stop(_event=None):
        pub.send_multipart([b"estop", estop_msg()])
        print("[control_panel] ESTOP sent")
        sent_text.set_text("ESTOP sent")
        fig.canvas.draw_idle()

    def _match_theta(_event=None):
        with state.lock:
            pp = state.payload_pose.copy()
        if not np.isnan(pp[2]):
            sl_th.set_val(pp[2])

    btn_send.on_clicked(_send)
    btn_stop.on_clicked(_stop)
    btn_match.on_clicked(_match_theta)

    # -------------------------------------------------------------------------
    # Animation loop
    # -------------------------------------------------------------------------
    _PAD = 0.5

    def _update(_frame):
        with state.lock:
            rp = state.robot_pose.copy()
            pp = state.payload_pose.copy()

        all_x = [_goal["x"]]
        all_y = [_goal["y"]]

        for i in range(n):
            x, y, theta = rp[i]
            vis = not np.isnan(x)
            robot_circles[i].set_visible(vis)
            robot_arrows[i].set_visible(vis)
            robot_labels[i].set_visible(vis)
            if vis:
                robot_circles[i].center = (x, y)
                robot_labels[i].set_position((x, y))
                dx = np.cos(theta) * ARROW_LEN
                dy = np.sin(theta) * ARROW_LEN
                robot_arrows[i].set_position((x, y))
                robot_arrows[i].xy = (x + dx, y + dy)
                robot_arrows[i].xytext = (x, y)
                all_x.append(x); all_y.append(y)

        if not np.isnan(pp[0]):
            payload_dot.set_data([pp[0]], [pp[1]])
            payload_dot.set_visible(True)
            dx = np.cos(pp[2]) * PAYLOAD_ARROW_LEN
            dy = np.sin(pp[2]) * PAYLOAD_ARROW_LEN
            payload_arrow.set_position((pp[0], pp[1]))
            payload_arrow.xy = (pp[0] + dx, pp[1] + dy)
            payload_arrow.xytext = (pp[0], pp[1])
            payload_arrow.set_visible(True)
            payload_theta_text.set_text(f"payload θ: {pp[2]:.3f} rad  ({np.degrees(pp[2]):.1f}°)")
            trail_x.append(pp[0]); trail_y.append(pp[1])
            if len(trail_x) > TRAIL_LEN:
                trail_x.pop(0); trail_y.pop(0)
            trail_line.set_data(trail_x, trail_y)
            all_x.append(pp[0]); all_y.append(pp[1])

            dist = float(np.linalg.norm(np.array([pp[0], pp[1]]) -
                                        np.array([_goal["x"], _goal["y"]])))
            reached = dist < _goal["tol"]
            pending = "" if _goal["sent"] else "  [PENDING — click Send Goal]"
            msg = f"dist to goal: {dist*100:.1f} cm{pending}"
            if reached and _goal["sent"]:
                msg += "  ✓ REACHED"
            status_text.set_text(msg)
            status_text.get_bbox_patch().set_facecolor(
                "lightgreen" if (reached and _goal["sent"]) else "white"
            )
        else:
            payload_dot.set_visible(False)
            payload_arrow.set_visible(False)
            status_text.set_text("waiting for payload pose…")
            payload_theta_text.set_text("payload θ: —")

        if all_x:
            xmin, xmax = min(all_x) - _PAD, max(all_x) + _PAD
            ymin, ymax = min(all_y) - _PAD, max(all_y) + _PAD
            cur_xl = ax.get_xlim()
            cur_yl = ax.get_ylim()
            ax.set_xlim(min(cur_xl[0], xmin), max(cur_xl[1], xmax))
            ax.set_ylim(min(cur_yl[0], ymin), max(cur_yl[1], ymax))

        return (robot_circles + robot_arrows + robot_labels +
                [payload_dot, payload_arrow, trail_line,
                 goal_star, goal_heading, status_text, sent_text, payload_theta_text])

    _anim = FuncAnimation(fig, _update, interval=50, blit=False)
    plt.show()


if __name__ == "__main__":
    main()
