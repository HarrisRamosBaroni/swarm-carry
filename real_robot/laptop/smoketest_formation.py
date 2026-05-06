"""
Formation smoketest — no MRCap controller.

1. Reads mocap poses for all robots + payload.
2. Computes and prints formation offsets (same geometry as central_runner).
3. Runs a simple proportional position controller: each robot drives toward
   its formation offset relative to the current payload centroid.
   No MPC, no horizon — just P control on world-frame position error.

Use this to verify:
  - Formation offsets are geometrically sensible (printed at start)
  - Robots hold formation without fighting (P controller exposes ID/frame issues
    without MRCap complexity)
  - World→body transform is correct under formation conditions

python real_robot/laptop/smoketest_formation.py \
    --config real_robot/config/network.yaml \
    --n-robots 2 \
    --duration 10.0

Ctrl+C sends zeros and prints a summary.
"""
import argparse
import time

import yaml
import zmq
import numpy as np
import msgpack

from real_robot.transport.messages import cmd_msg


PAYLOAD_ID = -1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="real_robot/config/network.yaml")
    parser.add_argument("--n-robots", type=int, default=2)
    parser.add_argument("--kp", type=float, default=1.0,
                        help="Proportional gain (world-frame position error → velocity)")
    parser.add_argument("--v-max", type=float, default=0.15,
                        help="Max velocity magnitude (m/s)")
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument("--duration", type=float, default=None,
                        help="Run for this many seconds (default: until Ctrl+C)")
    parser.add_argument("--verbose", action="store_true",
                        help="Print per-robot errors and commands every tick.")
    args = parser.parse_args()

    with open(args.config) as f:
        cfg = yaml.safe_load(f)

    n = args.n_robots
    dt = 1.0 / args.hz

    ctx = zmq.Context.instance()
    sub = ctx.socket(zmq.SUB)
    for r in cfg["robots"][:n]:
        sub.connect(f"tcp://{r['ip']}:{r['pub_port']}")
    sub.connect(f"tcp://{cfg['laptop']['ip']}:{cfg['laptop']['mocap_pub_port']}")
    sub.setsockopt_string(zmq.SUBSCRIBE, "pose")

    pub = ctx.socket(zmq.PUB)
    pub.bind(f"tcp://*:{cfg['laptop']['central_pub_port']}")

    robot_pose = np.full((n, 3), np.nan)   # x, y, theta per robot
    payload_pose = np.full(3, np.nan)       # x, y, theta of payload
    got_robot = np.zeros(n, dtype=bool)
    got_payload = False
    poller = zmq.Poller()
    poller.register(sub, zmq.POLLIN)

    def drain():
        nonlocal got_payload
        while dict(poller.poll(timeout=0)):
            _, raw = sub.recv_multipart()
            d = msgpack.unpackb(raw, raw=False)
            if d.get("t") != "pose":
                continue
            rid = d.get("id", 0)
            if rid == PAYLOAD_ID:
                payload_pose[:] = [d["x"], d["y"], d["theta"]]
                if not got_payload:
                    got_payload = True
                    print(f"[formation] payload:  x={d['x']:.3f}  y={d['y']:.3f}  "
                          f"θ={np.degrees(d['theta']):.1f}°")
            elif 0 <= rid < n:
                robot_pose[rid] = [d["x"], d["y"], d["theta"]]
                if not got_robot[rid]:
                    got_robot[rid] = True
                    print(f"[formation] robot {rid}: x={d['x']:.3f}  y={d['y']:.3f}  "
                          f"θ={np.degrees(d['theta']):.1f}°")

    def send_zeros():
        for i in range(n):
            pub.send_multipart([b"cmd", cmd_msg(i, 0.0, 0.0)])
        print("[formation] zero-velocity commands sent.")

    # ---- wait for all poses ----
    print(f"[formation] waiting for {n} robot(s) + payload…")
    while not (got_robot.all() and got_payload):
        drain()
        missing = ([f"robot {i}" for i in range(n) if not got_robot[i]]
                   + ([] if got_payload else ["payload"]))
        print(f"\r  waiting: {', '.join(missing)}    ", end="", flush=True)
        time.sleep(0.05)
    print()

    # ---- compute formation offsets in payload body frame ----
    px, py, pth = payload_pose
    c, s = np.cos(pth), np.sin(pth)
    R_inv = np.array([[c, s], [-s, c]])
    offsets = (robot_pose[:n, :2] - np.array([px, py])) @ R_inv.T  # (n, 2) in payload frame

    print("[formation] formation offsets (in payload body frame):")
    for i in range(n):
        ox, oy = offsets[i]
        # describe geometrically relative to payload facing direction
        print(f"  robot {i}: forward={ox:+.3f} m  left={oy:+.3f} m  "
              f"(world pos: {robot_pose[i,0]:.3f}, {robot_pose[i,1]:.3f}  "
              f"θ={np.degrees(robot_pose[i,2]):.1f}°)")
    print(f"  payload:  world pos: {px:.3f}, {py:.3f}  θ={np.degrees(pth):.1f}°")
    print()

    # ---- P-control formation hold ----
    print(f"[formation] holding formation — Kp={args.kp}  v_max={args.v_max}  "
          + (f"{args.duration}s" if args.duration else "until Ctrl+C"))

    next_tick = time.monotonic()
    t_start = time.monotonic()
    last_heartbeat = 0.0

    try:
        while True:
            drain()
            now = time.monotonic()

            if args.duration and (now - t_start) >= args.duration:
                print("[formation] duration elapsed.")
                break

            if now >= next_tick:
                # Desired world position for each robot = payload_centroid + R(pth) * offset_i
                ppx, ppy, ppth = payload_pose
                cp, sp = np.cos(ppth), np.sin(ppth)
                R = np.array([[cp, -sp], [sp, cp]])

                for i in range(n):
                    desired = np.array([ppx, ppy]) + R @ offsets[i]
                    err = desired - robot_pose[i, :2]
                    v_world = args.kp * err
                    mag = np.linalg.norm(v_world)
                    if mag > args.v_max:
                        v_world = v_world / mag * args.v_max

                    # world → robot body frame
                    th = robot_pose[i, 2]
                    cr, sr = np.cos(th), np.sin(th)
                    vx_b =  cr * v_world[0] + sr * v_world[1]
                    vy_b = -sr * v_world[0] + cr * v_world[1]

                    pub.send_multipart([b"cmd", cmd_msg(i, vx_b, vy_b)])

                    if args.verbose:
                        print(f"  r{i}: err=({err[0]:+.3f},{err[1]:+.3f})  "
                              f"v_world=({v_world[0]:+.3f},{v_world[1]:+.3f})  "
                              f"v_body=({vx_b:+.3f},{vy_b:+.3f})")

                if now - last_heartbeat >= 3.0:
                    last_heartbeat = now
                    errs = []
                    ppx, ppy, ppth = payload_pose
                    cp, sp = np.cos(ppth), np.sin(ppth)
                    R = np.array([[cp, -sp], [sp, cp]])
                    for i in range(n):
                        desired = np.array([ppx, ppy]) + R @ offsets[i]
                        errs.append(np.linalg.norm(desired - robot_pose[i, :2]))
                    err_str = "  ".join(f"r{i}={errs[i]*100:.1f}cm" for i in range(n))
                    print(f"[formation] position errors: {err_str}")

                next_tick += dt

            time.sleep(max(0.0, next_tick - time.monotonic()))

    except KeyboardInterrupt:
        print("\n[formation] interrupted.")

    send_zeros()


if __name__ == "__main__":
    main()
