"""
Smoketest: drive all robots at a fixed world-frame velocity vector.

Subscribes to mocap pose messages for each robot, rotates the commanded
world-frame velocity into each robot's body frame every tick, and sends
the result. No controller, no payload, no goal logic.

At the end, prints delta-x, delta-y, delta-theta for each robot so you
can verify the transform without watching the robots closely.

Use this to verify:
  - ZeroMQ network connectivity
  - Mocap pose stream is live
  - World-to-body frame transform is correct
    (all robots should show the same delta-x, delta-y regardless of heading)

python real_robot/laptop/smoketest_drive.py \
    --config real_robot/config/network.yaml \
    --n-robots 2 \
    --vx 0.1 --vy 0.0 \
    --duration 3.0

Ctrl+C sends zeros and exits cleanly, then prints deltas.
"""
import argparse
import sys
import threading
import time

import yaml
import zmq
import numpy as np
import msgpack

from real_robot.transport.messages import cmd_msg


def _angle_diff(a, b):
    """Signed shortest angular difference a - b, in degrees."""
    return float(np.degrees(np.arctan2(np.sin(np.radians(a) - np.radians(b)),
                                       np.cos(np.radians(a) - np.radians(b)))))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="real_robot/config/network.yaml")
    parser.add_argument("--n-robots", type=int, default=2)
    parser.add_argument("--vx", type=float, default=0.1,
                        help="World-frame x velocity (m/s)")
    parser.add_argument("--vy", type=float, default=0.0,
                        help="World-frame y velocity (m/s)")
    parser.add_argument("--hz", type=float, default=20.0,
                        help="Command rate (Hz)")
    parser.add_argument("--duration", type=float, default=None,
                        help="Stop after this many seconds (default: run until Ctrl+C)")
    parser.add_argument("--no-transform", action="store_true",
                        help="Send world-frame vx/vy directly without rotating to body frame.")
    parser.add_argument("--verbose", action="store_true",
                        help="Print theta and computed body-frame commands every tick.")
    parser.add_argument("--wait", action="store_true",
                        help="Stream live robot headings until Enter is pressed, then drive.")
    args = parser.parse_args()

    with open(args.config) as f:
        cfg = yaml.safe_load(f)

    n = args.n_robots
    dt = 1.0 / args.hz
    vx_world, vy_world = args.vx, args.vy

    ctx = zmq.Context.instance()

    sub = ctx.socket(zmq.SUB)
    for r in cfg["robots"][:n]:
        sub.connect(f"tcp://{r['ip']}:{r['pub_port']}")
    sub.connect(f"tcp://{cfg['laptop']['ip']}:{cfg['laptop']['mocap_pub_port']}")
    sub.setsockopt_string(zmq.SUBSCRIBE, "pose")

    pub = ctx.socket(zmq.PUB)
    pub.bind(f"tcp://*:{cfg['laptop']['central_pub_port']}")

    pose = np.full((n, 3), np.nan)      # current x, y, theta (radians)
    pose_start = np.full((n, 3), np.nan)
    got_pose = np.zeros(n, dtype=bool)
    poller = zmq.Poller()
    poller.register(sub, zmq.POLLIN)

    def drain():
        while dict(poller.poll(timeout=0)):
            _, raw = sub.recv_multipart()
            d = msgpack.unpackb(raw, raw=False)
            if d.get("t") != "pose":
                continue
            rid = d.get("id", 0)
            if 0 <= rid < n:
                pose[rid] = [d["x"], d["y"], d["theta"]]
                if not got_pose[rid]:
                    got_pose[rid] = True
                    pose_start[rid] = pose[rid].copy()
                    print(f"[smoketest] robot {rid} start: "
                          f"x={d['x']:.3f}  y={d['y']:.3f}  "
                          f"θ={np.degrees(d['theta']):.1f}°")

    def send_zeros():
        for i in range(n):
            pub.send_multipart([b"cmd", cmd_msg(i, 0.0, 0.0)])
        print("[smoketest] zero-velocity commands sent.")

    def print_deltas():
        print("\n[smoketest] --- displacement summary ---")
        for i in range(n):
            if np.isnan(pose_start[i, 0]):
                print(f"  r{i}: no data")
                continue
            dx = pose[i, 0] - pose_start[i, 0]
            dy = pose[i, 1] - pose_start[i, 1]
            dth = _angle_diff(np.degrees(pose[i, 2]), np.degrees(pose_start[i, 2]))
            print(f"  r{i}: Δx={dx:+.3f} m  Δy={dy:+.3f} m  Δθ={dth:+.1f}°  "
                  f"(start θ={np.degrees(pose_start[i,2]):.1f}°  "
                  f"end θ={np.degrees(pose[i,2]):.1f}°)")
        print(f"  commanded world vector: vx={vx_world}  vy={vy_world}"
              + ("  [NO TRANSFORM]" if args.no_transform else ""))

    print(f"[smoketest] waiting for {n} robot pose(s)…")
    while not got_pose.all():
        drain()
        time.sleep(0.05)

    if args.wait:
        entered = threading.Event()
        def _wait_for_enter():
            input()
            entered.set()
        threading.Thread(target=_wait_for_enter, daemon=True).start()
        print("[smoketest] orient robots then press Enter to start driving…")
        while not entered.is_set():
            drain()
            parts = "  ".join(
                f"r{i}: θ={np.degrees(pose[i, 2]):+.1f}°" for i in range(n)
            )
            print(f"\r  {parts}    ", end="", flush=True)
            time.sleep(0.1)
        print()  # newline after the live readout

    print(f"[smoketest] driving vx={vx_world} vy={vy_world} (world frame) at {args.hz} Hz"
          + (f" for {args.duration} s" if args.duration else " until Ctrl+C")
          + (" [NO TRANSFORM]" if args.no_transform else ""))

    next_tick = time.monotonic()
    t_start = time.monotonic()

    try:
        while True:
            drain()

            now = time.monotonic()
            if args.duration and (now - t_start) >= args.duration:
                print("[smoketest] duration elapsed.")
                break

            if now >= next_tick:
                for i in range(n):
                    if args.no_transform:
                        vx_b, vy_b = vx_world, vy_world
                    else:
                        c, s = np.cos(pose[i, 2]), np.sin(pose[i, 2])
                        vx_b =  c * vx_world + s * vy_world
                        vy_b = -s * vx_world + c * vy_world
                    if args.verbose:
                        print(f"  r{i}: θ={np.degrees(pose[i,2]):+.1f}°  "
                              f"world=({vx_world:.3f},{vy_world:.3f})  "
                              f"body=({vx_b:.3f},{vy_b:.3f})")
                    pub.send_multipart([b"cmd", cmd_msg(i, vx_b, vy_b)])
                next_tick += dt

            time.sleep(max(0.0, next_tick - time.monotonic()))

    except KeyboardInterrupt:
        print("\n[smoketest] interrupted.")

    send_zeros()
    print_deltas()


if __name__ == "__main__":
    main()
