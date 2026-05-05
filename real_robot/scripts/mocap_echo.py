#!/usr/bin/env python3
"""
ZMQ pose listener — equivalent to 'ros2 topic echo /mocap/rigids'.

Subscribes to the mocap_pub ZMQ stream and prints incoming poses.
Run while mocap_pub.py is active.

  python -m real_robot.scripts.mocap_echo
  python -m real_robot.scripts.mocap_echo --host 192.168.1.55 --port 5560
  python -m real_robot.scripts.mocap_echo --id 2        # filter to one robot
"""
import argparse
import math
import time

import msgpack
import zmq


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=5560)
    parser.add_argument("--id", type=int, default=None,
                        help="Only show messages for this logical robot ID")
    args = parser.parse_args()

    ctx = zmq.Context.instance()
    sub = ctx.socket(zmq.SUB)
    sub.connect(f"tcp://{args.host}:{args.port}")
    sub.setsockopt(zmq.SUBSCRIBE, b"pose")

    print(f"Listening on tcp://{args.host}:{args.port}  (Ctrl+C to stop)\n")

    try:
        while True:
            topic, raw = sub.recv_multipart()
            msg = msgpack.unpackb(raw, raw=False)
            if args.id is not None and msg["id"] != args.id:
                continue
            lag = time.time() - msg["ts"]
            print(f"id={msg['id']:>3}  x={msg['x']:+.4f}  y={msg['y']:+.4f}"
                  f"  θ={math.degrees(msg['theta']):+.1f}°  lag={lag*1000:.1f}ms")
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
