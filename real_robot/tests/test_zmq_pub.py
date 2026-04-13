"""
Checks that the robot can bind its ZMQ PUB socket and serialise state/force
messages. Does NOT require the laptop or any other robot to be reachable.

python3 real_robot/tests/test_zmq_pub.py --config /home/ubuntu/network.yaml --id 0
"""
import sys, os, argparse, time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../.."))

import zmq
import yaml
from real_robot.transport.messages import state_msg, force_msg, unpack

parser = argparse.ArgumentParser()
parser.add_argument("--config", default="/home/ubuntu/network.yaml")
parser.add_argument("--id", type=int, default=0)
args = parser.parse_args()

with open(args.config) as f:
    cfg = yaml.safe_load(f)

my_port = next(r["pub_port"] for r in cfg["robots"] if r["id"] == args.id)

ctx = zmq.Context()
pub = ctx.socket(zmq.PUB)
pub.bind(f"tcp://*:{my_port}")
time.sleep(0.1)

# Publish a few messages and verify they serialise correctly
for i in range(3):
    raw = state_msg(args.id, float(i), 0.0, 0.1, 0.0, 0.0, 0.0)
    pub.send_multipart([b"state", raw])
    d = unpack(raw)
    assert d["id"] == args.id and d["t"] == "state"
    print(f"  published state #{i}: x={d['x']}")

raw_f = force_msg(args.id, [{"label": "horizontal", "value": 0.0},
                             {"label": "vertical",   "value": 0.0}])
pub.send_multipart([b"force", raw_f])
print(f"  published force OK")

pub.close()
print(f"ZMQ PUB on port {my_port} OK. If you have a second terminal, "
      f"you can verify receipt with: python3 -c \""
      f"import zmq,time; ctx=zmq.Context(); s=ctx.socket(zmq.SUB); "
      f"s.connect('tcp://127.0.0.1:{my_port}'); s.setsockopt_string(zmq.SUBSCRIBE,''); "
      f"time.sleep(0.2); print(s.recv_multipart(flags=zmq.NOBLOCK))\"")
