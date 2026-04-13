"""
Network connectivity test — laptop side.

Start this first, then run test_network_robot.py on the robot.

python3 real_robot/tests/test_network_laptop.py --config real_robot/config/network.yaml --robot-id 0
"""
import sys, os, argparse, time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../.."))

import zmq, yaml
from real_robot.transport.messages import cmd_msg, unpack

parser = argparse.ArgumentParser()
parser.add_argument("--config", default="real_robot/config/network.yaml")
parser.add_argument("--robot-id", type=int, default=0)
args = parser.parse_args()

with open(args.config) as f:
    cfg = yaml.safe_load(f)

robot     = next(r for r in cfg["robots"] if r["id"] == args.robot_id)
rob_ip    = robot["ip"]
rob_port  = robot["pub_port"]
lap_port  = cfg["laptop"]["central_pub_port"]

ctx = zmq.Context()

sub = ctx.socket(zmq.SUB)
sub.connect(f"tcp://{rob_ip}:{rob_port}")
sub.setsockopt_string(zmq.SUBSCRIBE, "state")

pub = ctx.socket(zmq.PUB)
pub.bind(f"tcp://*:{lap_port}")

print(f"Listening for state from robot {args.robot_id} at {rob_ip}:{rob_port}...")
print("(start test_network_robot.py on the robot now)")

sub.setsockopt(zmq.RCVTIMEO, 15000)
try:
    topic, raw = sub.recv_multipart()
    d = unpack(raw)
    assert d["t"] == "state", f"unexpected message type: {d['t']}"
    print(f"  received state: x={d['x']:.2f} y={d['y']:.2f}")
except zmq.Again:
    print("FAIL — no state received within 15s. Check robot is running and IPs are correct.")
    sys.exit(1)

print(f"Sending cmd to robot on port {lap_port}...")
time.sleep(0.3)
pub.send_multipart([b"cmd", cmd_msg(args.robot_id, 0.1, 0.0)])
time.sleep(0.2)

print("PASS")
pub.close(); sub.close()
