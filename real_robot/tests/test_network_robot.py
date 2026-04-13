"""
Network connectivity test — robot side.

Run AFTER starting test_network_laptop.py on the laptop.

python3 real_robot/tests/test_network_robot.py --config /home/ubuntu/network.yaml --id 0
"""
import sys, os, argparse, time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../.."))

import zmq, yaml
from real_robot.transport.messages import state_msg, cmd_msg, unpack

parser = argparse.ArgumentParser()
parser.add_argument("--config", default="/home/ubuntu/network.yaml")
parser.add_argument("--id", type=int, default=0)
args = parser.parse_args()

with open(args.config) as f:
    cfg = yaml.safe_load(f)

my_port  = next(r["pub_port"] for r in cfg["robots"] if r["id"] == args.id)
lap_ip   = cfg["laptop"]["ip"]
lap_port = cfg["laptop"]["central_pub_port"]

ctx = zmq.Context()

pub = ctx.socket(zmq.PUB)
pub.bind(f"tcp://*:{my_port}")

sub = ctx.socket(zmq.SUB)
sub.connect(f"tcp://{lap_ip}:{lap_port}")
sub.setsockopt_string(zmq.SUBSCRIBE, "cmd")

time.sleep(0.5)  # let connections establish

print(f"Publishing state on port {my_port}...")
for _ in range(5):
    pub.send_multipart([b"state", state_msg(args.id, 1.0, 2.0, 0.0, 0.0, 0.0, 0.0)])
    time.sleep(0.1)

print("Waiting for cmd from laptop (10s timeout)...")
sub.setsockopt(zmq.RCVTIMEO, 10000)
try:
    topic, raw = sub.recv_multipart()
    d = unpack(raw)
    assert d["t"] == "cmd", f"unexpected message type: {d['t']}"
    print(f"  received cmd: vx={d['vx']:.2f} vy={d['vy']:.2f}")
    print("PASS")
except zmq.Again:
    print("FAIL — no cmd received within 10s. Check laptop is running and IPs are correct.")
finally:
    pub.close(); sub.close()
