#!/usr/bin/env python3
import time
import zmq

from real_robot.transport.messages import cmd_msg

# Hard-coded network info (replace with your robot/laptop addresses)
TARGET_IP = "192.168.1.121"          # robot or relay IP to connect to
TARGET_PORT = 5561                # central_pub_port or robot PUB port
TOPIC = b"cmd"
ROBOT_ID = 1

# Desired body-frame velocities (m/s)
vx = 0.5
vy = 0.0

ctx = zmq.Context()
pub = ctx.socket(zmq.PUB)
pub.connect(f"tcp://{TARGET_IP}:{TARGET_PORT}")

# Short pause so the socket has time to connect (PUB/SUB needs it)
time.sleep(0.1)

# Send one command
pub.send_multipart([TOPIC, cmd_msg(ROBOT_ID, vx, vy)])
print(f"sent cmd to r{ROBOT_ID}: vx={vx} vy={vy} -> tcp://{TARGET_IP}:{TARGET_PORT}")

# Optionally send zeros after a short delay to stop robot
time.sleep(2.5)
pub.send_multipart([TOPIC, cmd_msg(ROBOT_ID, 0.0, 0.0)])
print("sent zero-velocity command")

pub.close()
ctx.term()