"""
Run on the myAGV with myagv_ros already running:
  roslaunch myagv_ros myagv_active.launch

Then in a second terminal:
  python3 real_robot/tests/test_ros1_bridge.py
"""
import sys, os, time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../.."))

from real_robot.robot.ros1_bridge import ROS1Bridge

print("Connecting to ROS1...")
bridge = ROS1Bridge()
time.sleep(1.0)

odom = bridge.get_odom()
print(f"Odom: {odom}")
assert isinstance(odom, dict) and "x" in odom, "odom missing expected keys"
print("  odom OK")

print("Sending cmd_vel (nudge forward 0.5s)...")
bridge.send_cmd(0.1, 0.0)
time.sleep(0.5)
bridge.send_cmd(0.0, 0.0)
print("  cmd_vel OK")

print("All passed. Robot should have nudged forward briefly.")
