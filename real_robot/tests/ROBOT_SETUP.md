# Robot bring-up — myAGV

Run everything here on the robot's Raspberry Pi over SSH.
If `myagv_ros` is pre-installed on the manufacturer image then no ROS setup needed. 
Otherwise see melodic-devel branch: https://github.com/elephantrobotics/myagv_ros/tree/melodic-devel#

## 1. Get the code

```bash
git pull
pip3 install pyzmq msgpack pyyaml qwiic_nau7802 qwiic_i2c
```

## 2. Calibrate load cells (once per robot)

```bash
python3 swarmlib/sensors/force/calibrate_vals.py
```

Follow the prompts. Save the printed `zeroOffset` / `calFactor` values into `/home/ubuntu/force_config.yaml` using `swarmlib/sensors/force/config.yaml.example` as a template.

## 3. Test ROS1 bridge

Terminal 1:
```bash
roslaunch myagv_ros myagv_active.launch
```

Terminal 2:
```bash
python3 real_robot/tests/test_ros1_bridge.py
```

Expected: prints odom dict, robot nudges forward ~0.5 s, prints "All passed."

## 4. Test load cells

```bash
python3 real_robot/tests/test_load_cells.py --config /home/ubuntu/force_config.yaml
```

Expected: prints 5 readings near zero. Press on the carriage — values should change.

## 5. Test ZMQ pub (robot only, no laptop needed)

```bash
python3 real_robot/tests/test_zmq_pub.py --config /home/ubuntu/network.yaml --id 0
```

Expected: prints 3 published state messages and exits cleanly.

## 6. Test network (robot ↔ laptop)

Make sure IPs in `network.yaml` are filled in on both sides first.

**Laptop:**
```bash
python3 real_robot/tests/test_network_laptop.py --config real_robot/config/network.yaml --robot-id 0
```

**Then robot (once laptop is waiting):**
```bash
python3 real_robot/tests/test_network_robot.py --config /home/ubuntu/network.yaml --id 0
```

Expected: laptop prints a received state, robot prints a received cmd, both print PASS.
