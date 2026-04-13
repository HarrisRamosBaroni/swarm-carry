"""
Run on the myAGV after calibrating:
  python3 swarmlib/sensors/force/calibrate_vals.py

Then:
  python3 real_robot/tests/test_load_cells.py --config /home/ubuntu/force_config.yaml
"""
import sys, os, argparse
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../.."))

parser = argparse.ArgumentParser()
parser.add_argument("--config", default="/home/ubuntu/force_config.yaml")
args = parser.parse_args()

from real_robot.robot.load_cell_reader import LoadCellReader

print(f"Loading config from {args.config}")
lc = LoadCellReader(config_path=args.config)
print("Taring...")
lc.tare()
print("Reading 5 samples (values should be near zero at rest):")
for i in range(5):
    r = lc.read()
    print(f"  {r}")
print("All passed. Press on the carriage and re-run to verify non-zero readings.")
