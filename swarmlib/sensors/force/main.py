import time
import yaml
import qwiic_i2c
from qwiic_nau7802 import QwiicNAU7802


def load_config(path="config.yaml"):
    with open(path, "r") as f:
        return yaml.safe_load(f)


def init_scale(bus_id, zero_offset, cal_factor):
    driver = qwiic_i2c.get_i2c_driver(iBus=bus_id)
    scale = QwiicNAU7802(i2c_driver=driver)
    if not scale.is_connected():
        raise RuntimeError(f"Scale on bus {bus_id} not connected")
    scale.begin()
    scale.set_zero_offset(zero_offset)
    scale.set_calibration_factor(cal_factor)
    return scale


def main():
    config = load_config("config.yaml")

    h_cfg = config["horizontal"]
    v_cfg = config["vertical"]

    print("Initialising horizontal load cell (bus 3)...")
    h_scale = init_scale(
        bus_id=3,
        zero_offset=h_cfg["zeroOffset"],
        cal_factor=h_cfg["calFactor"],
    )

    print("Initialising vertical load cell (bus 2)...")
    v_scale = init_scale(
        bus_id=2,
        zero_offset=v_cfg["zeroOffset"],
        cal_factor=v_cfg["calFactor"],
    )

    print("Reading force data. Press Ctrl+C to stop.\n")
    try:
        while True:
            h_reading = h_scale.get_weight(allow_negative=True, samples=1, timeout_ms=500)
            v_reading = v_scale.get_weight(allow_negative=True, samples=1, timeout_ms=500)
            print(f"Horizontal: {h_reading:.3f}  |  Vertical: {v_reading:.3f}")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nExiting...")


if __name__ == "__main__":
    main()
