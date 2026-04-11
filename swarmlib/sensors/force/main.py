import time
import pigpio
from nau7802_pi import NAU7802

SDA_PIN = 17
SCL_PIN = 27

pi = pigpio.pi()
if not pi.connected:
    raise SystemExit("pigpio not running")

try:
    scale = NAU7802(pi, SDA_PIN, SCL_PIN)
    if not scale.begin():
        print("NAU7802 not found or failed init")
    else:
        # tare (zero) example: read average and set as zero offset
        tare_val = scale.get_average(samples=10, timeout_ms=2000)
        scale.set_zero_offset(tare_val)
        scale.set_calibration_factor(1000.0)  # example: counts per kg

        for i in range(10):
            w = scale.get_weight(allow_negative=False, samples=3, timeout_ms=500)
            print(f"Weight: {w:.3f} (units based on calibration factor)")
            time.sleep(0.5)
finally:
    scale.close()
    pi.stop()
