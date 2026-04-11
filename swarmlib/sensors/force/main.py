# example_kernel_i2c.py
# Example using NAU7802 over kernel I2C bus 2

import time
from nau7802_smbus import NAU7802

bus = 2

scale = NAU7802(bus=bus)
try:
    if not scale.begin():
        print("NAU7802 init failed")
    else:
#        tare = scale.get_average(samples=10, timeout_ms=2000)
#        scale.set_zero_offset(tare)
        scale.set_calibration_factor(1.0)  # counts per unit (adjust)

        while True:
            w = scale.get_weight(allow_negative=True, samples=1, timeout_ms=500)
            print(f"Weight: {w:.3f}")
            time.sleep(0.5)
finally:
    scale.close()
