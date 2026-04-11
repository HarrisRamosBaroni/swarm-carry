import time
from qwiic_i2c import I2CDriver
from qwiic_nau7802 import QwiicNAU7802

bus = 2
driver = I2CDriver(bus)

scale = QwiicNAU7802(driver)
scale.begin()
try:
    if scale.is_connected() == False:
        print("Not connected")
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
except KeyboardInterrupt:
    print("Exiting...")
