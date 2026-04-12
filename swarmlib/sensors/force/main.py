import time
import qwiic_i2c
from qwiic_nau7802 import QwiicNAU7802

bus_id = 2
print(qwiic_i2c._drivers)
driver = qwiic_i2c.get_i2c_driver(iBus=bus_id)
print(driver)
print(driver.scan())

scale = QwiicNAU7802(i2c_driver=driver)
try:
    if scale.is_connected() == False:
        print("Not connected")
    print("12a", scale.is_connected())
    print("123a", scale._i2c.isDeviceConnected(0x2a))

    print(scale.begin())
    if False:
        pass
#    if not scale.begin():
#        print("NAU7802 init failed")
    else:
#        tare = scale.get_average(samples=10, timeout_ms=2000)
#        scale.set_zero_offset(tare)
        scale.set_calibration_factor(1.0)  # counts per unit (adjust)

        while True:
#            w = scale.get_weight(allow_negative=True, samples=1, timeout_ms=500)
            w = scale.get_reading()
            print(f"Weight: {w:.3f}")
            time.sleep(0.5)
except KeyboardInterrupt:
    print("Exiting...")
