import qwiic_i2c
from qwiic_nau7802 import QwiicNAU7802

bus_id = int(input("Enter I2C bus number: "))

driver = qwiic_i2c.get_i2c_driver(iBus=bus_id)
scale = QwiicNAU7802(i2c_driver=driver)

if not scale.is_connected():
    raise RuntimeError(f"Scale on bus {bus_id} not connected")

scale.begin()

input("Remove all weight from the scale, then press Enter...")
scale.calculate_zero_offset(64)

known_mass = float(input("Place a known weight on the scale. Enter the mass (in your chosen units): "))
scale.calculate_calibration_factor(known_mass, 64)

print(f"\nCalibration complete!")
print(f"  zeroOffset: {scale.get_zero_offset()}")
print(f"  calFactor:  {scale.get_calibration_factor()}")
