import pigpio
import time

# Pin configuration (BCM numbering)
SDA = 0
SCL = 11

I2C_ADDR = 0x2A  # NAU7802 default address

# NAU7802 registers
REG_PU_CTRL = 0x00
REG_ADCO_B2 = 0x12  # MSB of ADC result (3 bytes total)

pi = pigpio.pi()

if not pi.connected:
    raise RuntimeError("Could not connect to pigpio daemon")

# Open bit-banged I2C
bb_i2c = pi.bb_i2c_open(SDA, SCL, 50000)  # 50 kHz

if bb_i2c != 0:
    raise RuntimeError("Failed to open bit-banged I2C")

def i2c_read(register, count):
    # Write register address, then read
    (count_written, _, _) = pi.bb_i2c_zip(SDA, [
        4, I2C_ADDR,       # Set device address (write)
        2, register,       # Write register address
        4, I2C_ADDR | 1,   # Repeated start, read mode
        6, count,          # Read count bytes
        3                  # Stop
    ])
    return _

def read_adc_raw():
    data = i2c_read(REG_ADCO_B2, 3)
    if len(data) != 3:
        return None

    # Combine 24-bit value
    value = (data[0] << 16) | (data[1] << 8) | data[2]

    # Convert signed 24-bit
    if value & 0x800000:
        value -= 1 << 24

    return value

def initialise_scale():
    # Power up digital + analog
    pi.bb_i2c_zip(SDA, [
        4, I2C_ADDR,
        2, REG_PU_CTRL,
        2, 0x06,  # PUD + PUA bits
        3
    ])
    time.sleep(0.1)


try:
    # initialise_scale()
    
    while True:
        raw = read_adc_raw()
        if raw is not None:
            print(f"Raw ADC: {raw}")
        else:
            print("Read failed")

        time.sleep(0.5)

except KeyboardInterrupt:
    pass

finally:
    pi.bb_i2c_close(SDA)
    pi.stop()