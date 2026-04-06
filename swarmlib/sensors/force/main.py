import pigpio
import time

# GPIO (BCM)
SDA_PIN = 27
SCL_PIN = 23

I2C_ADDR = 0x2A

# Registers
PU_CTRL   = 0x00
CTRL1     = 0x01
CTRL2     = 0x02
ADC_B2    = 0x12

# Bit positions
PU_CTRL_RR    = 0
PU_CTRL_PUD   = 1
PU_CTRL_PUA   = 2
PU_CTRL_PUR   = 3
PU_CTRL_CS    = 4
PU_CTRL_CR    = 5

CTRL2_CALMOD  = 0
CTRL2_CALS    = 2
CTRL2_CAL_ERR = 3

# pigpio setup
pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("pigpio not running")

pi.bb_i2c_open(SDA_PIN, SCL_PIN, 100000)

# ---------- I2C helpers ----------
def write_reg(reg, value):
    pi.bb_i2c_zip(SDA_PIN, [
        4, I2C_ADDR,
        2, reg, value,
        3
    ])

def read_reg(reg, length=1):
    pi.bb_i2c_zip(SDA_PIN, [
        4, I2C_ADDR,
        2, reg,
        4, I2C_ADDR | 1,
        6, length,
        3
    ])
    count, data = pi.bb_i2c_zip(SDA_PIN, [])
    return data

def set_bit(reg, bit):
    val = read_reg(reg)[0]
    write_reg(reg, val | (1 << bit))

def clear_bit(reg, bit):
    val = read_reg(reg)[0]
    write_reg(reg, val & ~(1 << bit))

# ---------- Initialisation ----------
def initialise_nau7802():
    print("Initialising NAU7802...")

    # Reset
    set_bit(PU_CTRL, PU_CTRL_RR)
    time.sleep(0.1)
    clear_bit(PU_CTRL, PU_CTRL_RR)

    # Power up digital + analog
    set_bit(PU_CTRL, PU_CTRL_PUD)
    set_bit(PU_CTRL, PU_CTRL_PUA)

    # Wait until powered up (PUR bit)
    while True:
        val = read_reg(PU_CTRL)[0]
        if val & (1 << PU_CTRL_PUR):
            break
        time.sleep(0.01)

    # Set gain = 128 (recommended for load cell)
    write_reg(CTRL1, 0x07)

    # Set sample rate = 10 SPS (stable)
    write_reg(CTRL2, 0x30)

    # Enable internal LDO (3.3V)
    set_bit(PU_CTRL, PU_CTRL_CS)

    time.sleep(0.1)

    # Start calibration
    print("Calibrating...")
    set_bit(CTRL2, CTRL2_CALS)

    # Wait for calibration to finish
    while True:
        val = read_reg(CTRL2)[0]

        if not (val & (1 << CTRL2_CALS)):
            if val & (1 << CTRL2_CAL_ERR):
                raise RuntimeError("Calibration failed")
            break

        time.sleep(0.01)

    print("Initialisation complete")

# ---------- Read functions ----------
def data_ready():
    val = read_reg(PU_CTRL)[0]
    return (val & (1 << PU_CTRL_CR)) != 0

def read_adc():
    data = read_reg(ADC_B2, 3)
    raw = (data[0] << 16) | (data[1] << 8) | data[2]

    # Convert 24-bit signed
    if raw & 0x800000:
        raw -= 1 << 24

    return raw

# ---------- Main ----------
try:
    initialise_nau7802()

    while True:
        if data_ready():
            value = read_adc()
            print(f"Raw ADC: {value}")
        else:
            print("Not ready")

        time.sleep(1)

except KeyboardInterrupt:
    pass

finally:
    pi.bb_i2c_close(SDA_PIN)
    pi.stop()