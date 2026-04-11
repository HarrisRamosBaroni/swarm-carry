#!/usr/bin/env python3
import pigpio
import time

# Choose BCM GPIO pins for SDA and SCL
SDA = 17
SCL = 27

# bus parameters
BAUD = 100000  # target clock in Hz (e.g., 100000 for 100 kHz)

pi = pigpio.pi()
if not pi.connected:
    raise SystemExit("Cannot connect to pigpiod")

# Open an I2C bit-banged master on specified GPIOs.
# Returns a handle >=0 on success
handle = pi.bb_i2c_open(SDA, SCL, BAUD)
if handle < 0:
    pi.stop()
    raise SystemExit(f"bb_i2c_open failed: {handle}")

def probe_address(addr):
    """
    Probe 7-bit address by issuing a quick write of zero bytes.
    pigpio's bb_i2c_zip can perform a START+ADDR+STOP; here we attempt a single read/write op.
    We'll use the I2C_QUICK-like behavior: try a write of zero bytes.
    """
    # Build a zip command: start + addr write + stop
    # pigpio bb_i2c_zip command bytes:
    # 0x04 = I2C_START, 0x03 = I2C_STOP, 0x01 = I2C_ADDRESS, 0x02 = I2C_READ, 0x00 = I2C_WRITE
    # Simpler: use bb_i2c_zip to write zero bytes to address (address byte supplied with I2C_ADDRESS)
    # Format: [I2C_ADDRESS, addr<<1 | 0]   then [I2C_WRITE, length, data...]  then STOP
    I2C_ADDRESS = 0x01
    I2C_WRITE = 0x00
    I2C_STOP = 0x03

    # command: set address (write) then write zero bytes then stop
    cmd = bytes([I2C_ADDRESS, (addr << 1) | 0, I2C_WRITE, 0, I2C_STOP])
    # Execute zip; returns (count, data) on success where count>=0; on NACK gets PI_BAD_I2C_CMD or negative?
    try:
        count, data = pi.bb_i2c_zip(SDA, cmd)
        # If there was an ACK to the address the operation will return successfully; count==0 is expected (no data)
        return count >= 0
    except Exception:
        return False

# Alternative simpler probe: attempt a single-byte write and check return code
def probe_addr_write(addr):
    # bb_i2c_zip returns (count, data) or raises; many use bb_i2c_zip for complex ops.
    try:
        # pigpio also offers bb_i2c_zip with write: address + write 1 byte (0x00)
        I2C_ADDRESS = 0x01
        I2C_WRITE = 0x00
        I2C_STOP = 0x03
        cmd = bytes([I2C_ADDRESS, (addr << 1) | 0, I2C_WRITE, 1, 0x00, I2C_STOP])
        count, data = pi.bb_i2c_zip(SDA, cmd)
        return count >= 0
    except Exception:
        return False

try:
    found = []
    for a in range(0x03, 0x78):
        ok = probe_addr_write(a)
        if ok:
            found.append(a)
    if found:
        for d in found:
            print(f"Found device at 0x{d:02X}")
    else:
        print("No devices found.")
finally:
    pi.bb_i2c_close(SDA)  # close bit-banged I2C on those pins
    pi.stop()
