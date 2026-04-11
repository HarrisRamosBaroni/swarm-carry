# nau7802_smbus.py
# NAU7802 driver using kernel I2C via smbus2 (bus number configurable).
# Usage: from nau7802_smbus import NAU7802

import time
from smbus2 import SMBus, i2c_msg

# NAU7802 constants (7-bit address)
NAU7802_ADDR = 0x2A
NAU7802_ADCO_B2 = 0x12
NAU7802_ADC = 0x15
NAU7802_CTRL1 = 0x01
NAU7802_CTRL2 = 0x02
NAU7802_PU_CTRL = 0x00
NAU7802_PGA_PWR = 0x1C

NAU7802_PU_CTRL_RR = 0
NAU7802_PU_CTRL_PUD = 1
NAU7802_PU_CTRL_PUA = 2
NAU7802_PU_CTRL_PUR = 3
NAU7802_PU_CTRL_CS = 4
NAU7802_PU_CTRL_CR = 5
NAU7802_PU_CTRL_AVDDS = 7
NAU7802_CTRL1_VLDO = 3

# module-level calibration storage
_zero_offset = 0
_cal_factor = 1.0

class NAU7802:
    def __init__(self, bus: int = 2, address: int = NAU7802_ADDR):
        self.bus_num = bus
        self.addr = address
        self.bus = SMBus(bus)

    def close(self):
        self.bus.close()

    # low-level helpers
    def _write_register(self, reg: int, value: int) -> bool:
        try:
            self.bus.write_byte_data(self.addr, reg, value)
            return True
        except Exception:
            return False

    def _read_register(self, reg: int) -> int:
        try:
            return self.bus.read_byte_data(self.addr, reg)
        except Exception:
            return 0xFF

    def _read_registers(self, reg: int, length: int) -> bytes:
        # Combined write(register) then read(length) with repeated start
        try:
            write = i2c_msg.write(self.addr, [reg])
            read = i2c_msg.read(self.addr, length)
            self.bus.i2c_rdwr(write, read)
            return bytes(list(read))
        except Exception:
            return bytes([0]*length)

    def is_connected(self) -> bool:
        return self._read_register(NAU7802_PU_CTRL) != 0xFF

    def _set_bit(self, reg: int, bit: int) -> bool:
        v = self._read_register(reg)
        if v == 0xFF:
            return False
        v |= (1 << bit)
        return self._write_register(reg, v)

    def _clear_bit(self, reg: int, bit: int) -> bool:
        v = self._read_register(reg)
        if v == 0xFF:
            return False
        v &= ~(1 << bit)
        return self._write_register(reg, v)

    def _get_bit(self, reg: int, bit: int) -> bool:
        v = self._read_register(reg)
        if v == 0xFF:
            return False
        return (v & (1 << bit)) != 0

    def _get24(self, reg_msb: int) -> int:
        buf = self._read_registers(reg_msb, 3)
        if len(buf) != 3:
            return 0
        unsigned32 = (buf[0] << 16) | (buf[1] << 8) | buf[2]
        if unsigned32 & 0x00800000:
            unsigned32 |= 0xFF000000
        # convert to signed 32-bit
        if unsigned32 & 0x80000000:
            return -((~unsigned32 + 1) & 0xFFFFFFFF)
        return unsigned32 if unsigned32 < 0x80000000 else unsigned32 - 0x100000000

    def available(self) -> bool:
        return self._get_bit(NAU7802_PU_CTRL, NAU7802_PU_CTRL_CR)

    # High-level controls
    def reset(self) -> bool:
        if not self._set_bit(NAU7802_PU_CTRL, NAU7802_PU_CTRL_RR):
            return False
        time.sleep(0.001)
        if not self._clear_bit(NAU7802_PU_CTRL, NAU7802_PU_CTRL_RR):
            return False
        return True

    def power_up(self) -> bool:
        if not self._set_bit(NAU7802_PU_CTRL, NAU7802_PU_CTRL_PUD): return False
        if not self._set_bit(NAU7802_PU_CTRL, NAU7802_PU_CTRL_PUA): return False
        for _ in range(200):
            if self._get_bit(NAU7802_PU_CTRL, NAU7802_PU_CTRL_PUR):
                break
            time.sleep(0.001)
        return self._set_bit(NAU7802_PU_CTRL, NAU7802_PU_CTRL_CS)

    def set_ldo(self, ldo_value: int) -> bool:
        v = self._read_register(NAU7802_CTRL1)
        if v == 0xFF: return False
        v &= 0b11000111
        v |= ((ldo_value & 0x07) << NAU7802_CTRL1_VLDO)
        ok = self._write_register(NAU7802_CTRL1, v)
        if ok:
            ok = self._set_bit(NAU7802_PU_CTRL, NAU7802_PU_CTRL_AVDDS)
        return ok

    def set_gain(self, gain_value: int) -> bool:
        v = self._read_register(NAU7802_CTRL1)
        if v == 0xFF: return False
        v &= 0b11111000
        v |= (gain_value & 0x07)
        return self._write_register(NAU7802_CTRL1, v)

    def set_sample_rate(self, rate_value: int) -> bool:
        if rate_value > 0x07: rate_value = 0x07
        v = self._read_register(NAU7802_CTRL2)
        if v == 0xFF: return False
        v &= 0b10001111
        v |= (rate_value & 0x07) << 4
        return self._write_register(NAU7802_CTRL2, v)

    def begin(self) -> bool:
        if not self.is_connected():
            time.sleep(0.01)
            if not self.is_connected():
                return False
        ok = True
        ok &= self.reset()
        ok &= self.power_up()
        ok &= self.set_ldo(3)
        ok &= self.set_gain(7)
        ok &= self.set_sample_rate(3)
        adc = self._read_register(NAU7802_ADC)
        if adc != 0xFF:
            adc |= 0x30
            ok &= self._write_register(NAU7802_ADC, adc)
        # Enable PGA capacitor (bit 7 of PGA_PWR)
        try:
            self._set_bit(NAU7802_PGA_PWR, 7)
        except Exception:
            pass
        # Calibrate internal AFE (CTRL2 bit 2 = CALS; waits for it to clear)
        ok &= self._set_bit(NAU7802_CTRL2, 2)
        deadline = time.time() + 1.0
        while time.time() < deadline:
            if not self._get_bit(NAU7802_CTRL2, 2):
                break
            time.sleep(0.001)
        else:
            ok = False
        time.sleep(0.2)
        return bool(ok)

    def get_reading(self) -> int:
        return self._get24(NAU7802_ADCO_B2)

    def get_average(self, samples: int = 5, timeout_ms: int = 1000) -> int:
        total = 0
        got = 0
        start = time.time()
        while got < samples:
            if self.available():
                v = self.get_reading()
                total += v
                got += 1
            else:
                time.sleep(0.001)
            if timeout_ms and (time.time() - start) * 1000 > timeout_ms:
                break
        if got == 0:
            return 0
        return int(total / got)

    # calibration helpers
    @staticmethod
    def set_zero_offset(val: int):
        global _zero_offset
        _zero_offset = int(val)

    @staticmethod
    def get_zero_offset() -> int:
        return _zero_offset

    @staticmethod
    def set_calibration_factor(f: float):
        global _cal_factor
        _cal_factor = float(f)

    @staticmethod
    def get_calibration_factor() -> float:
        return _cal_factor

    def get_weight(self, allow_negative: bool = False, samples: int = 5, timeout_ms: int = 1000) -> float:
        on_scale = self.get_average(samples, timeout_ms)
        if not allow_negative and on_scale < _zero_offset:
            on_scale = _zero_offset
        return float(on_scale - _zero_offset) / _cal_factor
