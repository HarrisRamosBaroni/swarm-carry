# nau7802_pi.py
# Minimal NAU7802 driver using pigpio bit-banged (software) I2C on any two GPIO pins.
# Requires: pigpio (daemon running)

import time
import pigpio

# NAU7802 register addresses and bit defs (subset used)
NAU7802_ADDR = 0x2A  # 7-bit address
NAU7802_ADCO_B2 = 0x12
NAU7802_ADC = 0x10
NAU7802_CTRL1 = 0x0A
NAU7802_CTRL2 = 0x0B
NAU7802_PU_CTRL = 0x00
NAU7802_PGA_PWR = 0x0D
NAU7802_PU_CTRL_RR = 1
NAU7802_PU_CTRL_PUD = 2
NAU7802_PU_CTRL_PUA = 3
NAU7802_PU_CTRL_PUR = 4
NAU7802_PU_CTRL_CS = 0
NAU7802_PU_CTRL_AVDDS = 5
NAU7802_CTRL1_VLDO = 3

# local static storage for calibration
_zero_offset = 0
_cal_factor = 1.0

class NAU7802:
    def __init__(self, pi: pigpio.pi, sda_gpio: int, scl_gpio: int):
        self.pi = pi
        self.sda = sda_gpio
        self.scl = scl_gpio
        # open a software I2C on these pins
        self.handle = pi.bb_i2c_open(self.sda, self.scl, 100000)  # 100 kHz
        if self.handle < 0:
            raise RuntimeError("bb_i2c_open failed")

    def close(self):
        self.pi.bb_i2c_close(self.sda, self.scl)

    # low-level reads/writes
    def _write_register(self, reg: int, value: int) -> bool:
        # write register via I2C mem write: send [reg, value]
        status, data = self.pi.bb_i2c_zip(self.sda, [  # use zip for raw sequence
            0x04, NAU7802_ADDR << 1,  # start + address write
            0x02, 1, reg,             # write 1 byte (reg)
            0x02, 1, value,           # write 1 byte (value)
            0x06                      # stop
        ])
        return status >= 0

    def _read_register(self, reg: int) -> int:
        # combined write(reg) then read(1)
        seq = [
            0x04, NAU7802_ADDR << 1,       # start + address write
            0x02, 1, reg,                  # write reg
            0x04, (NAU7802_ADDR << 1) | 1, # restart + address read
            0x03, 1,                       # read 1 byte
            0x06                           # stop
        ]
        status, data = self.pi.bb_i2c_zip(self.sda, seq)
        if status >= 0 and data and len(data) == 1:
            return data[0]
        return 0xFF

    def _read_registers(self, reg: int, length: int) -> bytes:
        seq = [
            0x04, NAU7802_ADDR << 1,
            0x02, 1, reg,
            0x04, (NAU7802_ADDR << 1) | 1,
            0x03, length,
            0x06
        ]
        status, data = self.pi.bb_i2c_zip(self.sda, seq)
        if status >= 0 and data and len(data) == length:
            return bytes(data)
        return bytes([0]*length)

    def is_connected(self) -> bool:
        # attempt simple read of PU_CTRL
        v = self._read_register(NAU7802_PU_CTRL)
        return v != 0xFF

    def _set_bit(self, reg: int, bit: int) -> bool:
        v = self._read_register(reg)
        if v == 0xFF: return False
        v |= (1 << bit)
        return self._write_register(reg, v)

    def _clear_bit(self, reg: int, bit: int) -> bool:
        v = self._read_register(reg)
        if v == 0xFF: return False
        v &= ~(1 << bit)
        return self._write_register(reg, v)

    def _get_bit(self, reg: int, bit: int) -> bool:
        v = self._read_register(reg)
        if v == 0xFF: return False
        return (v & (1 << bit)) != 0

    def _get24(self, reg_msb: int) -> int:
        buf = self._read_registers(reg_msb, 3)
        if len(buf) != 3:
            return 0
        unsigned32 = (buf[0] << 16) | (buf[1] << 8) | buf[2]
        # sign extend if needed
        if unsigned32 & 0x00800000:
            unsigned32 |= 0xFF000000
        return int.from_bytes(unsigned32.to_bytes(4, 'big', signed=False), 'big', signed=True)

    def available(self) -> bool:
        return self._get_bit(NAU7802_PU_CTRL, NAU7802_PU_CTRL_CR) if True else self._get_bit(NAU7802_PU_CTRL, NAU7802_PU_CTRL_PUR)

    # Basic control functions
    def reset(self) -> bool:
        if not self._set_bit(NAU7802_PU_CTRL, NAU7802_PU_CTRL_RR): return False
        time.sleep(0.001)
        if not self._clear_bit(NAU7802_PU_CTRL, NAU7802_PU_CTRL_RR): return False
        return True

    def power_up(self) -> bool:
        if not self._set_bit(NAU7802_PU_CTRL, NAU7802_PU_CTRL_PUD): return False
        if not self._set_bit(NAU7802_PU_CTRL, NAU7802_PU_CTRL_PUA): return False
        # wait for PUR
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
        ok &= self.set_ldo(3)      # 3.3V
        ok &= self.set_gain(7)     # gain 128
        ok &= self.set_sample_rate(3)  # 80 SPS
        adc = self._read_register(NAU7802_ADC)
        if adc != 0xFF:
            adc |= 0x30
            ok &= self._write_register(NAU7802_ADC, adc)
        # enable PGA cap (best-effort)
        _ = self._set_bit(NAU7802_PGA_PWR, 0) or True
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

    # calibration helpers (module-level storage)
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
        weight = (float(on_scale - _zero_offset)) / _cal_factor
        return weight
