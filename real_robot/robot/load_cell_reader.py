"""
Load cell reader — SparkFun NAU7802 Qwiic scale via I2C on Raspberry Pi.

Two load cells wired to separate I2C buses:
  - horizontal: bus 3  (shear / wall force)
  - vertical:   bus 2  (base / normal force)

Calibration values (zeroOffset, calFactor) live in a config.yaml whose
format matches swarmlib/sensors/force/config.yaml.example. Run
swarmlib/sensors/force/calibrate_vals.py once per robot to generate them.

Returns readings in the same label/value format expected by agent_runner
and force_msg():
  [{"label": "horizontal", "value": <float>},
   {"label": "vertical",   "value": <float>}]
"""
import yaml
import qwiic_i2c
from qwiic_nau7802 import QwiicNAU7802


class LoadCellReader:
    def __init__(self, config_path: str = "config.yaml"):
        with open(config_path, "r") as f:
            cfg = yaml.safe_load(f)

        self._h_scale = self._init_scale(
            bus_id=3,
            zero_offset=cfg["horizontal"]["zeroOffset"],
            cal_factor=cfg["horizontal"]["calFactor"],
        )
        self._v_scale = self._init_scale(
            bus_id=2,
            zero_offset=cfg["vertical"]["zeroOffset"],
            cal_factor=cfg["vertical"]["calFactor"],
        )

    @staticmethod
    def _init_scale(bus_id: int, zero_offset: int, cal_factor: float) -> QwiicNAU7802:
        driver = qwiic_i2c.get_i2c_driver(iBus=bus_id)
        scale = QwiicNAU7802(i2c_driver=driver)
        if not scale.is_connected():
            raise RuntimeError(f"NAU7802 on I2C bus {bus_id} not connected")
        scale.begin()
        scale.set_zero_offset(zero_offset)
        scale.set_calibration_factor(cal_factor)
        return scale

    def read(self) -> list:
        """
        Return [{"label": "horizontal", "value": float},
                {"label": "vertical",   "value": float}].
        Blocking — call from control loop at desired rate.
        """
        return [
            {"label": "horizontal", "value": self._h_scale.get_weight()},
            {"label": "vertical",   "value": self._v_scale.get_weight()},
        ]

    def tare(self) -> None:
        """Re-zero both scales against current reading. Call once at startup."""
        self._h_scale.calculate_zero_offset(64)
        self._v_scale.calculate_zero_offset(64)
