"""
Load cell reader — HX711 via GPIO on Raspberry Pi.

HARDWARE TEAM: fill in read() once load cells are wired up.
Each entry in the returned list is {"label": <str>, "value": <float, Newtons>}.
Label naming convention to discuss with software team so labels match
what the controller expects.

Example expected output for two load cells:
  [{"label": "base", "value": 42.1}, {"label": "wall_x", "value": -1.3}]
"""


class LoadCellReader:
    def __init__(self):
        # HARDWARE TEAM: initialise HX711 channels here
        # e.g.: from hx711 import HX711
        #       self._hx = HX711(dout_pin=..., pd_sck_pin=...)
        #       self._hx.set_reading_format(...)
        #       self._tare = self._hx.get_raw_data_mean()
        self._tare = 0.0  # placeholder

    def read(self) -> list:
        """
        Return list of {"label": str, "value": float} dicts.
        Blocking read — call from control loop at desired rate.
        """
        # HARDWARE TEAM: replace with real HX711 read + tare subtraction
        raise NotImplementedError("LoadCellReader.read() not yet implemented")

    def tare(self) -> None:
        """Zero out current reading. Call once at startup."""
        raise NotImplementedError
