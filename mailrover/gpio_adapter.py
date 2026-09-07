# adapters/gpio_adapter.py

import time
import config

try:
    import gpiod
    _GPIOD_VERSION = tuple(int(x) for x in gpiod.__version__.split(".")[:2])
except Exception:
    gpiod = None
    _GPIOD_VERSION = (0, 0)


class GPIOAdapter:
    """
    GPIO adapter for the Pi 5 solenoid lock design.

    Hardware model:
      - Raspberry Pi GPIO drives a MOSFET gate.
      - MOSFET switches the 12V solenoid lock.
      - Flyback diode protects the MOSFET/Pi when the coil turns off.
      - The spring lock mechanically re-locks when power is removed.

    The app calls unlock_drawer() only after an authorized action. That pulses
    the configured GPIO for UNLOCK_PULSE_SEC, then returns it to the locked
    inactive level. For the confirmed board wiring, 0 means locked/safe and
    1 means unlock.
    """

    CHIP_PATH = getattr(config, "GPIO_CHIP_PATH", "/dev/gpiochip4")
    VALID_DRAWERS = ("D1",)

    UNLOCK_LINES = {
        "D1": config.SOLENOID_UNLOCK_LINE_MAP.get("D1"),
    }
    LOCK_STATE_LINES = {
        "D1": config.LOCK_STATE_LINE_MAP.get("D1"),
    }

    def __init__(self):
        self._real_gpio = False
        self._chip = None
        self._unlock_req = {}
        self._lock_state_req = {}
        self._power_button_req = None
        self._estop_req = None
        self._real_gpio_actuators_enabled = getattr(config, "USE_REAL_GPIO_ACTUATORS", False)
        self._unlock_active_low = getattr(config, "SOLENOID_UNLOCK_ACTIVE_LOW", False)
        self._lock_state_locked_high = getattr(config, "LOCK_STATE_LOCKED_HIGH", True)
        self._power_button_gpio = getattr(config, "POWER_BUTTON_GPIO", None)
        self._power_button_active_low = getattr(config, "POWER_BUTTON_ACTIVE_LOW", False)
        self._estop_gpio = getattr(config, "ESTOP_GPIO", None)
        self._estop_stopped_high = getattr(config, "ESTOP_STOPPED_HIGH", True)
        self._drawer_unlock_state = {"D1": False}

        if gpiod is None:
            print("GPIOAdapter: gpiod not available — solenoid lock running in development mode")
            return

        if not self._real_gpio_actuators_enabled:
            print("GPIOAdapter: real solenoid GPIO disabled — running in development mode")
            return

        if not any(v is not None for v in self.UNLOCK_LINES.values()):
            print("GPIOAdapter: no solenoid GPIO lines configured")
            return

        print("GPIOAdapter: initializing Pi solenoid lock GPIO")
        self._real_gpio = True

        if _GPIOD_VERSION < (2, 0):
            self._init_gpiod_v1()
        else:
            self._init_gpiod_v2()

    def configured_actuator_drawers(self):
        return sorted(d for d, line in self.UNLOCK_LINES.items() if line is not None)

    def _init_gpiod_v1(self):
        self._chip = gpiod.Chip(self.CHIP_PATH)

        for drawer, offset in self.UNLOCK_LINES.items():
            if offset is None:
                continue
            line = self._chip.get_line(offset)
            line.request(
                consumer=f"mailrover_solenoid_{drawer}",
                type=gpiod.LINE_REQ_DIR_OUT,
                default_vals=[self._gpio_value(False)],
            )
            self._unlock_req[drawer] = line

        for drawer, offset in self.LOCK_STATE_LINES.items():
            if offset is None:
                continue
            line = self._chip.get_line(offset)
            line.request(
                consumer=f"mailrover_lock_state_{drawer}",
                type=gpiod.LINE_REQ_DIR_IN,
            )
            self._lock_state_req[drawer] = line

        if self._power_button_gpio is not None:
            line = self._chip.get_line(self._power_button_gpio)
            line.request(
                consumer="mailrover_power_button",
                type=gpiod.LINE_REQ_DIR_IN,
            )
            self._power_button_req = line

        if self._estop_gpio is not None:
            line = self._chip.get_line(self._estop_gpio)
            line.request(
                consumer="mailrover_estop",
                type=gpiod.LINE_REQ_DIR_IN,
            )
            self._estop_req = line

    def _init_gpiod_v2(self):
        self._chip = self.CHIP_PATH

        for drawer, offset in self.UNLOCK_LINES.items():
            if offset is None:
                continue
            req = gpiod.request_lines(
                self._chip,
                consumer=f"mailrover_solenoid_{drawer}",
                config={
                    offset: gpiod.LineSettings(
                        direction=gpiod.line.Direction.OUTPUT,
                        output_value=self._gpiod_value(False),
                    )
                },
            )
            self._unlock_req[drawer] = (req, offset)

        for drawer, offset in self.LOCK_STATE_LINES.items():
            if offset is None:
                continue
            req = gpiod.request_lines(
                self._chip,
                consumer=f"mailrover_lock_state_{drawer}",
                config={
                    offset: gpiod.LineSettings(
                        direction=gpiod.line.Direction.INPUT,
                    )
                },
            )
            self._lock_state_req[drawer] = (req, offset)

        if self._power_button_gpio is not None:
            req = gpiod.request_lines(
                self._chip,
                consumer="mailrover_power_button",
                config={
                    self._power_button_gpio: gpiod.LineSettings(
                        direction=gpiod.line.Direction.INPUT,
                    )
                },
            )
            self._power_button_req = (req, self._power_button_gpio)

        if self._estop_gpio is not None:
            req = gpiod.request_lines(
                self._chip,
                consumer="mailrover_estop",
                config={
                    self._estop_gpio: gpiod.LineSettings(
                        direction=gpiod.line.Direction.INPUT,
                    )
                },
            )
            self._estop_req = (req, self._estop_gpio)

    def unlock_drawer(self, drawer_id: str):
        if drawer_id not in self.VALID_DRAWERS:
            return

        if self.UNLOCK_LINES.get(drawer_id) is None:
            print(f"WARNING: no solenoid GPIO configured for drawer={drawer_id}")
            return

        self._pulse(drawer_id, getattr(config, "UNLOCK_PULSE_SEC", 0.50))

    def lock_drawer(self, drawer_id: str):
        if drawer_id not in self.VALID_DRAWERS:
            return

        self._set_unlock_line(drawer_id, False)
        self._drawer_unlock_state[drawer_id] = False

    def ensure_locked(self):
        for drawer in self.configured_actuator_drawers():
            self.lock_drawer(drawer)

    def is_drawer_unlocked(self, drawer_id: str) -> bool:
        return bool(self._drawer_unlock_state.get(drawer_id, False))

    def read_lock_state(self, drawer_id: str) -> str:
        if drawer_id not in self.VALID_DRAWERS:
            return "UNKNOWN"

        if not self._real_gpio or not self._real_gpio_actuators_enabled:
            return "UNKNOWN"

        line = self._lock_state_req.get(drawer_id)
        if line is None:
            return "UNKNOWN"

        raw = self._read_lock_state_raw(line)
        locked = raw == 1 if self._lock_state_locked_high else raw == 0
        return "LOCKED" if locked else "UNLOCKED"

    def read_power_button(self) -> dict:
        if not self._real_gpio or not self._real_gpio_actuators_enabled:
            return {
                "configured": self._power_button_gpio is not None,
                "gpio": self._power_button_gpio,
                "raw": None,
                "pressed": False,
                "state": "UNKNOWN",
            }

        if self._power_button_req is None:
            return {
                "configured": False,
                "gpio": self._power_button_gpio,
                "raw": None,
                "pressed": False,
                "state": "NOT_CONFIGURED",
            }

        raw = self._read_line_raw(self._power_button_req)
        pressed = raw == 0 if self._power_button_active_low else raw == 1
        return {
            "configured": True,
            "gpio": self._power_button_gpio,
            "raw": raw,
            "pressed": pressed,
            "state": "PRESSED" if pressed else "RELEASED",
        }

    def read_estop(self) -> dict:
        if not self._real_gpio or not self._real_gpio_actuators_enabled:
            return {
                "configured": self._estop_gpio is not None,
                "gpio": self._estop_gpio,
                "raw": None,
                "stopped": False,
                "state": "UNKNOWN",
            }

        if self._estop_req is None:
            return {
                "configured": False,
                "gpio": self._estop_gpio,
                "raw": None,
                "stopped": False,
                "state": "NOT_CONFIGURED",
            }

        raw = self._read_line_raw(self._estop_req)
        stopped = raw == 1 if self._estop_stopped_high else raw == 0
        return {
            "configured": True,
            "gpio": self._estop_gpio,
            "raw": raw,
            "stopped": stopped,
            "state": "STOPPED" if stopped else "READY",
        }

    def _read_lock_state_raw(self, line) -> int:
        return self._read_line_raw(line)

    def _read_line_raw(self, line) -> int:
        if _GPIOD_VERSION < (2, 0):
            return int(line.get_value())

        req, offset = line
        value = req.get_value(offset)
        return 1 if value == gpiod.line.Value.ACTIVE else 0

    def _pulse(self, drawer_id: str, seconds: float):
        if drawer_id not in self.VALID_DRAWERS or seconds <= 0:
            return

        print(f"GPIOAdapter: pulsing solenoid drawer={drawer_id} seconds={seconds}")
        self._drawer_unlock_state[drawer_id] = True
        self._set_unlock_line(drawer_id, True)
        time.sleep(seconds)
        self._set_unlock_line(drawer_id, False)
        self._drawer_unlock_state[drawer_id] = False

    def _set_unlock_line(self, drawer_id: str, active: bool):
        if not self._real_gpio or not self._real_gpio_actuators_enabled:
            return

        line = self._unlock_req.get(drawer_id)
        if line is None:
            return

        if _GPIOD_VERSION < (2, 0):
            line.set_value(self._gpio_value(active))
            return

        req, offset = line
        req.set_value(offset, self._gpiod_value(active))

    def _gpio_value(self, active: bool) -> int:
        if self._unlock_active_low:
            return 0 if active else 1
        return 1 if active else 0

    def _gpiod_value(self, active: bool):
        value = self._gpio_value(active)
        return gpiod.line.Value.ACTIVE if value else gpiod.line.Value.INACTIVE

    def cleanup(self):
        self.ensure_locked()
