import fcntl
import os
import statistics
import threading
import time
from collections import deque


I2C_SLAVE = 0x0703


class BatteryADC:
    """Read the MailRover battery ADC and estimate 12 V LiFePO4 SOC."""

    # Resting-voltage curve based on the supplied 12 V LiFePO4 chart.
    # The chart lists both 40% and 50% at 13.0 V, represented as 45%.
    SOC_CURVE_MV = (
        (10000, 0),
        (12000, 10),
        (12800, 20),
        (12900, 30),
        (13000, 45),
        (13100, 60),
        (13200, 70),
        (13300, 80),
        (13400, 90),
        (13600, 100),
    )

    def __init__(self):
        self.enabled = self._env_bool("BATTERY_ADC_ENABLED", True)
        self.device = os.environ.get("BATTERY_I2C_DEVICE", "/dev/i2c-1")
        self.address = int(os.environ.get("BATTERY_ADC_ADDRESS", "0x48"), 0)
        self.poll_interval = max(
            1.0,
            float(os.environ.get("BATTERY_ADC_POLL_SECONDS", "5")),
        )
        self.sample_count = max(
            1,
            int(os.environ.get("BATTERY_ADC_SAMPLE_COUNT", "5")),
        )
        self.voltage_scale = float(
            os.environ.get("BATTERY_ADC_VOLTAGE_SCALE", "1.0")
        )
        self.minimum_valid_mv = int(
            os.environ.get("BATTERY_ADC_MIN_VALID_MV", "9000")
        )
        self.maximum_valid_mv = int(
            os.environ.get("BATTERY_ADC_MAX_VALID_MV", "15000")
        )

        self._lock = threading.Lock()
        self._voltage_history = deque(maxlen=3)
        self._last_read_at = 0.0
        self._last_result = {
            "ok": False,
            "counts": None,
            "voltage_mv": None,
            "percent": None,
            "error": "NOT_READ_YET",
        }

    @staticmethod
    def _env_bool(name: str, default: bool) -> bool:
        value = os.environ.get(name)
        if value is None:
            return default
        return value.strip().lower() in ("1", "true", "yes", "on")

    @classmethod
    def voltage_to_percent(cls, voltage_mv: int) -> int:
        curve = cls.SOC_CURVE_MV
        if voltage_mv <= curve[0][0]:
            return curve[0][1]
        if voltage_mv >= curve[-1][0]:
            return curve[-1][1]

        for (low_mv, low_percent), (high_mv, high_percent) in zip(
            curve,
            curve[1:],
        ):
            if low_mv <= voltage_mv <= high_mv:
                fraction = (voltage_mv - low_mv) / (high_mv - low_mv)
                percent = low_percent + fraction * (high_percent - low_percent)
                return max(0, min(100, int(round(percent))))

        return 0

    def _read_once(self):
        descriptor = os.open(self.device, os.O_RDWR)
        try:
            fcntl.ioctl(descriptor, I2C_SLAVE, self.address)
            payload = os.read(descriptor, 2)
        finally:
            os.close(descriptor)

        if len(payload) != 2:
            raise OSError(f"ADC returned {len(payload)} bytes instead of 2")

        counts = (payload[0] << 8) | payload[1]
        if counts > 0x0FFF:
            raise ValueError(f"ADC count out of range: 0x{counts:04x}")

        # The two least-significant count bits are random. Multiplication and
        # masking produce the specified 16 mV voltage resolution.
        voltage_mv = (counts << 2) & ~0x0F
        voltage_mv = int(round(voltage_mv * self.voltage_scale))

        if not self.minimum_valid_mv <= voltage_mv <= self.maximum_valid_mv:
            raise ValueError(
                f"battery voltage outside plausible range: {voltage_mv}mV"
            )

        return counts, voltage_mv

    def read(self, force: bool = False) -> dict:
        with self._lock:
            current_time = time.monotonic()
            if (
                not force
                and current_time - self._last_read_at < self.poll_interval
            ):
                return dict(self._last_result)

            self._last_read_at = current_time

            if not self.enabled:
                self._last_result = {
                    "ok": False,
                    "counts": None,
                    "voltage_mv": None,
                    "percent": None,
                    "error": "DISABLED",
                }
                return dict(self._last_result)

            try:
                samples = []
                counts_samples = []
                for sample_index in range(self.sample_count):
                    counts, voltage_mv = self._read_once()
                    counts_samples.append(counts)
                    samples.append(voltage_mv)
                    if sample_index + 1 < self.sample_count:
                        time.sleep(0.02)

                sample_voltage_mv = int(statistics.median(samples))
                self._voltage_history.append(sample_voltage_mv)
                filtered_voltage_mv = int(
                    statistics.median(self._voltage_history)
                )
                filtered_percent = self.voltage_to_percent(filtered_voltage_mv)

                self._last_result = {
                    "ok": True,
                    "counts": int(statistics.median(counts_samples)),
                    "voltage_mv": filtered_voltage_mv,
                    "percent": filtered_percent,
                    "error": "",
                }
            except Exception as error:
                self._last_result = {
                    "ok": False,
                    "counts": None,
                    "voltage_mv": None,
                    "percent": None,
                    "error": str(error),
                }

            return dict(self._last_result)
