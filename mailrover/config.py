import os


def env_bool(name: str, default: bool = False) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() in ("1", "true", "yes", "on")


def env_int(name: str, default: int) -> int:
    value = os.environ.get(name)
    if value is None or not value.strip():
        return default
    return int(value)


def env_float(name: str, default: float) -> float:
    value = os.environ.get(name)
    if value is None or not value.strip():
        return default
    return float(value)


POLL_INTERVAL_SECONDS = 1
COMM_TIMEOUT_SECONDS = 15
LOCKOUT_SECONDS = 60
CANCEL_CUTOFF_HOURS = env_int("CANCEL_CUTOFF_HOURS", 6)
ENFORCE_DELIVERY_WINDOW = env_bool("ENFORCE_DELIVERY_WINDOW", False)
DISPATCH_LEAD_MINUTES = env_int("DISPATCH_LEAD_MINUTES", 15)
CLEAR_PENDING_ON_STARTUP = env_bool("CLEAR_PENDING_ON_STARTUP", False)

LOW_BATTERY_BLOCK_PERCENT = 20
CRITICAL_BATTERY_PERCENT = 10

HOST_LOCAL = "127.0.0.1"
HOST_BBG = "0.0.0.0"
HOST_PI = "0.0.0.0"
HOST = os.environ.get("MAILROVER_HOST", HOST_PI)
PORT = env_int("MAILROVER_PORT", 8000)

LOG_DIR = "logs"
LOG_FILE = "events.log"
DATA_DIR = os.environ.get("MAILROVER_DATA_DIR", "data")
PACKAGE_DB_PATH = os.environ.get("MAILROVER_PACKAGE_DB_PATH", os.path.join(DATA_DIR, "mailrover.sqlite3"))
EMAIL_LOGO_PATH = os.environ.get("EMAIL_LOGO_PATH", os.path.join("static", "images", "rover-logo.png"))
GPIO_CHIP_PATH = os.environ.get("GPIO_CHIP_PATH", "/dev/gpiochip4")

USE_REAL_GPIO_ACTUATORS = env_bool("USE_REAL_GPIO_ACTUATORS", False)

SALT = os.environ.get("MAILROVER_SALT", "mailrover_salt")
DEMO_PIN = "1234"

UNLOCK_PULSE_SEC = env_float("UNLOCK_PULSE_SEC", 0.50)
SOLENOID_UNLOCK_ACTIVE_LOW = env_bool("SOLENOID_UNLOCK_ACTIVE_LOW", False)
LOCK_FEEDBACK_REQUIRED = env_bool("LOCK_FEEDBACK_REQUIRED", False)
LOCK_STATE_LOCKED_HIGH = env_bool("LOCK_STATE_LOCKED_HIGH", True)
POWER_BUTTON_GPIO = env_int("POWER_BUTTON_GPIO", 10)
POWER_BUTTON_ACTIVE_LOW = env_bool("POWER_BUTTON_ACTIVE_LOW", False)
ESTOP_GPIO = env_int("ESTOP_GPIO", 18)
ESTOP_STOPPED_HIGH = env_bool("ESTOP_STOPPED_HIGH", True)
# BCM GPIO line offsets for the electronic lock board on Pi 5.
# D1 coil command output: GPIO26, physical pin 37.
# D1 lock-state input: GPIO16, physical pin 36.
SOLENOID_UNLOCK_LINE_MAP = {"D1": env_int("D1_UNLOCK_GPIO", 26)}
LOCK_STATE_LINE_MAP = {"D1": env_int("D1_LOCK_STATE_GPIO", 16)}
DRAWER_IDS = sorted(SOLENOID_UNLOCK_LINE_MAP.keys())

UART_ENABLED = False
UART_PORT = "/dev/ttyS1"
UART_BAUDRATE = 115200

# ROS2 navigation bridge. When enabled, dispatch sends tasks to a local ROS2
# bridge which translates room numbers into Nav2 goals.
NAV_BRIDGE_ENABLED = env_bool("NAV_BRIDGE_ENABLED", False)
NAV_BRIDGE_URL = os.environ.get("NAV_BRIDGE_URL", "http://127.0.0.1:8765/task")
NAV_BRIDGE_TIMEOUT_SEC = env_float("NAV_BRIDGE_TIMEOUT_SEC", 2.0)

# Email delivery. Leave SMTP_HOST empty during local development; emails will
# be written to the event log instead of sent.
EMAIL_ENABLED = env_bool("EMAIL_ENABLED", False)
SMTP_HOST = os.environ.get("SMTP_HOST", "")
SMTP_PORT = env_int("SMTP_PORT", 587)
SMTP_USERNAME = os.environ.get("SMTP_USERNAME", "")
SMTP_PASSWORD = os.environ.get("SMTP_PASSWORD", "")
SMTP_USE_TLS = env_bool("SMTP_USE_TLS", True)
EMAIL_FROM = os.environ.get("EMAIL_FROM", "MailRover <mailrover@example.com>")
PUBLIC_BASE_URL = os.environ.get("PUBLIC_BASE_URL", "").rstrip("/")

# Access control. Set these in .env for product testing.
ADMIN_PASSWORD = os.environ.get("ADMIN_PASSWORD", "").strip()
SERVICE_KEY = os.environ.get("SERVICE_KEY", "").strip()
