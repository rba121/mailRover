# services/uart_bridge.py

import threading
import time
import serial


class UARTBridge:
    """
    UART bridge between BBG Flask app and Raspberry Pi ROS side.

    Responsibilities:
      - send task/status messages from BBG to Pi
      - listen for Pi status updates
      - invoke callback(message_dict) when valid messages arrive

    Message format (BBG -> Pi):
      TASK_CREATE|<task_id>|<destination>|<drawer_id>   (4 parts)
      TASK_CANCEL|<task_id>                             (2 parts)
      DELIVERY_COMPLETE|<task_id>                       (2 parts)

    Message format (Pi -> BBG):
      STATUS|<task_id>|<status>                         (3 parts)
      where status is one of: MOVING, ARRIVED, FAULT, BLOCKED, CANCELED, IDLE,
      LOCKED, UNLOCKED, LOCK_UNLOCKED, LOCK_LOCKED
      LOCK|<task_id>|<state>                            (3 parts)
      where state is one of: LOCKED, UNLOCKED

    Note: _parse_line handles inbound STATUS and LOCK messages (3-part).
    Outbound messages are never parsed — if the Pi echoes them back they
    will be logged as UART_PARSE_IGNORE which is expected and harmless.
    """

    # FIX 1: default port documented — in production this is always
    # overridden by config.UART_PORT passed from app.py.
    def __init__(self, port="/dev/ttyS1", baudrate=115200, on_message=None, logger=None):
        self.port = port
        self.baudrate = baudrate
        self.on_message = on_message
        self.logger = logger

        self.ser = None
        self.running = False
        self.thread = None

        # FIX 3: lock to protect self.ser from concurrent access between
        # the Flask send threads and the _read_loop thread. Without this
        # there is a race where send_line passes the "if not self.ser" check
        # and then _read_loop sets self.ser = None before the write, causing
        # an AttributeError crash.
        self._serial_lock = threading.Lock()

    def start(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.2)

            # FIX 2: set self.running = True only after Serial opens successfully
            # and just before thread.start(). Previously it was set before
            # thread.start(), so a rare start() failure would leave running=True
            # with no thread actually running, making the system think UART was alive.
            self.running = True
            self.thread = threading.Thread(target=self._read_loop, daemon=True)
            self.thread.start()
            self._log("UART_START", f"Opened {self.port} @ {self.baudrate}")
        except Exception as e:
            self.running = False
            self.ser = None
            self._log("UART_ERROR", f"Failed to open UART: {e}")

    def stop(self):
        self.running = False
        with self._serial_lock:
            if self.ser:
                try:
                    self.ser.close()
                except Exception:
                    pass
            self.ser = None
        self._log("UART_STOP", "UART bridge stopped")

    def send_line(self, line: str) -> bool:
        # FIX 3: acquire lock before checking and using self.ser so the
        # _read_loop thread cannot set it to None between the check and write.
        with self._serial_lock:
            if not self.ser:
                self._log("UART_SEND_FAIL", f"UART not open, line={line}")
                return False

            try:
                payload = (line.strip() + "\n").encode("utf-8")
                self.ser.write(payload)
                self.ser.flush()
                self._log("UART_TX", line.strip())
                return True
            except Exception as e:
                self._log("UART_SEND_FAIL", f"{e}; line={line}")
                return False

    def send_task_create(self, task_id: str, destination: str, drawer_id: str) -> bool:
        return self.send_line(f"TASK_CREATE|{task_id}|{destination}|{drawer_id}")

    def send_task_cancel(self, task_id: str) -> bool:
        return self.send_line(f"TASK_CANCEL|{task_id}")

    def send_delivery_complete(self, task_id: str) -> bool:
        return self.send_line(f"DELIVERY_COMPLETE|{task_id}")

    def _read_loop(self):
        buffer = ""

        while self.running:
            # Check if serial port is still open under lock
            with self._serial_lock:
                ser_open = self.ser is not None

            if not ser_open:
                break

            try:
                # Read outside the lock so we don't hold it during blocking I/O
                data = self.ser.read(128)
                if not data:
                    time.sleep(0.05)
                    continue

                buffer += data.decode("utf-8", errors="ignore")

                # FIX 5: guard against unbounded buffer growth if the Pi sends
                # malformed data with no newline terminator. Without this the
                # buffer grows indefinitely with every read(128) call.
                if len(buffer) > 1024:
                    self._log("UART_BUFFER_OVERFLOW", "clearing buffer — no newline received")
                    buffer = ""
                    continue

                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue

                    self._log("UART_RX", line)
                    msg = self._parse_line(line)
                    if msg and self.on_message:
                        self.on_message(msg)

            except Exception as e:
                self._log("UART_READ_FAIL", str(e))

                # Close and clear serial under lock
                with self._serial_lock:
                    try:
                        if self.ser:
                            self.ser.close()
                    except Exception:
                        pass
                    self.ser = None

                # FIX 4: attempt one reconnection before giving up entirely.
                # Previously a serial drop (cable glitch, BBG issue) would exit
                # the read loop permanently, requiring a full server restart.
                if not self.running:
                    break

                self._log("UART_RECONNECT", f"Waiting 2s then attempting reconnect on {self.port}")
                time.sleep(2)

                if not self.running:
                    break

                try:
                    new_ser = serial.Serial(self.port, self.baudrate, timeout=0.2)
                    with self._serial_lock:
                        if not self.running:
                            try:
                                new_ser.close()
                            except Exception:
                                pass
                            break
                        self.ser = new_ser
                    self._log("UART_RECONNECT", f"Reconnected to {self.port} successfully")
                    buffer = ""
                    continue
                except Exception as re:
                    self._log("UART_RECONNECT_FAIL", f"Could not reconnect: {re} — stopping read loop")
                    self.running = False
                    break

    def _parse_line(self, line: str):
        """
        Parses an inbound message from the Pi.
        Expected format: STATUS|<task_id>|<status> or LOCK|<task_id>|<state>
        Returns a dict on success, None on any parse failure.
        """
        parts = [p.strip() for p in line.split("|")]

        if len(parts) != 3:
            self._log("UART_PARSE_IGNORE", f"bad format (expected 3 parts): {line}")
            return None

        msg_type, task_id, status = parts

        if msg_type not in {"STATUS", "LOCK"}:
            self._log("UART_PARSE_IGNORE", f"unexpected msg_type={msg_type} line={line}")
            return None

        if not task_id:
            self._log("UART_PARSE_IGNORE", f"missing task_id: {line}")
            return None

        status = status.upper()
        if msg_type == "LOCK":
            valid_lock_states = {"LOCKED", "UNLOCKED"}
            if status not in valid_lock_states:
                self._log("UART_PARSE_IGNORE", f"unknown lock_state={status} line={line}")
                return None
            return {
                "type": "LOCK",
                "task_id": task_id,
                "lock_state": status,
            }

        valid = {
            "MOVING",
            "ARRIVED",
            "FAULT",
            "BLOCKED",
            "CANCELED",
            "IDLE",
            "LOCKED",
            "UNLOCKED",
            "LOCK_UNLOCKED",
            "LOCK_LOCKED",
        }
        if status not in valid:
            self._log("UART_PARSE_IGNORE", f"unknown status={status} line={line}")
            return None

        return {
            "type": "STATUS",
            "task_id": task_id,
            "status": status
        }

    def _log(self, tag: str, msg: str):
        if self.logger:
            self.logger.log(tag, msg)
        else:
            print(f"[{tag}] {msg}")
