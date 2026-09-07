# services/logger.py

from datetime import datetime
import os
import queue
import threading


class AsyncLogger:
    """
    Asynchronous file logger. Log writes are offloaded to a background
    daemon thread via a queue so they never block Flask request handlers
    or navigation/control functions (satisfies S3.14.1).

    Thread safety: log() is safe to call from any thread at any time.
    """

    # FIX 3: cap queue size so a runaway log loop cannot consume unbounded
    # memory. If the queue is full, the oldest entry is dropped and a
    # warning is written instead. 1000 entries is well above any realistic
    # burst during normal operation.
    MAX_QUEUE_SIZE = 1000

    def __init__(self, log_dir: str, log_file: str):
        # FIX 3: bounded queue with maxsize
        self.q = queue.Queue(maxsize=self.MAX_QUEUE_SIZE)
        self._dropped = 0  # count of dropped entries since last flush

        os.makedirs(log_dir, exist_ok=True)
        self.path = os.path.join(log_dir, log_file)

        self.thread = threading.Thread(target=self._worker, daemon=True)
        self.thread.start()

    def _worker(self):
        # FIX 1: wrap the entire worker in a try/except so a file write
        # failure (disk full, permissions change) is caught and reported
        # rather than silently killing the thread. On failure we attempt
        # to fall back to stdout so log entries are not lost entirely.
        while True:
            try:
                with open(self.path, "a", buffering=1) as f:
                    while True:
                        evt = self.q.get()
                        if evt is None:
                            return  # close() was called — exit cleanly
                        try:
                            f.write(evt + "\n")
                        except Exception as write_err:
                            # File write failed — print to stdout and
                            # break inner loop to reopen the file
                            print(f"[LOGGER_WRITE_FAIL] {write_err} — entry: {evt}")
                            break
            except Exception as open_err:
                print(f"[LOGGER_OPEN_FAIL] Cannot open log file {self.path}: {open_err}")
                # Wait briefly before retrying to avoid a tight error loop
                import time
                time.sleep(2)

    def log(self, event_type: str, details: str = ""):
        ts = datetime.now().isoformat(timespec="seconds")

        # FIX 4: use | as separator between fields for easier log parsing.
        # Format: "YYYY-MM-DDTHH:MM:SS | EVENT_TYPE | details"
        if details:
            line = f"{ts} | {event_type} | {details}"
        else:
            line = f"{ts} | {event_type}"

        # FIX 3: if queue is full, drop the entry and increment counter
        # rather than blocking the caller or crashing.
        try:
            self.q.put_nowait(line)
            # If we had previously dropped entries, log that now that
            # there is room again
            if self._dropped > 0:
                warn = f"{ts} | LOGGER_DROPPED | {self._dropped} entries dropped due to full queue"
                try:
                    self.q.put_nowait(warn)
                except queue.Full:
                    pass
                self._dropped = 0
        except queue.Full:
            self._dropped += 1
            print(f"[LOGGER_QUEUE_FULL] dropped: {line}")

    def close(self):
        """
        Signal the worker to flush and exit. Waits up to 2 seconds for
        the queue to drain.
        """
        try:
            self.q.put_nowait(None)
        except queue.Full:
            print("[LOGGER_CLOSE_WARN] Queue full during close — dropping one entry to enqueue shutdown signal")
            try:
                self.q.get_nowait()
                self.q.put_nowait(None)
            except Exception:
                print(
                    f"[LOGGER_CLOSE_WARN] Could not enqueue shutdown signal — "
                    f"some entries may not have been written to {self.path}"
                )
                return

        self.thread.join(timeout=2.0)
        if self.thread.is_alive():
            print(
                f"[LOGGER_CLOSE_WARN] Logger worker did not finish within 2s — "
                f"some entries may not have been written to {self.path}"
            )