# services/watchdog.py

from datetime import datetime, timedelta
import threading
import time


class CommWatchdog:
    """
    Monitors UI communication health by tracking the last time ping() was
    called. If no ping is received within timeout_seconds, on_lost() is
    called once. When pings resume, on_restored() is called once.

    Thread safety: ping() is safe to call from any thread.

    Note: callbacks (on_lost, on_restored) are invoked from the watchdog's
    own background thread, outside of self._lock. In app.py these callbacks
    acquire STATE_LOCK — ensure STATE_LOCK is never held when ping() is
    called to avoid lock ordering issues.
    """

    def __init__(self, timeout_seconds: int, on_lost, on_restored):
        self.timeout = timeout_seconds
        self.on_lost = on_lost
        self.on_restored = on_restored

        # FIX 8: initialize last_seen to None instead of datetime.now().
        # Previously the watchdog started counting from construction time,
        # so if the UI never connected within timeout_seconds, on_lost()
        # would fire even though communication was never established.
        # Now we skip expiry checks until the first ping() is received.
        self.last_seen = None

        self.comm_ok = True
        self._lock = threading.Lock()

        t = threading.Thread(target=self._loop, daemon=True)
        t.start()

    def ping(self):
        with self._lock:
            self.last_seen = datetime.now()

    def _loop(self):
        while True:
            now = datetime.now()
            callback = None

            # FIX 5: capture the callback inside the lock but call it
            # OUTSIDE the lock. Previously the callback was invoked while
            # self._lock was held. The callbacks (on_comm_lost/on_comm_restored)
            # acquire STATE_LOCK in app.py — if anything in that path ever
            # tried to acquire self._lock again it would deadlock.
            with self._lock:
                # FIX 8: skip expiry check until first ping has been received
                if self.last_seen is None:
                    callback = None
                else:
                    expired = (now - self.last_seen) > timedelta(seconds=self.timeout)

                    if expired:
                        if self.comm_ok:
                            self.comm_ok = False
                            callback = self.on_lost
                        # else: already in lost state, don't fire again
                    else:
                        if not self.comm_ok:
                            self.comm_ok = True
                            callback = self.on_restored
                        # else: already ok, nothing to do

            # FIX 5: call outside the lock
            if callback:
                callback()

            # FIX 7: scale sleep interval to timeout so the watchdog never
            # misses a timeout window. Previously a fixed sleep(1) meant
            # timeouts shorter than 1 second would never be detected.
            # Capped at 1s so the loop stays responsive.
            time.sleep(min(1.0, self.timeout / 5.0))