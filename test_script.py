#!/usr/bin/env python3
import argparse
import sys
import time

gpiod = None
GPIOD_VERSION = (0, 0)


def load_gpiod():
    global gpiod, GPIOD_VERSION
    try:
        import gpiod as loaded_gpiod
        gpiod = loaded_gpiod
        GPIOD_VERSION = tuple(int(x) for x in gpiod.__version__.split(".")[:2])
    except Exception as exc:
        print(f"ERROR: python gpiod module is not available: {exc}", file=sys.stderr)
        print("Install on the Pi with: sudo apt install -y python3-libgpiod gpiod", file=sys.stderr)
        sys.exit(1)


def parse_args():
    parser = argparse.ArgumentParser(description="Test MailRover electronic lock GPIO.")
    parser.add_argument("--chip", default="/dev/gpiochip4")
    parser.add_argument("--coil-gpio", type=int, default=26)
    parser.add_argument("--state-gpio", type=int, default=16)
    parser.add_argument("--pulse-sec", type=float, default=0.20)
    parser.add_argument("--watch-sec", type=float, default=5.0)
    parser.add_argument("--active-low-output", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--locked-high-input", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--hold-safe", action="store_true")
    return parser.parse_args()


class LockLines:
    def __init__(self, chip, coil_gpio, state_gpio, active_low_output):
        self.chip = chip
        self.coil_gpio = coil_gpio
        self.state_gpio = state_gpio
        self.active_low_output = active_low_output
        self._chip = None
        self._coil = None
        self._state = None

    def locked_output_value(self):
        return 1 if self.active_low_output else 0

    def unlock_output_value(self):
        return 0 if self.active_low_output else 1

    def open(self):
        if GPIOD_VERSION < (2, 0):
            self._chip = gpiod.Chip(self.chip)
            self._coil = self._chip.get_line(self.coil_gpio)
            self._state = self._chip.get_line(self.state_gpio)
            self._coil.request(
                consumer="mailrover_lock_test_coil",
                type=gpiod.LINE_REQ_DIR_OUT,
                default_vals=[self.locked_output_value()],
            )
            self._state.request(
                consumer="mailrover_lock_test_state",
                type=gpiod.LINE_REQ_DIR_IN,
            )
            return

        self._coil = gpiod.request_lines(
            self.chip,
            consumer="mailrover_lock_test_coil",
            config={
                self.coil_gpio: gpiod.LineSettings(
                    direction=gpiod.line.Direction.OUTPUT,
                    output_value=self._v2_value(self.locked_output_value()),
                )
            },
        )

        self._state = gpiod.request_lines(
            self.chip,
            consumer="mailrover_lock_test_state",
            config={
                self.state_gpio: gpiod.LineSettings(
                    direction=gpiod.line.Direction.INPUT,
                )
            },
        )

    def close(self):
        try:
            self.set_locked()
        finally:
            for req in (self._coil, self._state):
                if req is not None and hasattr(req, "release"):
                    req.release()

    def _v2_value(self, value):
        return gpiod.line.Value.ACTIVE if value else gpiod.line.Value.INACTIVE

    def set_locked(self):
        self.set_output(self.locked_output_value())

    def set_unlock(self):
        self.set_output(self.unlock_output_value())

    def set_output(self, value):
        if GPIOD_VERSION < (2, 0):
            self._coil.set_value(value)
            return

        self._coil.set_value(self.coil_gpio, self._v2_value(value))

    def read_state_raw(self):
        if GPIOD_VERSION < (2, 0):
            return int(self._state.get_value())

        value = self._state.get_value(self.state_gpio)
        return 1 if value == gpiod.line.Value.ACTIVE else 0


def state_label(raw, locked_high_input):
    locked = raw == 1 if locked_high_input else raw == 0
    return "LOCKED" if locked else "UNLOCKED"


def is_locked(raw, locked_high_input):
    return state_label(raw, locked_high_input) == "LOCKED"


def main():
    args = parse_args()
    load_gpiod()

    lines = LockLines(
        args.chip,
        args.coil_gpio,
        args.state_gpio,
        args.active_low_output,
    )

    print("MailRover lock test")
    print(f"  GPIO chip: {args.chip}")
    print(f"  coil output: BCM GPIO{args.coil_gpio}")
    print(f"  state input: BCM GPIO{args.state_gpio}")
    print(f"  output logic: {'0=unlock, 1=locked' if args.active_low_output else '1=unlock, 0=locked'}")
    print(f"  input logic: {'1=locked, 0=unlocked' if args.locked_high_input else '0=locked, 1=unlocked'}")

    lines.open()

    try:
        lines.set_locked()
        raw = lines.read_state_raw()
        print(f"Initial lock input: {raw} ({state_label(raw, args.locked_high_input)})")

        if args.hold_safe:
            print("Holding coil command at locked/safe level. Press Ctrl+C to exit.")
            while True:
                lines.set_locked()
                raw = lines.read_state_raw()
                print(f"  {time.strftime('%H:%M:%S')} input={raw} ({state_label(raw, args.locked_high_input)})")
                time.sleep(1.0)

        if args.dry_run:
            print("Dry run: not pulsing coil output.")
            return

        if not is_locked(raw, args.locked_high_input):
            print("ABORT: lock input is not locked. Not pulsing coil command.")
            print("Expected input=1 for locked with the default wiring.")
            return

        print(f"Sending unlock command for up to {args.pulse_sec:.2f}s...")
        started = time.monotonic()
        lines.set_unlock()

        saw_unlocked = False

        while time.monotonic() - started < args.pulse_sec:
            raw = lines.read_state_raw()
            label = state_label(raw, args.locked_high_input)

            if label == "UNLOCKED":
                saw_unlocked = True
                print(f"Lock input changed to UNLOCKED after {time.monotonic() - started:.3f}s.")
                break

            time.sleep(0.02)

        lines.set_locked()
        print("Coil command returned to locked/safe level.")

        if not saw_unlocked:
            raw = lines.read_state_raw()
            print(
                f"WARNING: did not observe UNLOCKED during pulse. "
                f"Current input: {raw} ({state_label(raw, args.locked_high_input)})"
            )

        print(f"Watching input for {args.watch_sec:.1f}s. It is locked when input=1.")
        end = time.monotonic() + args.watch_sec
        previous = None

        while time.monotonic() < end:
            raw = lines.read_state_raw()
            label = state_label(raw, args.locked_high_input)

            if label != previous:
                print(f"  {time.strftime('%H:%M:%S')} input={raw} {label}")
                previous = label

            time.sleep(0.1)

        raw = lines.read_state_raw()
        label = state_label(raw, args.locked_high_input)
        print(f"Final lock input: {raw} ({label})")

        if label != "LOCKED":
            print("WARNING: final input is not locked.")

    finally:
        lines.close()
        print("Safe exit: coil command is locked/safe.")


if __name__ == "__main__":
    main()
