#!/usr/bin/env python3
"""
Bench-test the MailRover electronic lock board on a Raspberry Pi.

Default wiring:
  coil command output: physical pin 37, BCM GPIO26
  lock state input:    physical pin 36, BCM GPIO16

Default logic:
  command output active-low: 1 = locked/safe, 0 = unlock request
  lock state input:          1 = locked, 0 = unlocked

The board's logic gates may cut coil drive once the lock reports unlocked.
This script only pulses if the lock input starts locked. It stops the pulse
after 200ms max or as soon as the input goes low, then always returns the
command output to the locked/safe level.
"""

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
    parser.add_argument("--chip", default="/dev/gpiochip4", help="GPIO chip path.")
    parser.add_argument("--coil-gpio", type=int, default=26, help="BCM GPIO for lock coil command output.")
    parser.add_argument("--state-gpio", type=int, default=16, help="BCM GPIO for lock state input.")
    parser.add_argument("--pulse-sec", type=float, default=0.20, help="Maximum unlock pulse duration.")
    parser.add_argument("--watch-sec", type=float, default=5.0, help="Seconds to watch lock state after pulse.")
    parser.add_argument(
        "--active-low-output",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Use 0 as unlock command and 1 as locked/safe command.",
    )
    parser.add_argument(
        "--locked-high-input",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Treat input 1 as locked and 0 as unlocked.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Read state but do not pulse the coil output.",
    )
    parser.add_argument(
        "--hold-safe",
        action="store_true",
        help="Hold the coil command at locked/safe level until Ctrl+C.",
    )
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
    lines = LockLines(args.chip, args.coil_gpio, args.state_gpio, args.active_low_output)

    print("MailRover lock test")
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
            print(f"WARNING: did not observe UNLOCKED during pulse. Current input: {raw} ({state_label(raw, args.locked_high_input)})")

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
