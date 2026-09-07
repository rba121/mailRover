# Pi 5 Solenoid Lock Setup

This setup is for a 12V spring cabinet/locker solenoid lock driven by a
logic-level MOSFET such as IRLZ44N.

This design does not use servo motors or a PCA9685 controller.

Do not connect the lock directly to a Raspberry Pi GPIO pin. The lock needs
12V and much more current than a GPIO pin can supply.

## Parts

- Raspberry Pi 5
- 12V DC power supply rated for the lock current
- 12V solenoid cabinet lock
- IRLZ44N or similar logic-level N-channel MOSFET
- Flyback diode from your diode kit, for example 1N4007, 1N5404, or 1N5408
- 220 ohm gate resistor
- 10k ohm gate pulldown resistor

## How The App Controls The Lock

The Pi GPIO pin never powers the lock directly. The GPIO pin only turns the
MOSFET on briefly.

- GPIO low: solenoid is off, latch is mechanically locked or ready to re-lock
- GPIO high: MOSFET turns on, 12V flows through the solenoid, latch releases
- After `UNLOCK_PULSE_SEC`, GPIO returns low automatically

The app pulses the solenoid only for:

- admin loading/service unlock from `/admin`
- recipient unlock after the correct arrival PIN is entered

The app does not pulse the lock on startup or reset.

Your tested electronic lock board is active-high for unlock:

```text
GPIO26 = 0 means locked / safe
GPIO26 = 1 means unlock command
```

Set `SOLENOID_UNLOCK_ACTIVE_LOW=false`. The app will hold GPIO26 low while
locked, pulse it high to unlock, then return it low.

## Wiring

Use these Raspberry Pi 5 pins for the electronic lock board:

```text
GPIO chip --------------------- /dev/gpiochip4
lock coil output / command ---- physical pin 37 / BCM GPIO26
lock input / state feedback --- physical pin 36 / BCM GPIO16
power button input ----------- physical pin 19 / BCM GPIO10

command logic: 0 = locked/safe, 1 = unlock request
input logic:   1 = locked,      0 = unlocked
```

The electronic lock board has logic gates that cut coil drive when it detects
the door/lock is already unlocked. The app should still return the command line
to locked/safe immediately after the pulse.

If you are using an external 12V supply, grounds must be common:

```text
Pi GND and 12V supply negative connect together.
```

## `.env` Settings On The Pi

```bash
USE_REAL_GPIO_ACTUATORS=true
GPIO_CHIP_PATH=/dev/gpiochip4
D1_UNLOCK_GPIO=26
D1_LOCK_STATE_GPIO=16
POWER_BUTTON_GPIO=10
POWER_BUTTON_ACTIVE_LOW=false
SOLENOID_UNLOCK_ACTIVE_LOW=false
LOCK_STATE_LOCKED_HIGH=true
LOCK_FEEDBACK_REQUIRED=true
```

## Demo Day Delivery Timing

For demo day, the recipient can choose a normal-looking future delivery window
while the admin can dispatch immediately:

```bash
ENFORCE_DELIVERY_WINDOW=false
```

For production, set this to `true` so dispatch is only allowed shortly before
the selected window:

```bash
ENFORCE_DELIVERY_WINDOW=true
DISPATCH_LEAD_MINUTES=15
```

## Robot Lock Feedback

If the robot already knows the lock line state, enable:

```bash
LOCK_FEEDBACK_REQUIRED=true
```

Then the app does not mark the delivery complete immediately after the PIN
unlock pulse. It waits until the robot reports the lock is secure again.

Accepted UART messages from the robot:

```text
LOCK|<task_id>|UNLOCKED
LOCK|<task_id>|LOCKED
```

The app also accepts these equivalent status messages:

```text
STATUS|<task_id>|LOCK_UNLOCKED
STATUS|<task_id>|LOCK_LOCKED
STATUS|<task_id>|UNLOCKED
STATUS|<task_id>|LOCKED
```

No hall sensor or servo settings are needed for this build.

## Install GPIO Library On Pi

Inside the project virtualenv on the Pi:

```bash
source .venv/bin/activate
pip install gpiod
```

If `gpiod` is unavailable on your Pi image, install the system package first:

```bash
sudo apt update
sudo apt install -y python3-libgpiod gpiod
```

## Test The Pulse

Before testing through the full app, run the standalone lock test on the Pi:

```bash
source .venv/bin/activate
python scripts/test_lock.py
```

If you copied it elsewhere as `test_script.py`, run:

```bash
sudo python3 test_script.py --chip /dev/gpiochip4 --active-low-output --locked-high-input
```

This test uses:

```text
GPIO chip:           /dev/gpiochip4
coil command output: BCM GPIO26 / physical pin 37
lock state input:    BCM GPIO16 / physical pin 36
```

It first confirms the input is locked (`1`). It then drives the command line
low for up to `200ms`, or stops early when the input goes unlocked (`0`), then
always returns the command line high before exiting.

Raw GPIO checks on Pi 5:

```bash
sudo gpioget 4 16
sudo gpioset 4 26=1
sudo gpioset 4 26=0
sudo gpioset 4 26=1
```

Start the app:

```bash
bash run_pi.sh
```

Open admin:

```text
http://10.255.255.2:8000/admin
```

Use **Mark Arrived** after dispatching a delivery, then enter the emailed unlock
code on the tracking page. The app will pulse GPIO26 for `UNLOCK_PULSE_SEC`.

For loading, open `/admin` and use **Open Locker for Loading**. With nothing
wired yet, this only updates the dev-mode state and logs the action. On the Pi
with `USE_REAL_GPIO_ACTUATORS=true`, it pulses GPIO26.

For a direct service test, use the app flow rather than manually touching GPIO,
because the app also checks delivery state and logs the action.

## Pulse Timing

The current default unlock pulse is:

```python
UNLOCK_PULSE_SEC = 0.50
```

If the lock needs longer, increase it carefully in `config.py`. Avoid holding a
12V solenoid on for too long unless the lock is rated for continuous duty.
