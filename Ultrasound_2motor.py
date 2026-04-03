from gpiozero import PWMOutputDevice, DigitalOutputDevice, LED, DistanceSensor
from time import sleep, monotonic

# ---- Motor driver (DFR0601 channel 1) ----
PWM1_PIN = 18
INA1_PIN = 23
INB1_PIN = 24

# ---- Motor driver (DFR0601 channel 2) ----
PWM2_PIN = 12
INA2_PIN = 16
INB2_PIN = 20

# ---- HC-SR04 ----
TRIG_PIN = 17
ECHO_PIN = 27

# ---- LED ----
LED_PIN = 22

STOP_DISTANCE_CM = 50.0

# Start with a lower guessed speed and tune it by testing
TARGET_SPEED = 110 / 255.0

RUNNING = 0
STOPPED_WAIT_CLEAR = 1
CLEARING_DELAY = 2

state = RUNNING
clear_start_time = 0.0
current_speed = 0.0

# Motor 1 setup
pwm1 = PWMOutputDevice(PWM1_PIN, frequency=1000, initial_value=0)
ina1 = DigitalOutputDevice(INA1_PIN)
inb1 = DigitalOutputDevice(INB1_PIN)

# Motor 2 setup
pwm2 = PWMOutputDevice(PWM2_PIN, frequency=1000, initial_value=0)
ina2 = DigitalOutputDevice(INA2_PIN)
inb2 = DigitalOutputDevice(INB2_PIN)

# Ultrasonic setup
sensor = DistanceSensor(
    echo=ECHO_PIN,
    trigger=TRIG_PIN,
    max_distance=2.0,
    threshold_distance=0.5
)

# LED setup
led = LED(LED_PIN)

def read_distance_cm():
    try:
        d = sensor.distance * 100.0
        if d <= 0:
            return -1
        return d
    except Exception:
        return -1

def motor_direction_forward():
    ina1.on()
    inb1.off()
    ina2.on()
    inb2.off()

def set_motor_speed(speed):
    global current_speed
    speed = max(0.0, min(1.0, speed))
    pwm1.value = speed
    pwm2.value = speed
    current_speed = speed

def ramp_to_speed(target, step=0.02, delay=0.05):
    global current_speed
    target = max(0.0, min(1.0, target))

    while abs(current_speed - target) > step:
        if current_speed < target:
            current_speed += step
        else:
            current_speed -= step

        current_speed = max(0.0, min(1.0, current_speed))
        pwm1.value = current_speed
        pwm2.value = current_speed
        sleep(delay)

    set_motor_speed(target)

def motors_forward_slow(target_speed):
    motor_direction_forward()
    ramp_to_speed(target_speed, step=0.02, delay=0.05)

def motors_stop_slow():
    ramp_to_speed(0.0, step=0.03, delay=0.03)

def main():
    global state, clear_start_time

    motor_direction_forward()
    set_motor_speed(0.0)
    motors_forward_slow(TARGET_SPEED)
    led.off()

    while True:
        d = read_distance_cm()
        obstacle = (d > 0 and d < STOP_DISTANCE_CM)

        print(f"d={d:.1f} cm, state={state}, speed={current_speed:.2f}")

        if state == RUNNING:
            if obstacle:
                motors_stop_slow()
                led.on()
                state = STOPPED_WAIT_CLEAR

        elif state == STOPPED_WAIT_CLEAR:
            if not obstacle:
                clear_start_time = monotonic()
                state = CLEARING_DELAY

        elif state == CLEARING_DELAY:
            if obstacle:
                state = STOPPED_WAIT_CLEAR
            elif monotonic() - clear_start_time >= 2.0:
                motors_forward_slow(TARGET_SPEED)
                led.off()
                state = RUNNING

        sleep(0.05)

try:
    main()
finally:
    set_motor_speed(0.0)
    led.off()
    ina1.off()
    inb1.off()
    ina2.off()
    inb2.off()