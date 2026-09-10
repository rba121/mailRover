

from gpiozero import PWMOutputDevice, DigitalOutputDevice, DistanceSensor, LED
from signal import pause
import time

# Pin definitions (BCM / GPIO)

# Motor 1 Left
PWM1_PIN = 18 #PIN12
INA1_PIN = 17 #PIN11
INB1_PIN = 27 #PIN13

# Motor 2 Right
PWM2_PIN = 12 #PIN32
INA2_PIN = 22 #PIN15
INB2_PIN = 23 #PIN16

# Ultrasonic sensor
TRIG_PIN = 24 #PIN18
ECHO_PIN = 25 #PIN22

# LED
LED_PIN = 16 #PIN36

# Settings
STOP_DISTANCE_CM = 50.0     # stop if object is closer than this
MOTOR_SPEED = 0.07           # 0.0 to 1.0
SAMPLE_DELAY = 0           # seconds between sensor reads

# Devices
motor1_pwm = PWMOutputDevice(PWM1_PIN, frequency=1000)
motor1_ina = DigitalOutputDevice(INA1_PIN)
motor1_inb = DigitalOutputDevice(INB1_PIN)

motor2_pwm = PWMOutputDevice(PWM2_PIN, frequency=1000)
motor2_ina = DigitalOutputDevice(INA2_PIN)
motor2_inb = DigitalOutputDevice(INB2_PIN)

sensor = DistanceSensor(
    echo=ECHO_PIN,
    trigger=TRIG_PIN,
    max_distance=2.0,      # meters
    threshold_distance=0.5
)

warning_led = LED(LED_PIN)

# Motor functions
def motor1_forward(speed):
    motor1_ina.on()
    motor1_inb.off()
    motor1_pwm.value = speed

def motor2_forward(speed):
    motor2_ina.off()
    motor2_inb.on()
    motor2_pwm.value = speed

def motor1_stop():
    motor1_pwm.value = 0
    motor1_ina.off()
    motor1_inb.off()

def motor2_stop():
    motor2_pwm.value = 0
    motor2_ina.off()
    motor2_inb.off()

def motors_forward(speed):
    motor1_forward(speed)
    motor2_forward(speed)

def motors_stop():
    motor1_stop()
    motor2_stop()

# Main loop
try:
    print("Starting robot test...")
    print("Press Ctrl+C to stop.\n")

    while True:
        distance_cm = sensor.distance * 100

        # Sometimes ultrasonic readings can glitch
        if distance_cm <= 0 or distance_cm > 400:
            print("Distance: invalid reading")
            time.sleep(SAMPLE_DELAY)
            continue

        print(f"Distance: {distance_cm:.1f} cm")

        if distance_cm < STOP_DISTANCE_CM:
            motors_stop()
            warning_led.on()
            print("Object detected: STOP, LED ON")
        else:
            motors_forward(MOTOR_SPEED)
            warning_led.off()
            print("Path clear: FORWARD, LED OFF")

        time.sleep(SAMPLE_DELAY)

except KeyboardInterrupt:
    print("\nStopping robot...")

finally:
    motors_stop()
    warning_led.off()
    print("Clean shutdown complete.")
