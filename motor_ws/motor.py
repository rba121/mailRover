from gpiozero import PWMOutputDevice, DigitalOutputDevice, LED, DistanceSensor
from time import sleep, monotonic

# Define your pins (you were missing these!)
INA1_PIN = 16
INB1_PIN = 20
INA2_PIN = 23
INB2_PIN = 24

# Motor 1 setup
pwm1 = PWMOutputDevice(12, frequency=100, initial_value=0)
ina1 = DigitalOutputDevice(INA1_PIN)
inb1 = DigitalOutputDevice(INB1_PIN)

# Motor 2 setup
pwm2 = PWMOutputDevice(18, frequency=100, initial_value=0)
ina2 = DigitalOutputDevice(INA2_PIN)
inb2 = DigitalOutputDevice(INB2_PIN)

try:
    pwm1.value = 0.01
    pwm2.value = 0
    sleep(5)

except Exception as e:
    print("Error:", e)

finally:
    pwm1.value = 0
    pwm2.value = 0
