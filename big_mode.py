import RPi.GPIO as GPIO
from gpiozero import DistanceSensor
import time
import math
import random

# Wheel Info
WHEEL_DISTANCE = 21  # Distance between wheels in cm
WHEEL_CIRCUMFERENCE = 22.86  # Wheel circumference

# Time to call the start the robot
alarm_time = 28800000
auto_shutoff = 10000

GPIO.setmode(GPIO.BCM)

DUTY_CYCLE_FORWARD_LEFT = 4.5
DUTY_CYCLE_BACKWARD_LEFT = 7.5
DUTY_CYCLE_STOP_LEFT = 0

DUTY_CYCLE_FORWARD_RIGHT = 50
DUTY_CYCLE_BACKWARD_RIGHT = 2.5
DUTY_CYCLE_STOP_RIGHT = 20.5


class Servo:
    def __init__(self, pin, pwm_object, forward, backward, stop):
        self.pin = pin
        self.pwm = pwm_object
        self.forwardDC = forward
        self.backwardDC = backward
        self.stopDC = stop

        self.pwm.start(0)

    def forward(self):
        self.pwm.ChangeDutyCycle(self.forwardDC)

    def backward(self):
        self.pwm.ChangeDutyCycle(self.backwardDC)

    def stop(self):
        self.pwm.ChangeDutyCycle(self.stopDC)

    def test(self):
        for i in range(0, 10, 1):
            print("testing: ", i)
            self.pwm.ChangeDutyCycle(i)
            time.sleep(0.1)
            print("testing: ", i + 0.5)
            self.pwm.ChangeDutyCycle(i + 0.5)
            time.sleep(1)


# create PWM object for left servo
servo_pin_left = 17
GPIO.setup(servo_pin_left, GPIO.OUT)
servo_pwm_left = GPIO.PWM(servo_pin_left, 50)
servo_left = Servo(servo_pin_left, servo_pwm_left, DUTY_CYCLE_FORWARD_LEFT, DUTY_CYCLE_BACKWARD_LEFT,
                   DUTY_CYCLE_STOP_LEFT)

# create PWM object for right servo
servo_pin_right = 27
GPIO.setup(servo_pin_right, GPIO.OUT)
servo_pwm_right = GPIO.PWM(servo_pin_right, 50)
servo_right = Servo(servo_pin_right, servo_pwm_right, DUTY_CYCLE_FORWARD_RIGHT, DUTY_CYCLE_BACKWARD_RIGHT,
                    DUTY_CYCLE_STOP_RIGHT)

# Set up the GPIO pins for the ultrasonic sensors
trigger_pin_right = 6
echo_pin_right = 5
trigger_pin_left = 24
echo_pin_left = 23
ultrasonic_left = DistanceSensor(echo=echo_pin_left, trigger=trigger_pin_left)
ultrasonic_right = DistanceSensor(echo=echo_pin_right, trigger=trigger_pin_right)


# Turn left by a given angle
def turn_left(angle):
    turn_distance = (angle / 360) * math.pi * WHEEL_DISTANCE
    turn_duration = turn_distance / WHEEL_CIRCUMFERENCE
    servo_right.forward()
    servo_left.backward()
    time.sleep(turn_duration)


# Turn right by a given angle
def turn_right(angle):
    turn_distance = (angle / 360) * math.pi * WHEEL_DISTANCE
    turn_duration = turn_distance / WHEEL_CIRCUMFERENCE
    servo_right.forward()
    servo_left.backward()
    time.sleep(turn_duration)


def test_turning():
    start_time = time.time()
    while math.abs(time.time() - start_time) < auto_shutoff:
        distance_left = ultrasonic_left.distance() * 100
        distance_right = ultrasonic_right.distance() * 100
        if distance_left < 20 or distance_right < 20:
            angle = random.randint(90, 180)
            direction = random.randint(0, 1)
            if direction == 0:
                turn_left(angle)
            elif direction == 1:
                turn_right(angle)
        else:
            servo_left.forward()
            servo_right.forward()
    stop()


def stop():
    servo_left.stop()
    servo_right.stop()


def test_motors():
    for i in range(10):
        servo_left.forward()
        servo_right.forward()
        time.sleep(5)
        servo_left.stop()
        servo_right.stop()
        time.sleep(5)
        servo_left.backward()
        servo_right.backward()
        time.sleep(5)


# Clean up the GPIO pins
GPIO.cleanup()





