import RPi.GPIO as GPIO
import time
from gpiozero import DistanceSensor

DUTY_CYCLE_FORWARD_LEFT = 4.5
DUTY_CYCLE_BACKWARD_LEFT = 7.5
DUTY_CYCLE_STOP_LEFT = 0

DUTY_CYCLE_FORWARD_RIGHT = 50
DUTY_CYCLE_BACKWARD_RIGHT = 2.5
DUTY_CYCLE_STOP_RIGHT = 20.5

GPIO.setmode(GPIO.BCM)


class Servo:
    def __init__(self, pin, pwm_object, forward, backward, stop):
        self.pin = pin
        self.pwm = pwm_object
        self.forwardDC = forward
        self.backwardDC = backward
        self.stopDC = stop

        self.pwm.start(0)
        # for i in range(0, 10, 1):
        # print("testing: ", i)
        # self.pwm.ChangeDutyCycle(i)
        # time.sleep(0.1)
        # print("testing: ", i + 0.5)
        # self.pwm.ChangeDutyCycle(i + 0.5)
        # time.sleep(1)

    def forward(self):
        self.pwm.ChangeDutyCycle(self.forwardDC)

    def backward(self):
        self.pwm.ChangeDutyCycle(self.backwardDC)

    def stop(self):
        self.pwm.ChangeDutyCycle(self.stopDC)

    def test(self):
        self.pwm.ChangeDutyCycle(100)


def test_servo():
    GPIO.setmode(GPIO.BCM)
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


# Set up the GPIO pins for the ultrasonic sensors

trigger_pin_left = 6
echo_pin_left = 5
ultrasonic_left = DistanceSensor(echo=echo_pin_left, trigger=trigger_pin_left)


def test_ultra():
    print("Distance:", ultrasonic_left.distance * 100)


def test_buzzer():
    buzzer = 16

    GPIO.setup(buzzer, GPIO.OUT)

    GPIO.output(buzzer, GPIO.HIGH)
    print("beep")
    sleep(0.5)
    GPIO.output(buzzer, GPIO.LOW)
    print("no beep")
    sleep(0.5)
