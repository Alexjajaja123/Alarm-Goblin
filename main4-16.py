import RPi.GPIO as GPIO
from gpiozero import DistanceSensor
from time import sleep
import time
import random
import math
import pygame

pygame.mixer.init()
pygame.mixer.music.load("Goofy_Noises.mp3")
pygame.mixer.music.play(-1)
pygame.mixer.music.pause()

DUTY_CYCLE_FORWARD_LEFT = 50
DUTY_CYCLE_BACKWARD_LEFT = 2.5
DUTY_CYCLE_STOP_LEFT = 20.5

DUTY_CYCLE_FORWARD_RIGHT = 4.5
DUTY_CYCLE_BACKWARD_RIGHT = 10
DUTY_CYCLE_STOP_RIGHT = 0

LEFT_TURN_RATIO = 2.25
RIGHT_TURN_RATIO = 2.5
BACKUP_TIME = 1
DETECTION_DISTANCE = 30.0
AUTO_SHUTOFF = 180

GPIO.setmode(GPIO.BCM)


class Servo:
    def __init__(self, pin, pwm_object, forward, backward, stop):
        self.pin = pin
        self.pwm = pwm_object
        self.forwardDC = forward
        self.backwardDC = backward
        self.stopDC = stop

        self.pwm.start(0)
        # for i in range(0, 100, 1):
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


GPIO.setmode(GPIO.BCM)
# create PWM object for left servo
servo_pin_left = 27
GPIO.setup(servo_pin_left, GPIO.OUT)
servo_pwm_left = GPIO.PWM(servo_pin_left, 50)
servo_left = Servo(servo_pin_left, servo_pwm_left, DUTY_CYCLE_FORWARD_LEFT, DUTY_CYCLE_BACKWARD_LEFT,
                   DUTY_CYCLE_STOP_LEFT)

# create PWM object for right servo
servo_pin_right = 17
GPIO.setup(servo_pin_right, GPIO.OUT)
servo_pwm_right = GPIO.PWM(servo_pin_right, 50)
servo_right = Servo(servo_pin_right, servo_pwm_right, DUTY_CYCLE_FORWARD_RIGHT, DUTY_CYCLE_BACKWARD_RIGHT,
                    DUTY_CYCLE_STOP_RIGHT)

# Set up the GPIO pins for the ultrasonic sensors
trigger_pin_left = 24
echo_pin_left = 23
trigger_pin_right = 6
echo_pin_right = 5
ultrasonic_left = DistanceSensor(echo=echo_pin_left, trigger=trigger_pin_left)
ultrasonic_right = DistanceSensor(echo=echo_pin_right, trigger=trigger_pin_right)

# Set up Button
GPIO.setup(26, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)


def test_servo():
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

        servo_left.stop()
        servo_right.stop()
        time.sleep(5)


def test_ultra():
    print("Distance from left:", ultrasonic_left.distance * 100, "Distance from right:",
          ultrasonic_right.distance * 100)


def test_button():
    try:
        while True:
            if GPIO.input(26) == GPIO.HIGH:
                print("pushed")
    except KeyboardInterrupt:
        pass


def button_delay(duration):
    for i in range(int(duration * 10)):
        if GPIO.input(26) == GPIO.HIGH:
            return True
        sleep(0.1)
    return False


def test_main_driver():
    pygame.mixer.music.unpause()
    while True:
        test_ultra()
        servo_left.forward()
        servo_right.forward()
        button_pressed = button_delay(5)
        if button_pressed:
            break

        servo_left.stop()
        servo_right.stop()
        button_pressed = button_delay(5)
        if button_pressed:
            break

        servo_left.backward()
        servo_right.backward()
        button_pressed = button_delay(5)
        if button_pressed:
            break

        servo_left.stop()
        servo_right.stop()
        button_pressed = button_delay(5)
        if button_pressed:
            break
    pygame.mixer.music.pause()
    stop()


def wall_detection():
    button_pressed = False
    while not button_pressed:
        servo_left.forward()
        servo_right.forward()
        button_pressed = button_delay(0.5)
        left_distance = ultrasonic_left.distance * 100.0
        right_distance = ultrasonic_right.distance * 100.0
        print("Left distance:", left_distance, ". Right distance:", right_distance)
        if left_distance < DETECTION_DISTANCE or right_distance < DETECTION_DISTANCE:
            angle = random.randint(90, 180)
            direction = random.randint(0, 1)
            if direction == 0:
                button_pressed = turn_left(angle)
            elif direction == 1:
                button_pressed = turn_right(angle)
        if button_pressed:
            break
    stop()
    return


def move_forward(duration=1):
    servo_left.forward()
    servo_right.forward()
    time.sleep(duration)
    servo_left.stop()
    servo_right.stop()


def move_back(duration=BACKUP_TIME):
    servo_left.backward()
    servo_right.backward()
    time.sleep(duration)
    servo_left.stop()
    servo_right.stop()


def turn_left(angle):
    move_back()
    wheel_distance = 10.0  # Distance between wheels
    wheel_circumference = 22.86  # Wheel circumference (in cm)
    turn_distance = (angle / 360.0) * math.pi * wheel_distance
    turn_duration = (turn_distance / wheel_circumference) * LEFT_TURN_RATIO
    servo_left.backward()
    servo_right.forward()
    button_pressed = button_delay(turn_duration)
    servo_left.stop()
    servo_right.stop()
    return button_pressed


# Turn right by a given angle
def turn_right(angle):
    move_back()
    wheel_distance = 10.0  # Distance between wheels
    wheel_circumference = 22.86  # Wheel circumference (in cm)
    turn_distance = (angle / 360.0) * math.pi * wheel_distance
    turn_duration = (turn_distance / wheel_circumference) * RIGHT_TURN_RATIO
    servo_left.forward()
    servo_right.backward()
    button_pressed = button_delay(turn_duration)
    servo_left.stop()
    servo_right.stop()
    return button_pressed


def stop():
    servo_left.stop()
    servo_right.stop()
    pygame.mixer.quit()
    restart_song()


def restart_song():
    pygame.mixer.init()
    pygame.mixer.music.load("Goofy_Noises.mp3")
    pygame.mixer.music.play(-1)
    pygame.mixer.music.pause()


def alarm():
    pygame.mixer.music.unpause()
    start_time = time.time()
    while True and time.time() - start_time < AUTO_SHUTOFF:
        servo_left.forward()
        servo_right.forward()
        button_pressed = button_delay(0.5)
        left_distance = ultrasonic_left.distance * 100.0
        right_distance = ultrasonic_right.distance * 100.0
        print("Left distance:", left_distance, ". Right distance:", right_distance)
        if left_distance < DETECTION_DISTANCE or right_distance < DETECTION_DISTANCE:
            angle = random.randint(90, 180)
            direction = random.randint(0, 1)
            if direction == 0:
                button_pressed = turn_left(angle)
            elif direction == 1:
                button_pressed = turn_right(angle)
        if button_pressed:
            break
    pygame.mixer.music.pause()
    stop()


def end():
    GPIO.cleanup()
