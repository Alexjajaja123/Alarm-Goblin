from gpiozero import DistanceSensor

# Set up the GPIO pins for the ultrasonic sensors
trigger_pin_right = 6
echo_pin_right = 5
trigger_pin_left = 24
echo_pin_left = 23
ultrasonic_left = DistanceSensor(echo=echo_pin_left, trigger=trigger_pin_left)
ultrasonic_right = DistanceSensor(echo=echo_pin_right, trigger=trigger_pin_right)