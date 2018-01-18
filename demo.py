# Limits:
# Hand -> 8 - 9.9
# Base -> 3 - 10
# Left -> 5 - 11
# Right -> 5 - 10

import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)

hand_pin = 11
base_pin = 16
left_pin = 12
right_pin = 15

GPIO.setup(hand_pin, GPIO.OUT)
GPIO.setup(base_pin, GPIO.OUT)
GPIO.setup(left_pin, GPIO.OUT)
GPIO.setup(right_pin, GPIO.OUT)

hand = GPIO.PWM(hand_pin, 50)
base = GPIO.PWM(base_pin, 50)
left = GPIO.PWM(left_pin, 50)
right = GPIO.PWM(right_pin, 50)

def startingPosition():
    hand.start(9.9)
    base.start(7.5)
    left.start(8)
    right.start(5)

def closeConn():
    hand.stop()
    base.stop()
    left.stop()
    right.stop()

startingPosition()
time.sleep(1)

hand.ChangeDutyCycle(8)
time.sleep(1)

hand.ChangeDutyCycle(9.9)
time.sleep(1)

base.ChangeDutyCycle(3)
time.sleep(1)

base.ChangeDutyCycle(10)
time.sleep(1)

base.ChangeDutyCycle(7.5)
time.sleep(1)

right.ChangeDutyCycle(8)
time.sleep(1)

right.ChangeDutyCycle(5)
time.sleep(1)

left.ChangeDutyCycle(5)
time.sleep(1)

left.ChangeDutyCycle(11)
time.sleep(1)

left.ChangeDutyCycle(8)
time.sleep(1)

closeConn()
