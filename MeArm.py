import RPi.GPIO as GPIO
import time

class MeArm:
    'An interface for GPIO operations in MeArm robotic arm'
    # Limits
    baseLimits = [3, 10]
    leftLimits = [5, 11]
    rightLimits = [5, 10]
    handLimits = [8, 9.9]
    # Servo Frequency
    servoFreq = 50


    def mapping(value, limits):
        'A function that translates a value within a range to the relative between 0 and 100'
        span = limits[1] - limits[0]
        return 100*float(value - limits[0])/float(span)

    def setBase(self, value):
        dutyCycle = mapping(value, baseLimits)
        self.base.ChangeDutyCycle(dutyCycle)
        time(1)

    def setLeft(self, value):
        dutyCycle = mapping(value, leftLimits)
        self.left.ChangeDutyCycle(dutyCycle)
        time(1)

    def setRight(self, value):
        dutyCycle = mapping(value, rightLimits)
        self.right.ChangeDutyCycle(dutyCycle)
        time(1)

    def setHand(self, value):
        dutyCycle = mapping(value, handLimits)
        self.right.ChangeDutyCycle(dutyCycle)
        time(1)

    
    def closeConn(self):
        'Stops PWM and closes GPIO connection'
        self.base.stop()
        self.left.stop()
        self.right.stop()
        self.hand.stop()
    
    def __init__(self, baseServoPin, leftServoPin, rightServoPin, handServoPin):
        # Setting pins as outputs
        GPIO.setup(baseServoPin, GPIO.OUT)
        GPIO.setup(leftServoPin, GPIO.OUT)
        GPIO.setup(rightServoPin, GPIO.OUT)
        GPIO.setup(handServoPin, GPIO.OUT)
        
        # Instantiating PWM
        self.base = GPIO.PWM(baseServoPin, servoFreq)
        self.left = GPIO.PWM(leftServoPin, servoFreq)
        self.right = GPIO.PWM(rightServoPin, servoFreq)
        self.hand = GPIO.PWM(handServoPin, servoFreq)
        
        # Starting PWM and defining initial Duty Cycle
        self.base.start(7.5)
        self.left.start(8)
        self.right.start(5)
        self.hand.start(9.9)

