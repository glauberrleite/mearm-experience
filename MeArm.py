import RPi.GPIO as GPIO
import time

class MeArm:
    'An interface for GPIO operations in MeArm robotic arm'
    # Servo DutyCycle Limits based on its place at the Arm 
    baseLimits = [4, 10]
    leftLimits = [5, 10]
    rightLimits = [3.5, 10]
    handLimits = [8, 9.9]
    # Angles limits for each servo
    baseAngles = [-45, 45]
    leftAngles = [-75, 0]
    rightAngles = [-20, 90]
    handAngles = [0, 100]
    # Servo Frequency
    servoFreq = 50


    def mapping(value,  anglesLimits, limits):
        'A function that translates a value within a range of angles to a value within the DutyCycle Limits'
        span = anglesLimits[1] - anglesLimits[0]
        
        scaled = float(value - anglesLimits[0])/float(span)
        return limits[0] + (scaled * (limits[1] - limits[0]))

    def setBase(self, value):
        dutyCycle = mapping(value, baseAngles, baseLimits)
        self.base.ChangeDutyCycle(dutyCycle)
        time(1)

    def setLeft(self, value):
        dutyCycle = mapping(value, leftAngles, leftLimits)
        self.left.ChangeDutyCycle(dutyCycle)
        time(1)

    def setRight(self, value):
        dutyCycle = mapping(value, rightAngles, rightLimits)
        self.right.ChangeDutyCycle(dutyCycle)
        time(1)

    def setHand(self, value):
        dutyCycle = mapping(value, handAngles, handLimits)
        self.right.ChangeDutyCycle(dutyCycle)
        time(1)

    
    def closeConn(self):
        'Stops PWM and closes GPIO connection'
        self.base.stop()
        self.left.stop()
        self.right.stop()
        self.hand.stop()
        GPIO.cleanup()
    
    def __init__(self, baseServoPin, leftServoPin, rightServoPin, handServoPin):
        GPIO.setmode(GPIO.BOARD)
        # Setting pins as outputs
        GPIO.setup(baseServoPin, GPIO.OUT)
        GPIO.setup(leftServoPin, GPIO.OUT)
        GPIO.setup(rightServoPin, GPIO.OUT)
        GPIO.setup(handServoPin, GPIO.OUT)
        
        # Instantiating PWM
        self.base = GPIO.PWM(baseServoPin, self.servoFreq)
        self.left = GPIO.PWM(leftServoPin, self.servoFreq)
        self.right = GPIO.PWM(rightServoPin, self.servoFreq)
        self.hand = GPIO.PWM(handServoPin, self.servoFreq)
        
        # Starting PWM and defining initial Duty Cycle
        self.base.start(7)
        self.left.start(8)
        self.right.start(4.7)
        self.hand.start(9.9)

