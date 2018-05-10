import RPi.GPIO as GPIO
import numpy
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
    rightAngles = [-90, 30]
    handAngles = [0, 100]
    # Servo Frequency
    servoFreq = 50

    def __mapping(self, value,  anglesLimits, limits):
        'A function that translates a value within a range of angles to a value within the DutyCycle Limits'
        span = anglesLimits[1] - anglesLimits[0]
        
        scaled = float(value - anglesLimits[0])/float(span)

        # A little limit to avoid miscalculation
        if scaled > 1:
            scaled = 1
        elif scaled < 0:
            scaled = 0

        return limits[0] + (scaled * (limits[1] - limits[0]))

    def setBase(self, value):
        dutyCycle = self.__mapping(value, self.baseAngles, self.baseLimits)
        self.base.ChangeDutyCycle(dutyCycle)
        time.sleep(0.5)

    def setLeft(self, value):
        dutyCycle = self.__mapping(value, self.leftAngles, self.leftLimits)
        self.left.ChangeDutyCycle(dutyCycle)
        time.sleep(0.5)

    def setRight(self, value):
        dutyCycle = self.__mapping(value, self.rightAngles, self.rightLimits)
        self.right.ChangeDutyCycle(dutyCycle)
        time.sleep(0.5)

    def setHand(self, value):
        dutyCycle = self.__mapping(value, self.handAngles, self.handLimits)
        self.hand.ChangeDutyCycle(dutyCycle)
        time.sleep(0.5)

    def setTheta(self, theta):
        self.setBase(theta[0]);
        self.setRight(theta[1]);
        self.setLeft(theta[2]);

    # Inverse Kinematics
    def __invkine(self, x, y, z, l1, l2, offset):
        z = z - offset

        theta1 = numpy.arctan2(y, x)

        theta2 = numpy.arctan2(z, numpy.sqrt(x**2 + y**2)) \
                + numpy.arccos((x**2 + y**2 + z**2 + l1**2 - l2**2)\
                /(2 * l1 * numpy.sqrt(x**2 + y**2 + z**2)))

        theta3 = numpy.arccos((numpy.sqrt(x**2 + y**2) - l1 * numpy.cos(theta2))/l2)

        # To degrees
        theta1 = theta1 * 180 / numpy.pi
        theta2 = theta2 * 180 / numpy.pi
        theta3 = -(theta3 * 180 / numpy.pi)
        
        return [theta1, theta2, theta3]

    def setPos(self, x, y, z):
        'Given x, y and z, will put the hand at that location'
        l1 = 8.0
        l2 = 8.0
        offset = 6.0

        [theta1, theta2, theta3] = self.__invkine(x, y, z, l1, l2, offset)

        if self.verbose:
            print("theta1 = " + str(theta1))
            print("theta2 = " + str(theta2))
            print("theta3 = " + str(theta3))

        self.setBase(theta1)
        self.setRight(theta2)
        self.setLeft(theta3)

    def closeConn(self):
        'Stops PWM and closes GPIO connection'
        self.base.stop()
        self.left.stop()
        self.right.stop()
        self.hand.stop()
        GPIO.cleanup()
    
    def __init__(self, baseServoPin, leftServoPin, rightServoPin, handServoPin, verbose=False):
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

        self.verbose = verbose

