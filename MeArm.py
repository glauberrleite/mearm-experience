import pigpio
import numpy
import time

class MeArm:
    'An interface for GPIO operations in MeArm robotic arm'
    # Servo DutyCycle Limits based on its place at the Arm 
    baseLimits = [900, 2100]
    leftLimits = [800, 1800]
    rightLimits = [700, 1900]
    handLimits = [1600, 2100]
    # Angles limits for each servo
    baseAngles = [-45, 45]
    leftAngles = [-65, 20]
    rightAngles = [-45, 90]
    handAngles = [0, 100]

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
        theta1 = self.__mapping(value, self.baseAngles, self.baseLimits)
        self.pi.set_servo_pulsewidth(self.baseServoPin, theta1)

    def setLeft(self, value):
        theta3 = self.__mapping(value, self.leftAngles, self.leftLimits)
        self.pi.set_servo_pulsewidth(self.leftServoPin, theta3)

    def setRight(self, value):
        theta2 = self.__mapping(value, self.rightAngles, self.rightLimits)
        self.pi.set_servo_pulsewidth(self.rightServoPin, theta2)

    def setHand(self, value):
        openess = self.__mapping(value, self.handAngles, self.handLimits)
        self.pi.set_servo_pulsewidth(self.handServoPin, openess)

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
        self.pi.stop()
    
    def __init__(self, baseServoPin, leftServoPin, rightServoPin, handServoPin, verbose=False):
        
        self.baseServoPin = baseServoPin
        self.leftServoPin = leftServoPin
        self.rightServoPin = rightServoPin
        self.handServoPin = handServoPin

        self.pi = pigpio.pi()

        # Setting pins as outputs
        self.pi.set_mode(baseServoPin, pigpio.OUTPUT)
        self.pi.set_mode(leftServoPin, pigpio.OUTPUT)
        self.pi.set_mode(rightServoPin, pigpio.OUTPUT)
        self.pi.set_mode(handServoPin, pigpio.OUTPUT)
        
        self.setTheta([0, 0, 0])
        self.setHand(0)

        self.verbose = verbose

