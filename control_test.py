from MeArmKinematics import MeArmKinematics
from MeArm import MeArm
from DQ import *
import numpy as np
import time
import sys

if len(sys.argv) == 1:
    print("Input the desired angles")
    quit()
else:
    thetad = np.array([float(sys.argv[1]) * np.pi / 180, float(sys.argv[2]) * np.pi / 180 , float(sys.argv[3]) * np.pi / 180])

kinematics = MeArmKinematics()
mearm = MeArm(baseServoPin = 4, rightServoPin = 17, leftServoPin = 27, handServoPin = 22)

theta = np.array([0, 0, 0])

xd = kinematics.fkm(thetad)

error = 1
epsilon = 0.1
dt = 0.1

while np.linalg.norm(error) > epsilon:
    x = kinematics.fkm(theta)
    J = kinematics.jacobian(theta)
    error = vec8(xd - x)
    theta = theta + np.linalg.pinv(J) @ error * dt

    mearm.setTheta(theta * 180/np.pi)

    time.sleep(0.0001)
    print(theta * 180 / np.pi)


mearm.setHand(95)
input()
mearm.closeConn()
