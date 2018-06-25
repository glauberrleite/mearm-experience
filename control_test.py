from MeArmKinematics import MeArmKinematics
from MeArm import MeArm
from DQ import *
import numpy as np
import time

kinematics = MeArmKinematics()
mearm = MeArm(baseServoPin = 4, rightServoPin = 17, leftServoPin = 27, handServoPin = 22)

theta = np.array([0, 0, 0])
thetad = np.array([np.pi/6, np.pi/6, -np.pi/4])

xd = kinematics.fkm(thetad)

error = 1
epsilon = 0.01
dt = 0.1

while np.linalg.norm(error) > epsilon:
    x = kinematics.fkm(theta)
    J = kinematics.jacobian(theta)
    error = vec8(xd - x)
    theta = theta + (np.linalg.pinv(J) @ error) * dt

    mearm.setTheta(theta * 180/np.pi)

    time.sleep(1)
    print(theta)


mearm.setHand(85)
mearm.closeConn()
