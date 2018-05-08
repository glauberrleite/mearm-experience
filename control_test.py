from MeArmKinematics import MeArmKinematics
from DQ import *
import numpy as np

kinematics = MeArmKinematics()

theta = np.array([0, 0, 0])
thetad = np.array([np.pi/2, -np.pi/6, 0])

xd = kinematics.fkm(thetad)

error = 1
epsilon = 0.001
K = 1

while np.linalg.norm(error) > epsilon:
    x = kinematics.fkm(theta)
    J = kinematics.jacobian(theta)
    error = vec8(xd - x)
    theta = theta + K * (np.linalg.pinv(J) @ error)

    print(theta)
