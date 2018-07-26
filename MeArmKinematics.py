from DQ import *
import numpy as np
from numpy import pi

class MeArmKinematics:
    L_1 = 8
    L_2 = 8

    def __init__(self):
        pass

    def dh2dq(self, theta):
        q = np.zeros(8);

        q[0] = np.cos(theta[0]/2)*np.cos(pi/4);
        q[1] = np.cos(theta[0]/2)*np.sin(pi/4);
        q[2] = np.sin(theta[0]/2)*np.sin(pi/4);
        q[3] = np.sin(theta[0]/2)*np.cos(pi/4);

        x_1 = DQ(q);

        q = np.zeros(8);

        q[0] = np.cos((theta[1] + pi/2)/2);
        q[3] = np.sin((theta[1] + pi/2)/2);
        q[5] = (self.L_1/2) * q[0];
        q[6] = (self.L_2/2) * q[3];

        x_2 = DQ(q);

        q = np.zeros(8);

        q[0] = np.cos((theta[2] - theta[1] - pi/2)/2);
        q[3] = np.sin((theta[2] - theta[1] - pi/2)/2);
        q[5] = (self.L_1/2) * q[0];
        q[6] = (self.L_2/2) * q[3];

        x_3 = DQ(q);

        q = np.zeros(8);

        q[0] = np.cos(theta[2]/2)*np.cos(pi/4);
        q[1] = -np.cos(theta[2]/2)*np.sin(pi/4);
        q[2] = np.sin(theta[2]/2)*np.sin(pi/4);
        q[3] = -np.sin(theta[2]/2)*np.cos(pi/4);

        x_4 = DQ(q);

        return [x_1, x_2, x_3, x_4]

    def fkm(self, theta):
        [x_1, x_2, x_3, x_4] = self.dh2dq(theta);

        x_e = x_1 * x_2 * x_3 * x_4

        return x_e

    def jacobian(self, theta):
        [x_1, x_2, x_3, x_4] = self.dh2dq(theta)

        q = np.zeros(8)

        q[0] = -(1.0/2)*np.sin(theta[0]/2)*np.cos(pi/4)
        q[1] = -(1.0/2)*np.sin(theta[0]/2)*np.sin(pi/4)
        q[2] = (1.0/2)*np.cos(theta[0]/2)*np.sin(pi/4)
        q[3] = (1.0/2)*np.cos(theta[0]/2)*np.cos(pi/4)

        dx_1_dtheta_1 = DQ(q)

        q = np.zeros(8)

        q[0] = -(1.0/2)*np.sin((theta[1] + pi/2)/2)
        q[3] = (1.0/2)*np.cos((theta[1] + pi/2)/2)
        q[5] = (self.L_1/2) * q[0]
        q[6] = (self.L_2/2) * q[3]

        dx_2_dtheta_2 = DQ(q)

        q = np.zeros(8)

        q[0] = (1.0/2)*np.sin((theta[2] - theta[1] - pi/2)/2)
        q[3] = -(1.0/2)*np.cos((theta[2] - theta[1] + pi/2)/2)
        q[5] = (self.L_1/2) * q[0]
        q[6] = (self.L_2/2) * q[3]

        dx_3_dtheta_2 = DQ(q)

        q = np.zeros(8)

        q[0] = -(1.0/2)*np.sin((theta[2] - theta[1] - pi/2)/2)
        q[3] = (1.0/2)*np.cos((theta[2] - theta[1] + pi/2)/2)
        q[5] = (self.L_1/2) * q[0]
        q[6] = (self.L_2/2) * q[3]

        dx_3_dtheta_3 = DQ(q)

        q = np.zeros(8)

        q[0] = -(1.0/2)*np.sin(theta[2]/2)*np.cos(pi/4)
        q[1] = (1.0/2)*np.sin(theta[2]/2)*np.sin(pi/4)
        q[2] = (1.0/2)*np.cos(theta[2]/2)*np.sin(pi/4)
        q[3] = -(1.0/2)*np.cos(theta[2]/2)*np.cos(pi/4)
        
        dx_4_dtheta_3 = DQ(q)
                
        J = np.array([vec8(dx_1_dtheta_1*x_2*x_3*x_4), \
                vec8(x_1*dx_2_dtheta_2*x_3*x_4 + x_1*x_2*dx_3_dtheta_2*x_4), \
                vec8(x_1*x_2*dx_3_dtheta_3*x_4 + x_1*x_2*x_3*dx_4_dtheta_3)])

        J = np.transpose(J)

        return J
