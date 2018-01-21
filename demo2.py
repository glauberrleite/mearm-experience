from MeArm import MeArm
import time

white = MeArm(16, 12, 15, 11)

time.sleep(1)

white.setBase(30)

white.setRight(45)

white.setHand(50)

time.sleep(0.5)

white.setHand(100)

white.setRight(0)

white.setBase(0)

white.setHand(50)

time.sleep(0.5)

white.setHand(100)

white.closeConn()
