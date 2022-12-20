import time
#import picobot_motors
from picobot_motors import MotorDriver
from picobot_servos import ArmServos
class PicoBot:
    def __init__(self):
        #self.m = picobot_motors.MotorDriver()
        self.arm = ArmServos()
        time.sleep(1)
        print("PicoBot arm driver initialized!")


    ## -----------------
    ##  Picobot Arm Control
    ## -----------------

    def initArm(self):
        self.arm.initialPosition()

    def rotateArmBase(self, angle):
        self.arm.rotateBaseServo(angle)
