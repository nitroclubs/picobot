'''
picobot.py

Class to encapsulate PicoBot control
'''
import time
#import picobot_motors
from picobot_motors import MotorDriver
from picobot_servos import ArmServos
class PicoBot:
    def __init__(self):
        #self.m = picobot_motors.MotorDriver()
        self.arm = ArmServos()
        time.sleep(1)
        self.motors = MotorDriver()
        time.sleep(1)
        print("PicoBot drivers initialized!")
        #Parameter 1: motor select:'LeftFront', 'LeftBack', 'RightFront', 'RightBack'
        #Parameter 2: turn dir:'forward', 'backward'
        #Parameter 3: motor speed: 0-100
        #Parameter 4: Running time: >0

    ## -----------------
    ##  Picobot Motor Control
    ## -----------------
    def stop_all_motors(self):
        self.motors.StopAll()
        
    def starf_left(self, speed = 50):
    # Default speed is 50    
        self.motors.Turn('LeftFront', 'backward', speed)
        self.motors.Turn('LeftBack', 'forward', speed) # rename to LeftRearMotor??????????
        self.motors.Turn('RightFront', 'forward', speed)
        self.motors.Turn('RightBack', 'backward', speed)

    def starf_right(self, speed = 50):
    # Default speed = 50 is 50 %    
        self.motors.Turn('LeftFront', 'forward', speed)
        self.motors.Turn('LeftBack', 'backward', speed)
        self.motors.Turn('RightFront', 'backward', speed)
        self.motors.Turn('RightBack', 'forward', speed)
        

    def goForward(self, speed = 50):
        self.motors.Turn('LeftFront', 'forward', speed)
        self.motors.Turn('LeftBack', 'forward', speed)
        self.motors.Turn('RightFront', 'forward', speed)
        self.motors.Turn('RightBack', 'forward', speed)
        
    def goBackwad(self, speed = 50):
        self.motors.Turn('LeftFront', 'backward', speed)
        self.motors.Turn('LeftBack', 'backward', speed)
        self.motors.Turn('RightFront', 'backward', speed)
        self.motors.Turn('RightBack', 'backward', speed)

    def moveRight(self, speed = 50):
        self.motors.Turn('LeftFront', 'forward', speed)
        self.motors.Turn('LeftBack', 'backward', speed)
        self.motors.Turn('RightFront', 'backward', speed)
        self.motors.Turn('RightBack', 'forward', speed)
        
    def moveLeft(self, speed = 50):
        self.motors.Turn('LeftFront', 'backward', speed)
        self.motors.Turn('LeftBack', 'forward', speed)
        self.motors.Turn('RightFront', 'forward', speed)
        self.motors.Turn('RightBack', 'backward', speed)
        
    def moveRightForward(self, speed = 50):
        self.motors.Turn('LeftFront', 'forward', speed)
        self.motors.Turn('RightBack', 'forward', speed)
        
    def moveRightBackward(self, speed = 50):
        self.motors.Turn('LeftBack', 'backward', speed)
        self.motors.Turn('RightFront', 'backward', speed)
        
    def moveLeftForward(self, speed = 50):
        self.motors.Turn('LeftBack', 'forward', speed)
        self.motors.Turn('RightFront', 'forward', speed)
        
    def moveLeftBackward(self, speed = 50):
        self.motors.Turn('LeftFront', 'backward', speed)
        self.motors.Turn('RightBack', 'backward', speed)
        
    def rotateRight(self, speed = 50):
        self.motors.Turn('LeftFront', 'forward', speed)
        self.motors.Turn('LeftBack', 'forward', speed)
        self.motors.Turn('RightFront', 'backward', speed)
        self.motors.Turn('RightBack', 'backward', speed)

    def rotateLeft(self, speed = 50):
        self.motors.Turn('LeftFront', 'backward', speed)
        self.motors.Turn('LeftBack', 'backward', speed)
        self.motors.Turn('RightFront', 'forward', speed)
        self.motors.Turn('RightBack', 'forward', speed)

    def stopRobot(self, delay_ms = 10): #??? do we need delay
        self.motors.StopAll()
        time.sleep(delay_ms)
        
    def hardStop(self):
        self.motors.StopAll()

    ## -----------------
    ##  Picobot Arm Control
    ## -----------------

    def initArm(self):
        self.arm.initialPosition()

    def rotateArmBase(self, angle):
        self.arm.rotateBaseServo(angle)


