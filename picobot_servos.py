import time
from machine import Pin, I2C
import math
import ustruct


class ArmController_PCA9685:
    """
    This class models the PCA9685 board, used to control up to 16
    servos, using just 2 wires for control over the I2C interface
    """

    def __init__(self, i2c, address=0x40):
        """
        class constructor

        Args:
            i2c ([I2C Class, from the build in machine library]): This is used to
            bring in the i2c object, which can be created by
            > i2c = I2C(id=0, sda=Pin(0), scl=Pin(1))
            address (hexadecimal, optional): [description]. Defaults to 0x40.
        """
        self.i2c = i2c
        self.address = address
        self.reset()

    def _write(self, address, value):
        self.i2c.writeto_mem(self.address, address, bytearray([value]))

    def _read(self, address):
        return self.i2c.readfrom_mem(self.address, address, 1)[0]

    def reset(self):
        self._write(0x00, 0x00)  # Mode1

    def freq(self, freq=None):
        if freq is None:
            return int(25000000.0 / 4096 / (self._read(0xfe) - 0.5))
        prescale = int(25000000.0 / 4096.0 / freq + 0.5)
        old_mode = self._read(0x00)  # Mode 1
        self._write(0x00, (old_mode & 0x7F) | 0x10)  # Mode 1, sleep
        self._write(0xfe, prescale)  # Prescale
        self._write(0x00, old_mode)  # Mode 1
        time.sleep_us(5)
        self._write(0x00, old_mode | 0xa1)  # Mode 1, autoincrement on

    def pwm(self, index, on=None, off=None):
        if on is None or off is None:
            data = self.i2c.readfrom_mem(self.address, 0x06 + 4 * index, 4)
            return ustruct.unpack('<HH', data)
        data = ustruct.pack('<HH', on, off)
        self.i2c.writeto_mem(self.address, 0x06 + 4 * index, data)

    def duty(self, index, value=None, invert=False):
        if value is None:
            pwm = self.pwm(index)
            if pwm == (0, 4096):
                value = 0
            elif pwm == (4096, 0):
                value = 4095
            value = pwm[1]
            if invert:
                value = 4095 - value
            return value
        if not 0 <= value <= 4095:
            raise ValueError("Out of range")
        if invert:
            value = 4095 - value
        if value == 0:
            self.pwm(index, 0, 4096)
        elif value == 4095:
            self.pwm(index, 4096, 0)
        else:
            self.pwm(index, 0, value)


###---------------------------------------------------

class ServoControl:
    def __init__(self, address=0x40, freq=50, min_us=600, max_us=2400,
                 degrees=180):
        self.period = 1000000 / freq
        self.min_duty = self._us2duty(min_us)
        self.max_duty = self._us2duty(max_us)
        self.degrees = degrees
        self.freq = freq
        self.i2c = I2C(1, scl=Pin(3), sda=Pin(2), freq=self.freq)
        self.pca9685 = ArmController_PCA9685(self.i2c, address)
        self.pca9685.freq(freq)

    def _us2duty(self, value):
        return int(4095 * value / self.period)

    def position(self, index, degrees=None, radians=None, us=None, duty=None):
        span = self.max_duty - self.min_duty
        if degrees is not None:
            duty = self.min_duty + span * degrees / self.degrees
        elif radians is not None:
            duty = self.min_duty + span * radians / math.radians(self.degrees)
        elif us is not None:
            duty = self._us2duty(us)
        elif duty is not None:
            pass
        else:
            return self.pca9685.duty(index)
        duty = min(self.max_duty, max(self.min_duty, int(duty)))
        self.pca9685.duty(index, duty)

    def release(self, index):
        self.pca9685.duty(index, 0)


###---------------------------------------------------

class ArmServos:
    def __init__(self, base_servo_index=0, base_servo_right_angle=0, base_servo_center_angle=90,
                 base_servo_left_angle=180,
                 joint_servo_index=1, joint_servo_extended_angle=0, joint_servo_retracted_angle=150,
                 joint_servo_initial_angle=150,
                 gripper_servo_index=2, gripper_servo_open_angle=45, gripper_servo_closed_angle=158):
        self.base_servo_index = base_servo_index
        self.base_servo_angle_center = base_servo_center_angle
        self.base_servo_angle_left = base_servo_left_angle
        self.base_servo_angle_right = base_servo_right_angle
        self.joint_servo_index = joint_servo_index
        self.joint_servo_initial_angle = joint_servo_initial_angle
        self.joint_servo_extended_angle = joint_servo_extended_angle
        self.joint_servo_retracted_angle = joint_servo_retracted_angle
        self.gripper_servo_index = gripper_servo_index
        self.gripper_servo_closed = gripper_servo_closed_angle
        self.gripper_servo_open = gripper_servo_open_angle
        self.currentBaseServoPosition = 90

        self.servo = ServoControl()

    def initialPosition(self):
        self.servo.position(index=self.base_servo_index, degrees=self.base_servo_angle_center)
        self.currentBaseServoPosition = self.base_servo_angle_center
        time.sleep_ms(500)
        self.servo.position(index=self.joint_servo_index, degrees=self.joint_servo_initial_angle)
        time.sleep_ms(500)
        self.servo.position(index=self.gripper_servo_index, degrees=self.gripper_servo_closed)
        time.sleep_ms(500)

    def rotateBaseServo(self, angle):
        sleep_time_ms = 10
        if angle == self.currentBaseServoPosition:
            print('target angle =  self.currentBaseServoPosition - SKIPPING')
            return
        elif angle > self.currentBaseServoPosition:
            targetPositionDiff = angle - self.currentBaseServoPosition
            print('target angle >  self.currentBaseServoPosition - target diff = ', targetPositionDiff)
            for angl in range(self.currentBaseServoPosition + 1, angle + 1, 1):
                self.servo.position(index=self.base_servo_index, degrees=angl)
                self.currentBaseServoPosition = angl
                time.sleep_ms(sleep_time_ms)
                print('for loop > | angl = ', angl)

        else:
            targetPositionDiff = self.currentBaseServoPosition - angle
            print('target angle <  self.currentBaseServoPosition - target diff = {}', targetPositionDiff)
            for angl in range(self.currentBaseServoPosition - 1, angle - 1, -1):
                self.servo.position(index=self.base_servo_index, degrees=angl)
                self.currentBaseServoPosition = angl
                time.sleep_ms(sleep_time_ms)
                print('for loop < | angl = ', angl)

    def rotateJointServo(self, angle, speed):
        pass

    def rotateGripperServo(self, angle, speed):
        pass
