import time
from picobot_arm import PicoBot

p = PicoBot()
p.initArm()
print('rotate to 90')
p.rotateArmBase(90)
time.sleep_ms(500)
print('rotate to 180')
p.rotateArmBase(180)
time.sleep_ms(500)
print('rotate to 0')
p.rotateArmBase(0)
time.sleep_ms(500)
print('rotate to 180')
p.rotateArmBase(180)
time.sleep_ms(500)
print('rotate to 90')
p.rotateArmBase(90)
time.sleep_ms(500)


