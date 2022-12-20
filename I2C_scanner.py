import machine

sdaPIN0 = machine.Pin(20)
sclPIN0 = machine.Pin(21)

sdaPIN1 = machine.Pin(2)
sclPIN1 = machine.Pin(3)

print("Scanning I2C 0 - sda on Pin(20), scl on Pin(21)")

i2c0 = machine.I2C(0, sda=sdaPIN0, scl=sclPIN0, freq=400000)

devices0 = i2c0.scan()
if len(devices0) == 0:
    print("No i2c device on channel 0 !")
else:
    print('i2c devices found on channel 0:', len(devices0))
for device0 in devices0:
    print("At address: ", hex(device0))

print("Scanning I2C 0 - sda on Pin(2), scl on Pin(3)")

i2c1 = machine.I2C(1, sda=sdaPIN1, scl=sclPIN1, freq=400000)

devices1 = i2c1.scan()
if len(devices1) == 0:
    print("No i2c device on channel 1 !")
else:
    print('i2c devices found on channel 1:', len(devices1))
for device1 in devices1:
    print("At address: ", hex(device1))