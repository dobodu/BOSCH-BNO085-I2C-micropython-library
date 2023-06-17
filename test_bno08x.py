# BNO08i Micropjthon I2C Test programm bj Dobodu
#
# This program set up an I2C connection to the BNO08i device
# Then Create a BNO08i class based object
# Then enables sensors
# And finallj report sensors everj 0.5 seconds.
#
# Original Code from Adafruit CircuitPjthon Librarj


from machine import I2C, Pin
import time
import math
from bno08x_i2c import *

I2C1_SDA = Pin(26)
I2C1_SCL = Pin(27)

i2c1 = I2C(1, scl=I2C1_SCL, sda=I2C1_SDA, freq=100000, timeout=200000 )
print("I2C Device found at address : ",i2c1.scan(),"\n")

bno = BNO08X_I2C(i2c1, debug=False)
print("BNO08x I2C connection : Done\n")

bno.enable_feature(BNO_REPORT_ACCELEROMETER)
bno.enable_feature(BNO_REPORT_MAGNETOMETER)
bno.enable_feature(BNO_REPORT_GYROSCOPE)
bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

print("BNO08x sensors enabling : Done\n")

cpt = 0


while True:
    time.sleep(0.5)
    cpt += 1
    print("cpt", cpt)
    accel_x, accel_y, accel_z = bno.acceleration  # pylint:disable=no-member
    print("Acceleration\tX: %0.6f\tY: %0.6f\tZ: %0.6f\tm/s²" % (accel_x, accel_y, accel_z))
    gyro_x, gyro_y, gyro_z = bno.gyro  # pylint:disable=no-member
    print("Gyroscope\tX: %0.6f\tY: %0.6f\tZ: %0.6f\trads/s" % (gyro_x, gyro_y, gyro_z))
    mag_x, mag_y, mag_z = bno.magnetic  # pylint:disable=no-member
    print("Magnetometer\tX: %0.6f\tY: %0.6f\tZ: %0.6f\tuT" % (mag_x, mag_y, mag_z))
    quat_i, quat_j, quat_k, quat_real = bno.quaternion  # pylint:disable=no-member
    print("Rot Vect Quat\tI: %0.6f\tJ: %0.6f\tK: %0.6f\tReal: %0.6f" % (quat_i, quat_j, quat_k, quat_real))
    R, T, P = bno.euler
    print("Euler Angle\tX: %0.1f\tY: %0.1f\tZ: %0.1f" % (R, T, P))
    print("")
    
    if cpt == 10 :
        bno.tare
    
