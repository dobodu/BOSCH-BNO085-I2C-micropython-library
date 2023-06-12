# BNO08X Micropython I2C Test programm by Dobodu
#
# This program set up an I2C connection to the BNO08x device
# Then Create a BNO08X class based object
# Then enables sensors
# And finally report sensors every 0.5 seconds.
#
# Original Code from Adafruit CircuitPython Library


from machine import I2C, Pin
import time
import math
from bno08x_i2c import *

I2C0_SDA = Pin(16)
I2C0_SCL = Pin(17)

def quaternion_to_euler(i, j, k, w):
    jsqr = j * j

    t0 = +2.0 * (w * i + j * k)
    t1 = +1.0 - 2.0 * (i * i + jsqr)
    Roll = math.degrees(math.atan2(t0, t1))

    t2 = +2.0 * (w * j - k * i)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Tilt = math.degrees(math.asin(t2))

    t3 = +2.0 * (w * k + i * j)
    t4 = +1.0 - 2.0 * (jsqr + k * k)
    Pan = math.degrees(math.atan2(t3, t4))

    return Roll, Tilt, Pan

i2c0 = I2C(0, scl=I2C0_SCL, sda=I2C0_SDA, freq=100000, timeout=200000 )
print("I2C Device found at address : ",i2c0.scan(),"\n")

bno = BNO08X_I2C(i2c0, debug=False)
print("BNO08x I2C connection : Done\n")

bno.enable_feature(BNO_REPORT_ACCELEROMETER)
bno.enable_feature(BNO_REPORT_MAGNETOMETER)
bno.enable_feature(BNO_REPORT_GYROSCOPE)
bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

print("BNO08x sensors enabling : Done\n")

while True:
    time.sleep(0.5)
    accel_x, accel_y, accel_z = bno.acceleration  # pylint:disable=no-member
    print("Acceleration\tX: %0.6f\tY: %0.6f\tZ: %0.6f\tm/s²" % (accel_x, accel_y, accel_z))
    gyro_x, gyro_y, gyro_z = bno.gyro  # pylint:disable=no-member
    print("Gyroscope\tX: %0.6f\tY: %0.6f\tZ: %0.6f\trads/s" % (gyro_x, gyro_y, gyro_z))
    mag_x, mag_y, mag_z = bno.magnetic  # pylint:disable=no-member
    print("Magnetometer\tX: %0.6f\tY: %0.6f\tZ: %0.6f\tuT" % (mag_x, mag_y, mag_z))
    quat_i, quat_j, quat_k, quat_real = bno.quaternion  # pylint:disable=no-member
    print("Rot Vect Quat\tI: %0.6f\tJ: %0.6f\tK: %0.6f\tReal: %0.6f" % (quat_i, quat_j, quat_k, quat_real))
    R, T, P = quaternion_to_euler(quat_i, quat_j, quat_k, quat_real)
    print("Euler Angle\tX: %0.1f\tY: %0.1f\tZ: %0.1f" % (R, T, P))
    print("")
