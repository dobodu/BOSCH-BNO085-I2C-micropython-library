# raw_sensors.py
#
# BNO08x Micropython I2C example program
# Magnetometer report and query magnetic accuracy

from bno08x import *
from machine import I2C, Pin
from utime import ticks_ms, sleep_ms

I2C0_SDA = Pin(12)
I2C0_SCL = Pin(13)

i2c0 = I2C(0, scl=I2C0_SCL, sda=I2C0_SDA, freq=100_000, timeout=200_000)

bno = BNO08X(i2c0, debug=False)
print("BNO08x I2C connection : Done\n")

bno.enable_feature(BNO_REPORT_MAGNETOMETER, 10)
print("BNO086 Magnetometer report enabled\n")

print("Begin calibration\n")
bno.calibration()

cpt = 0
timer_origin = ticks_ms()
average_delay = -1

while True:
    cpt += 1
    print("cpt", cpt)

    mag_x, mag_y, mag_z = bno.mag
    print(f"Magnetometer  X: {mag_x:.6f}  Y: {mag_y:.6f}  Z: {mag_z:.6f} uT")

    calibration_status = bno.calibration_status

    # print accuracy string and value
    print(f"Mag Calibration: {REPORT_ACCURACY_STATUS[calibration_status]}={calibration_status}")
