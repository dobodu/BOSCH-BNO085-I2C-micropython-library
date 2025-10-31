# raw_sensors.py
#
# BNO08x Micropython I2C example program
# Raw device reports: Raw_Accelerometer, Raw_Magnetometer, Raw_Gyroscope
# acc_raw amd mag_raw return 3 values and time_stamp
# gyro_raw return 3 values, celsius, and time_stamp

from bno08x import *
from machine import I2C, Pin
from utime import ticks_ms, sleep_ms

I2C0_SDA = Pin(12)
I2C0_SCL = Pin(13)

i2c0 = I2C(0, scl=I2C0_SCL, sda=I2C0_SDA, freq=100_000, timeout=200_000)

bno = BNO08X(i2c0, debug=False)
print("BNO08x I2C connection : Done\n")

bno.enable_feature(BNO_REPORT_RAW_ACCELEROMETER, 20)
bno.enable_feature(BNO_REPORT_RAW_MAGNETOMETER, 20)
bno.enable_feature(BNO_REPORT_RAW_GYROSCOPE, 20)

print("BNO08x sensors enabled : Done\n")

print(f"{bno.calibration=}")
print(f"{bno.calibration_status}")

cpt = 0
timer_origin = ticks_ms()
average_delay = -1

while True:
    cpt += 1
    print("cpt", cpt)
    accel_x, accel_y, accel_z, time_stamp = bno.acc_raw
    print(f"Raw Acceleration\tX: {accel_x:+.3f}\tY: {accel_y:+.3f}\tZ: {accel_z:+.3f}\t {time_stamp=}")
    gyro_x, gyro_y, gyro_z, celsius, time_stamp = bno.gyro_raw
    print(f"Raw Gyroscope\tX: {gyro_x:+.3f}\tY: {gyro_y:+.3f}\tZ: {gyro_z:+.3f}\t {celsius=} {time_stamp=}")
    mag_x, mag_y, mag_z, time_stamp = bno.mag_raw
    print(f"Raw Magnetometer\tX: {mag_x:+.3f}\tY: {mag_y:+.3f}\tZ: {mag_z:+.3f}\t {time_stamp=}")

    print("===================================")
    timer = ticks_ms()
    average_delay = (timer - timer_origin) / cpt
    print(f"average delay times (ms) : {average_delay:.1f}")
    print("===================================")

    if cpt == 10:
        bno.tare
