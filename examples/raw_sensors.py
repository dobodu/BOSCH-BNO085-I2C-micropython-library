# raw_sensors.py
#
# BNO08x Micropython I2C example program
# Raw device reports: Raw_Accelerometer, Raw_Magnetometer, Raw_Gyroscope
# acc_raw amd mag_raw return 3 values and time_stamp
# gyro_raw return 3 values, Celsius, and time_stamp

from bno08x import *
from machine import I2C, Pin
from utime import ticks_ms

i2c0 = I2C(0, scl=Pin(13), sda=Pin(12), freq=100_000, timeout=200_000)
bno = BNO08X(i2c0, debug=False)
print("BNO08x I2C connection : Done\n")
print("I2C devices found:", [hex(d) for d in i2c0.scan()])

bno.enable_feature(BNO_REPORT_RAW_ACCELEROMETER, 20)
bno.enable_feature(BNO_REPORT_RAW_MAGNETOMETER, 20)
bno.enable_feature(BNO_REPORT_RAW_GYROSCOPE, 20)
print("BNO08x sensors enabled\n")

cpt = 0
timer_origin = ticks_ms()
average_delay = -1

while True:
    cpt += 1
    print("\ncpt", cpt)

    accel_x, accel_y, accel_z, time_stamp = bno.acc_raw
    print(f"Raw Acceleration:  X: {accel_x:#06x}  Y: {accel_y:#06x}  Z: {accel_z:#06x}  {time_stamp=}")

    mag_x, mag_y, mag_z, time_stamp = bno.mag_raw
    print(f"Raw Magnetometer:  X: {mag_x:#06x}  Y: {mag_y:#06x}  Z: {mag_z:#06x}  {time_stamp=}")

    gyro_x, gyro_y, gyro_z, celsius, time_stamp = bno.gyro_raw
    print(f"Raw Gyroscope:     X: {gyro_x:#06x}  Y: {gyro_y:#06x}  Z: {gyro_z:#06x}  {celsius=}  {time_stamp=}")

    print("===================================")
    timer = ticks_ms()
    average_delay = (timer - timer_origin) / cpt
    print(f"average delay times (ms) : {average_delay:.1f}")
    print("===================================")

    if cpt == 10:
        bno.tare
