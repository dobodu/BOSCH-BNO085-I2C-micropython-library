# Adafruit_MicroPython_BNO08x
Adafruit Micropython I2C library for BNO08X

100% inspired by the original Adafruit Circuitpython I2C library for BNO08X
Copyright (c) 2020 Bryan Siepert for Adafruit Industries


I NEED HELP FOR DEBUGGING THIS PROJECT



Need an I2C setup like

    i2c0 = I2C(0, scl=I2C1_SCL, sda=I2C1_SDA, freq=100000, timeout=200000 )

BNO is declared thanks to

    bno = BNO08X_I2C(i2c0)

Implentation of sensors is done throught

    bno.enable_feature(BNO_REPORT_ACCELEROMETER)  for accelerometer
    
Sensors values are obtained with

    accel_z = bno.acceleration
