# BOSCH-BNO085-I2C-micropython-library
Micropython I2C library for 9 dof BOSCH BNO08X sensors ( BNO080, BNO085, aso )

100% inspired by the original Adafruit Circuitpython I2C library for BNO08X
Copyright (c) 2020 Bryan Siepert for Adafruit Industries

THIS LIBRARY IS WORKING ON RASPBERRY PICO AND ALSO ON ESP32-S3*
*ESP32 S3 need a firmware compiled with ESP-IDF 5.3.2 (maybe 5.3.1 will also work)

Need an I2C setup like

    i2c0 = I2C(0, scl=I2C1_SCL, sda=I2C1_SDA, freq=100000, timeout=200000 )

BNO is declared this way

    bno = BNO08X_I2C(i2c0)
    
but can be completed by optinal conditions

    bno = BNO08X_I2C(i2c_bus, address=None, reset_pin=None, debug=False)
    
Mandatory :     

i2c_bus

Optionnal :    

address : will try to find by itself, but if using 2 BNO08x you need to define it

reset_pin : if a pin identifier is defined, will try to hard reset, otherwise, soft reset only

debug : just in case...  

Implentation of sensors is done throught

    bno.enable_feature(BNO_REPORT_ACCELEROMETER)  for accelerometer
    
Please check #Reports Summary on top of bno08x_i2c to find your needs
    
Sensors values are obtained with

    accel_x, accel_y, accel_z = bno.acceleration

Roll Tilt and Pan are obtained with

    roll, tilt, pan = bno.euler
