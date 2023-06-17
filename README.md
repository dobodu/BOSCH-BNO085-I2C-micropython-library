# Adafruit_MicroPython_BNO08x
Adafruit Micropython I2C library for BNO08X ( BNO080, BNO085, aso )

100% inspired by the original Adafruit Circuitpython I2C library for BNO08X
Copyright (c) 2020 Bryan Siepert for Adafruit Industries


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
