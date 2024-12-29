# BOSCH-BNO085-I2C-micropython-library
## Micropython I2C library for 9 dof BOSCH BNO08X sensors

- 100% inspired by the original Adafruit Circuitpython I2C library for BNO08X
- Copyright (c) 2020 Bryan Siepert for Adafruit Industries

## This library is working with:

|  Manufacturer |  Chips  |  Observations |
| ------------ | ------------ | ------------ | 
|  Raspberry | Pico, Pico W,  Pico 2,  Pico 2W   |   |
|  Espressif | Esp32 S2, Esp32 S3 |  See Requirements |

ESP32 S3 need a firmware compiled with ESP-IDF 5.3.2 (maybe 5.3.1 will also work)
[Firmware for Lilygo AMOLED displays](https://github.com/dobodu/Lilygo-Amoled-Micropython/blob/main/firmware/firmware_2024_12_28.bin "Firmware for Lilygo AMOLED displays")

## How to setup

Need an I2C setup like

        #import the library
    import bno08x

    #setup the  I2C bus
    i2c0 = I2C(0, scl=I2C1_SCL, sda=I2C1_SDA, freq=100000, timeout=200000 )

    #setup the BNO sensor
    bno = BNO08x(i2c0)


but can be completed by optinal conditions

    bno = BNO08X_I2C(i2c_bus, address=None, reset_pin=None, debug=False)

**Mandatory :**

- i2c_bus

**Optionnal :**

- address : will try to find by itself, but if using 2 BNO08x you need to define it
- reset_pin : if a pin identifier is defined, will try to hard reset, otherwise, soft reset only
- debug : just in case...  

**Implentation of sensors is done throught**

    bno.enable_feature(BNO_REPORT_ACCELEROMETER)  # for accelerometer
    
Available sensors reports are :

    BNO_REPORT_ACCELEROMETER
    BNO_REPORT_GYROSCOPE
    BNO_REPORT_MAGNETOMETER
    BNO_REPORT_LINEAR_ACCELERATION
    BNO_REPORT_ROTATION_VECTOR
    BNO_REPORT_GRAVITY
    BNO_REPORT_UNCALIBRATED_GYROSCOPE
    BNO_REPORT_GAME_ROTATION_VECTOR
    BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR
    BNO_REPORT_PRESSURE
    BNO_REPORT_AMBIENT_LIGHT
    BNO_REPORT_HUMIDITY
    BNO_REPORT_PROXIMITY
    BNO_REPORT_TEMPERATURE
    BNO_REPORT_UNCALIBRATED_MAGNETOMETER
    BNO_REPORT_TAP_DETECTOR
    BNO_REPORT_STEP_COUNTER
    BNO_REPORT_SIGNIFICANT_MOTION
    BNO_REPORT_STABILITY_CLASSIFIER
    BNO_REPORT_RAW_ACCELEROMETER
    BNO_REPORT_RAW_GYROSCOPE
    BNO_REPORT_RAW_MAGNETOMETER
    BNO_REPORT_SAR
    BNO_REPORT_STEP_DETECTOR
    BNO_REPORT_SHAKE_DETECTOR
    BNO_REPORT_FLIP_DETECTOR
    BNO_REPORT_PICKUP_DETECTOR
    BNO_REPORT_STABILITY_DETECTOR
    BNO_REPORT_ACTIVITY_CLASSIFIER
    BNO_REPORT_SLEEP_DETECTOR
    BNO_REPORT_TILT_DETECTOR
    BNO_REPORT_POCKET_DETECTOR
    BNO_REPORT_CIRCLE_DETECTOR
    BNO_REPORT_HEART_RATE_MONITOR
    BNO_REPORT_ARVR_STABILIZED_ROTATION_VECTOR
    BNO_REPORT_ARVR_STABILIZED_GAME_ROTATION_VECTOR
    BNO_REPORT_GYRO_INTEGRATED_ROTATION_VECTOR
    
Sensors values can be reached with

    accel_x, accel_y, accel_z = bno.acc

Roll Tilt and Pancan be obtained with

    roll, tilt, pan = bno.euler
