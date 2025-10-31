# BOSCH-BNO085-I2C-micropython-library
## Micropython I2C library for 9 dof BOSCH BNO08X sensors

- 100% inspired by the original Adafruit Circuitpython I2C library for BNO08X
- Copyright (c) 2020 Bryan Siepert for Adafruit Industries

## This library is working with:

|  Manufacturer |  Chips  |  Observations |
| ------------ | ------------ | ------------ | 
|  Raspberry | Pico, Pico W,  Pico 2,  Pico 2 W   |   |
|  Espressif | Esp32 S2, Esp32 S3 |  See Requirements |

This library has been tested with BNO080, BNO085, and BNO086 sensors.

ESP32 S3 need a firmware compiled with ESP-IDF 5.3.2 (maybe 5.3.1 will also work)
[Firmware for Lilygo AMOLED displays](https://github.com/dobodu/Lilygo-Amoled-Micropython/blob/main/firmware/firmware_2024_12_28.bin "Firmware for Lilygo AMOLED displays")

## How to setup

Need to set up the I2C

    #import the library
    import bno08x

    #setup the  I2C bus
    i2c0 = I2C(0, scl=I2C0_SCL, sda=I2C0_SDA, freq=100000, timeout=200000)

    #setup the BNO sensor
    bno = BNO08x(i2c0)

but can be completed by optional conditions

    bno = BNO08X_I2C(i2c_bus, address=None, rst_pin=14, debug=False)

**Mandatory :**

- i2c_bus

**Optional :**

- address : will find by itself, but if using 2 BNO08x you need to define it
- rst_pin : if a pin identifier (Pin number, not Pin object) is defined, will do a hard reset, otherwise, soft reset will be used
- debug : print logs from driver 

**Enable the sensor reports**

    bno.enable_feature(BNO_REPORT_ACCELEROMETER)  # for accelerometer
    
Available sensors reports are :

        BNO_REPORT_ACCELEROMETER
        BNO_REPORT_GYROSCOPE
        BNO_REPORT_MAGNETOMETER
        BNO_REPORT_LINEAR_ACCELERATION
        BNO_REPORT_ROTATION_VECTOR
        BNO_REPORT_GRAVITY
        BNO_REPORT_GAME_ROTATION_VECTOR
        BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR
        BNO_REPORT_PRESSURE
        BNO_REPORT_AMBIENT_LIGHT
        BNO_REPORT_HUMIDITY
        BNO_REPORT_PROXIMITY
        BNO_REPORT_TEMPERATURE
        BNO_REPORT_STEP_COUNTER
        BNO_REPORT_SHAKE_DETECTOR
        BNO_REPORT_STABILITY_CLASSIFIER
        BNO_REPORT_ACTIVITY_CLASSIFIER
        BNO_REPORT_RAW_ACCELEROMETER
        BNO_REPORT_RAW_GYROSCOPE
        BNO_REPORT_RAW_MAGNETOMETER
        BNO_REPORT_UNCALIBRATED_GYROSCOPE
        BNO_REPORT_UNCALIBRATED_MAGNETOMETER
        BNO_REPORT_ARVR_STABILIZED_ROTATION_VECTOR
        BNO_REPORT_ARVR_STABILIZED_GAME_ROTATION_VECTOR
        BNO_REPORT_GYRO_INTEGRATED_ROTATION_VECTOR

Yet unsupported sensors reports are :

        BNO_REPORT_SIGNIFICANT_MOTION
        BNO_REPORT_SAR
        BNO_REPORT_STEP_DETECTOR
        BNO_REPORT_SHAKE_DETECTOR
        BNO_REPORT_FLIP_DETECTOR
        BNO_REPORT_PICKUP_DETECTOR
        BNO_REPORT_STABILITY_DETECTOR
        BNO_REPORT_SLEEP_DETECTOR
        BNO_REPORT_TILT_DETECTOR
        BNO_REPORT_POCKET_DETECTOR
        BNO_REPORT_CIRCLE_DETECTOR
        BNO_REPORT_HEART_RATE_MONITOR
    
Sensors values can be reached with

    accel_x, accel_y, accel_z = bno.acc

Roll Tilt and Pan can be obtained with

    roll, tilt, pan = bno.euler

**Examples of other sensor reports**

See examples directory for sample code. The following functions use on-chip sensor fusion for accuracy.

    x, y, z = bno.acc  # acceleration 3-tuple of x,y,z float returned
    x, y, z = bno.acc_linear  # linear accel 3-tuple of x,y,z float returned
    x, y, z = bno.gyro  # acceleration 3-tuple of x,y,z float returned
    x, y, z = bno.mag  # acceleration 3-tuple of x,y,z float returned
    roll, pitch, yaw = bno.euler  # rotation 3-tuple of x,y,z float returned
    x, y, z = bno.gravity  # vector 3-tuple of x,y,z float returned
    i, j, k, real = bno.quaternion  # rotation 4-tuple of i,j,k,real float returned
    i, j, k, real = bno.geomagnetic_quat  # rotation 4-tuple of i,j,k,real float returned
    i, j, k, real = bno.game_quat  # rotation 4-tuple of i,j,k,real float returned
    num = bno.steps  # number of steps since initialization returned
    state = bno.shaken  # boolean of state since last read returned
    stability_str = bno.stability_classif  # string of stability classification returned
    activity_str = bno.activity_classif  # string of activity classification returned

The following functions provide raw values directly from individual sensors:

    # raw data sensor tuple of x,y,z, float and time_stamp int returned
    x, y, z, usec = bno.acc_raw 
    x, y, z, usec = bno.mag_raw
    
    # raw data gyro tuple of x,y,z, celsius float, and time_stamp int returned
    x, y, z, temp_c, usec = bno.gyro_raw
    
The following functions can be used to control and test sensor:

    bno.tare  # tare the sensor

    bno.calibration  # begin calibration
    mag_accuracy = bno.calibration_status  # magnetic calibration status int returned
    print(f"Mag Calibration: {REPORT_ACCURACY_STATUS[mag_accuracy]} = {mag_accuracy}")
    bno.calibration_save  # Save calibration

    status = bno.ready  # test sensor status, boolean returned
