# more_reports.py
#
# BNO08x Micropython I2C example program
#
# Original Code from Adafruit CircuitPython Library
# Bryan Siepert for Adafruit Industries

from time import sleep

from bno08x import *
from machine import I2C, Pin

i2c0 = I2C(0, scl=Pin(13), sda=Pin(12), freq=100_000, timeout=200_000)
bno = BNO08X(i2c0, debug=False)

bno.enable_feature(BNO_REPORT_STEP_COUNTER)
bno.enable_feature(BNO_REPORT_STABILITY_CLASSIFIER)
bno.enable_feature(BNO_REPORT_ACTIVITY_CLASSIFIER)
bno.enable_feature(BNO_REPORT_SHAKE_DETECTOR)
print("BNO08x reports enabled\n")

while True:
    sleep(0.1)

    print(f"\nTotal Steps detected: {bno.steps=}")
    print(f"Stability classification: {bno.stability_classif=}")

    activity_classification = bno.activity_classif
    most_likely = activity_classification["most_likely"]
    confidence = activity_classification.get(most_likely, 0)  # safe default
    print(f"Activity classification: {most_likely}, confidence: {confidence}/100")

    print("sleep for 0.5 sec, then test shake")
    sleep(0.5)
    if bno.shake:
        print("Shake Detected! \n")
