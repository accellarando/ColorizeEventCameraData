# Hello World Example
#
# Welcome to the OpenMV IDE! Click on the green run arrow button below to run the script!

import sensor, image, time, io, rpc, os
import pyb

sensor.reset()                      # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.VGA)   # Set frame size to QVGA (320x240)
sensor.skip_frames(time = 2000)     # Wait for settings take effect.
#sensor.ioctl(sensor.IOCTL_SET_TRIGGERED_MODE, False)

pin = pyb.Pin("P1", pyb.Pin.IN, pull=pyb.Pin.PULL_UP)
prevVal = pin.value()

sd = pyb.SDCard()
os.mount(sd,"/sdcard")
dirname = "/sdcard"

imgCounter = 0

while True:
    if pin.value() != prevVal:
        img = sensor.snapshot()
        filename = '{}/img{:08d}.jpg'.format(dirname, imgCounter)
        img.save(filename)         # Take a picture and save the image.
        imgCounter += 1
    prevVal = pin.value();
