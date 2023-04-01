# Hello World Example
#
# Welcome to the OpenMV IDE! Click on the green run arrow button below to run the script!

import sensor, image, time, io, rpc, os, micropython
import pyb
from pyb import ExtInt, Pin

sensor.reset()                      # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.VGA)    # Set frame size to VGA (640x480)
sensor.skip_frames(time = 2000)     # Wait for settings take effect.
#sensor.ioctl(sensor.IOCTL_SET_TRIGGERED_MODE, False)

pin = Pin("P1", pyb.Pin.IN, pull=pyb.Pin.PULL_NONE)
prevVal = pin.value()

sd = pyb.SDCard()
os.mount(sd,"/sdcard")
dirname = "/sdcard"
imgCounter = 0
img = sensor.snapshot()

def image_snapshot(arg):
    #print(arg)

    global imgCounter, img, dirname, sd

    try:
        #suffix = "pos" if pin.value() else "neg"
        #print(suffix)
        img = sensor.snapshot()
        filename = '{}/img{:05d}.jpg'.format(dirname, imgCounter)
        #print(filename)
        img.save(filename)         # Take a picture and save the image.
    except:
       pass
    imgCounter += 1




def capture_image(lineNumber):
    #print(lineNumber)
    global imgCounter
    try:
        micropython.schedule(image_snapshot, lineNumber)
    except:
        imgCounter += 1




ext = ExtInt(Pin("P1"), ExtInt.IRQ_RISING_FALLING, Pin.PULL_NONE, capture_image)
ext.enable()
#pin.callback(pyb.ExtInt.IRQ_RISING_FALLING, lambda pin: capture_image())



while True:
    pyb.delay(500)
    #pyb.wfi()
