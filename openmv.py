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

sd = pyb.SDCard()
os.mount(sd,"/sdcard")
dirname = "/sdcard"

clock = time.clock()                # Create a clock object to track the FPS.
imgCounter = 0

for i in range(0,10000):
    clock.tick()                    # Update the FPS clock.
    img = sensor.snapshot()
    filename = '{}/img{:08d}-{}.jpg'.format(dirname, imgCounter,clock.fps())
    img.save(filename)         # Take a picture and save the image.
    imgCounter += 1
