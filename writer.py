import io 
import os 
import time 
import picamera 
import cv2 
import numpy as np
from array import array 
from time import sleep

import serial

#To fix exposure time, set the shutter_speed attribute to a reasonable value.
#To fix exposure gains, let analog_gain and digital_gain settle on reasonable values, then set exposure_mode to 'off'.
#To fix white balance, set the awb_mode to 'off', then set awb_gains to a (red, blue) tuple of gains.
#Optionally, set iso to a fixed value.

ser = serial.Serial('/dev/ttyUSB0', 115200)  # open serial port
print(ser.name)         # check which port was really used

with picamera.PiCamera() as camera:
    #camera.start_preview()
    x = 320
    y = 240
    camera.resolution = (x, y)
    time.sleep(2)

    while True:
        inputdata = ser.read() 
        print("new data %s" % inputdata)
        if (inputdata == 'h') :
          print('hi-res')
          x = 640
          y = 240
          camera.resolution = (x,y)
        if (inputdata == 's'):
          print('sport mode')
        if (inputdata == 'i'):
          print('iso low')
        image = np.empty((x*y*3,), dtype=np.uint8)
        camera.capture(image, format='bgr')
        image = image.reshape((x, y, 3))
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        cv2.imwrite('img.jpg',gray_image, [int(cv2.IMWRITE_JPEG_QUALITY), 40])
        with open('img.jpg', 'rb') as f:
          byte = f.read(1)
          i = 0
          while byte:
            bts = bytes(byte)
            i = i + 1
            print('send  %d' % i)
            ser.write(byte)
            byte = f.read(1)
        print('sent')


