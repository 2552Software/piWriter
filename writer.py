#!/usr/bin/python3
import io 
import os 
import time 
import picamera 
import cv2 
import _thread
import numpy as np
from array import array 
from time import sleep
import serial

#To fix exposure time, set the shutter_speed attribute to a reasonable value.
#To fix exposure gains, let analog_gain and digital_gain settle on reasonable values, then set exposure_mode to 'off'.
#To fix white balance, set the awb_mode to 'off', then set awb_gains to a (red, blue) tuple of gains.
#Optionally, set iso to a fixed value.

ser = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=10 )  # open serial port, timeout is in seconds
print(ser.name)         # check which port was really used

def send(filename):
  statinfo = os.stat(filename)
  print('send %s, size %d' % (filename, statinfo.st_size))
  with open(filename, 'rb') as f:
          byte = f.read(1)
          i = 0
          while byte:
            #bts = bytes(byte)
            i = i + 1
            print('send  %d' % i)
            print('byte %s' % byte)
            ser.write(byte)
            byte  = f.read(1)
            # keep the buffers safe
            
with picamera.PiCamera() as camera:
    #camera.start_preview()
    camera.resolution = (320, 240)
    #camera.exposure_mode = 'sports'
    #camera.iso = 100
    camera.framerate=24
    # speed in in micro seconds, 6000000us, 6000ms, 6s
    #camera.exposure_speed = 100
    #camera.shutter_speed = camera.exposure_speed
    #sign on, let cam start
    time.sleep(5)
    print('cam signed on')
    while True:
        print('capture')
        image = np.empty((320*240*3,), dtype=np.uint8)
        camera.capture(image, format='bgr')
        image = image.reshape((240, 320, 3))
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        cv2.imwrite('img.jpg',gray_image, [int(cv2.IMWRITE_JPEG_QUALITY), 40])
        try:
            _thread.start_new_thread(send, ('img.jpg'))
        except:
            print('eror')



