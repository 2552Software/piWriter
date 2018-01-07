import io
import os
import time
import picamera
import cv2
import numpy as np
import gzip
from array import array
from time import sleep

import serial

#To fix exposure time, set the shutter_speed attribute to a reasonable value.
#To fix exposure gains, let analog_gain and digital_gain settle on reasonable values, then set exposure_mode to 'off'.
#To fix white balance, set the awb_mode to 'off', then set awb_gains to a (red, blue) tuple of gains.
#Optionally, set iso to a fixed value.

ser = serial.Serial('/dev/ttyUSB0', 115200)  # open serial port
print(ser.name)         # check which port was really used

# Create the in-memory stream
stream = io.BytesIO()
with picamera.PiCamera() as camera:
    #camera.start_preview()
    camera.resolution = (320, 240)
    camera.framerate = 24
    time.sleep(2)
    while True:
        image = np.empty((240*320*3,), dtype=np.uint8)
        camera.capture(image, format='bgr')
        image = image.reshape((240, 320, 3))
        # Construct a numpy array from the stream
        #data = np.fromstring(stream.getvalue(), dtype=np.uint8)
        # "Decode" the image from the array, preserving colour
        #image = cv2.imdecode(data, 1)
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        cv2.imwrite('img.jpg',gray_image, [int(cv2.IMWRITE_JPEG_QUALITY), 40])
        data2  = array('B')
        file = open('img.jpg', 'rb') 
        statinfo = os.stat('img.jpg')
        print('data size %d' %  statinfo.st_size)
        data2.read(file, statinfo.st_size)
        i = 1
        for x in np.nditer(data2):
                print i, hex(x)
                i = i + 1
                ser.write(chr(x))
                if ((i % 500) == 0):
                        sleep(0.15)

        #image2 = cv2.imread('img_CV2_90.jpg', cv2.IMREAD_GRAYSCALE )
        print gray_image.size

#cv2.imshow('image',image)
