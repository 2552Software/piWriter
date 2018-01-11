#!/usr/bin/python3
import io 
import os 
import time 
import picamera 
import cv2 
import numpy as np
from array import array 
from time import sleep
import serial
from queue import Queue
from threading import Thread

def send(filename):
  statinfo = os.stat(filename)
  print('send %s, size %d' % (filename, statinfo.st_size))
  t0 = time.time()
  with open(filename, 'rb') as f:
          byte = f.read(1)
          i = 0
          while byte:
            i = i + 1
            #print('send  %d %s' % (i, byte))
            ser.write(byte)
            byte  = f.read(1)
            #is buffer really 16? hoping 64 is a good size since we are sending and reading from a file, not sure, time will tell
            if ((i % 64) == 0):
              sleep(.03)
  x = ser.read()  
  t1 = time.time()       
  print('%s sent, x = %s time is %d' % (filename, x, t1-t0))

def sender(i, q):
    """This is the worker thread function.
    It processes items in the queue one after
    another.  These daemon threads go into an
    infinite loop, and only exit when
    the main thread ends.
    """
    while True:
        filename = q.get()
        if (len(filename) > 0):
          print ('%s: sending' %  filename)
          send(filename)
          q.task_done()

#To fix exposure time, set the shutter_speed attribute to a reasonable value.
#To fix exposure gains, let analog_gain and digital_gain settle on reasonable values, then set exposure_mode to 'off'.
#To fix white balance, set the awb_mode to 'off', then set awb_gains to a (red, blue) tuple of gains.
#Optionally, set iso to a fixed value.

ser = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=60 )  # open serial port, timeout is in seconds
print(ser.name)         # check which port was really used

with picamera.PiCamera() as camera:
    #camera.start_preview()
    # bw 320x240, 3 sec on Pi3, lossy of 20 3 sec (no change)
    # bw 640x480, 10 sec on Pi3
    # color 640x480, 12 seconds
    # color 1280x720, 27 seconds, with lossy of 20 17 sec, bw 20 lossy 14 sec
    x = 320
    y = 240
    camera.resolution = (x, y)
    camera.exposure_mode = 'sports'
    #camera.iso = 100
    #camera.framerate=24
    # speed in in micro seconds, 6000000us, 6000ms, 6s
    #camera.exposure_speed = 100
    #camera.shutter_speed = camera.exposure_speed
    #sign on, let cam start
    Q = Queue()
    for i in range(1):
      worker = Thread(target=sender, args=(i, Q,))
      worker.setDaemon(True)
      worker.start()
    print('cam signed on')
    i = 1
    while True:
        print('capture')
        image = np.empty((x*y*3,), dtype=np.uint8)
        camera.capture(image, format='bgr')
        image = image.reshape((y, x, 3))
        #maybe for inmemory some day? encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
        #encimg = cv2.imencode('.jpg', img, encode_param)
        #gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        #gray_image = image
        filename = "img" + str(i) + ".jpg"
        print('create %s' % filename)
        cv2.imwrite(filename,image, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        Q.put(filename)
