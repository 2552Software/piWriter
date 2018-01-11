
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
import queue
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



def sender(q):
    """This is the worker thread function.
    It processes items in the queue one after
    another.  These daemon threads go into an
    infinite loop, and only exit when
    the main thread ends.
    """
    while True:
        filename = q.get()
        print ('%s: sending' %  filename)
        send(filename)
        # instead of really downloading the URL,
        # we just pretend and sleep
        time.sleep( 2)
        q.task_done()

#To fix exposure time, set the shutter_speed attribute to a reasonable value.
#To fix exposure gains, let analog_gain and digital_gain settle on reasonable values, then set exposure_mode to 'off'.
#To fix white balance, set the awb_mode to 'off', then set awb_gains to a (red, blue) tuple of gains.
#Optionally, set iso to a fixed value.

ser = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=60 )  # open serial port, timeout is in seconds
print(ser.name)         # check which port was really used

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



