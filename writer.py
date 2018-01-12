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
import picamera.array

x = 640
y = 480
threshold = 10  # How Much pixel changes
sensitivity = 100  # How many pixels change
streamWidth = 128  # motion scan stream Width
streamHeight = 80


ser = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=60 )  # open serial port, timeout is in seconds

def takeStreamImage(width, height):
    with picamera.PiCamera() as camera:
        print('take stream image %d, %d' % (x,y,))
        camera.resolution = (width, height)
        with picamera.array.PiRGBArray(camera) as stream:
            camera.exposure_mode = 'auto'
            camera.awb_mode = 'auto'
            print('cap')
            camera.capture(stream, format='rgb')
            return stream.array

def scanMotion(width, height):
    motionFound = False
    print('scan motion')
    data1 = takeStreamImage(width, height)
    while not motionFound:
        print('no motion')
        data2 = takeStreamImage(width, height)
        diffCount = 0;
        for w in range(0, width):
            for h in range(0, height):
                # get the diff of the pixel. Conversion to int
                # is required to avoid unsigned short overflow.
                diff = abs(int(data1[h][w][1]) - int(data2[h][w][1]))
                #print('dif %d' % diff)
                if  diff > threshold:
                    diffCount += 1
            if diffCount > sensitivity:
                break; #break outer loop.
        if diffCount > sensitivity:
            print('motion!')
            motionFound = True
        else:
            data1 = data2
    return motionFound


def send(filename):
  statinfo = os.stat(filename)
  
  print('send %s, size %d' % (filename, statinfo.st_size))
  t0 = time.time()
  with open(filename, 'rb') as f:
          byte = f.read(1)
          i = 0
          print('reading from %s' % filename)
          while byte:
            i = i + 1
            #print('send %d %s' % (i, byte))
            ser.write(byte)
            byte  = f.read(1)
            #is buffer really 16? hoping 64 is a good size since we are sending and reading from a file, not sure, time will tell
            if ((i % 512) == 0):
              sleep(.25)
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

def shoot(count):
  with picamera.PiCamera() as camera:
      #camera.start_preview()
      # bw 320x240, 3 sec on Pi3, lossy of 20 3 sec (no change)
      # bw 640x480, 10 sec on Pi3
      # color 640x480, 12 seconds
      # color 1280x720, 27 seconds, with lossy of 20 17 sec, bw 20 lossy 14 sec
      camera.resolution = (x, y)
      camera.exposure_mode = 'sports'
      #camera.iso = 100
      #camera.framerate=24
      # speed in in micro seconds, 6000000us, 6000ms, 6s
      #camera.exposure_speed = 100
      #camera.shuttle_speed = camera.exposure_speed
      #sign on, let cam start
      Q = Queue()
      for i in range(1):
        worker = Thread(target=sender, args=(i, Q,))
        worker.setDaemon(True)
        worker.start()
      time.sleep(2)
      print('cam signed on')
      i = 1
      while True:
        print('capture')
        image = np.empty((x*y*3,), dtype=np.uint8)
        camera.capture(image, format='bgr')
        image = image.reshape((y, x, 3))
        #maybe for inmemory some day? encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
        #encimg = cv2.imencode('.jpg', img, encode_param)
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        #gray_image = image
        filename = "img" + str(i) + ".jpg"
        i = i + 1
        print('create %s' % filename)
        cv2.imwrite(filename,gray_image, [int(cv2.IMWRITE_JPEG_QUALITY), 25])
        Q.put(filename)
        if (i > count):
         break
        sleepTime = 1  # min of 1 second
        print('nap')
        time.sleep(sleepTime)
      print('wait for q')
      Q.join()
      print('all done!')
    
#To fix exposure time, set the shutter_speed attribute to a reasonable value.
#To fix exposure gains, let analog_gain and digital_gain settle on reasonable values, then set exposure_mode to 'off'.
#To fix white balance, set the awb_mode to 'off', then set awb_gains to a (red, blue) tuple of gains.
#Optionally, set iso to a fixed value.

# Start Main Program Logic
if __name__ == '__main__':
    try:
      print(ser.name)         # check which port was really used
      while True:
        print('motion check')
        if scanMotion(streamWidth, streamHeight):
          print('shoot')
          shoot(3)
    except:
      print("")
      print("+++++++++++++++")
      print("Exiting FUMi")
      print("+++++++++++++++")









