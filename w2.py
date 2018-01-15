
#!/usr/bin/python3
#cv only motion cam
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
from picamera.array import PiRGBArray
import logging
logging.basicConfig(format='%(asctime)s %(name)s %(levelname)s %(message)s', level=logging.INFO)
log = logging.getLogger('fumi')
ser = serial.Serial('/dev/ttyUSB0', baudrate=57600, timeout=60 )  # open serial port, timeout is in seconds

x = 320
y = 240
sleepTime = 1  # time for camera to wait between pictures in seconds (can be .1 etc also)
def sendBinary(filename):
    BLOCKSIZE = 1024
    result = []
    current = ''
    statinfo = os.stat(filename)
    log.info('send %s, size %d' % (filename, statinfo.st_size))
    bytes = 0
    with open(filename, 'rb') as fp:
      for block in iter(lambda: fp.read(BLOCKSIZE), ''):
        if (len(block) == 0):
            log.info('bye')
            break
        bytes = bytes + len(block)
        c = ser.write(block)
        log.info('send %d of %d' % (c, bytes))
    x = ser.read()  

def takeStreamImage(camera, width, height, fmt):
    #log.info('take stream image %d, %d' % (x,y,))
    with picamera.array.PiRGBArray(camera) as stream:
        #log.info('cap %s' % fmt)
        # bgr or rgb
        camera.capture(stream, format=fmt)
        return stream.array
      
def send(filename):
  statinfo = os.stat(filename)
#  
  log.info('send %s, size %d' % (filename, statinfo.st_size))
  t0 = time.time()
  with open(filename, 'rb') as f:
          log.info('file open')
          byte = f.read(1)
          i = 0
          while byte:
            i = i + 1
            #print('write %s' % byte)
            ser.write(byte)
            byte  = f.read(1)
            #is buffer really 16? hoping 64 is a good size since we are sending and reading from a file, not sure, time will tell
            #if ((i % 64) == 0):
             # sleep(.30)
          sleep(1)
  log.info('wait')      
  x = ser.read()  
  t1 = time.time()       
  log.info('%s sent, x = %s time is %d' % (filename, x, t1-t0))

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
          log.info ('%s: sending' %  filename)
          sendbinary(filename)
          q.task_done()
          sleep(0.1)

#To fix exposure time, set the shutter_speed attribute to a reasonable value.
#To fix exposure gains, let analog_gain and digital_gain settle on reasonable values, then set exposure_mode to 'off'.
##To fix white balance, set the awb_mode to 'off', then set awb_gains to a (red, blue) tuple of gains.
#Optionally, set iso to a fixed value.

def scanMotionOpenCV(camera):
    log.info('scan')  
    avg = None
    picCount = 0
    Q = Queue()
    for i in range(1):
        worker = Thread(target=sender, args=(i, Q,))
        worker.setDaemon(True)
        worker.start()
#    log.info('start threads')
    while True:
          frame = np.empty((x*y*3,), dtype=np.uint8)
          camera.capture(frame, format='bgr')
          frame = frame.reshape((y, x, 3))        
          #log.info('next frame')  
          # resize, grayscale & blur out noise
          gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
          blur = cv2.GaussianBlur(gray, (21, 21), 0)
#          #cv2.imwrite('HI.jpg',gray, [int(cv2.IMWRITE_JPEG_QUALITY), 25])
          # if the average frame is None, initialize it
          if avg is None:
                  #log.info("setup average frame")
                  avg = blur.copy().astype("float")
                  continue
          # accumulate the weighted average between the current frame and
          # previous frames, then compute the difference between the current
          # frame and running average
          cv2.accumulateWeighted(blur, avg, 0.5)
          frame_delta = cv2.absdiff(blur, cv2.convertScaleAbs(avg))
          # threshold the delta image, dilate the thresholded image to fill
          # in holes, then find contours on thresholded image
          thresh = cv2.threshold(frame_delta, 5, 255, cv2.THRESH_BINARY)[1]
          thresh = cv2.dilate(thresh, None, iterations=2)
          (_, contours, _) = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
          for c in contours:
              # if the contour is too small, ignore it
              if cv2.contourArea(c) < 5000:
                  continue
              #log.info("Motion detected")
              filename = "img2" + str(picCount) + ".jpg"
              picCount = picCount + 1
              #log.info('create %s' % filename)
              #gray = gray.reshape((y, x, 3))            
              cv2.imwrite(filename,gray, [int(cv2.IMWRITE_JPEG_QUALITY), 20])
              Q.put(filename)
              sleep(1)
              break
          if (sleepTime) :
              #log.info('nap %d seconds' % sleepTime)
              time.sleep(sleepTime)              
          
# Start Main Program Logic
if __name__ == '__main__':
    #try:
      log.info(ser.name)         # check which port was really used
     
      with picamera.PiCamera() as camera:
          camera.resolution = (x,y)
          camera.exposure_mode = 'sports'
          sleep(2)
          scanMotionOpenCV(camera)
    #except:
      log.info("Exiting FUMi")

















