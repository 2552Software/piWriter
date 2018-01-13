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
import logging
logging.basicConfig(format='%(asctime)s %(name)s %(levelname)s %(message)s', level=logging.INFO)
log = logging.getLogger('fumi')
ser = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=60 )  # open serial port, timeout is in seconds

x = 640
y = 480

# for no opencv motion detection
threshold = 10  # How Much pixel changes
sensitivity = 100  # How many pixels change
streamWidth = 128  # motion scan stream Width
streamHeight = 80

sleepTime = 1  # time for camera to wait between pictures in seconds (can be .1 etc also)

def takeStreamImage(width, height, fmt):
    with picamera.PiCamera() as camera:
        log.info('take stream image %d, %d' % (x,y,))
        camera.resolution = (width, height)
        with picamera.array.PiRGBArray(camera) as stream:
            camera.exposure_mode = 'auto'
            camera.awb_mode = 'auto'
            log.info('cap %s' % fmt)
            # bgr or rgb
            camera.capture(stream, format=fmt)
            return stream.array

#reference https://github.com/timatooth/catscanface
def scanMotionOpenCV(width, height):
    avg = None
    log.info('scan motion using OpenCV')
    while True:
        log.info('take test image')
        image = takeStreamImage(width, height, 'bgr')
        # resize, grayscale & blur out noise
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (21, 21), 0)

        # if the average frame is None, initialize it
        if avg is None:
                log.info("setup average frame")
                avg = gray.copy().astype("float")
                continue

        # accumulate the weighted average between the current frame and
        # previous frames, then compute the difference between the current
        # frame and running average
        log.info('weight')
        cv2.accumulateWeighted(gray, avg, 0.5)
        frame_delta = cv2.absdiff(gray, cv2.convertScaleAbs(avg))
        log.info("threshold")
        # threshold the delta image, dilate the thresholded image to fill
        # in holes, then find contours on thresholded image
        thresh = cv2.threshold(frame_delta, args.delta_threshold, 255, cv2.THRESH_BINARY)[1]
        thresh = cv2.dilate(thresh, None, iterations=2)
        (contours, _) = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        motion = False
        log.info('check contours')
        for c in contours:
            log.info('contour')
            # if the contour is too small, ignore it
            if cv2.contourArea(c) < args.min_area:
                continue

            log.info("Motion detected")
            return True

       
def scanMotion(width, height):
    motionFound = False
    log.info('scan motion')
    data1 = takeStreamImage(width, height, 'rgb')
    while not motionFound:
        log.info('no motion')
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
            log.info('motion!')
            motionFound = True
        else:
            data1 = data2
    return motionFound


def send(filename):
  statinfo = os.stat(filename)
  
  log.info('send %s, size %d' % (filename, statinfo.st_size))
  t0 = time.time()
  with open(filename, 'rb') as f:
          byte = f.read(1)
          i = 0
          log.info('reading from %s' % filename)
          while byte:
            i = i + 1
            #log.info('send %d %s' % (i, byte))
            ser.write(byte)
            byte  = f.read(1)
            #is buffer really 16? hoping 64 is a good size since we are sending and reading from a file, not sure, time will tell
            if ((i % 512) == 0):
              sleep(.25)
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
          send(filename)
          q.task_done()

def shoot(count, Q):
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
      time.sleep(2)
      log.info('cam signed on')
      i = 1
      while True:
        log.info('capture')
        image = np.empty((x*y*3,), dtype=np.uint8)
        camera.capture(image, format='bgr')
        image = image.reshape((y, x, 3))
        #maybe for inmemory some day? encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
        #encimg = cv2.imencode('.jpg', img, encode_param)
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        #gray_image = image
        filename = "img" + str(i) + ".jpg"
        i = i + 1
        log.info('create %s' % filename)
        cv2.imwrite(filename,gray_image, [int(cv2.IMWRITE_JPEG_QUALITY), 25])
        Q.put(filename)
        if (i > count):
         break
        if (sleepTime) :
          log.info('nap %d seconds' % sleepTime)
          time.sleep(sleepTime)
      log.info('done shooting for now')
    
#To fix exposure time, set the shutter_speed attribute to a reasonable value.
#To fix exposure gains, let analog_gain and digital_gain settle on reasonable values, then set exposure_mode to 'off'.
#To fix white balance, set the awb_mode to 'off', then set awb_gains to a (red, blue) tuple of gains.
#Optionally, set iso to a fixed value.

# Start Main Program Logic
if __name__ == '__main__':
    try:
      log.info(ser.name)         # check which port was really used
      log.info('start threads')
      Q = Queue()
      for i in range(1):
        worker = Thread(target=sender, args=(i, Q,))
        worker.setDaemon(True)
        worker.start()

      while True:
        log.info('motion check')
        if scanMotionOpenCV(x,y):
          log.info('shoot')
          shoot(3, Q)
    except:
      log.info('wait for q')
      Q.join()
      log.info('all done!')
      log.info("Exiting FUMi")









