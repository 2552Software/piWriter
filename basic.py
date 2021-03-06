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
import picamera.array
from picamera.array import PiRGBArray
import logging
logging.basicConfig(format='%(asctime)s %(name)s %(levelname)s %(message)s', level=logging.INFO)
log = logging.getLogger('basic')
x = 640
y = 480
sleepTime = 0  # time for camera to wait between pictures in seconds (can be .1 etc also)

#To fix exposure time, set the shutter_speed attribute to a reasonable value.
#To fix exposure gains, let analog_gain and digital_gain settle on reasonable values, then set exposure_mode to 'off'.
#To fix white balance, set the awb_mode to 'off', then set awb_gains to a (red, blue) tuple of gains.
#Optionally, set iso to a fixed value.

def scanMotionOpenCV(camera):
    log.info('scan')  
    avg = None
    picCount = 0
    raw_capture = PiRGBArray(camera, size=(x,y))
    for f in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
          log.info('next frame')  
          frame = f.array
          # resize, grayscale & blur out noise
          gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
          gray = cv2.GaussianBlur(gray, (21, 21), 0)
          #cv2.imwrite('HI.jpg',gray, [int(cv2.IMWRITE_JPEG_QUALITY), 25])

          # if the average frame is None, initialize it
          if avg is None:
                  #log.info("setup average frame")
                  avg = gray.copy().astype("float")
                  raw_capture.truncate(0)
                  continue
          # accumulate the weighted average between the current frame and
          # previous frames, then compute the difference between the current
          # frame and running average
          cv2.accumulateWeighted(gray, avg, 0.5)
          frame_delta = cv2.absdiff(gray, cv2.convertScaleAbs(avg))
          # threshold the delta image, dilate the thresholded image to fill
          # in holes, then find contours on thresholded image
          thresh = cv2.threshold(frame_delta, 5, 255, cv2.THRESH_BINARY)[1]
          thresh = cv2.dilate(thresh, None, iterations=2)
          (_, contours, _) = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
          for c in contours:
              # if the contour is too small, ignore it
              if cv2.contourArea(c) < 5000:
                  continue
              log.info("Motion detected")
              filename = "img" + str(picCount) + ".jpg"
              picCount = picCount + 1
              log.info('create %s' % filename)
              cv2.imwrite(filename,frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
              if (sleepTime) :
                log.info('nap %d seconds' % sleepTime)
                time.sleep(sleepTime)
              break
          raw_capture.truncate(0)  

# Start Main Program Logic
if __name__ == '__main__':
    try:
      with picamera.PiCamera() as camera:
          camera.resolution = (x,y)
          camera.exposure_mode = 'sports'
          sleep(2)
          scanMotionOpenCV(camera)
    except:
          log.info("stop")

