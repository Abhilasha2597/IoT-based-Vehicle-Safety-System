

#!/usr/bin/env python
#
#
import time
import os
import sys
import RPi.GPIO as GPIO
from decimal import *
import atexit
import signal
import subprocess
import datetime
from datetime import datetime
from datetime import timedelta
import socket

import numpy
sys.path.append('/usr/local/lib/python3.4/site-packages')
sys.path.append('/usr/local/lib/python2.7/dist-packages')
sys.path.append('/usr/local/lib/python3.4/dist-packages')
import cv2
import time
from picamera.array import PiRGBArray
from picamera import PiCamera
from scipy.spatial import distance as dist
from imutils import perspective
from imutils import contours
import numpy as np
import argparse
import imutils

from scipy.spatial import distance as dist
import argparse

global firstTime
global centerX
#global centerY
global loopC
global cX
np = numpy

firstTime=0

tempX=0
scan=1
scan1=1
P_TEST=0               
THREEPLACES = Decimal(10) ** -3

GPIO.setwarnings(False)

GPIO.setmode(GPIO.BCM)

subprocess.call('setterm -cursor off', shell=True)
subprocess.call('spincl -ib', shell=True)
GPIO_TRIGGER = 2
GPIO_ECHO    = 3
BUZZER    = 12
RELAY= 17
KEYS1=0
KEYS2=5
KEYS3=6
KEYS4=13
KEYS5=26
KEYS6=1
LED=19

Alco=16
Gas=20
Light=21

# Define GPIO to LCD mapping
LCD_RS = 7
LCD_E  = 22
LCD_D4 = 25
LCD_D5 = 24
LCD_D6 = 23
LCD_D7 = 18



# Define some device constants
LCD_WIDTH = 16    # Maximum characters per line
LCD_CHR = True
LCD_CMD = False
 
LCD_LINE_1 = 0x80 # LCD RAM address for the 1st line
LCD_LINE_2 = 0xC0 # LCD RAM address for the 2nd line
LCD_LINE_3 = 0x90 # LCD RAM address for the 1st line
LCD_LINE_4 = 0xd0 # LCD RAM address for the 2nd line

GPIO.setmode(GPIO.BCM)       # Use BCM GPIO numbers
GPIO.setup(LCD_E, GPIO.OUT)  # E
GPIO.setup(LCD_RS, GPIO.OUT) # RS
GPIO.setup(LCD_D4, GPIO.OUT) # DB4
GPIO.setup(LCD_D5, GPIO.OUT) # DB5
GPIO.setup(LCD_D6, GPIO.OUT) # DB6
GPIO.setup(LCD_D7, GPIO.OUT) # DB7


GPIO.setup(KEYS1, GPIO.IN)
GPIO.setup(KEYS2, GPIO.IN)
GPIO.setup(KEYS3, GPIO.IN)
GPIO.setup(KEYS4, GPIO.IN)
GPIO.setup(KEYS5, GPIO.IN)
GPIO.setup(KEYS6, GPIO.IN)

GPIO.setup(Alco, GPIO.IN)
GPIO.setup(Gas, GPIO.IN)
GPIO.setup(Light, GPIO.IN)

GPIO.setup(RELAY, GPIO.OUT)

GPIO.setup(LED, GPIO.OUT)

GPIO.output(LED, False)  
# Timing constants
E_PULSE = 0.0005
E_DELAY = 0.0005
#for IP---------------------

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
camera.hflip=True
camera.vflip=True
rawCapture = PiRGBArray(camera, size=(640, 480))
firstTime=1
centerX=0
loopC=0


#Sums up all the pixels under a feature's circle and averages them
#Darkest first
def sort_features_by_brightness(image, features):
    global cx
    global cy
    image=255-image
##    cv2.imshow("Image3", image)
##    (thresh, image) = cv2.threshold(image, 128, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    thresh = 20
    image = cv2.threshold(image, thresh, 255, cv2.THRESH_BINARY)[1]
    image=255-image
    thresh=image
    # find contours in the thresholded image
    cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if imutils.is_cv2() else cnts[1]
    
    # loop over the contours
    cX=0
    for c in cnts:
        # compute the center of the contour
        M = cv2.moments(c)
        if(M["m00"]>0):
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            
            print("cx,cy")
            print(cX)
            print(cY)
            # draw the contour and center of the shape on the image
            cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
            cv2.circle(image, (cX, cY), 7, (255, 255, 255), -1)
            cv2.putText(image, "center", (cX - 20, cY - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            # show the image
            cv2.imshow("Image2", image)
            cv2.waitKey(1)
    return cX
##    features_and_brightnesses = [(find_average_brightness_of_feature(image, feature), feature) for feature in features]
##    features_and_brightnesses.sort(key = lambda x:x[0])
##    return [fb[1] for fb in features_and_brightnesses]

def draw_circle_for_feature(image, feature, color=255, thickness=1):
    cv2.circle(image, (int(feature.pt[0]), int(feature.pt[1])), int(feature.size/2), color, thickness)
    #print("Feature")
    #print(int(feature.pt[0]))
    #print(int(feature.pt[1]))


def find_pupil(gray_image, minsize=.15, maxsize=.4):
    global firstTime
    global centerX
    #global centerY
    global loopC
    global cX1
    print('Find pupil')
    #detector = cv2.FeatureDetector_create  ('MSER')
    detector = cv2.MSER_create()        #detector features
    features_all = detector.detect(gray_image)
    features_big = [feature for feature in features_all if feature.size > gray_image.shape[0]*minsize]
    features_small = [feature for feature in features_big if feature.size < gray_image.shape[0]*maxsize]
    print('Sort1')
    
    if len(features_small) == 0:
            lcd_string("drowsinessdetect",LCD_LINE_2)
    
            return None
    print('Sort')
    cX1=sort_features_by_brightness(gray_image, features_small)
    if firstTime==1:
                print('In11')
                if loopC < 10:
                    loopC=loopC+1
                    print(loopC)
                elif loopC < 15:
                    centerX=centerX+cX1
                    #centerY=centerY+pupil.pt[1]
                    loopC=loopC+1
                    print(loopC)
                else:
                    firstTime=2
                    centerX=centerX/5
                    #centerY=centerY/10
                    print("Calib:")
##                    print(int(centerX))
                    print(int(centerX))
                    print("Done")
    else:
               print('Compare')
               print(centerX)
               print(cX1)
               if(cX1==0):
                    print("Stop")##                    print("stop")
                    lcd_string("drowsinessdetect",LCD_LINE_2)
    
               else:
                    print("center")
               


def circle_pupil(color_image, output_image = None):
    if output_image is None:
        output_image = color_image
    gray_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2GRAY)  
    find_pupil(gray_image)
        

def draw(photo):    #show original image
    image_to_show = photo.copy()
    circle_pupil(image_to_show)
    cv2.imshow('Image', image_to_show)
    if cv2.waitKey(10) > 0: #If we got a key press in less than 10ms
        return 1
    return 0

def findPupil():
     for frame in camera.capture_continuous(rawCapture, format="bgr",use_video_port=True):
            img = frame.array             
            #cv2.imshow('img',img)
            if draw(img) > 0:
                break
            rawCapture.truncate(0)
            break

#------------------------------

def lcd_init():
  # Initialise display
  lcd_byte(0x33,LCD_CMD) # 110011 Initialise
  lcd_byte(0x32,LCD_CMD) # 110010 Initialise
  lcd_byte(0x06,LCD_CMD) # 000110 Cursor move direction
  lcd_byte(0x0C,LCD_CMD) # 001100 Display On,Cursor Off, Blink Off
  lcd_byte(0x28,LCD_CMD) # 101000 Data length, number of lines, font size
  lcd_byte(0x01,LCD_CMD) # 000001 Clear display
  time.sleep(E_DELAY)
 
def lcd_byte(bits, mode):
  
  GPIO.output(LCD_RS, mode) # RS
 
  # High bits
  GPIO.output(LCD_D4, False)
  GPIO.output(LCD_D5, False)
  GPIO.output(LCD_D6, False)
  GPIO.output(LCD_D7, False)
  if bits&0x10==0x10:		#0001 0000 0010  0100 10000
    GPIO.output(LCD_D4, True)
  if bits&0x20==0x20:
    GPIO.output(LCD_D5, True)
  if bits&0x40==0x40:
    GPIO.output(LCD_D6, True)
  if bits&0x80==0x80:
    GPIO.output(LCD_D7, True)
 
  # Toggle 'Enable' pin
  lcd_toggle_enable()
 
  # Low bits
  GPIO.output(LCD_D4, False)
  GPIO.output(LCD_D5, False)
  GPIO.output(LCD_D6, False)
  GPIO.output(LCD_D7, False)
  if bits&0x01==0x01:
    GPIO.output(LCD_D4, True)
  if bits&0x02==0x02:
    GPIO.output(LCD_D5, True)
  if bits&0x04==0x04:
    GPIO.output(LCD_D6, True)
  if bits&0x08==0x08:
    GPIO.output(LCD_D7, True)
 
  # Toggle 'Enable' pin
  lcd_toggle_enable()
 
def lcd_toggle_enable():
  # Toggle enable
  time.sleep(E_DELAY)
  GPIO.output(LCD_E, True)
  time.sleep(E_PULSE)
  GPIO.output(LCD_E, False)
  time.sleep(E_DELAY)
 
def lcd_string(message,line):
  # Send string to display
 
  message = message.ljust(LCD_WIDTH," ")
 
  lcd_byte(line, LCD_CMD)
 
  for i in range(LCD_WIDTH):
    lcd_byte(ord(message[i]),LCD_CHR)


														
def dispnum(num,stcol,endcol,dp,line):
    str1 =[]
    #str1[16]
    #str1 =[]
    for i in range(0,16):
        str1.append(0)
        j=0 
    for i in range(endcol+1,stcol,-1):# Step -1:
       #print i
        lcd_byte(line+endcol-j, LCD_CMD)
        if(dp == (i-1)):
            #str1.insert(i-1,'.')
            x= 0x2E
        else:
            a=(num % 10)
            x=int(a)+int(0x30) 
            num =int(num/10)
        lcd_byte(int(x),LCD_CHR)
        j=j+1


subprocess.call('clear', shell=True)

def measure():
  # This function measures a distance

  GPIO.output(GPIO_TRIGGER, True)
  time.sleep(0.00001)   #100msec
  GPIO.output(GPIO_TRIGGER, False)
  start = time.time()
  
  while GPIO.input(GPIO_ECHO)==0:
    start = time.time()

  while GPIO.input(GPIO_ECHO)==1:
    stop = time.time()

  elapsed = stop-start
  distance = (elapsed * 34300)/2

  return distance

def measure_average():
  # This function takes 3 measurements and
  
  distance1=measure()
  return distance1

def getSW():
      if not GPIO.input(KEYS1):
        print("Key1")
        lcd_string("DOOR1 closed    ",LCD_LINE_3)
        time.sleep(1)
        #time.sleep(2)
      else:
        bit1=1
      if not GPIO.input(KEYS2):
        
        lcd_string("Dickey closed         ",LCD_LINE_3)
        print("Key2")
        time.sleep(1) #time.sleep(3)
      if not GPIO.input(KEYS3):
        
        lcd_string("Door3 closed         ",LCD_LINE_3)
        print("Key3")
        time.sleep(1) #time.sleep(3)
      else:
        bit1=1
      if not GPIO.input(KEYS4):
        
        lcd_string("Door4 detected       ",LCD_LINE_3)
        print("Key4")
        time.sleep(1)
      else:
        bit1=1
      if not GPIO.input(KEYS5):
        
        lcd_string("Door2 closed        ",LCD_LINE_3)
        print("Key5")
        time.sleep(1)
        #time.sleep(3)
      else:
        bit1=1
      if not GPIO.input(KEYS6):
        lcd_string("seatbelt weared        ",LCD_LINE_3)
        print("Key6")#time.sleep(3)
        time.sleep(1)
      else:
        bit1=1
        
      if not GPIO.input(Gas):
        bit1=2
        
        lcd_string("Gas Leak detect ",LCD_LINE_2)
        print("Gas Leak detect")#time.sleep(3)
        time.sleep(15)
        Beep2()
        GPIO.output(RELAY, False)
        while True:
          doNothing=0
      #else:
        #lcd_string("               ",LCD_LINE_4)
      if not GPIO.input(Alco):
        bit1=2
        lcd_string("Alcohol detected",LCD_LINE_2)
        print("Alcohol detected")#time.sleep(3)
        time.sleep(1)
        #GPIO.output(RELAY,False)
      else:  
         GPIO.output(RELAY,True)
        # lcd_string("               ",LCD_LINE_4)
      if not GPIO.input(Light):
        
        lcd_string("Low Light detect",LCD_LINE_4)
        GPIO.output(LED, False)
        
        print("Low Light detected")#time.sleep(3)
      else:
        lcd_string("               ",LCD_LINE_4)
        GPIO.output(LED, True)

def chkAll():

   lcd_string("Detecting alcohol",LCD_LINE_1) 
   if not GPIO.input(Gas):
        bit1=1
        lcd_string("Gas Leak detect ",LCD_LINE_2)
        print("Gas Leak detect")#time.sleep(3)
   time.sleep(5)     
   lcd_string("Detecting alcohol",LCD_LINE_1)
   if not GPIO.input(Alco):
        bit1=1
        lcd_string("Alcohol detected",LCD_LINE_2)
        print("Alcohol detected")#time.sleep(3)
   time.sleep(5)     

   lcd_string("Detecting Dickey",LCD_LINE_1)
   if not GPIO.input(KEYS5):
        bit1=1
        lcd_string("Dickey closed        ",LCD_LINE_2)
        print("Key5")#time.sleep(3)
   time.sleep(2) 
   if not GPIO.input(KEYS1):
        bit1=1
        print("Key1")
        lcd_string("DOOR1 closed    ",LCD_LINE_3)
        #time.sleep(2)
   time.sleep(2) 
   if not GPIO.input(KEYS2):
        bit1=1
        lcd_string("Door2 closed         ",LCD_LINE_3)
        print("Key2")#time.sleep(3)
   time.sleep(2)
   if not GPIO.input(KEYS3):
        bit1=1
        lcd_string("Door3 closed         ",LCD_LINE_3)
        print("Key3")#time.sleep(3)
   time.sleep(2)
   if not GPIO.input(KEYS4):
        bit1=1
        lcd_string("Door4 closed         ",LCD_LINE_3)
        print("Key4")
   time.sleep(2)  
   if not GPIO.input(KEYS6):
        bit1=1
        lcd_string("Seatbelt  weared         ",LCD_LINE_3)
        print("Key6")#time.sleep(3)
          
def beep1():
  for i in range(5):
    
    GPIO.output(BUZZER,False)
    time.sleep(0.5)
    GPIO.output(BUZZER,True)
    time.sleep(0.5)

def beep2():
  for i in range(3):
    
    GPIO.output(BUZZER,False)
    time.sleep(1)
    GPIO.output(BUZZER,True)
    time.sleep(0.5)
def beep3():
  for i in range(3):
    
    GPIO.output(BUZZER,False)
    time.sleep(0.5)
    GPIO.output(BUZZER,True)
    time.sleep(1)    
  

def main():
    
    GPIO.setup(GPIO_TRIGGER,GPIO.OUT)  # Trigger
    GPIO.setup(GPIO_ECHO,GPIO.IN)      # Echo
   	GPIO.output(GPIO_TRIGGER, False)
    
	GPIO.setup(RELAY,GPIO.OUT)      # Echo
    GPIO.output(RELAY,False)
    
	GPIO.setup(BUZZER,GPIO.OUT)      # Echo
    GPIO.output(BUZZER,True)
   
    THREEPLACES = Decimal(10) ** -3 # 12 bit
      # Initialise display
    lcd_init()
    scan=1
    scan1=1
    abc=0
    lcd_string("Rasbperry Pi Based",LCD_LINE_1)
    lcd_string("Veh. Safety Sys  ",LCD_LINE_2)
    downT = datetime.now().replace(microsecond=0)
    #print downT
    bit1=0
  
    while True: #"A"
      
	  chkAll()
      if bit1==0:
        break
      else:
        GPIO.output(BUZZER,False)
    GPIO.output(RELAY,True)
    GPIO.output(BUZZER,True)
 
    while True: # "A"LOOP

                curTime = datetime.now().replace(microsecond=0)
                elapse=curTime - downT
               
                timedelta(0, 8, 562000)
                (mins,secs)=divmod(elapse.days * 86400 + elapse.seconds, 60)
                
                
                if secs >= 1:
                    downT=curTime
                    abc=1
                    if scan==1:
                         getSW(); #S/W 1 Sensor 2 Ultra 3 Pupil
                         
                         scan=2
                        
                    elif scan==2:
                        scan=3
                        distance = measure_average()
                        #print "Distance  : %.1f" % distance
                        lcd_string("Distance :",LCD_LINE_1)
                        lcd_string(str(distance),LCD_LINE_1)
                        if(distance < 100): #10.0 cm
                       
					    bit1=3
                       
                    elif scan==3:
                      scan=1  
                      findPupil()
                   
				    if(bit1==1):
                      beep1()
                    elif(bit1==2):
                      beep2()
                    elif(bit1==3):
                      beep3()
                    else:
                      GPIO.output(BUZZER,True)
                      
                      
                    
               
    #serialPort.close()
    exit (0)

if __name__ == '__main__':
 
  try:
    main()
  except KeyboardInterrupt:
    pass
  finally:
    lcd_byte(0x01, LCD_CMD)
    lcd_string("Goodbye!",LCD_LINE_1)
    GPIO.cleanup()

