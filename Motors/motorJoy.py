#!/usr/bin/env python
# -*- coding: utf8 -*-
#
# Copyright (c) 2013 Nautilabs
#
# Licensed under the MIT License,
# https://github.com/baptistelabat/robokite
# Authors: Baptiste LABAT
import time
import serial
import numpy as np
import pygame
from pygame.locals import *

pygame.init()
fenetre = pygame.display.set_mode((300,300))

pygame.joystick.init()
nb_joysticks = pygame.joystick.get_count()
mon_joystick = pygame.joystick.Joystick(0)
mon_joystick.init() #Initialisation

def computeXORChecksum(chksumdata):
	# Inspired from http://doschman.blogspot.fr/2013/01/calculating-nmea-sentence-checksums.html
    # Initializing XOR counter
    csum = 0
    
    # For each char in chksumdata, XOR against the previous 
    # XOR char.  The final XOR of the last char will be the
    # checksum  
    for c in chksumdata:
        # Makes XOR value of counter with the next char in line
        # and stores the new XOR value in csum
        csum ^= ord(c)
    h = hex(csum)    
    return h[2:]#get hex data without 0x prefix
    

dt = 0.01
locations=['/dev/ttyACM0','/dev/ttyACM1','/dev/ttyACM2','/dev/ttyACM3','/dev/ttyACM4','/dev/ttyACM5','/dev/ttyUSB0','/dev/ttyUSB1','/dev/ttyUSB2','/dev/ttyUSB3','/dev/ttyS0','/dev/ttyS1','/dev/ttyS2','/dev/ttyS3']
for device in locations:
  try:
    print "Trying...", device
    ser = serial.Serial(device, baudrate=19200, timeout=1)
    print "Connected on ", device
    break
  except:
    print "Failed to connect on ", device
time.sleep(1.5)
ser.write('i1')
t0 = time.time()

msg1 = "ORPW1"+","+str(0.00)
msg1 = "$"+msg1 +"*"+ computeXORChecksum(msg1) + chr(13).encode('ascii')
msg2 = "ORPW2"+","+str(0.00)
msg2 = "$"+msg2 +"*"+ computeXORChecksum(msg2) + chr(13).encode('ascii')
while True:
  for event in pygame.event.get():
    if event.type == JOYAXISMOTION:
      if event.axis == 2:
        #print "direction control ", event.value
        alpha2 = np.round(event.value, 2)
        msg2 = "ORPW2"+","+str(alpha2)
        msg2 = "$"+msg2 +"*"+ computeXORChecksum(msg2) + chr(13).encode('ascii')
      if event.axis == 3:
        #print "power control ", event.value
        alpha1 = np.round(event.value, 2)
        msg1 = "ORPW1"+","+str(alpha1)
        msg1 = "$"+msg1 +"*"+ computeXORChecksum(msg1) + chr(13).encode('ascii')
  if time.time()-t0 > dt:
    ser.write(msg1)
    print msg1
    ser.write(msg2)
    print msg2
    t0 = time.time()

  try: #The ressource can be temporarily unavailable
    line = ser.readline()
    print "Received from arduino: ", line
  except Exception, e:
    print("Error reading from serial port" + str(e))
	  
ser.close()
