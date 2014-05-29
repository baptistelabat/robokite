#!/usr/bin/env python
# -*- coding: utf8 -*-
#
# Copyright (c) 2013 Nautilabs
#
# Licensed under the MIT License,
# https://github.com/baptistelabat/robokite
# Authors: Baptiste LABAT
import serial
import time

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

while True:
  try: #The ressource can be temporarily unavailable
    line = ser.readline()
    print "Received from arduino: ", line
  except Exception, e:
    print("Error reading from serial port" + str(e))
      
ser.close()
