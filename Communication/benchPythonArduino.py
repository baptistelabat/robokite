#!/usr/bin/env python
# -*- coding: utf8 -*-
#
# Copyright (c) 2013 Nautilabs
#
# Licensed under the MIT License,
# https://github.com/baptistelabat/robokite
# Authors: Baptiste LABAT, 
#
# This file is a benchmark of the different python libraries enabling high level communication with arduino.
# The following libraries were tested:
#
#   firmata                       https://github.com/lupeke/python-firmata/
#   pyfirmata                     https://github.com/tino/pyFirmata
#   Python-Arduino-Proto-API-v2   https://github.com/vascop/Python-Arduino-Proto-API-v2
#   Python-Arduino-Command-API    https://github.com/thearn/Python-Arduino-Command-API 
#   
#   The benchmark is only based on the capability (frequency) to read analog signal from Arduino

import time
import os
import sys
import re

def guess_port():
    """
    Tries to guess port from system platform.
    Taken from https://github.com/tino/pyFirmata
    Inspired form https://github.com/rwldrn/johnny-five
    """
    ports = os.listdir("/dev")
    prefix = "cu" if sys.platform == "darwin" else "tty"
    #pattern = '^%s\.(usb|acm|ACM).+$' % (prefix)
    pattern = '%s(usb|acm|ACM)' % (prefix)
    port = ""
    for p in ports:
        if re.match(pattern, p):
            port = p
            break
    return "/dev/%s" % port

# The library to be tested is given as argument
if len(sys.argv) > 1:
  nlib = int(sys.argv[1])
else:
  nlib = 1
print nlib
# The choice are among (need to be installed)
firmata = 0                 # https://github.com/lupeke/python-firmata/
pyfirmata = 1               # https://github.com/tino/pyFirmata
protoApiV2 = 2              # https://github.com/vascop/Python-Arduino-Proto-API-v2
pythonArduinoCommandAPI = 3 # https://github.com/thearn/Python-Arduino-Command-API 

# Only one pin is used for the benchmark
pin = 0

# Load library to be tested
if nlib == firmata:
  from firmata import Arduino, OUTPUT
elif nlib == pyfirmata:
  from pyfirmata import util, Arduino
elif nlib == protoApiV2:
  sys.path.append('/home/bat/Python-Arduino-Proto-API-v2/arduino') # There is no installation script, so this is hard coded
  from arduino import Arduino
elif nlib == pythonArduinoCommandAPI:
  from Arduino import Arduino

# Open serial connection
device = guess_port()	# Worked so far. Replace with port if special case
board = None
try:
      print "Trying connection on...", device
      if (nlib==firmata) or (nlib==pyfirmata) or (nlib==protoApiV2):
        board = Arduino(device)
        print "Connected on ", device
       # break
      elif pythonArduinoCommandAPI:
        board = Arduino('9600', port=device)
        print "Connected on ", device
        #break
except:
      print "Failed to connect on ", device
      
if board:
  # Initialize pin
  print "Initialising pin"
  if nlib == firmata:
    board.pin_mode(pin, OUTPUT)
  elif nlib == pyfirmata:
      it= util.Iterator(board)
      it.start()
      board.analog[pin].enable_reporting()
  elif nlib == protoApiV2:
    board.output([])
  elif pythonArduinoCommandAPI:
      board.pinMode(pin, "INPUT")
    
  #  Loop and read data
  t0 = time.time()
  n = 0
  t = time.time()
  print "Looping over data"
  while t-t0<10:  
    if nlib == firmata:
      board.parse()
      value = board.analog_read(pin)
    elif nlib == pyfirmata:
      value = board.analog[pin].read()
    elif nlib == protoApiV2:
      value = board.analogRead(pin) # Reading from analog pin #0
    elif nlib == pythonArduinoCommandAPI:
      value = board.analogRead(pin)
    n = n+1
    print "n: ",n, value
    t = time.time()
   
  # Closing connection properly
  if nlib == firmata:
    print "No close"
  elif nlib == pyfirmata:
    board.exit()
  elif nlib == protoApiV2:
    board.close()

