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
    return h[2:].zfill(2)#get hex data without 0x prefix
    
def NMEA(message_type, value, talker_id= "OR"):
  msg = talker_id + message_type +","+ str(value)
  msg = "$"+ msg +"*"+ computeXORChecksum(msg) + str(chr(13).encode('ascii')) + str(chr(10).encode('ascii'))
  return msg
    
msg1 = NMEA("PW1", 0.00, "OR")
msg2 = NMEA("PW2", 0.00, "OR")
mfb = NMEA("FBR", 0, "OR")
dt = 0.01
locations=['/dev/ttyACM0','/dev/ttyACM1','/dev/ttyACM2','/dev/ttyACM3','/dev/ttyACM4','/dev/ttyACM5','/dev/ttyUSB0','/dev/ttyUSB1','/dev/ttyUSB2','/dev/ttyUSB3','/dev/ttyS0','/dev/ttyS1','/dev/ttyS2','/dev/ttyS3']
ORDER_SAMPLE_TIME = 0.5 #seconds Sample time to send order without overwhelming arduino 
FEEDBACK_SAMPLE_TIME = 0.5
for device in locations:
  try:
    print("Trying...", device)
    ser = serial.Serial(device, baudrate=57600, timeout=1)
    print("Connected on ", device)
    break
  except:
    print("Failed to connect on ", device)
time.sleep(1.5)
t0 = time.time()
previousTime = t0
last_feedback_time = 0
n = 0
while True:

  if (time.time()-previousTime > ORDER_SAMPLE_TIME):
    previousTime = time.time()
    n = n+1
    t = time.time()-t0
    print("n= ", n, ", t= ", t)
    order = 0.4*np.sin(t)
    alpha = np.round(order, 2)
    msg1 = NMEA("PW1", int(alpha*127), "OR")
    msg2 = NMEA("PW2", int(alpha*127), "OR")
    print(msg1)
    ser.write(msg1.encode())
    ser.write(msg2.encode())
    print("Message sent")

  if (time.time()-last_feedback_time > FEEDBACK_SAMPLE_TIME) :
    last_feedback_time = time.time()
    try:
      ser.write(mfb.encode())
      print("Feedback requested")
    except Exception as e:
      print("Error sending order: " + str(e))
      #ser.close()
      break

  try: # The ressource can be temporarily unavailable
    if ser.inWaiting() > 0:
      line = ser.readline()
      #print("Received from arduino: ", line)
      fdbk = line.split(',')
      print(fdbk)
  except Exception as e:
    #ser.close()
    print("Error reading from serial port: " + str(e))
ser.close()
