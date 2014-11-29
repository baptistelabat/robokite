#!/usr/bin/env python
# -*- coding: utf8 -*-
#
# Copyright (c) 2013 Nautilabs
#
# Licensed under the MIT License,
# https://github.com/baptistelabat/robokite
# Authors: Baptiste LABAT
#
# This script order from a USB joystick and send them to an arduino board
# using NMEA messages for robustness
# Press first button to switch to manual order
# Press second button to switch to automatic order
#
# Warning: the joystick has to be in neutral position when plugged,
# otherwise it might be badly initialized
#
# Supports automatic reconnection of joystick and serial connection
# Only tested on ubuntu 14.04

import time
import serial
import numpy as np
import pygame
from pygame.locals import *
import os
try:
  from pymavlink import mavutil
  isMavlinkInstalled = True
except:
  isMavlinkInstalled = False

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
    return h[2:].zfill(2)# Get hex data without 0x prefix
    
def NMEA(message_type, value, talker_id= "OR"):
  msg = talker_id + message_type +","+ str(value)
  msg = "$"+ msg +"*"+ computeXORChecksum(msg) + chr(13).encode('ascii') + chr(10).encode('ascii')
  return msg

# Parameters for the serial connection
locations = ['/dev/ttyACM0','/dev/ttyACM1','/dev/ttyACM2','/dev/ttyACM3','/dev/ttyACM4','/dev/ttyACM5','/dev/ttyUSB0','/dev/ttyUSB1','/dev/ttyUSB2','/dev/ttyUSB3','/dev/ttyS0','/dev/ttyS1','/dev/ttyS2','/dev/ttyS3']
baudrate = 57600

ORDER_SAMPLE_TIME = 0.01 #seconds Sample time to send order without overwhelming arduino

MANUAL    = 0 # Control lines are released to enable manual control
JOY_OL    = 1 # Joystick controls voltage applied to motors (open loop control)
JOY_CL    = 2 # Joystick controls kite bar position in closed loop
AUTO      = 3 # Kite roll is stabilised thanks to IMU measurements, joystick controls the kite roll angle
mode = JOY_OL

joy_OL_offset_forward = 0
joy_OL_offset_right    = 0
joy_CL_offset_forward = 0
joy_CL_offset_right   = 0
auto_offset_forward   = 0
auto_offset_right   = 0

global msg1, msg2, mfb, power1, power2, mode
def resetOrder():
  global msg1, msg2, mfb, power1, power2, mode
  # Define the NMEA message in use
  if mode==JOY_OL:
    msg1 = NMEA("PW1", 0, "OR") # Order to first motor
    msg2 = NMEA("PW2", 0, "OR") # Order to second motor
  if mode==JOY_CL:
    msg1 = NMEA("SP1", 0, "OR") # Order to first motor
    msg2 = NMEA("SP2", 0, "OR") # Order to second motor
  if mode==AUTO:
    msg1 = NMEA("PW1", 0, "OR") # Order to first motor
    msg2 = NMEA("PW2", 0, "OR") # Order to second motor
  mfb  = NMEA("FBR", 0, "OR") # Feedback request

  power1 = 0
  power2 = 0

# Use pygame for the joystick
pygame.init()
JOY_RECONNECT_TIME = 2 #seconds
FORWARD_BACKWARD = 2
LEFT_RIGHT = 3
RESET_OFFSET = 8

# Read mavlink messages
if isMavlinkInstalled:
  try:
    master = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600, source_system=255)
  except:
    isMavlinkInstalled = False
rollspeed = 0
roll = 0

# This loop is here for robustness in case of deconnection
while True:
    for device in locations:
      try:
        print "Trying...", device
        resetOrder()
        # This is necessary to unsure automatic reconnection after disconnection
        os.system("stty -F "+ device + " " + str(baudrate) + " cs8 cread clocal")
        
        # Note that by default arduino is restarting on serial connection
        ser = serial.Serial(device, baudrate=baudrate, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=2, xonxoff=0, rtscts=0, interCharTimeout=None)   
        print "Connected on ", device
        break
      except:
        print "Failed to connect on ", device
    # Wait so that arduino is fully awoken
    time.sleep(1.5)
    try:
        ser.write('i1')
    except:
        print "Can not write i1"
        
    # Reset the timer    
    t0 = time.time()
    last_event_time = 0


    # This is the main program loop
    while True:

      # Deals with joystick deconnection and reconnection      
      if time.time()-last_event_time > JOY_RECONNECT_TIME:
         print "No joystick event for x seconds, trying reconnection"
         last_event_time = time.time()
         resetOrder()
         pygame.quit()
         pygame.init()
         pygame.joystick.init()
         actual_nb_joysticks = pygame.joystick.get_count()
         print actual_nb_joysticks
         if actual_nb_joysticks > 0:
            mon_joystick = pygame.joystick.Joystick(0)
            mon_joystick.init()
            nb_joysticks = actual_nb_joysticks
            print "Joystick reinit"
      
      # Deals with gamepad events 
      for event in pygame.event.get():

        last_event_time = time.time()
        # Button events
        if event.type == JOYBUTTONDOWN:
            if event.button == JOY_OL:
                mode = JOY_OL
                print "JOY_OL"
            if event.button == JOY_CL:
                mode = JOY_CL
                print "JOY_CL"
            if event.button == AUTO:
                mode = AUTO
                print "AUTO"
            if event.button == RESET_OFFSET:
                if mode == JOY_OL:
                  joy_OL_offset_forward = 0
                  joy_OL_offset_right   = 0
                if mode == JOY_CL:
                  joy_CL_offset_forward = 0
                  joy_CL_offset_right   = 0
                if mode == AUTO:
                  auto_offset_forward   = 0
                  auto_offset_bacward   = 0                  
        # Joystick events  
        if event.type == JOYAXISMOTION:
          if event.axis == FORWARD_BACKWARD:
            #print "power control ", event.value
            power1 = event.value*127
          if event.axis == LEFT_RIGHT :
            #print "direction control ", event.value
            power2 = event.value*127
            
        # Trim events
        if event.type == JOYHATMOTION:
            print event
            if mode == JOY_OL:
              joy_OL_offset_forward += -event.value[1]
              joy_OL_offset_right    += event.value[0]
            if mode == JOY_CL:
              joy_CL_offset_forward += -event.value[1]
              joy_CL_offset_right   += event.value[0]
            if mode == AUTO:
              auto_offset_forward   += -event.value[1]
              auto_offset_right     += event.value[0]
        
        # Create messages to be sent   
        if mode == JOY_OL:
            msg2 = NMEA("PW2", int(power2 + joy_OL_offset_forward), "OR")
        if mode == JOY_CL:
            msg2 = NMEA("SP2", int(power2 + joy_CL_offset_forward), "OR")
        if mode == AUTO:
            msg2 = NMEA("PW2", int(power2 + auto_offset_forward), "OR")  
        if mode == JOY_OL:
            msg1 = NMEA("PW1", int(power1 + joy_OL_offset_right), "OR")
        if mode == JOY_CL:
            msg1 = NMEA("SP1", int(power1 + joy_CL_offset_right), "OR")
        if mode == AUTO:
            msg1 = NMEA("PW1",  int(power1 + auto_offset_right + roll*180/np.pi), "OR")
            
      # Mavlink messages
      if isMavlinkInstalled:  
        msg = master.recv_match(type='ATTITUDE', blocking=False)
        if msg!=None:
          rollspeed = msg.rollspeed
          roll = msg.roll
          if mode == AUTO:
            msg1 = NMEA("PW1",  int(power1 + auto_offset_right + roll*180/np.pi), "OR")
    
      # Send messages 
      if time.time()-t0 > ORDER_SAMPLE_TIME:
        try:
            ser.write(msg1)
            #print msg1
            ser.write(msg2)
            #print msg2
            ser.write(mfb)
            #print mfb
            t0 = time.time()
        except:
            print "break"
            ser.close()
            break

      try: # The ressource can be temporarily unavailable
        if ser.inWaiting() > 0:
            line = ser.readline()
            #print "Received from arduino: ", line
      except Exception, e:
        ser.close()
        print("Error reading from serial port" + str(e))
      
ser.close()
print "Closing"
