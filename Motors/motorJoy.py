#!/usr/bin/env python
# -*- coding: utf8 -*-
#
# Copyright (c) 2013 Nautilabs
#
# Licensed under the MIT License,
# https://github.com/baptistelabat/robokite
# Authors: Baptiste LABAT
#
# This script receives order from a USB joystick and send them to an arduino board
# using NMEA messages for robustness
# Press first button to switch to manual order
# Press second button to switch to open loop control
# Press third button to switch to closed loop control
# Press fourth button to switch to fully automatic control
# Use forward/backward motion to control one axis
# Use left/right motion to control other axis
# Use cross to trim joystick (independant for each mode)
# Use reset button to reset trim
#
# Warning: the joystick has to be in neutral position when plugged,
# otherwise it might be badly initialized
#
# Supports automatic reconnection of joystick and serial connection
# Tested on ubuntu 14.04

import os
import time
import serial
import numpy as np
import pygame
from pygame.locals import *
try:
    from scipy.interpolate import interp1d
    isScipyInstalled = True
except:
    isScipyInstalled = False
try:
  from pymavlink import mavutil
  isMavlinkInstalled = True
except:
  isMavlinkInstalled = False

global msg1, msg2, mfb, cmd1, cmd2, mode, add_deadband, line
line="0, 0, 0, 0, 0"

# Define a linear interpolation function to create a deadband
xi = [-2, -1, -0.75, 0.75, 1, 2]
yi = [-1, -1, 0, 0, 1, 1]
if isScipyInstalled:
    add_deadband = interp1d(xi, yi, kind='linear')

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
  msg = "$"+ msg +"*"+ computeXORChecksum(msg) + str(chr(13).encode('ascii')) + str(chr(10).encode('ascii'))
  return msg

# Parameters for the serial connection
locations = ['/dev/ttyACM0','/dev/ttyACM1','/dev/ttyACM2','/dev/ttyACM3','/dev/ttyACM4','/dev/ttyACM5','/dev/ttyUSB0','/dev/ttyUSB1','/dev/ttyUSB2','/dev/ttyUSB3','/dev/ttyS0','/dev/ttyS1','/dev/ttyS2','/dev/ttyS3','COM1','COM2','COM3']
baudrate = 57600

ORDER_SAMPLE_TIME = 0.05 #seconds Sample time to send order without overwhelming arduino

# Definition of gamepad button in use
MANUAL              = 0 # Control lines are released to enable manual control
JOY_OL              = 1 # Joystick controls voltage applied to motors (open loop control)
JOY_CL              = 2 # Joystick controls kite bar position in closed loop
AUTO                = 3 # Kite roll is stabilised thanks to IMU measurements, joystick controls the kite roll angle
INC_BUTTON_LEFT     = 4 # Increase derivative gain
INC_BUTTON_RIGHT    = 5 # Increase proportional gain
DEC_BUTTON_LEFT     = 6 # Decrease derivative gain
DEC_BUTTON_RIGHT    = 7 # Decrease proportional gain
RESET_OFFSET_BUTTON = 8 # Reset offet
mode = JOY_OL

FORWARD_BACKWARD_AXIS = 2
LEFT_RIGHT_AXIS       = 3

# Variables to save offset
joy_OL_offset_forward = 0
joy_OL_offset_right   = 0
joy_CL_offset_forward = 0
joy_CL_offset_right   = 0
auto_offset_forward   = 0
auto_offset_right     = 0
inc                   = 0.05 # Increment normalized

ROBOKITE_SYSTEM = 0
GROUND_UNIT = 0
FLYING_UNIT = 1

mfb  = NMEA("FBR", 0, "OR") # Feedback request

Kp = 1
Kd = 1
inc_factor = 2

def resetOrder():
  global msg1, msg2, mfb, cmd1, cmd2, mode
  # Define the NMEA message in use
  if mode==JOY_OL:
    msg1 = NMEA("PW1", 0, "OR") # Order to first motor
    msg2 = NMEA("PW2", 0, "OR") # Order to second motor
  elif mode==JOY_CL:
    msg1 = NMEA("SP1", 0, "OR") # Order to first motor
    msg2 = NMEA("SP2", 0, "OR") # Order to second motor
  elif mode==AUTO:
    msg1 = NMEA("PW1", 0, "OR") # Order to first motor
    msg2 = NMEA("PW2", 0, "OR") # Order to second motor

  cmd1 = 0
  cmd2 = 0

# Use pygame for the joystick

JOY_RECONNECT_TIME = 2 #seconds. Time to reconnect if no joystick motion

pygame.init()

# Read mavlink messages
if isMavlinkInstalled:
  try:
    embedded_device = '/dev/ttyUSB0'
    master = mavutil.mavlink_connection(embedded_device, baud=57600, source_system=254) # 255 is ground station
    master_forward = mavutil.mavlink_connection('localhost:14555', baud=57600, source_system=254) # 255 is ground station
    print("Connected to embbeded device on", embedded_device)
  except:
    isMavlinkInstalled = False
    print("Mavlink connection failed")
rollspeed = 0
roll = 0

# This loop is here for robustness in case of deconnection
while True:
    for device in locations:
      try:
        print("Trying...", device)
        resetOrder()
        
        # First open with  baudrate zero and close to enable reconnection after deconnection
		# Thanks to Rémi Moncel for this trick http://robokite.blogspot.fr/2014/11/reconnexion-automatique-du-port-serie.html#comment-form
        ser = serial.Serial(device, baudrate=0, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=2, xonxoff=0, rtscts=0, interCharTimeout=None)   
        ser.close()
        # Note that by default arduino is restarting on serial connection
        ser = serial.Serial(device, baudrate=baudrate, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=2, xonxoff=0, rtscts=0, interCharTimeout=None)   
        print("Connected to ground device on ", device)
        break
      except:
        print("Failed to connect on ", device)
    # Wait so that arduino is fully awoken
    time.sleep(1.5)
        
    # Reset the timer    
    t0 = time.time()
    last_event_time = 0


    # This is the main program loop
    while True:

      # Deals with joystick deconnection and reconnection      
      if time.time()-last_event_time > JOY_RECONNECT_TIME:
         print("No joystick event for x seconds, trying reconnection")
         last_event_time = time.time()
         resetOrder()
         pygame.quit()
         pygame.init()
         pygame.joystick.init()
         actual_nb_joysticks = pygame.joystick.get_count()
         print("Number of joystick: ", actual_nb_joysticks)
         if actual_nb_joysticks > 0:
            my_joystick = pygame.joystick.Joystick(0)
            my_joystick.init()
            nb_joysticks = actual_nb_joysticks
            print("Joystick reinit")
      
      # Deals with gamepad events 
      for event in pygame.event.get():

        last_event_time = time.time()
        # Button events
        if event.type == JOYBUTTONDOWN:
            if event.button == MANUAL:
                mode = MANUAL
                print("MANUAL")
            elif event.button == JOY_OL:
                mode = JOY_OL
                print("JOY_OL: JOYSTICK OPEN LOOP")
            elif event.button == JOY_CL:
                mode = JOY_CL
                print("JOY_CL: JOYSTICK to BAR POSITION CLOSE LOOP")
            elif event.button == AUTO:
                mode = AUTO
                print("AUTO: JOYSTICK to KITE ROLL CLOSE LOOP")
            elif event.button == INC_BUTTON_LEFT:
                Kd = Kd * inc_factor
                print("Kd: ", Kd)
            elif event.button == INC_BUTTON_RIGHT:
                Kp = Kp * inc_factor
                print("Kp: ", Kp)
            elif event.button == DEC_BUTTON_LEFT:
                Kd = Kd / inc_factor
                print("Kd: ", Kd)
            elif event.button == DEC_BUTTON_RIGHT:
                Kp = Kp / inc_factor
                print("Kp: ", Kp)
            elif event.button == RESET_OFFSET_BUTTON:
                if mode == JOY_OL:
                  joy_OL_offset_forward = 0
                  joy_OL_offset_right   = 0
                elif mode == JOY_CL:
                  joy_CL_offset_forward = 0
                  joy_CL_offset_right   = 0
                elif mode == AUTO:
                  auto_offset_forward   = 0
                  auto_offset_right     = 0                  
        # Joystick events  
        if event.type == JOYAXISMOTION:
          if event.axis == FORWARD_BACKWARD_AXIS:
            #print("power control ", event.value)
            cmd1 = event.value
          elif event.axis == LEFT_RIGHT_AXIS :
            #print("direction control ", event.value)
            cmd2 = event.value
            if isScipyInstalled:
                cmd2 = add_deadband(cmd2)
            
        # Trim events
        if event.type == JOYHATMOTION:
            if mode == JOY_OL:
              joy_OL_offset_forward -= event.value[1]*inc
              joy_OL_offset_right   += event.value[0]*inc
            elif mode == JOY_CL:
              joy_CL_offset_forward -= event.value[1]*inc
              joy_CL_offset_right   += event.value[0]*inc
            elif mode == AUTO:
              auto_offset_forward   -= event.value[1]*inc
              auto_offset_right     += event.value[0]*inc
        
        # Create messages to be sent   
        if mode == JOY_OL:
            msg1 = NMEA("PW1", int((cmd1 + joy_OL_offset_right)  *127), "OR")
            msg2 = NMEA("PW2", int((cmd2 + joy_OL_offset_forward)*127), "OR")
        elif mode == JOY_CL:
            msg1 = NMEA("SP1", int((cmd1 + joy_CL_offset_right)  *127), "OR")
            msg2 = NMEA("SP2", int((cmd2 + joy_CL_offset_forward)*127), "OR")
        elif mode == AUTO:
            msg1 = NMEA("PW1", int((auto_offset_right -Kp*(roll-cmd1*roll_max_excursion) - Kd*rollspeed)*127), "OR")
            msg2 = NMEA("PW2", int((cmd2 + auto_offset_forward)*127),   "OR")  # \todo: add regulation based on line tension?
        elif mode == MANUAL:
            msg1 = NMEA("PW1", 0, "OR")
            msg2 = NMEA("PW2", 0, "OR")     
            
      # Mavlink messages
      if isMavlinkInstalled:  
        msg = master.recv_match(type='ATTITUDE', blocking=False)
        if msg!=None:
          master_forward.mav.send(msg)
          rollspeed = msg.rollspeed
          roll = msg.roll
          if mode == AUTO:
            msg1 = NMEA("PW1", int((auto_offset_right -Kp*(roll-cmd1*roll_max_excursion) - Kd*rollspeed)*127), "OR")
        msg = master_forward.recv_match(type='SCALED_PRESSURE', blocking=False)
        if msg!=None:
          master_forward.mav.send(msg)
        master_forward.mav.local_position_ned_send(10, 0, 0, 0, 0, 0, 0 )
      # Send messages 
      if time.time()-t0 > ORDER_SAMPLE_TIME:
        try:
            t0 = time.time()

            ser.write(msg1.encode())
            #print(msg1)
            ser.write(msg2.encode())
            #print(msg2)
            ser.write(mfb.encode())
            #print(mfb)

            fdbk = line.split(',')
            time_us = int(time.time()*1e6)
            time_ms = int(time_us/1000)
            group_mlx = 0
            press_abs = 1025#hpa
            press_diff = 0 #hpa
            temperature = 20# Celsius deg
            if isMavlinkInstalled:
                master_forward.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                  0, 0, 0)

                master_forward.mav.actuator_control_target_send(time_us, group_mlx, [0, 0, float(fdbk[2]), float(fdbk[3]), float(fdbk[4]), 0 ,0, 0 ])
                master_forward.mav.set_actuator_control_target_send(time_us, group_mlx, ROBOKITE_SYSTEM, GROUND_UNIT, [float(fdbk[0]), float(fdbk[1]), 0, 0, 0, 0 ,0, 0 ])
        except Exception as e:
            print("Error sending order: " + str(e))
            #ser.close()
            break

      try: # The ressource can be temporarily unavailable
        if ser.inWaiting() > 0:
            line = ser.readline()
            print("Received from arduino: ", line)
      except Exception as e:
        #ser.close()
        print("Error reading from serial port: " + str(e))
      
ser.close()
print("Closing")
