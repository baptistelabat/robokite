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
import sys
import time
import serial
import numpy as np
sys.path.append('../../mavlink/pymavlink')

import rotmat
from math import radians, atan2, hypot, pi

import socket
def getIP():
  s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  s.connect(("8.8.8.8",80))
  ip = s.getsockname()[0]
  s.close()
  return ip

m = rotmat.Matrix3()
v = rotmat.Vector3(0,0,-1)
f = rotmat.Vector3(1,0,0)

try:
  from pymavlink import mavutil
  isMavlinkInstalled = True
except:
  isMavlinkInstalled = False

global msg1, msg2, mfb, cmd1, cmd2, mode
cmd1 = 0
cmd2 = 0

line="0, 0, 0, 0, 0"
rollspeed = 0
roll = 0
mustUpdateOrder = False

def bitfield16(n):
  a = [0 for i in range(16)]
  b = [1 if digit=='1' else 0 for digit in bin(n)[2:]]
  for i in range(len(b)):
    a[i] = b[-i-1]
  return a

def saturation(mini, x, maxi):
  return min(max(mini, x), maxi)
  
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
HEARTBEAT_SAMPLE_TIME = 1
# Definition of gamepad button in use
Nmode = 4  
MANUAL              = 0 # Control lines are released to enable manual control
JOY_OL              = 1 # Joystick controls voltage applied to motors (open loop control)
JOY_CL              = 2 # Joystick controls kite bar position in closed loop
AUTO                = 3 # Kite roll is stabilised thanks to IMU measurements, joystick controls the kite roll angle
INC_BUTTON_LEFT     = 4 # Increase derivative gain
INC_BUTTON_RIGHT    = 5 # Increase proportional gain
DEC_BUTTON_LEFT     = 6 # Decrease derivative gain
DEC_BUTTON_RIGHT    = 7 # Decrease proportional gain
RESET_OFFSET_BUTTON = 8 # Reset offset
HAT_LEFT            = 12
HAT_BACKWARD        = 13
HAT_RIGHT           = 14
HAT_FORWARD         = 15
buttons_state = bitfield16(0)
previous_buttons_state = buttons_state
mode = JOY_OL

# Variables to save offset
joy_offset_forward = [0 for i in range(Nmode)]
joy_offset_right   = [0 for i in range(Nmode)]
inc                   = 0.05 # Increment normalized

ROBOKITE_SYSTEM = 0
GROUND_UNIT     = 0
FLYING_UNIT     = 1
embedded_device  = '/dev/ttyUSB1'
ground_station   = 'udpout:localhost:14550'
joystick_address = 'udpin:'+getIP()+':14556'

mfb  = NMEA("FBR", 0, "OR") # Feedback request

Kp_ini = 1.
Kd_ini = 0.1
Kp = Kp_ini
Kd = Kd_ini
inc_factor = 2.
Kp_mini = Kp_ini/inc_factor**5
Kp_maxi = Kp_ini*inc_factor**5
Kd_mini = Kd_ini/inc_factor**5
Kd_maxi = Kd_ini*inc_factor**5
roll_max_excursion = 45*np.pi/180

def resetOrder():
  global msg1, msg2, mfb, cmd1, cmd2, mode
  # Define the NMEA message in use
  if mode==JOY_OL:
    msg1 = NMEA("PW1", 0, "OR") # Order to first motor    
  elif mode==JOY_CL:
    msg1 = NMEA("SP1", 0, "OR") # Order to first motor
  elif mode==AUTO:
    msg1 = NMEA("PW1", 0, "OR") # Order to first motor
  msg2 = NMEA("PW2", 0, "OR") # Order to second motor

  cmd1 = 0
  cmd2 = 0

# Open mavlink connection
if isMavlinkInstalled:
  try:
    master = mavutil.mavlink_connection(embedded_device, baud=57600, source_system=254) # 255 is ground station
    isConnectedToEmbeddedDevice = True
    print("Mavlink connection to embbeded device on", embedded_device)
  except:
    isConnectedToEmbeddedDevice = False
    print("Connected (mavlink) to embbed device failed")
  try:
    master_forward_ground = mavutil.mavlink_connection(ground_station, baud=57600, source_system=254) # 255 is ground station
    master_forward_embedded = mavutil.mavlink_connection(ground_station, baud=57600, source_system=100) # 255 is ground station
    master_forward_joystick = mavutil.mavlink_connection(ground_station, baud=57600, source_system=253) # 255 is ground station
    isConnectedToGroundStation = True
    print("Connected (mavlink) to ground station on ", ground_station)
  except:
    isConnectedToGroundStation = False
    print("Mavlink connection to ground station failed")
  try:
    print joystick_address
    mav_joystick = mavutil.mavlink_connection(joystick_address, baud=57600, source_system=254) # 255 is ground station
    isConnectedToJoystick = True
    print("Connected (mavlink) to joystick on ", joystick_address)
  except:
    isConnectedToJoystick = False
    print("Mavlink connection to joystick failed")


# This loop is here for robustness in case of deconnection
while True:
  for device in locations:
    try:
      print("Trying...", device)
      resetOrder()
      
      # First open with  baudrate zero and close to enable reconnection after deconnection
      # Thanks to Rémi Moncel for this trick http://robokite.blogspot.fr/2014/11/reconnexion-automatique-du-port-serie.html#comment-form
      ser = serial.Serial(device, baudrate=0)#, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=2, xonxoff=0, rtscts=0, interCharTimeout=None)   
      ser.close()
      # Note that by default arduino is restarting on serial connection
      ser = serial.Serial(device, baudrate=baudrate)#, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=2, xonxoff=0, rtscts=0, interCharTimeout=None)   
      print("Connected (serial) to ground device on ", device)
      break
    except:
      print("Failed to connect on ", device)
    # Wait so that arduino is fully awoken
    time.sleep(1.5)
        
  # Reset the timers    
  t0 = 0
  last_event_time = 0
  thb = 0
    # This is the main program loop
  while True:
    # Mavlink messages
    if isConnectedToGroundStation:
      if time.time()-thb > HEARTBEAT_SAMPLE_TIME:
        thb = time.time()
        print "Sending heartbeat"
        master_forward_embedded.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_KITE, mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                  0, 0, 0)
        master_forward_ground.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                  0, 0, 0)
        master_forward_joystick.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                  0, 0, 0)
      #msg = master_forward.recv_match(type='HEARTBEAT', blocking=False)

    if isConnectedToEmbeddedDevice:
      msg = master.recv_match(type='ATTITUDE', blocking=False)
      if msg!=None:
        #print msg
        m.from_euler(msg.roll, msg.pitch, msg.yaw)
        pos = m*v
        speed= m*f
        azimuth = atan2(pos.y, pos.x)
        elevation = atan2(pos.z, hypot(pos.x, pos.y))
        if isConnectedToGroundStation:
          master_forward_embedded.mav.send(msg)
          master_forward_embedded.mav.local_position_ned_send(10, pos.x, pos.y, pos.z, speed.x, speed.y, speed.z )
        rollspeed = msg.rollspeed
        roll = msg.roll
        if mode == AUTO:
          mustUpdateOrder = True
      msg = master.recv_match(type='SCALED_PRESSURE', blocking=False)
      if msg!=None:
        if isConnectedToGroundStation:
          master_forward_embedded.mav.send(msg)      
      
    if isConnectedToJoystick:
      msg = mav_joystick.recv_match(type='MANUAL_CONTROL', blocking=False)
      if msg is not None:
        mustUpdateOrder = True
        cmd1 = msg.x/1000.
        cmd2 = msg.y/1000.
        buttons_state = bitfield16(msg.buttons)
        buttons_down_event = [buttons_state[i]-previous_buttons_state[i] for i in range(16)]
        previous_buttons_state = buttons_state
        if isConnectedToGroundStation:
          master_forward_joystick.mav.send(msg)

        # Button events
        if buttons_state[MANUAL]:
          mode = MANUAL
          print("MANUAL")
        elif buttons_state[JOY_OL]:
          mode = JOY_OL
          print("JOY_OL: JOYSTICK OPEN LOOP")
        elif buttons_state[JOY_CL]:
          mode = JOY_CL
          print("JOY_CL: JOYSTICK to BAR POSITION CLOSE LOOP")
        elif buttons_state[AUTO]:
          mode = AUTO
          print("AUTO: JOYSTICK to KITE ROLL CLOSE LOOP")
            
        if buttons_state[RESET_OFFSET_BUTTON]:
          joy_offset_forward[mode] = 0
          joy_offset_right[mode]   = 0
        elif buttons_down_event[HAT_LEFT]==1:
          joy_offset_right[mode]   -= inc
        elif buttons_down_event[HAT_RIGHT]==1:
          joy_offset_right[mode]   += inc
        elif buttons_down_event[HAT_RIGHT]==1:
          joy_offset_right[mode]   += inc
        elif buttons_down_event[HAT_RIGHT]==1:
          joy_offset_right[mode]   += inc
        elif buttons_down_event[HAT_FORWARD]==1:        
          joy_offset_forward[mode] -= inc
        elif buttons_down_event[HAT_BACKWARD]==1:        
          joy_offset_forward[mode] += inc
                    
        if mode == AUTO:  
          if buttons_down_event[INC_BUTTON_LEFT]==1:
            Kd = saturation(Kd_mini, Kd * inc_factor, Kd_maxi)
            print("Kd: ", Kd)
          elif buttons_down_event[INC_BUTTON_RIGHT]==1:
            Kp = saturation(Kp_mini, Kp * inc_factor, Kp_maxi)
            print("Kp: ", Kp)
          elif buttons_down_event[DEC_BUTTON_LEFT]==1:
            Kd = saturation(Kd_mini, Kd / inc_factor, Kd_maxi)
            print("Kd: ", Kd)
          elif buttons_down_event[DEC_BUTTON_RIGHT]==1:
            Kp = saturation(Kp_mini, Kp / inc_factor, Kp_maxi)
            print("Kp: ", Kp)
          
    # Send messages 
    if (time.time()-t0 > ORDER_SAMPLE_TIME) or (mustUpdateOrder):
      t0 = time.time()
      try:
        # Create messages to be sent   
        if mode == JOY_OL:
          msg1 = NMEA("PW1", int((cmd1 + joy_offset_right[mode])  *127), "OR")
          msg2 = NMEA("PW2", int((cmd2 + joy_offset_forward[mode])*127), "OR")
        elif mode == JOY_CL:
          msg1 = NMEA("SP1", int((cmd1 + joy_offset_right[mode])  *127), "OR")
          msg2 = NMEA("PW2", int((cmd2 + joy_offset_forward[mode])*127), "OR")
        elif mode == AUTO:
          msg1 = NMEA("PW1", int((joy_offset_right[mode] -Kp*(roll-cmd1*roll_max_excursion) - Kd*rollspeed)*127), "OR")
          msg2 = NMEA("PW2", int((cmd2 + joy_offset_forward[mode])*127),   "OR")  # \todo: add regulation based on line tension?
        elif mode == MANUAL:
          msg1 = NMEA("PW1", 0, "OR")
          msg2 = NMEA("PW2", 0, "OR") 

        ser.write(msg1.encode())
        #print(msg1)
        ser.write(msg2.encode())
        #print(msg2)
        ser.write(mfb.encode())
        #print(mfb)
        mustUpdateOrder = False
      except Exception as e:
        print("Error sending order: " + str(e))
        #ser.close()
        break

    try: # The ressource can be temporarily unavailable
      if ser.inWaiting() > 0:
        line = ser.readline()
        print("Received from arduino: ", line)
        if isConnectedToGroundStation:
          fdbk = line.split(',')
          time_us = int(time.time()*1e6)
          time_ms = int(time_us/1000)
          group_mlx = 0
          if len(fdbk) == 8:
            master_forward_ground.mav.actuator_control_target_send(time_us, group_mlx, [0, 0, float(fdbk[2]), float(fdbk[3]), float(fdbk[4]), float(fdbk[5]) ,float(fdbk[6]), float(fdbk[7]) ])
            master_forward_ground.mav.set_actuator_control_target_send(time_us, group_mlx, ROBOKITE_SYSTEM, GROUND_UNIT, [float(fdbk[0]), float(fdbk[1]), 0, 0, 0, 0 ,0, 0 ])
    except Exception as e:
      #ser.close()
      print("Error reading from serial port: " + str(e))
      
ser.close()
print("Closing")
