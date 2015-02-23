#simple script intended to test/check the joystick messages sent.
import time
import pygame
from pygame.locals import *

from pymavlink import mavutil

def bitfield16(n):
  a = [0 for i in range(16)]
  b = [1 if digit=='1' else 0 for digit in bin(n)[2:]]
  for i in range(len(b)):
    a[i] = b[-i-1]
  return a
     
print bitfield16(123)
# Read mavlink messages
joystick_address = 'udpin:localhost:14556'
mav_joystick = mavutil.mavlink_connection(joystick_address, baud=57600, source_system=254) # 255 is ground station
print("Connected to joystick on ", joystick_address)

    

while True:
  msg = mav_joystick.recv_match(type='MANUAL_CONTROL', blocking=False)
  if msg is not None:
    print msg.x, msg.y, bitfield16(msg.buttons)
      
