import time
import pygame
from pygame.locals import *
import serial

#try:
try:
  from pymavlink import mavutil
  isMavlinkInstalled = True
except:
  isMavlinkInstalled = False
HEARTBEAT_SAMPLE_TIME = 1
ORDER_SAMPLE_TIME = 0.05
  
buttons_state             = 0 # Variable to store buttons state

# Use pygame for the keyboard
pygame.init()
pygame.display.set_caption('Keyboard fall back')
size = [480, 480]
screen = pygame.display.set_mode(size)
key_array = pygame.key.get_pressed()
global cmd1, cmd2
cmd1 = 0
cmd2 = 0
def resetOrder():
  global cmd1, cmd2
  cmd1 = 0
  cmd2 = 0
  
def saturation(mini, x, maxi):
  return max(mini, min(x, maxi))

# Read mavlink messages
if isMavlinkInstalled:
  try:
    ground_station = 'udpout:localhost:14556'
    master_forward = mavutil.mavlink_connection(ground_station, baud=57600, source_system=254) # 255 is ground station
    isConnectedToGroundStation = True
    print("Connected to ground station on ", ground_station)
  except:
    isConnectedToGroundStation = False
    print("Mavlink connection to ground station failed")

t0 = 0
t_last_order = 0
t = time.time()
while True:
  if time.time()-t0 > HEARTBEAT_SAMPLE_TIME:
    t0 = time.time()
    if isConnectedToGroundStation:
      msg = master_forward.recv_match(type='HEARTBEAT', blocking=False)
      master_forward.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                              0, 0, 0)
  pygame.key.get_focused()
  # Deals with gamepad events 
  for event in pygame.event.get():
    if event.type == KEYDOWN or event.type == KEYUP:
      key_array = pygame.key.get_pressed()
      #for i in range(len(key_array)):
        #print i, key_array[i]

      buttons_state = 0


      Kf = range(5)
      Kf[1] = 38 # Based on keyboard reverse engineering
      Kf[2] = 233
      Kf[3] = 34
      Kf[4] = 39
      for i in range(1,5):
        buttons_state += key_array[Kf[i]]*2**(i-1)
      buttons_state += key_array[K_l]*2**4
      buttons_state += key_array[K_i]*2**5
      buttons_state += key_array[K_j]*2**6
      buttons_state += key_array[K_k]*2**7
      buttons_state += key_array[K_r]*2**8
      buttons_state += key_array[K_s]*2**12
      buttons_state += key_array[K_d]*2**13
      buttons_state += key_array[K_f]*2**14
      buttons_state += key_array[K_e]*2**15

      if isConnectedToGroundStation:
          print cmd1, cmd2, buttons_state
          master_forward.mav.manual_control_send(0, cmd1*1000, cmd2*1000, 0, 0, buttons_state)
  if time.time()-t_last_order > ORDER_SAMPLE_TIME:
    t_last_order = time.time()
    cmd1 = saturation(-1, cmd1 + (-key_array[K_LEFT] + key_array[K_RIGHT] -cmd1)*(time.time()-t)*3, 1)
    cmd2 = saturation(-1, cmd2 + (-key_array[K_UP]   + key_array[K_DOWN] -cmd2)*(time.time()-t)*3, 1)
    if -key_array[K_LEFT] + key_array[K_RIGHT] == 0:
      cmd1 = 0
    if -key_array[K_UP]   + key_array[K_DOWN] == 0:
      cmd2 = 0
    master_forward.mav.manual_control_send(0, cmd1*1000, cmd2*1000, 0, 0, buttons_state)
    print cmd1, cmd2, buttons_state
    t = time.time()


