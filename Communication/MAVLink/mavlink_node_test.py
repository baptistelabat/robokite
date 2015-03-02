# This a minimal test to send and receive mavlink messages from python
# To test it open a Ground Control Station
# Open Qgroundcontrolstation
# Go to Communication
# Got to Add link
# Select UDP as Link Type
# Click on Add IP
# Keep the default value (localhost:14555)
# Click on OK, then click on Connect

from pymavlink import mavutil
import time
from math import sin, radians

freq_out  = 10#Hz

# Open UDP connection on default port
master = mavutil.mavlink_connection('udpout:localhost:14557', baud=57600, source_system=100) # 255 is ground station

last_sent_time = time.time()
t0 = time.time()
while True:
  # Read incoming messages
  msg = master.recv_match(type='HEARTBEAT', blocking=False)
  if msg is not None:
    print ">>", msg
  if msg is not None:
    print ">>", msg
  if abs(time.time() - last_sent_time)>1./freq_out:
    last_sent_time = time.time()
		# Send heartbeat
    master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                  0, 0, 0)
		# Send message 1st method
		# (function for messages available can be found in mavlink/pymavlink/dialects/v10/common.py)
    msg_out = master.mav.manual_control_encode(1, 10,10,10,10,10)
    msg_out = master.mav.attitude_encode((time.time()-t0)*1000, radians(20)*sin(time.time()), 0, 0, 0, 0, 0)					
    master.mav.send(msg_out)
    print "<<", msg_out
		
    # Send message 2nd method
    #master.mav.manual_control_send(1, 10,10,10,10,10) 	


	 



