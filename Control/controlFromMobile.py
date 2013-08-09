import os
import sys
import threading
import time
import numpy as np
sys.path.append(os.getcwd())
sys.path.append('/media/bat/DATA/Baptiste/Nautilab/kite_project/robokite/ObjectTracking')
import mobileState
import serial

# Get the mobile orientation
mobile = mobileState.mobileState()
a = threading.Thread(None, mobileState.mobileState.checkUpdate, None, (mobile,))
a.start()

# Open the serial port
locations=['/dev/ttyACM0','/dev/ttyACM1','/dev/ttyACM2','/dev/ttyUSB0','/dev/ttyUSB1','/dev/ttyUSB2','/dev/ttyUSB3','/dev/ttyS0','/dev/ttyS1','/dev/ttyS2','/dev/ttyS3']
for device in locations:
  try:
    print "Trying...",device
    ser = serial.Serial(
	port = device,
	baudrate = 9600
	)
    break
  except:
    print "Failed to connect on",device
time.sleep(1.5) # Arduino is reset when opening port so wait before communicating
ser.flush()
ser.write('i1')
dt = 0.1
while True:
    mobile.computeRPY()
    ser.write(
	      str(	
                   np.max([-0.99, np.min([0.99, np.floor(100*mobile.roll/2.0)/100.0])]
                          )
	          )
        )
    print mobile.roll
    time.sleep(dt)
