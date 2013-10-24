import os
import sys
import threading
import time
import numpy as np
sys.path.append(os.getcwd())
sys.path.append('../ObjectTracking')
import mobileState
import serial

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
    return h[2:]#get hex data without 0x prefix
    
    
try:
  # Get the mobile orientation
  mobile = mobileState.mobileState()
  a = threading.Thread(None, mobileState.mobileState.checkUpdate, None, (mobile,))
  a.start()

  # Open the serial port
  locations=['/dev/ttyACM0','/dev/ttyACM1','/dev/ttyACM2','/dev/ttyUSB0','/dev/ttyUSB1','/dev/ttyUSB2','/dev/ttyUSB3','/dev/ttyS0','/  dev/ttyS1','/dev/ttyS2','/dev/ttyS3']
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
      ser.flush()
      alpha = np.max([-0.99, np.min([0.99, np.floor(100*mobile.pitch/2.0)/100.0])]
                          )
      msg = "ORPWM"+","+str(alpha)
      msg = "$"+msg +"*"+ computeXORChecksum(msg) + chr(13).encode('ascii')
      ser.write(msg
        )
      print msg
      time.sleep(dt)
except KeyboardInterrupt:
 mobile.stop_requested = True
