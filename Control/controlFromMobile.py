import os
import sys
import threading
import time
import numpy as np
sys.path.append(os.getcwd())
sys.path.append('/media/bat/DATA/Baptiste/Nautilab/kite_project/robokite/ObjectTracking')
import mobileState
import serial
mobile = mobileState.mobileState()
a = threading.Thread(None, mobileState.mobileState.checkUpdate, None, (mobile,))
a.start()
ser = serial.Serial('/dev/ttyACM1', 9600)
dt = 0.1
time.sleep(dt)
ser.write('i1')
while True:
    mobile.computeRPY()
    ser.write(str(np.floor(100*mobile.roll/3)/100.0))
    print mobile.roll
    time.sleep(dt)
