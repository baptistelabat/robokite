# -------------------------------------------------------
import socket, traceback
import time
import json

import numpy as np
from scipy.spatial.transform import Rotation as R

host = ''
port = 2390

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
s.bind((host, port))

filein = open('saveUDP.txt', 'w')
t0 = time.time()

# Place IMU x-axis into wind going direction when launching script
is_init_done = False
wind_yaw = 0
while time.time()-t0 < 200:
    try:
        message, address = s.recvfrom(4096)
        #print(message)
        msg = json.loads(message.decode("utf-8"))
        if is_init_done==False:
          wind_yaw = msg["Yaw"]
          is_init_done = True
        msg['Yaw'] = msg['Yaw']-wind_yaw
        print(msg)
        
        ypr = [msg['Yaw'], msg['Pitch'], msg['Roll']]
        seq = 'ZYX' # small letters from intrinsic rotations

        r = R.from_euler(seq, ypr, degrees=True)
        
        # Compute coordinates in NED (could be useful to compare position with GPS position for example)
        line_length = 10
        base_to_kite = [0, 0, line_length]
        base_to_kite_in_NED = r.apply(base_to_kite)
        
        # Express kite coordinates as great roll, great pitch and small yaw angles
        grpy=r.as_euler(seq="XYZ")
        print(grpy*180/np.pi)

        filein.write('%s\n' % (message))
    except (KeyboardInterrupt, SystemExit):
       raise
    except:
       traceback.print_exc()
filein.close()
# -------------------------------------------------------
