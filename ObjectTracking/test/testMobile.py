import os
import sys
import time
sys.path.append(sys.path.append('../../Sensors'))

import mobileState
m = mobileState.mobileState()
m.open()
t0 = time.time()
while True:#time.time()-t0 < 1:
  try:
	  m.checkUpdate()
	  print m.isToUpdate
	  if m.isToUpdate:
		m.computeRPY()
	  print m.roll, m.pitch, m.yaw
	  time.sleep(0.001)
  except KeyboardInterrupt:
	  m.close()
