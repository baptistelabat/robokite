import os
import sys
import time
sys.path.append(sys.path.append('../../Sensors'))

import mobileState
m = mobileState.mobileState()
t0 = time.time()
while True:#time.time()-t0 < 1:
  m.checkUpdate()
  if m.isToUpdate:
    m.computeRPY()
  print m.roll, m.pitch, m.yaw
