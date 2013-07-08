import csv
import time
import math
import datetime
import os

filename = datetime.datetime.utcnow().strftime("%Y%m%d_%Hh%M_")+ 'simpleTrack'+'.csv'
try:
  os.remove(filename)
except:
  print('Creating file ' + filename) 
fileobj = file(filename, 'a')
csv_writer = csv.writer(fileobj)
csv_writer.writerow(['Time (s)', 'x (px)', 'y (px)', 'Orientation (rad)', 'Elevation (rad)', 'Bearing (rad)', 'ROT (rad/s)'])
t0 = time.time()
while time.time() -t0 <  10:
  t = time.time()
  csv_writer.writerow([t, math.sin(t), math.cos(t)])
  time.sleep(0.1)
fileobj.close()
