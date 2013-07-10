

import h5py

'''# inspired from http://www.h5py.org/docs/intro/quick.html
f = h5py.File('myfile.hdf5', 'a')


dset = f.create_dataset("MyDataset", (100, 100), 'i')
dset[...] = 42
print dset.shape
print dset.dtype
print dset.name

import scipy as sp
import numpy as np
arr = sp.rand(10,10)
dset1 = f.create_dataset('AnotherDataset', data=arr)

subgroup = f.create_group("SubGroup")
subsubgroup = subgroup.create_group("SubSubGroup")
print list(f)
dset2 = subsubgroup.create_dataset("LastDataset", data = np.zeros((1,2)))
dset2.attrs["Name"] = "AnyName"
f.close()'''

import csv
import time
import math
import datetime
import os

filename = datetime.datetime.utcnow().strftime("%Y%m%d_%Hh%M_")+ 'simpleTrack'+'.hdf5'
try:
  os.remove(filename)
except:
  print('Creating file ' + filename) 
f = h5py.File(filename, 'a')
#This create an extendable dataset
dset = f.create_dataset('kite', (2,2), maxshape=(None,7))
#csv_writer.writerow(['Time (s)', 'x (px)', 'y (px)', 'Orientation (rad)', 'Elevation (rad)', 'Bearing (rad)', 'ROT (rad/s)'])
t0 = time.time()
i=0
while time.time() -t0 <  100:
  i = i+1
  t = time.time()
  dset.resize((i,7))
  dset[i-1,:] = [t, math.sin(t), math.cos(t), t, t, t, t]
  time.sleep(0.1)
f.close()


