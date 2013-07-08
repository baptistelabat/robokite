# inspired from http://www.h5py.org/docs/intro/quick.html

import h5py
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
f.close()
