import h5py
import time
from SimpleCV import Image
recordFilename = '20130727_17h34_simpleTrack'
print recordFilename + '.hdf5'
#recordFile = h5py.File('20130722_21h53_simpleTrack.hdf5')
recordFile = h5py.File(recordFilename + '.hdf5', 'r') 
imgs = recordFile.get('image')
img = imgs[100,:,:,:]
r = img[:,:,0]
g = img[:,:,1]
b = img[:,:,2]
im = Image(img)
im.show()
time.sleep(10)

