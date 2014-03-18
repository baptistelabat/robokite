import matplotlib.pyplot as plt
import numpy as np

f = open('saveInput.txt','r')
ts=[]
ds=[]
lines = f.readlines()
[ds.append(float(line.replace('\n', '').split(',')[0])) for line in lines]

[ts.append(float(line.replace('\n', '').split(',')[1])) for line in lines]

f = open('saveOutput.txt','r')
tout=[]
dout=[]
lines = f.readlines()
for line in lines:
  [t, d] = line.replace('\n', '').split(',')
  dout.append(float(t))
  tout.append(float(d))

plt.hold()
plt.plot(np.array(ts)-ts[0],np.array(ds))
plt.plot(np.array(ts)-ts[0],np.array(ds),np.array(tout)-ts[0], dout, 'r')
plt.show()
