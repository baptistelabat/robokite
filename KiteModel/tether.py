import numpy as np

x0 = 0
y0 = 0
z0 = 0

x1 = 100
y1 = 100
z1 = 100

dx = x1-x0
dy = y1-y0
dz = z1-z0

l = np.sqrt(dx**2 + dy**2+ dz**2)
l0 = 150

epsilon = l/l0
E = 10 # Young modulus
T=E*epsilon

print T
Fz = T*np.sin(atan2(z, np.sqrt(x**2+y**2))
Fy = T*np.sin(atan2(y, np.sqrt(x**2+z**2))
Fx = T*np.sin(atan2(x, np.sqrt(z**2+y**2))
