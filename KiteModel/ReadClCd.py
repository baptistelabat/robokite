# -*- coding: utf-8 -*-

# Read the data file
import numpy as np
from numpy import pi
from scipy.optimize import leastsq
import matplotlib.pyplot as plt
import os

# Input file [Angle_deg Cl Cd Cm]
folder = "."
os.chdir(folder)
filename = "xf-naca2412-il-1000000.txt"

coefs = np.genfromtxt(filename, dtype=float, skip_header=12) # for xfoil files

# Convert first column to radians
coefs[:, 0] = pi/180*coefs[:, 0]
angles  = coefs[:, 0]
CL      = coefs[:, 1]
CD      = coefs[:, 2]
angles = np.append(angles, [angles[-1] + 5*pi/180, pi/2, pi, 3*pi/2])
CL = np.append(CL, [CL[-1]/2, 0, 0, 0])
print CL
CD = np.append(CD, [1, 1, 0, 1])
print CD

filename=filename.replace('.dat', '.npy')
np.save(filename, coefs)

def poly_trigo(a, theta, N=5):
    p = a[0]    
    for i in range(1, N+1):
        p = p + a[2*i-1]*np.cos(i*theta)
    for i in range(1, N+1):
        p = p + a[2*i]*np.sin(i*theta)
    return p
    

def residual(a, y, theta, N):
    err = y - poly_trigo(a, theta, N=N)
    return err

N = 2
a = np.ones(2*N+1)
CL_poly = leastsq(residual, a, args=(CL, angles, N))
CL_poly = CL_poly[0]
filename = filename.replace('.npy', '_CL.npy')
np.save(filename, CL_poly)
print(CL_poly)

CD_poly = leastsq(residual, a, args=(CD, angles, N))
CD_poly = CD_poly[0]
filename = filename.replace('CL', 'CD')
np.save(filename, CD_poly)
print(CD_poly)

angles_extrap = np.arange(0, 360)*pi/180.
plt.plot(angles*180/pi, CL, 'r')
plt.plot(angles*180/pi, poly_trigo(CL_poly, angles, N), 'r+')
plt.plot(angles_extrap*180/pi, poly_trigo(CL_poly, angles_extrap, N), 'r+')
plt.plot(angles*180/pi, CD, 'b')
plt.plot(angles*180/pi, poly_trigo(CD_poly, angles, N), 'b+')
plt.show()
