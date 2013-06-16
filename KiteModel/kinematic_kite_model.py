from SimpleCV import Image, Display, Color
import numpy as np
import scipy as sp
import time
import PID

def windForce(X, D):
  ''' Computes a factor for the wind force depending on the postion on the window
      Inputs :
	  - X: vector position
	  - D: diameter of the window of flight'''
  d = sp.sqrt(X[1]**2 + X[2]**2)
  return sp.cos(d/D*sp.pi/2) # kind of circular shape

def speed(X):
  ''' Computes the speed of kite based on its position and orientation'''
  F = windForce(X, 350)
  return F -0.1*sp.cos(sp.arctan2(X[2], X[1])-sp.pi/2 -X[0])
  
order = 1
X = np.array([[0, 0, 0]]).transpose()

#Create a black and a white image
logo = Image('logo').resize(800, 600)
black = logo-logo
white = black.invert()
kite_base = white.crop(0, 0, 30, 10)
disp = Display()
k = -4
dt = 0.1
kdrift  = 50
i_loop = 0
pid = PID.PID(1, 1, 0.1)
offset = sp.pi/3
dX = 0*X
while disp.isNotDone():
  setpoint = sp.pi/1.7*sp.sin(2*sp.pi/7*time.time())+offset
  i_loop = i_loop +1
  #order = 0+sp.randn(1)/5#2.0*disp.mouseX/white.width-1 
  error = X[0] -setpoint
  order = sp.randn(1)/100 +  pid.computeCorrection(error, dX[0]/dt-0)
  pid.incrementTime(error, dt)
  angle = X[0]
  V = 150*speed(X)
  print setpoint -X[0]
  dX = dt*np.array([k*(windForce(X, 400) +0.1)*(order-0.3*sp.sin(sp.arctan2(X[2], X[1])-sp.pi/2)), -V*sp.sin(angle) + kdrift*order*sp.cos(angle) , V*sp.cos(angle) + kdrift*order*sp.sin(angle)])
  X = X + dX
  kite = kite_base.rotate(sp.rad2deg(angle), fixed=False)
  #kite.save(disp)d
  #white.blit(kite, (799,0)).save(disp)

  toDisplay = black.blit(kite, (max(-kite.width +1, min(white.width-1, int(X[1]+white.width/2-kite.width/2))), max(-kite.height+1, min(white.height-1, int(white.height-X[2]-200)))))
  toDisplay.drawText(str(i_loop*dt), 0, 0, color = Color.RED, fontsize=60)
  toDisplay.save(disp)
  time.sleep(dt)
  


