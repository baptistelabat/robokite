from SimpleCV import Image, Display, Color
import numpy as np
import scipy as sp
import time
#import PID

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
  return F #-0.1*sp.cos(sp.arctan2(X[2], X[1])-sp.pi/2 -X[0])
  
order = 1


class kiteModel:
  def __init__(self):
    self.X = np.array([0, 0, 0])
    self.dt = 0.1
    self.k = -4
    self.kdrift  = 0
  
  def update(self, order, dt=0.1):
    angle = self.X[0]
    V = 150*speed(self.X)
    order_stick = order + sp.arctan2(self.X[2], self.X[1])/sp.pi/2
    dx0 = self.k*(windForce(self.X, 400) +0.)*(order_stick-0.*sp.sin(sp.arctan2(self.X[2], self.X[1])-sp.pi/2))
    dx1 = -V*sp.sin(angle) + self.kdrift*order*sp.cos(angle) 
    dx2 = V*sp.cos(angle) + self.kdrift*order*sp.sin(angle)-30
    dX = dt*np.array([dx0[0], dx1[0], dx2[0]])
    self.X = self.X + dX
    self.X = np.array([self.X[0], self.X[1], np.max([0, self.X[2]])])

if __name__ == '__main__': 
  #Create a black and a background image
  background = Image('http://fond-d-ecran-gratuit.org/photos/plage-mer-petites-vagues.jpg').resize(800, 600)
  kite_base = Image('http://www.winds-up.com/images/annonces/7915_1.jpg').resize(50, 50).invert()
  disp = Display()

  i_loop = 0
  #pid = PID.PID(1, 1, 0.1)
  offset = sp.pi/3*0
  kite_model = kiteModel()
  dX = 0*kite_model.X
  while disp.isNotDone():
    setpoint = sp.pi/1.7*sp.sin(2*sp.pi/7*time.time())+offset
    i_loop = i_loop +1
    order = 0+0*sp.randn(1)/5+2.0*disp.mouseX/background.width-1 
    #error = X[0] -setpoint
    #order = sp.randn(1)/100 +  pid.computeCorrection(error, dX[0]/dt-0)
    #pid.incrementTime(error, dt)
    dt = 0.1
    kite_model.update(order, dt)
    print kite_model.X
    kite = kite_base.rotate(sp.rad2deg(kite_model.X[0]), fixed=False).invert()
    #kite.save(disp)d
    #background.blit(kite, (799,0)).save(disp)

    toDisplay = background.blit(kite.invert(), (max(-kite.width +1, min(background.width-1, int(kite_model.X[1]+background.width/2-kite.width/2))), max(-kite.height+1, min(background.height-1, int(background.height-kite_model.X[2]-200)))), mask = kite.binarize())

    toDisplay.drawText(str(i_loop*dt), 0, 0, color = Color.RED, fontsize=60)
    toDisplay.save(disp)
    time.sleep(dt)
  


