from SimpleCV import Image, Color, Display
import scipy as sp
import time

logo = Image("logo")
black = logo-logo
black = black.resize(1920, 400)
white = black.invert()
white.drawCircle((200, 200), 100, color = Color.BLUE, thickness = -1)
white.drawCircle((400, 200), 100, color = Color.YELLOW, thickness = -1)
black.show()

inputList = []
timeList = []
disp = Display()
filein = open('saveInput.txt', 'w')
t0 = time.time()
while time.time()-t0 < 20:
  black = black - black
  white = black.invert()
  white.drawCircle((900, 200), 100, color = Color.BLUE, thickness = -1)
  dist = 4 + sp.sin(time.time())
  white.drawCircle((100*(9 + dist), 200), 100, color = Color.YELLOW, thickness = -1)
  im = white.applyLayers()
  im.show()
  inputList.append(dist)
  timeList.append(time.time())
  filein.write('%f, %f\n' % (time.time(), dist))
filein.close()
