from SimpleCV import Display, Camera, Image, DrawingLayer, VirtualCamera, Color, JpegStreamCamera
import scipy as sp
import time
import matplotlib.pyplot as plt

cam = JpegStreamCamera('http://192.168.43.1:8080/videofeed')#640 * 480

inputList = []
outputList = []
timeList = []
disp = Display()

fileout = open('saveOutput.txt', 'w')
t0 = time.time()
while time.time()-t0 < 20:
  
  #img = Image("blueRed") #cam.getImage()
  img = cam.getImage()

  blue = img.hueDistance(color = Color.YELLOW).threshold(20).invert().erode(2).dilate(2)
  red = img.hueDistance(color = Color.BLUE).threshold(10).invert().erode(2).dilate(2)
  blueBlobs = blue.findBlobs(minsize = 100)
  redBlobs = red.findBlobs(minsize = 100)

  im = red.sideBySide(blue, side='bottom')
  im.show()
  if blueBlobs and redBlobs:
      redRadius = redBlobs[-1].radius()
      blueRadius = blueBlobs[-1].radius()
      distance = blueBlobs[-1].distanceFrom(redBlobs[-1].coordinates())
      relative_distance = distance/redRadius
      outputList.append(relative_distance)
      timeList.append(time.time())
      fileout.write('%f, %f\n' % (relative_distance, time.time()))
fileout.close()
    
    #relative_distance = distance/sp.mean([blueRadius, redRadius])

plt.plot(timeList, outputList)
plt.show()
