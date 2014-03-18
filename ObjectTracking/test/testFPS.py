# -*- coding: utf8 -*-
from SimpleCV import Camera, Image, VirtualCamera, Display, DrawingLayer, Color, JpegStreamCamera
import time
import os
import sys
import threading
sys.path.append(os.getcwd())
sys.path.append('../Sensors')
import mobileState
# Open video to analyse or live stream
#cam = VirtualCamera('/media/bat/DATA/Baptiste/Nautilab/kite_project/zenith-wind-power-read-only/KiteControl-Qt/videos/kiteFlying.avi','video')
#cam = VirtualCamera('/media/bat/DATA/images/2013/05/18/00169.MTS','video')

cam = JpegStreamCamera('http://192.168.43.1:8080/videofeed')#640 * 480
#cam = Camera(0)
img = cam.getImage()
disp = Display((img.width*2, img.height*2))
t0 = time.time()
previousTime = t0
FPSList = []
mobile = mobileState.mobileState()
#a = threading.Thread(None, mobileState.mobileState.checkUpdate, None, (mobile,))
#a.start()
i = 0
while time.time()-t0<10:
  mobile.computeRPY()
  i = i+1
  t = time.time()
  img = cam.getImage()
  print img.getPixel(0,0)
  FPS = 1/(t-previousTime)
  print FPS
  FPSList.append(FPS)
  previousTime = t
  img.save(disp)
print i
print img.width, img.height
  
