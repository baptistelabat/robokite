from SimpleCV import JpegStreamer, Display, VirtualCamera, VideoStream
import time
import numpy as np

cam = VirtualCamera('../Recording/Videos/Flying kite images (for kite steering unit development)-YTMgX1bvrTo.flv','video')
port = 8000+np.random.randint(100)
print port
js = JpegStreamer(hostandport=port
, st=0.1)
vs = VideoStream("out.avi", fps=15)
endReached = False
t0 = time.time()
FPS = 25
while not(endReached):
  t = time.time()-t0
  toDisplay = cam.getFrame(np.ceil(t*FPS))
  if toDisplay.size()!=(0,0):
    toDisplay.save(js)
  else:
	  endReached = True
  time.sleep(0.05)
