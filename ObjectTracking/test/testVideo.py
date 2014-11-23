from SimpleCV import *
cam = Camera()
vs = VideoStream("myvideo.avi", 25, True)

i=0
while(i<300):
    #for 300 frames
    i = cam.getImage()
    i.save(vs)
    i+=1
