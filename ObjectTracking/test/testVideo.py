from SimpleCV import *
#cam = VirtualCamera('/media/bat/DATA/images/2013/03/15/00137.MTS','video')
cam = Camera(0)
#vs = VideoStream("cartoon.avi", 15, False)
disp = Display((320*3,240*3))
i=1
while disp.isNotDone():
	img=cam.getImage()
	i=i+1
	#img.drawText(str(i), x=0, y=0, fontsize=60)
	img.save(disp)
	#img.save(vs)
	#vs.writeFrame(img)

