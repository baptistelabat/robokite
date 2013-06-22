
from SimpleCV import ColorSegmentation, Image, Camera, VirtualCamera, Display

# Open reference video
cam=VirtualCamera('/media/bat/DATA/Baptiste/Nautilab/kite_project/zenith-wind-power-read-only/KiteControl-Qt/videos/kiteFlying.avi','video')
# Select reference image
img=cam.getFrame(50)
modelImage = img.crop(255, 180, 70, 20)
modelImage = Image('kite_detail.jpg')
ts = []
disp=Display()
for i in range(0,50):
    img = cam.getImage()
while (disp.isNotDone()):
        img = cam.getImage()
	bb = (255, 180, 70, 20)
        ts = img.track("camshift",ts,modelImage,bb, num_frames = 1)
        # now here in first loop iteration since ts is empty,
        # img0 and bb will be considered.
        # New tracking object will be created and added in ts (TrackSet)
        # After first iteration, ts is not empty and hence the previous
        # image frames and bounding box will be taken from ts and img0
        # and bb will be ignored.
	ts.drawPath()
	img.show()


