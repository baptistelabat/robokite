from SimpleCV import ColorSegmentation, Image, Camera, VirtualCamera

# Open reference video
cam=VirtualCamera('/media/bat/DATA/Baptiste/Nautilab/kite_project/zenith-wind-power-read-only/KiteControl-Qt/videos/kiteFlying.avi','video')
# Select reference image
img=cam.getFrame(50)
modelImage = img.crop(255, 180, 70, 20)
cs = ColorSegmentation()

cs.addToModel(modelImage)


img = cam.getImage()
cs.addImage(img)
res=cs.getSegmentedImage()
res.show()
