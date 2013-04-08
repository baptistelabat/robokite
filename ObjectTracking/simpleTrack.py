from SimpleCV import *
# Open reference video
cam=VirtualCamera('/media/bat/DATA/Baptiste/Nautilab/kite_project/zenith-wind-power-read-only/KiteControl-Qt/videos/kiteFlying.avi','video')
# Select reference image
img=cam.getFrame(50)
# Automatically detect kite
mor=img.morphClose().morphClose().invert()
kite_invert=mor.findBlobs(minsize=1000, maxsize=5000)
# Save kite color
color_invert=kite_invert.meanColor()

# Open video to analyse or live stream
cam=VirtualCamera('/media/bat/DATA/Baptiste/Nautilab/kite_project/zenith-wind-power-read-only/KiteControl-Qt/videos/kiteTest.avi','video')
disp=Display()
while disp.isNotDone():
    img=cam.getImage()
    myLayer = DrawingLayer((img.width,img.height))
    toShow = img.invert().colorDistance(color=color_invert[0]).invert().threshold(200)
    kite_part0 = img.hueDistance(color=(142,50,65)).invert().threshold(150)
    kite_part1 = img.hueDistance(color=(93,16,28)).invert().threshold(150)
    kite_part2 = img.hueDistance(color=(223,135,170)).invert().threshold(150)
    kite_raw_img = kite_part0+kite_part1+kite_part2
    kite_img = kite_raw_img#.morphClose().morphClose().morphClose().morphClose().morphClose().morphClose().morphClose().morphClose().morphClose().morphClose().morphClose()
    kite=kite_img.findBlobs(minsize=1000)
    if kite:
        kite[0].draw()
        kite[0].drawMinRect(layer=myLayer, color=Color.RED)
        myLayer.circle(filled=True, center=(10,10), color=Color.RED, radius=50)
    kite_img.addDrawingLayer(myLayer)
    kite_img.show()
    #---------------------------- myLayer = DrawingLayer((img.width,img.height))
    #------------------------------------------------------------------ if kite:
        #-------------------------------------------- #kite[0].drawMaskToLayer()
        #------------------- kite[0].drawMinRect(layer=myLayer, color=Color.RED)
        #------------------------------ kite[0].drawMaskToLayer(layer = myLayer)
        # myLayer.circle(filled=True, center=(10,10), color=Color.RED, radius=50)
    #---------------------------------------------- img.addDrawingLayer(myLayer)
    #---------------------------------------------------------------- img.show()
