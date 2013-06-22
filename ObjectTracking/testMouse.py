from SimpleCV import Display, Camera, Image, DrawingLayer, VirtualCamera
disp = Display((600,800))
#cam = Camera()
cam = VirtualCamera('/media/bat/DATA/Baptiste/Nautilab/kite_project/zenith-wind-power-read-only/KiteControl-Qt/videos/kiteTest.avi','video')
isPaused = False
updateSelection = False
while(disp.isNotDone()):  
  if not isPaused:
    img_flip = cam.getImage().flipHorizontal()
    img = img_flip.edges(150, 100).dilate()
  if disp.rightButtonDown:
    isPaused = not(isPaused)
  selectionLayer = DrawingLayer((img.width, img.height))
  if disp.leftButtonDown:
	corner1 = (disp.mouseX, disp.mouseY)
        updateSelection = True
  if updateSelection:
    corner2 = (disp.mouseX, disp.mouseY)
    bb = disp.pointsToBoundingBox(corner1, corner2)
    if disp.leftButtonUp: 
      updateSelection = False
    if corner1!=corner2:
     selectionLayer.rectangle((bb[0],bb[1]),(bb[2],bb[3]))
  img.addDrawingLayer(selectionLayer)
  img.save(disp)
  img.removeDrawingLayer(0)

