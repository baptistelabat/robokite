from SimpleCV import Camera, Image, VirtualCamera, Display, DrawingLayer, Color
import scipy as sp
import numpy as np
import matplotlib.pyplot as plt
import datetime

def modNeg90To90(angle):
    """Return a modulo between -90 and 90"""
    return (angle+90)%180-90
def unwrap180(angle, previous_angle):
    """Unwrap angle based on previous angle when jump is more than 90"""
    delta_angle = angle-previous_angle
    return previous_angle + modNeg90To90(delta_angle)


# Open reference image
target_detail = Image('kite_detail.jpg')

# Get RGB color palette of target (was found to work better than using hue)
pal = target_detail.getPalette(bins = 3, hue = False)



# Open video to analyse or live stream
cam = VirtualCamera('/media/bat/DATA/Baptiste/Nautilab/kite_project/zenith-wind-power-read-only/KiteControl-Qt/videos/kiteFlying.avi','video')
#cam = VirtualCamera('/media/bat/DATA/Baptiste/Nautilab/kite_project/robokite/ObjectTracking/00095.MTS', 'video')
#cam = VirtualCamera('output1.avi', 'video')
#cam = Camera()
FPS = 25 # Number of frame per second
maxRelativeMotionPerFrame = 2 # How much the target can moved between two succesive frames

# Initialize variables
previous_angle = 0 # target has to be upright when starting. Target width has to be larger than target heigth.
previous_coord = (0, 0) # Initialized to top left corner, which always exists
previous_dCoord = previous_coord
previous_dAngle = previous_angle
angles = []
coords = []
times = []
wasTargetFoundInPreviousFrame = False


# Automatically detect kite
#imgModel.hueDistance(imgModel.getPixel(10,10)).binarize().invert().erode(2).dilate(2).show()

# Create a pygame display
disp = Display((640*2, 480*2))
isPaused = False
updateSelection = False
displayDebug = False

print "Press right mouse button to pause or play"
print "Use left mouse button to select target"
print "Target color must be different from background"
print "Target must have width larger than height"

# Loop while not canceled by user
while disp.isNotDone():
#for i_loop in range(0, 500):
    # Get an image from camera
    if not isPaused:
      img = cam.getImage()
      toDisplay = img
      #img = img.resize(800,600)
    if disp.rightButtonDown:
      isPaused = not(isPaused)
    # Create a layer to add a selection
    selectionLayer = DrawingLayer((img.width, img.height))
    i_frame = 0#cam.getFrameNumber()
    time = datetime.datetime.utcnow()
    if img:

	    # Create a new layer to host information retrieved from video
	    layer = DrawingLayer((img.width, img.height))
	    if disp.leftButtonDown:
	      corner1 = (disp.mouseX, disp.mouseY)
              updateSelection = True
  	    if updateSelection:
              corner2 = (disp.mouseX, disp.mouseY)
              bb = disp.pointsToBoundingBox(corner1, corner2)
              if disp.leftButtonUp: 
                updateSelection = False
		selection = img.crop(bb[0],bb[1],bb[2],bb[3])
		if selection!=None:
		  pal = selection.getPalette(bins = 3, hue = False)
   		  wasTargetFoundInPreviousFrame = False
		  previous_coord = (bb[0] + bb[2]/2, bb[1] + bb[3]/2)
              if corner1!=corner2:
                selectionLayer.rectangle((bb[0],bb[1]),(bb[2],bb[3]), width = 5, color = Color.YELLOW)
            

	    if wasTargetFoundInPreviousFrame:
                ROITopLeftCorner = (max(0,previous_coord[0]-maxRelativeMotionPerFrame/2*width), max(0, previous_coord[1] -height*maxRelativeMotionPerFrame/2))
		 # Reduce the Region Of Interest around predicted position to save computation time
		ROI = img.crop(ROITopLeftCorner[0], ROITopLeftCorner[1], maxRelativeMotionPerFrame*width, maxRelativeMotionPerFrame*height, centered = False)
		layer.rectangle((previous_coord[0]-maxRelativeMotionPerFrame/2*width, previous_coord[1]-maxRelativeMotionPerFrame/2*height), (maxRelativeMotionPerFrame*width, maxRelativeMotionPerFrame*height), color = Color.GREEN, width = 2)

	    else:
		# Search on the whole image if no clue
		ROITopLeftCorner = (0, 0)
		ROI = img

	    '''#Option 1
	    kite_part0 = ROI.hueDistance(color=(142,50,65)).invert().threshold(150)
	    kite_part1 = ROI.hueDistance(color=(93,16,28)).invert().threshold(150)
	    kite_part2 = ROI.hueDistance(color=(223,135,170)).invert().threshold(150)
	    kite_raw_img = kite_part0+kite_part1+kite_part2
	    kite_img = kite_raw_img.erode(5).dilate(5)

	    #Option 2
	    kite_img = ROI.hueDistance(imgModel.getPixel(10,10)).binarize().invert().erode(2).dilate(2)'''

	    #Option 3
	    ini = ROI-ROI #black image
	    		
	    # Loop through palette of kite colors
	    for col in pal: 
	      c = tuple([int(col[i]) for i in range(0,3)])
	      target_img = ini + ROI.hueDistance(color = c).threshold(100)

	    # Get a black background with with white target foreground
	    target_img = target_img.invert().threshold(150)#.erode(2).dilate(2)

	    # Search for binary large objects representing potential target
	    target = target_img.findBlobs(minsize = 1000)

	    ini = target_img.resize(int(img.width/(len(pal)+1)),  int(img.height/(len(pal)+1)))
            for i_col in range(0, len(pal)): 
		      col = pal[i_col]
		      c = tuple([int(col[i]) for i in range(0,3)])
		      [R, G, B] = ROI.hueDistance(color = c).threshold(30).invert().splitChannels()
		      white = (R-R).invert()
		      r = R*1.0/255*c[0]
		      g = B*1.0/255*c[1]
		      b = G*1.0/255*c[2]
		      tmp = R.mergeChannels(r, b, g) #Order has to be changed here for unknown reason
		      ini = ini.sideBySide(tmp.resize(int(img.width/(len(pal)+1)), int(img.height/(len(pal)+1))), side = 'bottom')
            ini = ini.adaptiveScale((int(img.width), int(img.height)))

	    
	    if target: # If a target was found
		if wasTargetFoundInPreviousFrame:
		    predictedTargetPosition = (width*maxRelativeMotionPerFrame/2, height*maxRelativeMotionPerFrame/2) # Target will most likely be close to the center of the ROI   
		else:
		    predictedTargetPosition = previous_coord
		target = target.sortDistance(predictedTargetPosition)

		# Get target coordinates according to minimal bounding rectangle or centroid.
		coordMinRect = ROITopLeftCorner + np.array((target[0].minRectX(), target[0].minRectY()))
		coord = ROITopLeftCorner + np.array(target[0].centroid())
		# Get minimum bounding rectangle for display purpose
		minR = ROITopLeftCorner + np.array(target[0].minRect())

		contours = target[0].contour()
		contours = [ ROITopLeftCorner + np.array(contour) for contour in contours]
	
		# Get target features
		angle = sp.deg2rad(target[0].angle())
		angle =  unwrap180(angle, previous_angle)
		width = target[0].width()
		height = target[0].height()
		alpha = 0.1
		dCoord = np.array(previous_dCoord)*(1-alpha) + alpha*(np.array(coord) - previous_coord) # related to the speed only if cam is fixed
	        dAngle = np.array(previous_dAngle)*(1-alpha) + alpha*(np.array(angle) - previous_angle)
		#print coord, angle, width, height, dCoord
	
		# Save important data
		times.append(time)
		coords.append(coord)
		angles.append(angle)
		
		# Save for initialisation of next step
		previous_dCoord = dCoord
		previous_angle = angle
		previous_coord = (int(coord[0]), int(coord[1]))
		wasTargetFoundInPreviousFrame = True
	
		# Add target features to layer
		layer.polygon(minR[(0, 1, 3, 2), :], color = Color.RED, width = 5)
		layer.circle((int(coordMinRect[0]), int(coordMinRect[1])), 10, filled = True, color = Color.RED)
		layer.circle((int(coord[0]), int(coord[1])), 10, filled = True, color = Color.BLUE)
		layer.line((int(coord[0]), int(coord[1])), (int(coord[0]+20*dCoord[0]), int(coord[1]+20*dCoord[1])), width = 3)
		layer.polygon(contours, color = Color.BLUE, width = 5)
		# Line giving angle
		layer.line((int(coord[0]+200*sp.cos(angle)), int(coord[1]+200*sp.sin(angle))), (int(coord[0]-200*sp.cos(angle)), int(coord[1]-200*sp.sin(angle))), color = Color.RED)
		# Line giving rate of turn
		#layer.line((int(coord[0]+200*sp.cos(angle+dAngle*10)), int(coord[1]+200*sp.sin(angle+dAngle*10))), (int(coord[0]-200*sp.cos(angle + dAngle*10)), int(coord[1]-200*sp.sin(angle+dAngle*10))))
	    else:
		wasTargetFoundInPreviousFrame = False

            if displayDebug:
		    toDisplay = img.sideBySide(ini)
	    else:
            	toDisplay = img
	    # Add the layer to the raw image 
	    toDisplay.addDrawingLayer(layer)
	    toDisplay.addDrawingLayer(selectionLayer)

	    # Add time metadata
	    #toDisplay.drawText(str(i_frame)+" "+ str(time), x=0, y=0, fontsize=60)
	    
	    # Use show instead of save to display to be able to display the layer
	    
	    toDisplay.save(disp)
    toDisplay.removeDrawingLayer(1)
    toDisplay.removeDrawingLayer(0)
	    
plt.subplot(211)
plt.plot(times, coords)
plt.subplot(212)
plt.plot(times, angles)
plt.show()


