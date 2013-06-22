# -*- coding: utf8 -*-
from SimpleCV import Camera, Image, VirtualCamera, Display, DrawingLayer, Color, JpegStreamCamera
import scipy as sp
import numpy as np
import datetime
import os
import sys
from mpl_toolkits.basemap import Basemap
sys.path.append(os.getcwd())
import mobileState

def modNeg90To90(angle):
    """Returns a modulo between -90 and 90"""
    return (angle+90)%180-90

def unwrap180(angle, previous_angle):
    """Unwraps angle based on previous angle when jump is more than 90°"""
    delta_angle = angle-previous_angle
    return previous_angle + modNeg90To90(delta_angle)

def isPixelInImage((x,y), image):
    return (x>0 and x<image.width and y>0 and y<image.height)

# Open reference image: this is used at initlalisation
target_detail = Image('kite_detail.jpg')

# Get RGB color palette of target (was found to work better than using hue)
pal = target_detail.getPalette(bins = 3, hue = False)

# Open video to analyse or live stream
#cam = VirtualCamera('/media/bat/DATA/Baptiste/Nautilab/kite_project/zenith-wind-power-read-only/KiteControl-Qt/videos/kiteFlying.avi','video')
#cam = VirtualCamera('/media/bat/DATA/Baptiste/Nautilab/kite_project/robokite/ObjectTracking/00095.MTS', 'video')
#cam = VirtualCamera('output1.avi', 'video')
#cam = Camera()
cam = JpegStreamCamera('http://192.168.43.1:8080/videofeed')#640 * 480
img = cam.getImage()
print img.width, img.height
FPS = 25 # Number of frame per second
maxRelativeMotionPerFrame = 2 # How much the target can moved between two succesive frames
pixelPerRadians = 640
radius = pixelPerRadians

# Initialize variables
previous_angle = 0 # target has to be upright when starting. Target width has to be larger than target heigth.
previous_coord_px = (0, 0) # Initialized to top left corner, which always exists
previous_dCoord = previous_coord_px
previous_dAngle = previous_angle
angles = []
coords_px = []
target_elevations = []
target_bearings = []
times = []
wasTargetFoundInPreviousFrame = False


# Automatically detect target
#imgModel.hueDistance(imgModel.getPixel(10,10)).binarize().invert().erode(2).dilate(2).show()

# Create a pygame display
disp = Display((img.width*2, img.height*2))
isPaused = False
selectionInProgress = False
displayDebug = False
imageToRotate = True

print "Press right mouse button to pause or play"
print "Use left mouse button to select target"
print "Target color must be different from background"
print "Target must have width larger than height"


mobile = mobileState.mobileState()
# Loop while not canceled by user
while disp.isNotDone():
#for i_loop in range(0, 500):

    # Receive orientation of the camera
    isUDPConnection = True # Currently switched manually in the code
    if isUDPConnection:
      mobile.checkUpdate()
      if mobile.isToUpdate:
        mobile.computeRPY()
# ------------------------------
    # Get an image from camera
    if not isPaused:
      img = cam.getImage()
      if imageToRotate:
        img = img.rotate(-sp.rad2deg(mobile.roll), fixed = False)
      # else do not rotate image to save computation time

      toDisplay = img
      #img = img.resize(800,600)
    if disp.rightButtonDown:
      isPaused = not(isPaused)
    # Create a layer to enable user to make a selection of the target
    selectionLayer = DrawingLayer((img.width, img.height))
    i_frame = 0#cam.getFrameNumber()
    time = datetime.datetime.utcnow()
    if img:

	    # Create a new layer to host information retrieved from video
	    layer = DrawingLayer((img.width, img.height))
            # Selection is a rectangle drawn while holding mouse left button down
	    if disp.leftButtonDown:
	      corner1 = (disp.mouseX, disp.mouseY)
              selectionInProgress = True
  	    if selectionInProgress:
              corner2 = (disp.mouseX, disp.mouseY)
              bb = disp.pointsToBoundingBox(corner1, corner2)
              if disp.leftButtonUp: # User has finished is selection
                selectionInProgress = False
		selection = img.crop(bb[0], bb[1], bb[2], bb[3])
		if selection != None:
                  # The 3 main colors in the area selected are considered. Note that the selection should be included in the target and not contain background
		  try:
		    pal = selection.getPalette(bins = 3, hue = False)
		  except: #getPalette is sometimes bugging and raising LinalgError because matrix not positive definite
                    pal = pal
   		  wasTargetFoundInPreviousFrame = False
		  previous_coord_px = (bb[0] + bb[2]/2, bb[1] + bb[3]/2)
              if corner1 != corner2:
                selectionLayer.rectangle((bb[0], bb[1]), (bb[2], bb[3]), width = 5, color = Color.YELLOW)
            

	    if wasTargetFoundInPreviousFrame:
                ROITopLeftCorner = (max(0, previous_coord_px[0]-maxRelativeMotionPerFrame/2*width), max(0, previous_coord_px[1] -height*maxRelativeMotionPerFrame/2))
		# Reduce the Region Of Interest around predicted position to save computation time
		ROI = img.crop(ROITopLeftCorner[0], ROITopLeftCorner[1], maxRelativeMotionPerFrame*width, maxRelativeMotionPerFrame*height, centered = False)
		# Draw the rectangle corresponding to the ROI on the complete image
		layer.rectangle((previous_coord_px[0]-maxRelativeMotionPerFrame/2*width, previous_coord_px[1]-maxRelativeMotionPerFrame/2*height), (maxRelativeMotionPerFrame*width, maxRelativeMotionPerFrame*height), color = Color.GREEN, width = 2)

	    else:
		# Search on the whole image if no clue of where is the target
		ROITopLeftCorner = (0, 0)
		ROI = img

	    '''#Option 1
	    target_part0 = ROI.hueDistance(color=(142,50,65)).invert().threshold(150)
	    target_part1 = ROI.hueDistance(color=(93,16,28)).invert().threshold(150)
	    target_part2 = ROI.hueDistance(color=(223,135,170)).invert().threshold(150)
	    target_raw_img = target_part0+target_part1+target_part2
	    target_img = target_raw_img.erode(5).dilate(5)

	    #Option 2
	    target_img = ROI.hueDistance(imgModel.getPixel(10,10)).binarize().invert().erode(2).dilate(2)'''

	    # Option 3
	    ini = ROI-ROI # Black image
	    		
	    # Loop through palette of target colors
	    for col in pal: 
	      c = tuple([int(col[i]) for i in range(0,3)])
              # Search the target based on color
	      target_img = ini + ROI.hueDistance(color = c).threshold(50)
            #target_img = ROI.hueDistance(color = Color.RED).threshold(10).invert()

	    # Get a black background with with white target foreground
	    target_img = target_img.threshold(150)#.erode(2).dilate(2)

	    # Search for binary large objects representing potential target
	    target = target_img.findBlobs(minsize = 1000)
	    if displayDebug:
	      ini = target_img.resize(int(img.width/(len(pal)+1)),  int(img.height/(len(pal)+1)))
              for i_col in range(len(pal)): 
		      col = pal[i_col]
		      c = tuple([int(col[i]) for i in range(3)])
		      [R, G, B] = ROI.hueDistance(color = c).threshold(50).invert().splitChannels()
		      white = (R-R).invert()
		      r = R*1.0/255*c[0]
		      g = B*1.0/255*c[1]
		      b = G*1.0/255*c[2]
		      tmp = R.mergeChannels(r, b, g) # Order had to be changed here for unknown reason
		      ini = ini.sideBySide(tmp.resize(int(img.width/(len(pal)+1)), int(img.height/(len(pal)+1))), side = 'bottom')
              ini = ini.adaptiveScale((int(img.width), int(img.height)))

	    
	    if target: # If a target was found
		if wasTargetFoundInPreviousFrame:
		    predictedTargetPosition = (width*maxRelativeMotionPerFrame/2, height*maxRelativeMotionPerFrame/2) # Target will most likely be close to the center of the ROI   
		else:
		    predictedTargetPosition = previous_coord_px
                # If there are several targets in the image, take the one which is the closest of the predicted position
		target = target.sortDistance(predictedTargetPosition)

		# Get target coordinates according to minimal bounding rectangle or centroid.
		coordMinRect = ROITopLeftCorner + np.array((target[0].minRectX(), target[0].minRectY()))
		coord_px = ROITopLeftCorner + np.array(target[0].centroid())
		# Rotate the coordinates of roll angle around the middle of the screen
		ctm = np.array([[sp.cos(mobile.roll*(not(imageToRotate))), -sp.sin(mobile.roll*(not(imageToRotate)))],[sp.sin(mobile.roll*(not(imageToRotate))), sp.cos(mobile.roll*(not(imageToRotate)))]])
                rot_coord_px = np.dot(ctm, coord_px - np.array([img.width/2, img.height/2])) + np.array([[img.width/2], [img.height/2]])
		m = Basemap(width=img.width, height=img.height, projection='aeqd',
		    lat_0=sp.rad2deg(mobile.pitch),lon_0=sp.rad2deg(mobile.yaw), rsphere = radius)
		coord_deg = m(rot_coord_px[0], img.height-rot_coord_px[1], inverse = True)
		target_bearing_deg, target_elevation_deg = coord_deg
		# Get minimum bounding rectangle for display purpose
		minR = ROITopLeftCorner + np.array(target[0].minRect())

		contours = target[0].contour()
		contours = [ ROITopLeftCorner + np.array(contour) for contour in contours]
	
		# Get target features
		angle = sp.deg2rad(target[0].angle()) + mobile.roll
		angle =  unwrap180(angle, previous_angle)
		width = target[0].width()
		height = target[0].height()
		
                # Filter the data
		alpha = 0.1
                if not(isPaused):
		  dCoord = np.array(previous_dCoord)*(1-alpha) + alpha*(np.array(coord_px) - previous_coord_px) # related to the speed only if cam is fixed
	          dAngle = np.array(previous_dAngle)*(1-alpha) + alpha*(np.array(angle) - previous_angle)
		else : 
		  dCoord = np.array([0, 0])
		  dAngle = np.array([0]) 
#print coord_px, angle, width, height, dCoord
	
		# Save important data
		times.append(time)
		coords_px.append(coord_px)
		angles.append(angle)
  		target_elevations.append(sp.deg2rad(target_elevation_deg))
		target_bearings.append(sp.deg2rad(target_bearing_deg))
		
		# Save for initialisation of next step
		previous_dCoord = dCoord
		previous_angle = angle
		previous_coord_px = (int(coord_px[0]), int(coord_px[1]))
		wasTargetFoundInPreviousFrame = True
	
		# Add target features to layer
 		# Minimal rectange and its center in RED
		layer.polygon(minR[(0, 1, 3, 2), :], color = Color.RED, width = 5)
		layer.circle((int(coordMinRect[0]), int(coordMinRect[1])), 10, filled = True, color = Color.RED)
		
                # Target contour and centroid in BLUE
		layer.circle((int(coord_px[0]), int(coord_px[1])), 10, filled = True, color = Color.BLUE)
                layer.polygon(contours, color = Color.BLUE, width = 5)

		# Speed vector in BLACK
		layer.line((int(coord_px[0]), int(coord_px[1])), (int(coord_px[0]+20*dCoord[0]), int(coord_px[1]+20*dCoord[1])), width = 3)
		
		# Line giving angle
		layer.line((int(coord_px[0]+200*sp.cos(angle)), int(coord_px[1]+200*sp.sin(angle))), (int(coord_px[0]-200*sp.cos(angle)), int(coord_px[1]-200*sp.sin(angle))), color = Color.RED)

		# Line giving rate of turn
		#layer.line((int(coord_px[0]+200*sp.cos(angle+dAngle*10)), int(coord_px[1]+200*sp.sin(angle+dAngle*10))), (int(coord_px[0]-200*sp.cos(angle + dAngle*10)), int(coord_px[1]-200*sp.sin(angle+dAngle*10))))

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
	    toDisplay.drawText(str(i_frame)+" "+ str(time), x=0, y=0, fontsize=20)
	    # Line giving horizon
            #layer.line((0, int(img.height/2 + mobile.pitch*pixelPerRadians)),(img.width, int(img.height/2 + mobile.pitch*pixelPerRadians)), width = 3, color = Color.RED)
	    # plot parallels
	    for lat in range(-90, 90, 30):
	      r = range(0, 361, 10)
	      l = m (r, [lat]*len(r))
	      pix = [np.array(l[0]), img.height-np.array(l[1])]
	      #pix = np.dot(ctm, np.array([l[0], np.array(img.height)-l[1]]) - np.array([img.width/2, img.height/2])) + np.array([img.width/2, img.height/2])

	      for i in range(len(r)-1):
                if isPixelInImage((pix[0][i],pix[1][i]), img) or isPixelInImage((pix[0][i+1],pix[1][i+1]), img):
	          layer.line((pix[0][i],pix[1][i]), (pix[0][i+1], pix[1][i+1]), color=Color.WHITE, width = 2)
	    # plot meridians
	    for lon in range(0, 360, 30):
	      r = range(-90, 91, 10)
	      l = m ([lon]*len(r), r)
 	      pix = [np.array(l[0]), img.height-np.array(l[1])]
	      #pix = np.dot(ctm,[np.array(l[0]), img.height-np.array(l[1])]- np.array([img.width/2, img.height/2])) + np.array([img.width/2, img.height/2])

	      for i in range(len(r)-1):
                if isPixelInImage((pix[0][i],pix[1][i]), img) or isPixelInImage((pix[0][i+1],pix[1][i+1]), img):
	          layer.line((pix[0][i],pix[1][i]), (pix[0][i+1], pix[1][i+1]), color=Color.WHITE, width = 2)

	    # Text giving heading
	    for heading in range(0, 360, 30):
              layer.text(str(heading), ( img.width/2-(sp.mod(mobile.yaw-sp.deg2rad(heading)+sp.pi,2*sp.pi)-sp.pi)*pixelPerRadians,int(img.height/2 + mobile.pitch*pixelPerRadians)), color = Color.RED)
	    

	    toDisplay.save(disp)
    toDisplay.removeDrawingLayer(1)
    toDisplay.removeDrawingLayer(0)
	    
