#!/usr/bin/env python
# -*- coding: utf8 -*-
#
# Copyright (c) 2013 Nautilabs
#
# Licensed under the MIT License,
# https://github.com/baptistelabat/robokite
# Authors: Baptiste LABAT

# Import module from standard library
import datetime
import time
import os
import sys
import threading

# Import external library
from SimpleCV import Camera, Image, VirtualCamera, Display, DrawingLayer, Color, JpegStreamCamera, JpegStreamer
import scipy as sp
import numpy as np
import csv
try:
  from mpl_toolkits.basemap import Basemap
  useBasemap = True
except ImportError:
  useBasemap = False
try:
  import h5py
  useHDF5 = True
except ImportError:
  useHDF5 = False

# Import robokite specific lib
sys.path.append(os.getcwd())
sys.path.append('../Control')
sys.path.append('../Sensors')
import mobileState
import PID

def localProjection(lon, lat, radius, lon_0, lat_0, inverse = False):
  """ This function was written to use instead of Basemap which is very slow"""
  if inverse: 
    x, y = lon, lat
    lat = np.fmin( np.fmax(lat_0 + y/radius, -sp.pi/2), sp.pi/2)
    lon = sp.mod(lon_0 + x/(radius*sp.cos(lat_0))+sp.pi, 2*sp.pi) -sp.pi
    return (lon, lat)
  else:
    y = (lat-lat_0)*radius
    x = (sp.mod(lon-lon_0+sp.pi, 2*sp.pi)-sp.pi)*radius*sp.cos(lat_0)
  return (x, y)

def modNeg90To90(angle_deg):
    """Returns a modulo between -90 and 90"""
    return (angle_deg + 90)%180-90

def unwrap180(angle_deg, previous_angle_deg):
    """Unwraps angle based on previous angle when jump is more than 90°"""
    delta_angle_deg = angle_deg-previous_angle_deg
    return previous_angle_deg + modNeg90To90(delta_angle_deg)

def isPixelInImage((x,y), image):
    return (x>0 and x<image.width and y>0 and y<image.height)
    
def computeXORChecksum(chksumdata):
    # Inspired from http://doschman.blogspot.fr/2013/01/calculating-nmea-sentence-checksums.html
    # Initializing XOR counter
    csum = 0
    
    # For each char in chksumdata, XOR against the previous 
    # XOR char.  The final XOR of the last char will be the
    # checksum  
    for c in chksumdata:
        # Makes XOR value of counter with the next char in line
        # and stores the new XOR value in csum
        csum ^= ord(c)
    h = hex(csum)    
    return h[2:].zfill(2)#get hex data without 0x prefix
    

class Kite:
 def __init__(self):
   self.elevation = 0
   self.bearing = 0
   self.distance = 0
   self.speed = 0
   self.ROT = 0
   self.orientation = 0
   self.lastUpdateTime = 0
   self.filterTimeConstant = 0.5

 def track(self):
  print "Press right mouse button to pause or play"
  print "Use left mouse button to select target"
  print "Target color must be different from background"
  print "Target must have width larger than height"
  print "Target can be upside down"

  #Parameters
  isUDPConnection = False # Currently switched manually in the code
  display = True
  displayDebug = True
  useBasemap = False
  maxRelativeMotionPerFrame = 2 # How much the target can moved between two succesive frames
  pixelPerRadians = 320
  radius = pixelPerRadians
  referenceImage = '../ObjectTracking/kite_detail.jpg'
  scaleFactor = 0.5
  isVirtualCamera = True
  useHDF5 = False

  # Open reference image: this is used at initlalisation
  target_detail = Image(referenceImage)

  # Get RGB color palette of target (was found to work better than using hue)
  pal = target_detail.getPalette(bins = 2, hue = False)

  # Open video to analyse or live stream
  #cam = JpegStreamCamera('http://192.168.1.29:8080/videofeed')#640 * 480
  if isVirtualCamera:
    #cam = VirtualCamera('../../zenith-wind-power-read-only/KiteControl-Qt/videos/kiteFlying.avi','video')
    #cam = VirtualCamera('/media/bat/DATA/Baptiste/Nautilab/kite_project/robokite/ObjectTracking/00095.MTS', 'video')
    #cam = VirtualCamera('output.avi', 'video')
    cam = VirtualCamera('../Recording/Videos/Flying kite images (for kite steering unit development)-YTMgX1bvrTo.flv','video')
    virtualCameraFPS = 25
  else:
    cam = JpegStreamCamera('http://192.168.43.1:8080/videofeed')#640 * 480
    #cam = Camera() 

  # Get a sample image to initialize the display at the same size
  img = cam.getImage().scale(scaleFactor)
  print img.width, img.height
  # Create a pygame display
  if display:
   if img.width>img.height:
     disp = Display((27*640/10,25*400/10))#(int(2*img.width/scaleFactor), int(2*img.height/scaleFactor)))
   else:
     disp = Display((810,1080))
  #js = JpegStreamer()



  # Initialize variables
  previous_angle = 0 # target has to be upright when starting. Target width has to be larger than target heigth.
  previous_coord_px = (0, 0) # Initialized to top left corner, which always exists
  previous_dCoord = previous_coord_px
  previous_dAngle = previous_angle
  angles = []
  coords_px = []
  coord_px = [0, 0]
  angle = 0
  target_elevations = []
  target_bearings = []
  times = []
  wasTargetFoundInPreviousFrame = False
  i_frame = 0
  isPaused = False
  selectionInProgress = False
  th = [100, 100, 100]
  skycolor = Color.BLUE
  timeLastTarget = 0

  # Prepare recording
  recordFilename = datetime.datetime.utcnow().strftime("%Y%m%d_%Hh%M_")+ 'simpleTrack'
  if useHDF5:
    try:
      os.remove(recordFilename + '.hdf5') 
    except:
      print('Creating file ' + recordFilename + '.hdf5')
    """ The following line is used to silence the following error (according to http://stackoverflow.com/questions/15117128/h5py-in-memory-file-and-multiprocessing-error)
    #000: ../../../src/H5F.c line 1526 in H5Fopen(): unable to open file
    major: File accessability
    minor: Unable to open file"""
    h5py._errors.silence_errors()
    recordFile = h5py.File(os.path.join(os.getcwd(), 'log', recordFilename + '.hdf5'), 'a') 
    hdfSize = 0    
    dset = recordFile.create_dataset('kite', (2,2), maxshape=(None,7))
    imset = recordFile.create_dataset('image', (2,img.width,img.height,3 ), maxshape=(None, img.width, img.height, 3))
  else:
    try:
      os.remove(recordFilename + '.csv')   
    except:
      print('Creating file ' + recordFilename + '.csv') 
    recordFile = file(os.path.join(os.getcwd(), 'log', recordFilename + '.csv'), 'a')
    csv_writer = csv.writer(recordFile)
    csv_writer.writerow(['Time (s)', 'x (px)', 'y (px)', 'Orientation (rad)', 'Elevation (rad)', 'Bearing (rad)', 'ROT (rad/s)'])

  # Launch a thread to get UDP message with orientation of the camera
  mobile = mobileState.mobileState()
  if isUDPConnection:
    mobile.open()
  # Loop while not canceled by user
  t0 = time.time()
  previousTime = t0
  while not(display) or disp.isNotDone():
    t = time.time()
    deltaT = (t-previousTime)
    FPS = 1.0/deltaT
    #print 'FPS =', FPS
    if isVirtualCamera:
      deltaT = 1.0/virtualCameraFPS
    previousTime = t
    i_frame = i_frame + 1
    timestamp = datetime.datetime.utcnow()

    # Receive orientation of the camera
    if isUDPConnection:
      mobile.computeRPY([2, 0, 1], [-1, 1, 1])
    ctm = np.array([[sp.cos(mobile.roll), -sp.sin(mobile.roll)], \
            [sp.sin(mobile.roll), sp.cos(mobile.roll)]]) # Coordinate transform matrix

    if useBasemap:
    # Warning this really slows down the computation
      m = Basemap(width=img.width, height=img.height, projection='aeqd',
            lat_0=sp.rad2deg(mobile.pitch), lon_0=sp.rad2deg(mobile.yaw), rsphere = radius)

    # Get an image from camera
    if not isPaused:
      img = cam.getImage()
      img = img.resize(int(scaleFactor*img.width), int(scaleFactor*img.height))
    
    if display:
      # Pause image when right button is pressed
      dwn = disp.rightButtonDownPosition()
      if dwn is not None:
        isPaused = not(isPaused)
        dwn = None

    if display:
    # Create a layer to enable user to make a selection of the target
      selectionLayer = DrawingLayer((img.width, img.height))

    if img:
      if display: 
      # Create a new layer to host information retrieved from video
        layer = DrawingLayer((img.width, img.height))
          # Selection is a rectangle drawn while holding mouse left button down
        if disp.leftButtonDown:
          corner1 = (disp.mouseX, disp.mouseY)
          selectionInProgress = True
        if selectionInProgress:
          corner2 = (disp.mouseX, disp.mouseY)
          bb = disp.pointsToBoundingBox(corner1, corner2)# Display the temporary selection
          if disp.leftButtonUp: # User has finished is selection
            selectionInProgress = False
            selection = img.crop(bb[0], bb[1], bb[2], bb[3])
            if selection != None:
                    # The 3 main colors in the area selected are considered.
            # Note that the selection should be included in the target and not contain background
              try:
                selection.save('../ObjectTracking/'+ 'kite_detail_tmp.jpg')
                img0 = Image("kite_detail_tmp.jpg") # For unknown reason I have to reload the image...
                pal = img0.getPalette(bins = 2, hue = False)
              except: # getPalette is sometimes bugging and raising LinalgError because matrix not positive definite
                pal = pal
              wasTargetFoundInPreviousFrame = False
              previous_coord_px = (bb[0] + bb[2]/2, bb[1] + bb[3]/2)
          if corner1 != corner2:
            selectionLayer.rectangle((bb[0], bb[1]), (bb[2], bb[3]), width = 5, color = Color.YELLOW)
                       
   
      # If the target was already found, we can save computation time by
      # reducing the Region Of Interest around predicted position
      if wasTargetFoundInPreviousFrame:
        ROITopLeftCorner = (max(0, previous_coord_px[0]-maxRelativeMotionPerFrame/2*width), \
                  max(0, previous_coord_px[1] -height*maxRelativeMotionPerFrame/2))
        ROI = img.crop(ROITopLeftCorner[0], ROITopLeftCorner[1],                          \
                             maxRelativeMotionPerFrame*width, maxRelativeMotionPerFrame*height, \
                 centered = False)
        if display :
      # Draw the rectangle corresponding to the ROI on the complete image
          layer.rectangle((previous_coord_px[0]-maxRelativeMotionPerFrame/2*width,  \
                                   previous_coord_px[1]-maxRelativeMotionPerFrame/2*height), \
                                (maxRelativeMotionPerFrame*width, maxRelativeMotionPerFrame*height), \
                 color = Color.GREEN, width = 2)
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
    
          # Find sky color
      sky = (img-img.binarize()).findBlobs(minsize=10000)
      if sky:
       skycolor = sky[0].meanColor()
      # Option 3
      target_img = ROI-ROI # Black image
              
      # Loop through palette of target colors
      if display and displayDebug:
            decomposition = []
      i_col = 0
      for col in pal: 
        c = tuple([int(col[i]) for i in range(0,3)])
            # Search the target based on color
        ROI.save('../ObjectTracking/'+ 'ROI_tmp.jpg')
        img1 = Image('../ObjectTracking/'+ 'ROI_tmp.jpg')
        filter_img = img1.colorDistance(color = c)
        h = filter_img.histogram(numbins=256)
        cs = np.cumsum(h)
        thmax = np.argmin(abs(cs- 0.02*img.width*img.height)) # find the threshold to have 10% of the pixel in the expected color
        thmin = np.argmin(abs(cs- 0.005*img.width*img.height)) # find the threshold to have 10% of the pixel in the expected color
        if thmin==thmax:
          newth = thmin
        else:
          newth = np.argmin(h[thmin:thmax]) + thmin
        alpha = 0.5
        th[i_col] = alpha*th[i_col]+(1-alpha)*newth
        filter_img = filter_img.threshold(max(40,min(200,th[i_col]))).invert()
        target_img = target_img + filter_img
        #print th
        i_col = i_col + 1
        if display and displayDebug:
          [R, G, B] = filter_img.splitChannels()
          white = (R-R).invert()
          r = R*1.0/255*c[0]
          g = G*1.0/255*c[1]
          b = B*1.0/255*c[2]
          tmp = white.mergeChannels(r, g, b)
          decomposition.append(tmp)

      # Get a black background with with white target foreground
      target_img = target_img.threshold(150)
  
      target_img = target_img - ROI.colorDistance(color = skycolor).threshold(80).invert()

      if display and displayDebug:
        small_ini = target_img.resize(int(img.width/(len(pal)+1)),  int(img.height/(len(pal)+1)))
        for tmp in decomposition:
          small_ini = small_ini.sideBySide(tmp.resize(int(img.width/(len(pal)+1)), int(img.height/(len(pal)+1))), side = 'bottom')
        small_ini = small_ini.adaptiveScale((int(img.width), int(img.height)))
        toDisplay = img.sideBySide(small_ini)
      else:
        toDisplay = img
          #target_img = ROI.hueDistance(color = Color.RED).threshold(10).invert()

      # Search for binary large objects representing potential target
      target = target_img.findBlobs(minsize = 500)
      
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
        rot_coord_px = np.dot(ctm, coord_px - np.array([img.width/2, img.height/2])) + np.array([img.width/2, img.height/2])
        if useBasemap:
          coord = sp.deg2rad(m(rot_coord_px[0], img.height-rot_coord_px[1], inverse = True))
        else:
          coord = localProjection(rot_coord_px[0]-img.width/2, img.height/2-rot_coord_px[1], radius, mobile.yaw, mobile.pitch, inverse = True)
        target_bearing, target_elevation = coord

      # Get minimum bounding rectangle for display purpose
        minR = ROITopLeftCorner + np.array(target[0].minRect())

        contours = target[0].contour()

        contours = [ ROITopLeftCorner + np.array(contour) for contour in contours]


  
        # Get target features
        angle = sp.deg2rad(target[0].angle()) + mobile.roll
        angle =  sp.deg2rad(unwrap180(sp.rad2deg(angle), sp.rad2deg(previous_angle)))
        width = target[0].width()
        height = target[0].height()

        # Check if the kite is upside down
        # First rotate the kite
        ctm2 = np.array([[sp.cos(-angle+mobile.roll), -sp.sin(-angle+mobile.roll)], \
            [sp.sin(-angle+mobile.roll), sp.cos(-angle+mobile.roll)]]) # Coordinate transform matrix
        rotated_contours = [np.dot(ctm2, contour-coordMinRect) for contour in contours]  
        y = [-tmp[1] for tmp in rotated_contours]
        itop = np.argmax(y) # Then looks at the points at the top
        ibottom = np.argmin(y) # and the point at the bottom
        # The point the most excentered is at the bottom
        if abs(rotated_contours[itop][0])>abs(rotated_contours[ibottom][0]):
          isInverted = True
        else:
          isInverted = False    
        
        if isInverted:
            angle = angle + sp.pi    

        
                # Filter the data
        alpha = 1-sp.exp(-deltaT/self.filterTimeConstant)
        if not(isPaused):
          dCoord = np.array(previous_dCoord)*(1-alpha) + alpha*(np.array(coord_px) - previous_coord_px) # related to the speed only if cam is fixed
          dAngle = np.array(previous_dAngle)*(1-alpha) + alpha*(np.array(angle) - previous_angle)
        else : 
          dCoord = np.array([0, 0])
          dAngle = np.array([0]) 
#print coord_px, angle, width, height, dCoord
    
        # Record important data
        times.append(timestamp)
        coords_px.append(coord_px)
        angles.append(angle)
        target_elevations.append(target_elevation)
        target_bearings.append(target_bearing)
        
        # Export data to controller
        self.elevation = target_elevation
        self.bearing = target_bearing
        self.orientation = angle
        dt = time.time()-timeLastTarget
        self.ROT = dAngle/dt
        self.lastUpdateTime = t
        
        # Save for initialisation of next step
        previous_dCoord = dCoord
        previous_angle = angle
        previous_coord_px = (int(coord_px[0]), int(coord_px[1]))
        wasTargetFoundInPreviousFrame = True
        timeLastTarget = time.time()
      
      else:
        wasTargetFoundInPreviousFrame = False
        
      if useHDF5:
        hdfSize = hdfSize+1
        dset.resize((hdfSize, 7))
        imset.resize((hdfSize, img.width, img.height, 3))
        dset[hdfSize-1,:] = [time.time(), coord_px[0], coord_px[1], angle, self.elevation, self.bearing, self.ROT]
        imset[hdfSize-1,:,:,:] = img.getNumpy()
        recordFile.flush()
      else:
        csv_writer.writerow([time.time(), coord_px[0], coord_px[1], angle, self.elevation, self.bearing, self.ROT])



      if display :
        if target:
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
            
      # Add the layer to the raw image 
        toDisplay.addDrawingLayer(layer)
        toDisplay.addDrawingLayer(selectionLayer)

      # Add time metadata
        toDisplay.drawText(str(i_frame)+" "+ str(timestamp), x=0, y=0, fontsize=20)

      # Add Line giving horizon
          #layer.line((0, int(img.height/2 + mobile.pitch*pixelPerRadians)),(img.width, int(img.height/2 + mobile.pitch*pixelPerRadians)), width = 3, color = Color.RED)

      # Plot parallels
        for lat in range(-90, 90, 15):
          r = range(0, 361, 10)
          if useBasemap:
            # \todo improve for high roll
            l = m (r, [lat]*len(r))
            pix = [np.array(l[0]), img.height-np.array(l[1])]
          else:
            l = localProjection(sp.deg2rad(r), \
                    sp.deg2rad([lat]*len(r)), \
                    radius, \
                    lon_0 = mobile.yaw, \
                    lat_0 = mobile.pitch, \
                    inverse = False)
            l = np.dot(ctm, l)
            pix = [np.array(l[0])+img.width/2, img.height/2-np.array(l[1])]

          for i in range(len(r)-1):
            if isPixelInImage((pix[0][i],pix[1][i]), img) or isPixelInImage((pix[0][i+1],pix[1][i+1]), img):
              layer.line((pix[0][i],pix[1][i]), (pix[0][i+1], pix[1][i+1]), color=Color.WHITE, width = 2)

      # Plot meridians
        for lon in range(0, 360, 15):
          r = range(-90, 91, 10)
          if useBasemap:
        # \todo improve for high roll
            l = m ([lon]*len(r), r)
            pix = [np.array(l[0]), img.height-np.array(l[1])]
          else:
            l= localProjection(sp.deg2rad([lon]*len(r)), \
                    sp.deg2rad(r), \
                    radius, \
                    lon_0 = mobile.yaw, \
                    lat_0 = mobile.pitch, \
                    inverse = False)
            l = np.dot(ctm, l)
            pix = [np.array(l[0])+img.width/2, img.height/2-np.array(l[1])]

          for i in range(len(r)-1):
            if isPixelInImage((pix[0][i],pix[1][i]), img) or isPixelInImage((pix[0][i+1],pix[1][i+1]), img):
              layer.line((pix[0][i],pix[1][i]), (pix[0][i+1], pix[1][i+1]), color=Color.WHITE, width = 2)

      # Text giving bearing
      # \todo improve for high roll
        for bearing_deg in range(0, 360, 30):
          l = localProjection(sp.deg2rad(bearing_deg), sp.deg2rad(0), radius, lon_0 = mobile.yaw, lat_0 = mobile.pitch, inverse = False)
          l = np.dot(ctm, l)
          layer.text(str(bearing_deg), ( img.width/2+int(l[0]), img.height-20), color = Color.RED)

      # Text giving elevation
      # \todo improve for high roll
        for elevation_deg in range(-60, 91, 30):
          l = localProjection(0, sp.deg2rad(elevation_deg), radius, lon_0 = mobile.yaw, lat_0 = mobile.pitch, inverse = False)
          l = np.dot(ctm, l)
          layer.text(str(elevation_deg), ( img.width/2 ,img.height/2-int(l[1])), color = Color.RED)

        #toDisplay.save(js)
        toDisplay.save(disp)
    if display : 
      toDisplay.removeDrawingLayer(1)
      toDisplay.removeDrawingLayer(0)
  recordFile.close()


if __name__ == '__main__':
  import serial
  kite = Kite()
  a = threading.Thread(None, Kite.track, None, (kite,))
  print 'here'
  a.start()
#
        
