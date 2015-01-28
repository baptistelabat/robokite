#!/usr/bin/env python
# -*- coding: utf8 -*-
#
# Copyright (c) 2013 Nautilabs
#
# Licensed under the MIT License,
# https://github.com/baptistelabat/robokite
# Authors: Baptiste LABAT


from SimpleCV import Camera, Image, VirtualCamera, Display, DrawingLayer, Color, JpegStreamCamera, JpegStreamer
import scipy as sp
import numpy as np

cam = VirtualCamera('../Recording/Videos/Kite with leds in night-LgvpmMt-SA0.mp4','video')
img = cam.getImage()
disp = Display((810,1080))
display = True
predictedTargetPosition = (img.size()[0]/2, img.size()[1]/2)
while (not(display) or disp.isNotDone()) and img.size()!= (0, 0) :
    img = cam.getImage()
    if img.size()!= (0, 0):
     if img:
      if display: 
	    # Create a new layer to host information retrieved from video
	      layer = DrawingLayer((img.width, img.height))
      maskred = img.colorDistance(color=(200,50,70)).invert().threshold(170)
      imgred = (img*(maskred/255)).dilate(3)
      targetred=imgred.findBlobs(maxsize=200)
      maskwhite = img.colorDistance(color=(200,200,200)).invert().threshold(230)
      imgwhite = (img*(maskwhite/255)).dilate(3)
      targetwhite=imgwhite.findBlobs(maxsize=200)
      
      
      if targetred:
        targetred.draw()
        print targetred.meanColor()
        targetred = targetred.sortDistance(predictedTargetPosition)
        coord_px = np.array(targetred[0].centroid())
        layer.circle((int(coord_px[0]), int(coord_px[1])), 10, filled = False, color = Color.RED)
      if targetwhite:
        print "white ", targetwhite.meanColor()
        targetwhite = targetwhite.sortDistance(predictedTargetPosition)
        coord_px = np.array(targetwhite[0].centroid())
        layer.circle((int(coord_px[0]), int(coord_px[1])), 10, filled = False, color = Color.WHITE)
      img.addDrawingLayer(layer)
      img.save(disp)
