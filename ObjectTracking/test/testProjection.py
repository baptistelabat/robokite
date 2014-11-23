import scipy as sp
import time
from mpl_toolkits.basemap import Basemap
from SimpleCV import Display, Image, Color, JpegStreamCamera
import numpy as np
import matplotlib.pyplot as plt
import os
import sys
sys.path.append(os.getcwd())
sys.path.append('../../Sensors')
import mobileState
import threading

def isPixelInImage((x,y), image):
    return (x>0 and x<image.width and y>0 and y<image.height)

width = 640
lon_0 = 270
lat_0 = 80
pixelPerRadians = 640
height = 480
radius = pixelPerRadians

max_length = 0
cam = JpegStreamCamera('http://192.168.1.3:8080/videofeed')#640 * 480
mobile = mobileState.mobileState()
mobile.open()
disp = Display()


while disp.isNotDone():

    if mobile.isToUpdate:
        mobile.computeRPY()
    print mobile.roll
    image = cam.getImage().rotate(-sp.rad2deg(mobile.roll), fixed = False)
    m = Basemap(width=image.width, height=image.height, projection='aeqd',
    lat_0 = sp.rad2deg(mobile.pitch), lon_0=sp.rad2deg(mobile.yaw), rsphere = radius)
    # fill background.
    #m.drawmapboundary(fill_color='aqua')
    # draw coasts and fill continents.
    #m.drawcoastlines(linewidth=0.5)
    #m.fillcontinents(color='coral',lake_color='aqua')
    # 20 degree graticule.
    # m.drawparallels(np.arange(-90,90,30))
    #m.drawmeridians(np.arange(-180,180,30))
    # draw a black dot at the center.
    #xpt, ypt = m(heading_deg, elevation_deg)
    #m.plot([xpt],[ypt],'ko')
    # draw the title.
    #plt.title('Azimuthal Equidistant Projection')
    #plt.show()

    topLeftCorner_deg = m(-image.width/2, image.height/2, inverse = True)
    topRightCorner_deg = m(image.width/2, image.height/2, inverse = True)
    bottomLeftCorner_deg = m(-image.width/2, -image.height/2, inverse = True)
    bottomRightCorner_deg = m(image.width/2, -image.height/2, inverse = True)
    m2 = Basemap(width=640*2,height=480*2,projection='aeqd',
    lat_0=0,lon_0=0, rsphere = radius)
    
    logo = Image('logo')
    black = (logo-logo).resize(640*2,480*2)
    [x,y] = m2(sp.rad2deg(mobile.yaw), sp.rad2deg(mobile.pitch))
    pos = (min(black.width-1, max(-black.width+1, int(x+image.width/2))), min(black.height-1, max(-black.height+1, black.height-int(y+image.height/2))))
    print pos
    print black.size()
    print image.size()
    black = black.blit(image, pos)

    # plot parallels
    for lat in range(-90,90,15):
      r = range(0,361,10)
      l = m2 (r,[lat]*len(r))
      pix = [np.array(l[0]), black.height-np.array(l[1])]

      for i in range(len(r)-1):
        if isPixelInImage((pix[0][i],pix[1][i]), black) or isPixelInImage((pix[0][i+1],pix[1][i+1]), black):
            black.drawLine((pix[0][i],pix[1][i]), (pix[0][i+1], pix[1][i+1]), color=Color.RED)
    # plot meridians
    for lon in range(0,360,15):
      r = range(-90,91,10)
      l = m2 ([lon]*len(r),r)
      pix = [np.array(l[0]), black.height-np.array(l[1])]

      for i in range(len(r)-1):
        if isPixelInImage((pix[0][i],pix[1][i]), black) or isPixelInImage((pix[0][i+1],pix[1][i+1]), black):
            black.drawLine((pix[0][i],pix[1][i]), (pix[0][i+1], pix[1][i+1]), color=Color.RED)

    black = black.applyLayers()

    #image.crop(image.width/2-int(480/2/sp.sqrt(2)), image.height/2-int(480/2/sp.sqrt(2)), int(480/sp.sqrt(2)), int(480/sp.sqrt(2))).show()
    black.save(disp)
mobile.close()



