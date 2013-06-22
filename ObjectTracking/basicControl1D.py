from SimpleCV import *
import time
import serial
cam = JpegStreamCamera('http://192.168.1.6:8080/videofeed')
disp=Display()

"""This script was used for the demonstration of doing control with visual feedback
   A android mobile phone was used with ipcam application to stream the video
   A green fresbee was attached to a line rolled over the axis of the motor which was controlled"""

ser = serial.Serial('/dev/ttyACM2', 9600)
alpha = 0.8
time.sleep(1)
previous_z = 200;
while True:
    img = cam.getImage()
    myLayer = DrawingLayer((img.width,img.height))
    disk_img = img.hueDistance(color=Color.GREEN).invert().morphClose().morphClose().threshold(200)
    disk = disk_img.findBlobs(minsize=2000)
    if disk:
        disk[0].drawMinRect(layer=myLayer, color=Color.RED)
        disk_img.addDrawingLayer(myLayer)
        position = disk[0].centroid()
        print position
        z = alpha*position[1]+(1-alpha)*previous_z
        ser.write(str((z-200)*0.03))
        previous_z=z
    disk_img.save(disp)
    
    time.sleep(0.01)
