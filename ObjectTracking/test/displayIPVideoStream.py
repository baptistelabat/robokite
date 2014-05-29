#!/usr/bin/python
# coding=utf-8
# vlc -vvv "/media/bat/DATA/videos/PERSEPOLIS.avi" --sout '#transcode{vcodec=mjpg,vb=2500,width=640,height=480,acodec=none}:standard{access=http,mux=mpjpeg,dst=localhost:8080/videofeed}'

from SimpleCV import JpegStreamCamera, Display, Image
cam = JpegStreamCamera('http://192.168.1.3:8080/videofeed')
disp = Display()
while disp.isNotDone():
    img = cam.getImage()
    img.save(disp)
