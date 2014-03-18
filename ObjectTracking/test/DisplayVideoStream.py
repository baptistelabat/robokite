#!/usr/bin/python
# coding=utf-8
import cv2
import cv2.cv as cv
import datetime

cv2.namedWindow("preview")

vc = cv2.VideoCapture("/media/bat/DATA/images/2013/06/27/00176.MTS")
width, height = 640, 480
writer = cv2.VideoWriter(filename="outputVideo.avi",
fourcc=cv.CV_FOURCC('M','J', 'P', 'G'),
fps=15,
frameSize=(width, height))                    

if vc.isOpened(): # try to get the first frame
    rval, frame = vc.read()
else:
    rval = False

while rval:
    cv2.imshow("preview", frame)
   
    rval, frame = vc.read()
    im=cv.fromarray(frame)
    im=cv.Flip(im, flipMode=-1)
    cv.ShowImage('180_rotation', im)
    key = cv2.waitKey(20)
    key -= 0x100000 # Corrects bug in openCV...
    if key==27: # Esc key to stop
        break
    elif key==115:  # s key for snapshot
        cv2.imwrite(datetime.datetime.utcnow().strftime("%Yy%mm%dd%Hh%Mm%Ss")+'.jpg', frame)
    writer.write(frame)
cv2.destroyAllWindows()
