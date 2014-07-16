#!/usr/bin/env python
# -*- coding: utf8 -*-
#
# Copyright (c) 2013 Baptiste LABAT
#
# Licensed under the MIT License,
# https://github.com/baptistelabat/robokite
# Authors: Baptiste LABAT
#
# Used http://www.linuxforu.com/2012/04/getting-started-with-html5-websockets/
 
import tornado.web
import tornado.websocket
import tornado.ioloop
import os
import datetime
import time
import json
import math
import sys
import threading
import serial
sys.path.append(os.getcwd())
clients = []
global t0
global ser
t0 = time.time()
import serial

def openSerial():
  global ser
  
  # Loop over varying serial port till you find one (assume you have only one device connected)
  locations = ['/dev/ttyACM0','/dev/ttyACM1','/dev/ttyACM2','/dev/ttyACM3','/dev/ttyACM4','/dev/ttyACM5','/dev/ttyUSB0','/dev/ttyUSB1','/dev/ttyUSB2','/dev/ttyUSB3','/dev/ttyS0','/dev/ttyS1','/dev/ttyS2','/dev/ttyS3']
  for device in locations:
    try:
      print "Trying...",device
      ser = serial.Serial(device, baudrate=115200, timeout=1)
      print "Connected on ", device
      time.sleep(1.5) # Arduino is reset when opening port so wait before communicating
      # An alternative would be to listen to a message from the arduino saying it is ready
      ser.write('i1') # i to start serial control, 1 is the minimum expecting message frequency (in house protocol)

      break
    except:
      print "Failed to connect on ", device
      

def checkSerial():
    global ser
    global serialPending
    global t0
    t = time.time()-t0 
    try:
        s = ser.readline()
        print "Received from arduino: " + s
    except Exception, e:
        print("Error reading from serial port" + str(e))
        openSerial()
        
        d = str(-512)   
        s = d+', ' + d + ', ' + d + ', ' +d
    if len(s):
      a = s.split(',') 
      print a     
      for c in clients:
        c.write_message( json.dumps({'x':t, 'q0':float(a[0]), 'q1':float(a[1]), 'q2':float(a[2]), 'q3':float(a[3])}))


class MainHandler(tornado.web.RequestHandler):
    def get(self):
        self.render("webgl_IMU.html")
 
class WebSocketHandler(tornado.websocket.WebSocketHandler):
    def on_message(self, message):  
      print "message received"     

    def open(self):
      self.vote = 50
      clients.append(self)
      #self.write_message(u"Connected")
      print "open"
      
    def on_close(self):
      clients.remove(self)
      print "close"

handlers = [
    (r"/", MainHandler),
    (r"/websocket", WebSocketHandler),
]
settings = dict(
            static_path=os.path.join(os.path.dirname(__file__), "static"),
)
application = tornado.web.Application(handlers, **settings)

def timer():
    global mobile
    for c in clients:
        #c.write_message(datetime.datetime.utcnow().strftime("%Y%m%d_%Hh%Mm_%Ss"))
        t = time.time()
        mobile.computeRPY()
        #xrotation is through the screen by default, positive anticlockwise
        #yrotation is up by default, positive anticlockwise
        #z-axis is to the right
        msg =  json.dumps({'x':0, 'y':0, 'z':0,\
         'xrotation':mobile.roll, 'yrotation':mobile.pitch, 'zrotation':mobile.yaw,\
          'q0':mobile.q[0], 'q1':mobile.q[1], 'q2':mobile.q[2], 'q3':mobile.q[3],\
           'accx':mobile.acceleration_filtered[0], 'accy':mobile.acceleration_filtered[1], 'accz':mobile.acceleration_filtered[2],\
           'magx':mobile.magnetic_filtered[0], 'magy':mobile.magnetic_filtered[1], 'magz':mobile.magnetic_filtered[2]\
           })
        #print msg
        c.write_message(msg)
 
if __name__ == "__main__":

  application.listen(8080)
  mainLoop = tornado.ioloop.IOLoop.instance()
  scheduler = tornado.ioloop.PeriodicCallback(checkSerial, 1, io_loop = mainLoop)
  scheduler.start()
  mainLoop.start()

    

