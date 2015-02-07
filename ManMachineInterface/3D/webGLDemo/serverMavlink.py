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
global master
t0 = time.time()
import serial
from pymavlink import mavutil

def openSerial():
  global master
  
  # Loop over varying serial port till you find one (assume you have only one device connected)
  locations = ['/dev/ttyACM0','/dev/ttyACM1','/dev/ttyACM2','/dev/ttyACM3','/dev/ttyACM4','/dev/ttyACM5','/dev/ttyUSB0','/dev/ttyUSB1','/dev/ttyUSB2','/dev/ttyUSB3','/dev/ttyS0','/dev/ttyS1','/dev/ttyS2','/dev/ttyS3']
  for device in locations:
    try:
      print "Trying...",device
      master = mavutil.mavlink_connection(device, baud=57600, source_system=255) # 255 is ground station
      print "Connected on ", device
      break
    except:
      print "Failed to connect on ", device
      

def checkSerial():
    global master
    global serialPending
    global t0
    t = time.time()-t0 
    try:
		msg = master.recv_match(type='HEARTBEAT', blocking=True)
		print(msg)
		msg = master.recv_match(type='ATTITUDE', blocking=False)
		print(msg)
    except Exception, e:
        print("Error reading from serial port" + str(e))
        openSerial()
    if msg is not None:
      for c in clients:
        c.write_message( json.dumps({'t':t, 'roll':msg.roll, 'pitch':msg.pitch, 'yaw':msg.yaw, 'coordinates': 'EulerAngles'}))


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
 
if __name__ == "__main__":

  application.listen(8080)
  mainLoop = tornado.ioloop.IOLoop.instance()
  scheduler = tornado.ioloop.PeriodicCallback(checkSerial, 1, io_loop = mainLoop)
  scheduler.start()
  mainLoop.start()

    

