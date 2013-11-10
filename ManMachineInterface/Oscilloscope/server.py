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
from pyfirmata import Arduino, util
clients = []
global t0
global board
t0 = time.time()

def openSerial():
  global board
  
  # Loop over varying serial port till you find one (assume you have only one device connected)
  locations = ['/dev/ttyACM0','/dev/ttyACM1','/dev/ttyACM2','/dev/ttyACM3','/dev/ttyACM4','/dev/ttyACM5','/dev/ttyUSB0','/dev/ttyUSB1','/dev/ttyUSB2','/dev/ttyUSB3','/dev/ttyS0','/dev/ttyS1','/dev/ttyS2','/dev/ttyS3']
  for device in locations:
    if True:
      print "Trying...",device
      board = Arduino(device)
      print "Connected on ", device
      it= util.Iterator(board)
      it.start()
      board.analog[0].enable_reporting()
      break
    #except:
    #  print "Failed to connect on ", device
  
def checkSerial():
    global board
    value = board.analog[0].read()
    global t0
    for c in clients:
        t = time.time()-t0
        c.write_message( json.dumps({'x':t, 'y':value}))
        
class MainHandler(tornado.web.RequestHandler):
    def get(self):
        self.render("graph.html")
 
class WebSocketHandler(tornado.websocket.WebSocketHandler):
    def on_message(self, message):  
      print "message received"     

    def open(self):
      clients.append(self)
      self.write_message(u"Connected")
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
    global t0
    for c in clients:
        #c.write_message(datetime.datetime.utcnow().strftime("%Y%m%d_%Hh%Mm_%Ss"))
        t = time.time()-t0
        c.write_message( json.dumps({'x':t, 'y':500+500*math.sin(t)})
)
 
if __name__ == "__main__":
    openSerial()
    application.listen(8080)
    mainLoop = tornado.ioloop.IOLoop.instance()
    #scheduler = tornado.ioloop.PeriodicCallback(timer, 100, io_loop = mainLoop)
    scheduler = tornado.ioloop.PeriodicCallback(checkSerial, 1, io_loop = mainLoop)
    scheduler.start()
    mainLoop.start()
    

