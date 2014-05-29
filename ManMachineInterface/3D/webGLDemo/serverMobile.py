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
sys.path.append(os.getcwd())
sys.path.append('../../../Sensors')
import mobileState
clients = []
global mobile

try:
  # Get the mobile orientation
  mobile = mobileState.mobileState()
  a = threading.Thread(None, mobileState.mobileState.checkUpdate, None, (mobile,))
  a.start()
except KeyboardInterrupt:
 mobile.stop_requested = True


class MainHandler(tornado.web.RequestHandler):
    def get(self):
        self.render("webgl_mobile.html")
 
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
    try:
      application.listen(8080)
      mainLoop = tornado.ioloop.IOLoop.instance()
      scheduler = tornado.ioloop.PeriodicCallback(timer, 100, io_loop = mainLoop)
      scheduler.start()
      mainLoop.start()
    except KeyboardInterrupt:
      mobile.stop_requested = True
    

