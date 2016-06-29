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
clients = []

class MainHandler(tornado.web.RequestHandler):
    def get(self):
        self.render("webgl_IMU.html")
 
class WebSocketHandler(tornado.websocket.WebSocketHandler):
    def on_message(self, message):  
      print "message received"     

    def open(self):
      self.vote = 50
      clients.append(self)
      self.write_message(u"Connected")
      print "open"
      
    def on_close(self):
      clients.remove(self)
      print "close"
	
    def check_origin(self, origin):
      return True

handlers = [
    (r"/", MainHandler),
    (r"/websocket", WebSocketHandler),
]
settings = dict(
            static_path=os.path.join(os.path.dirname(__file__), "static"),
)
application = tornado.web.Application(handlers, **settings)

def timer():
    for c in clients:
        #c.write_message(datetime.datetime.utcnow().strftime("%Y%m%d_%Hh%Mm_%Ss"))
        t = time.time()
        c.write_message( json.dumps({'x':30*math.sin(t), 'y':30*math.sin(t), 'z':30*math.sin(t), 'xrotation':math.sin(t), 'yrotation':math.cos(t), 'zrotation':0})
)
 
if __name__ == "__main__":
    application.listen(8080)
    mainLoop = tornado.ioloop.IOLoop.instance()
    scheduler = tornado.ioloop.PeriodicCallback(timer, 100, io_loop = mainLoop)
    scheduler.start()
    mainLoop.start()
    

