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
import ssl
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
        c.write_message( json.dumps({'x':0*math.sin(t), 'y':0*math.sin(t), 'z':0*math.sin(t), 'roll':math.sin(t), 'pitch':math.cos(t), 'yaw':0, 'coordinates':'EulerAngles'})
)
 
if __name__ == "__main__":
    data_dir = "C:/OpenSSL-Win32/bin"
    ssl_ctx = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
    ssl_ctx.load_cert_chain(os.path.join(data_dir, "cert.pem"),
                        os.path.join(data_dir, "key.pem"))
    application.listen(8080,ssl_options=ssl_ctx)
    mainLoop = tornado.ioloop.IOLoop.instance()
    scheduler = tornado.ioloop.PeriodicCallback(timer, 100, io_loop = mainLoop)
    scheduler.start()
    mainLoop.start()
    

