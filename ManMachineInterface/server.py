#!/usr/bin/env python
# -*- coding: utf8 -*-
#
# Copyright (c) 2013 Nautilabs
#
# Licensed under the MIT License,
# http://code.google.com/p/robokite/source/checkout
# Authors: Baptiste LABAT
#
# Used http://www.linuxforu.com/2012/04/getting-started-with-html5-websockets/
 
import tornado.web
import tornado.websocket
import tornado.ioloop
import datetime
import os
import serial
import numpy as np
import time
clients = []
 
class MainHandler(tornado.web.RequestHandler):
    def get(self):
        self.render("ui.html")
 
class WebSocketHandler(tornado.websocket.WebSocketHandler):
    def on_message(self, message):
        self.write_message(u"Status OK " + message)
        global ser
	try:
	  ser.write(str(np.floor((float(message)-128)/128.*100)/100.0))
        except serial.SerialTimeoutException:
          print "time out exception"
        print str(np.floor((float(message)-128)/128.*100)/100.0)

    def open(self):
 	global ser
	# Open the serial port
	locations=['/dev/ttyACM0','/dev/ttyACM1','/dev/ttyACM2','/dev/ttyUSB0','/dev/ttyUSB1','/dev/ttyUSB2','/dev/ttyUSB3','/dev/ttyS0','/dev/ttyS1','/dev/ttyS2','/dev/ttyS3']
	for device in locations:
  	  try:
    	    print "Trying...",device
            ser = serial.Serial(
	port = device,
	baudrate = 9600,
        timeout = 0,
        writeTimeout = 0.1
	)
 	    print "Connected on ", device
            break
  	  except:
    	    print "Failed to connect on ", device
        time.sleep(1.5) # Arduino is reset when opening port so wait before communicating
        ser.flush()
	ser.write('i') # i to start serial control
        clients.append(self)
	self.write_message(u"Connected")
        print "open"
    def close(self):
	self.write_message(u"Connection closed")
 	clients.remove(self)
        print "close"

    def test(self):
        self.write_message(u"2")
        print "2"

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
        c.write_message(datetime.datetime.utcnow().strftime("%Y%m%d_%Hh%Mm_%Ss"))

 
if __name__ == "__main__":
    application.listen(8080)
    mainLoop = tornado.ioloop.IOLoop.instance()
    #scheduler = tornado.ioloop.PeriodicCallback(timer, 1000, io_loop = mainLoop)
    #scheduler.start()
    mainLoop.start()
    

