#!/usr/bin/python2
#http://www.linuxforu.com/2012/04/getting-started-with-html5-websockets/
 
import tornado.web
import tornado.websocket
import tornado.ioloop
import datetime
import os
import serial
import numpy as np
clients = []
 
class MainHandler(tornado.web.RequestHandler):
    def get(self):
        self.render("ui.html")
 
class WebSocketHandler(tornado.websocket.WebSocketHandler):
    def on_message(self, message):
        self.write_message(u"Status OK " + message)
        global ser
	ser.write(str(np.floor((float(message)-128)/128.*100)/100.0))
        print str(np.floor((float(message)-128)/128.)/100.0)

    def open(self):
 	global ser
	ser = serial.Serial('/dev/ttyACM0', 9600)
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
    

