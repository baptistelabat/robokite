#!/usr/bin/python2
#http://www.linuxforu.com/2012/04/getting-started-with-html5-websockets/
 
import tornado.web
import tornado.websocket
import tornado.ioloop
import datetime

clients = []
 
class MainHandler(tornado.web.RequestHandler):
    def get(self):
        self.render("ui.html")
 
class WebSocketHandler(tornado.websocket.WebSocketHandler):
    def on_message(self, message):
        self.write_message(u"Status OK " + message)
    def open(self):
        clients.append(self)
	self.write_message(u"Connected")
    def close(self):
	self.write_message(u"Connection closed")
 	clients.remove(self)

    def test(self):
        self.write_message(u"2")
        print "2"
 
application = tornado.web.Application([
    (r"/", MainHandler),
    (r"/websocket", WebSocketHandler),
])

def timer():
    for c in clients:
        c.write_message(datetime.datetime.utcnow().strftime("%Y%m%d_%Hh%Mm_%Ss"))

 
if __name__ == "__main__":
    application.listen(8888)
    mainLoop = tornado.ioloop.IOLoop.instance()
    scheduler = tornado.ioloop.PeriodicCallback(timer, 1000, io_loop = mainLoop)
    scheduler.start()
    mainLoop.start()
    

