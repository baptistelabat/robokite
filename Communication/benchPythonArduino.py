import time
import os
import sys
import re

def guess_port():
    """
    Tries to guess port from system platform.
    Taken from https://github.com/tino/pyFirmata
    Inspired form https://github.com/rwldrn/johnny-five
    """
    ports = os.listdir("/dev")
    prefix = "cu" if sys.platform == "darwin" else "tty"
    #pattern = '^%s\.(usb|acm|ACM).+$' % (prefix)
    pattern = '%s(usb|acm|ACM)' % (prefix)
    port = ""
    for p in ports:
        if re.match(pattern, p):
            port = p
            break
    return "/dev/%s" % port

firmata = 0 # https://github.com/lupeke/python-firmata/
pyfirmata = 1 # https://github.com/tino/pyFirmata
protoApiV2 = 2 # https://github.com/vascop/Python-Arduino-Proto-API-v2
pythonArduinoCommandAPI = 3 # https://github.com/thearn/Python-Arduino-Command-API 

pin = 0
nlib = int(sys.argv[1])
print nlib
# Load library to be tested
if nlib == firmata:
  from firmata import Arduino, OUTPUT
elif nlib == pyfirmata:
  from pyfirmata import util, Arduino
elif nlib == protoApiV2:
  sys.path.append('/home/bat/Python-Arduino-Proto-API-v2/arduino')
  from arduino import Arduino
elif nlib == pythonArduinoCommandAPI:
  from Arduino import Arduino


# Open serial connection
device = guess_port()	
try:
      print "Trying...", device
      if (nlib==firmata) or (nlib==pyfirmata) or (nlib==protoApiV2):
        board = Arduino(device)
        print "Connected on ", device
       # break
      elif pythonArduinoCommandAPI:
        board = Arduino('9600', port=device)
        print "Connected on ", device
        #break
except:
      print "Failed to connect on ", device
 
 
# Initialize pin
print "Initialising pin"
if nlib == firmata:
  board.pin_mode(pin, OUTPUT)
elif nlib == pyfirmata:
    it= util.Iterator(board)
    it.start()
    board.analog[pin].enable_reporting()
elif nlib == protoApiV2:
  board.output([])
elif pythonArduinoCommandAPI:
    board.pinMode(pin, "INPUT")
  
#  Loop and read data
t0 = time.time()
n = 0
t = time.time()
print "Looping over data"
while t-t0<10:  
  if nlib == firmata:
    board.parse()
    value = board.analog_read(pin)
  elif nlib == pyfirmata:
    value = board.analog[pin].read()
  elif nlib == protoApiV2:
    value = board.analogRead(pin) # Reading from analog pin #0
  elif nlib == pythonArduinoCommandAPI:
    value = board.analogRead(pin)
  n = n+1
  print "n: ",n, value
  t = time.time()
 
# Closing connection properly
if nlib == firmata:
  print "No close"
elif nlib == pyfirmata:
  board.exit()
elif nlib == protoApiV2:
  board.close()

