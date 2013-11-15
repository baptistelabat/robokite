import time
locations = ['/dev/ttyACM0','/dev/ttyACM1','/dev/ttyACM2','/dev/ttyACM3','/dev/ttyACM4','/dev/ttyACM5','/dev/ttyUSB0','/dev/ttyUSB1','/dev/ttyUSB2','/dev/ttyUSB3','/dev/ttyS0','/dev/ttyS1','/dev/ttyS2','/dev/ttyS3']

firmata = 0 # https://github.com/lupeke/python-firmata/
pyfirmata = 1 # https://github.com/tino/pyFirmata
protoApiV2 = 2 # https://github.com/vascop/Python-Arduino-Proto-API-v2
pythonArduinoCommandAPI = 3 # https://github.com/thearn/Python-Arduino-Command-API 

pin = 0
nlib = 0

# Load library to be tested
if nlib == firmata:
  from firmata import Arduino, OUTPUT
elif nlib == pyfirmata:
  from pyfirmata import util, Arduino
elif nlib == protoApiV2:
  import sys
  sys.path.append('/home/bat/Python-Arduino-Proto-API-v2/arduino')
  from arduino import Arduino
elif nlib == pythonArduinoCommandAPI:
  from Arduino import Arduino


# Open serial connection		
for device in locations:
   try:
      print "Trying...", device
      if (nlib==firmata) or (nlib==pyfirmata) or (nlib==protoApiV2):
        board = Arduino(device)
        print "Connected on ", device
        break
      elif pythonArduinoCommandAPI:
        board = Arduino('9600', port=device)
        print "Connected on ", device
        break
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

		

