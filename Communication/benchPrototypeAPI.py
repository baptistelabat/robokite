import sys
sys.path.append('/home/bat/Python-Arduino-Proto-API-v2/arduino')
import time
locations = ['/dev/ttyACM0','/dev/ttyACM1','/dev/ttyACM2','/dev/ttyACM3','/dev/ttyACM4','/dev/ttyACM5','/dev/ttyUSB0','/dev/ttyUSB1','/dev/ttyUSB2','/dev/ttyUSB3','/dev/ttyS0','/dev/ttyS1','/dev/ttyS2','/dev/ttyS3']

from firmata import Arduino, OUTPUT
for device in locations:
   try:
      print "Trying...",device
      board = Arduino(device)
      break
   except:
      print "Failed to connect on ", device
      

'''
pin = 0

#declare output pins as a list/tuple
board.output([pin])

t0 = time.time()
n = 0
t = time.time()
while t-t0<10:
    value = board.analogRead(pin)
    n = n+1
    print "n: ",n, value
    t = time.time()

b.close()

from pyfirmata import util, Arduino
t0 = time.time()
n = 0
t = time.time()
while t-t0<10:
  it= util.Iterator(board)
  it.start()
  board.analog[0].enable_reporting()
  value = board.analog[0].read()
  n = n+1
  print "n: ",n, value
  #board.pass_time(1)
  t = time.time()
  

board.exit()
'''

board.pin_mode(0, OUTPUT)

t0 = time.time()
n = 0
t = time.time()
while t-t0<10:
    board.parse()
    value = board.analog_read(0) # Reading from analog pin #0
    n = n+1
    print "n: ",n,  
