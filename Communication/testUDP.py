# -------------------------------------------------------
import socket, traceback
import time

host = ''
port = 12345

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
s.bind((host, port))

filein = open('saveUDP.txt', 'w')
t0 = time.time()
while time.time()-t0 < 20:
    try:
        message, address = s.recvfrom(9000)
	print message
	filein.write('%s\n' % (message))
    except (KeyboardInterrupt, SystemExit):
       raise
    except:
	traceback.print_exc()
filein.close()
# -------------------------------------------------------
