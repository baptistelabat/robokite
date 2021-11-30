# -------------------------------------------------------
import socket, traceback
import time
import json

host = ''
port = 2390

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
s.bind((host, port))

filein = open('saveUDP.txt', 'w')
t0 = time.time()
while time.time()-t0 < 200:
    try:
        message, address = s.recvfrom(4096)
        print(message)
        json.loads(message.decode("utf-8"))
        filein.write('%s\n' % (message))
    except (KeyboardInterrupt, SystemExit):
       raise
    except:
       traceback.print_exc()
filein.close()
# -------------------------------------------------------
