from pymavlink import mavutil

fileout = open('acc.txt', 'w')
filemag = open('mag.txt', 'w')
filegyro = open('gyro.txt', 'w')

# create a mavlink serial instance
master = mavutil.mavlink_connection('/dev/ttyUSB1', baud=57600, source_system=255)
try:
    while True:
        msg = master.recv_match(type='HEARTBEAT', blocking=True)
        msg = master.recv_match(type='HIGHRES_IMU', blocking=False)
        print msg
        if msg is not None:
            fileout.write('%f %f %f\n' % (msg.xacc, msg.yacc, msg.zacc))
            filemag.write('%f %f %f\n' % (msg.xmag, msg.ymag, msg.zmag))
            filegyro.write('%f %f %f\n' % (msg.xgyro, msg.ygyro, msg.zgyro))
except KeyboardInterrupt:
    fileout.close()
    filemag.close()
    filegyro.close()
