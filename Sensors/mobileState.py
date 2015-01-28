from math import atan2, sin, cos
import scipy as sp
import socket, traceback
import threading
import time
import esoq2p1
def decimalstr2float(decimalstrs):
    """ This function converts a string to float (probably exists elsewhere)"""
    decimal = []
    if type(decimalstrs) == str: 
        decimal = float(decimalstrs.replace(',', '.'))
    else : 
        for dec in decimalstrs:
            decimal.append(decimalstr2float(dec))
    return decimal 

class mobileState:
  """This class stores and process information about mobile phone"""
  def __init__(self):
    self.acceleration_raw = [0, 0, 0]
    self.acceleration_filtered = sp.array([0.0, 0.0, 0.0])
    self.magnetic_raw = [0, 0, 0]
    self.magnetic_filtered = sp.array([0.0, 0.0, 0.0])
    self.isToUpdate = False
    self.roll = 0
    self.pitch = 0
    self.yaw = 0
    self.filterTimeConstant = 0.3
    self.time_acceleration = 0
    self.time_magnetic = 0
    self.stop_requested = False

  def computeRPY(self, order=[0, 2, 1], sign=[1, 1, 1]):
    """Computes roll, pitch and yaw. See "Implementing a Tilt-Compensated
    eCompass using Accelerometer and
    Magnetometer Sensors" reference AN4248 from Freescale""" 
    # 0 2 1
    Gpx = -self.acceleration_filtered[order[0]]*sign[0]
    Gpy = -self.acceleration_filtered[order[1]]*sign[1]
    Gpz = self.acceleration_filtered[order[2]]*sign[2]
    Bpx = -self.magnetic_filtered[order[0]]*sign[0]
    Bpy = -self.magnetic_filtered[order[1]]*sign[1]
    Bpz = self.magnetic_filtered[order[2]]*sign[2]
    # These parameters could be used to compensate for hard iron effect, if not already compensated
    Vx = 0
    Vy = 0
    Vz = 0
    phi = atan2(Gpy, Gpz) # According to Eqn 13
    theta = atan2(-Gpx, Gpy*sin(phi)+Gpz*cos(phi)) # According to Eqn 15
    psi = atan2((Bpz-Vz)*sin(phi)-(Bpy-Vy)*cos(phi), (Bpx-Vx)*cos(theta)+(Bpy-Vy)*sin(theta)*sin(phi)+(Bpz-Vz)*sin(theta)*cos(phi))   #According to Eqn 22
    self.roll = phi
    self.pitch = theta
    self.yaw = psi # Sign changed to get correct output \todo find the first bug
    [q, loss] = esoq2p1.esoq2p1( sp.array([[Gpx, Bpx], [Gpy, Bpy], [Gpz, Bpz]]), sp.array([[0, -17], [0, 0], [-9.8, -42]]), sp.array([1,1]))
    #print q
    self.q = q
    self.isToUpdate = False

  def decodeMessageSensorUDP(self, msg):
    """ This is used to decode message from sensorUDP application from the android market.
    The orientation field was first used, but its conventions were unclear.
    So now acceleration and magnetic vectors should be used"""
    data = msg.split(', ')
    if data[0]=='G':
        # This is GPS message
        time = decimalstr2float(data[2])
        latitude_deg = decimalstr2float(data[3])
        longitude_deg = decimalstr2float(data[4])
        altitude = decimalstr2float(data[5])
        hdop = decimalstr2float(data[7]) # Horizontal dilution of precision
        vdop = decimalstr2float(data[8]) # Vertical dilution of precision
        print time, latitude_deg, longitude_deg, altitude, hdop, vdop
    if data[0]=='O':
        # \note This is no more used as orientation convention were unclear
        #  'O, 146, 1366575961732, 230,1182404, -075,2031250, 001,7968750'
        [ u, u,    # data not used                                         \    
        heading_deg, # pointing direction of top of phone                    \ 
        roll_deg,    # around horizontal axis, positive clockwise [-180:180] \   
        pitch_deg] = decimalstr2float(data[1:])  # around vertical axis [_90:90]
        elevation_deg = -sp.rad2deg(sp.arctan2(                     \
            sp.cos(sp.deg2rad(pitch_deg))*sp.cos(sp.deg2rad(roll_deg)),     \
            sp.sqrt(1+sp.cos(sp.deg2rad(roll_deg))**2*(sp.sin(sp.deg2rad(pitch_deg))**2-1)))) #positive up
        inclinaison_deg = pitch_deg #positive clockwise
        print heading_deg, roll_deg, pitch_deg, elevation_deg, inclinaison_deg
    if data[0] == 'A':
    # Accelerometer data
    # Index and sign are adjusted to obtain x through the screen, and z down
        deltaT = decimalstr2float(data[2])/1000 - self.time_acceleration
        if self.filterTimeConstant == 0.0:
          alpha = 1
        else:
          alpha = 1-sp.exp(-deltaT/self.filterTimeConstant)
          
        self.time_acceleration = decimalstr2float(data[2])/1000
        self.acceleration_raw[0] = decimalstr2float(data[3])
        self.acceleration_raw[1] = decimalstr2float(data[4])
        self.acceleration_raw[2] = decimalstr2float(data[5])
        # Filter the data
        self.acceleration_filtered +=alpha*(sp.array(self.acceleration_raw)-self.acceleration_filtered)
    if data[0] == 'M':
    # Magnetometer data
    # Index and sign are adjusted to obtain x through the screen, and z down
        deltaT =  decimalstr2float(data[2])/1000-self.time_magnetic
        if self.filterTimeConstant == 0.0:
          alpha = 1
        else:
          alpha = 1-sp.exp(-deltaT/self.filterTimeConstant)
          
        self.time_magnetic = decimalstr2float(data[2])/1000
        self.magnetic_raw[0] = decimalstr2float(data[3])
        self.magnetic_raw[1] = decimalstr2float(data[4])
        self.magnetic_raw[2] = -decimalstr2float(data[5])# Adapt to a bug in sensorUDP?
        # Filter the data
        self.magnetic_filtered += alpha*(sp.array(self.magnetic_raw)-self.magnetic_filtered)

  def checkUpdate(self):
    while not(self.stop_requested):
        try:
          message, address = self.socket.recvfrom(9000)
          self.decodeMessageSensorUDP(message)
          self.isToUpdate = True
        except (KeyboardInterrupt, SystemExit):
          raise
          self.close()
        except:
          traceback.print_exc()
          
  def open(self):
    host = ''
    port = 12345
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    s.bind((host, port))
    self.socket = s
    print("Socket opened")
    a = threading.Thread(None, self.checkUpdate, None, ())
    a.start()
    
  def close(self):
    self.stop_requested = True
    self.socket.close()
    

if __name__ == '__main__':
  max_length = 0
  timeList = []
  rollList = []
  pitchList = []
  headingList = []
  fileout = open('acc.txt', 'w')
  filemag = open('mag.txt', 'w')
  import matplotlib.pyplot as plt
  import numpy as np
  import time
  t0 = time.time()
  mobile = mobileState()
  mobile.open()
  try:
    while time.time()-t0 < 60:
        mobile.computeRPY()
        print mobile.acceleration_raw
        print mobile.roll
        timeList.append(time.time())
        headingList.append(mobile.yaw)
        rollList.append(mobile.roll)
        pitchList.append(mobile.pitch)
        time.sleep(0.1)
        fileout.write('%f %f %f\n' % (mobile.acceleration_raw[0], mobile.acceleration_raw[1], mobile.acceleration_raw[2]))
        filemag.write('%f %f %f\n' % (mobile.magnetic_raw[0], mobile.magnetic_raw[1], mobile.magnetic_raw[2]))
  except KeyboardInterrupt:
    mobile.close()
    fileout.close()
    filemag.close()
    
  plt.hold()
  plt.plot(np.array(timeList)-timeList[0], np.array(headingList))
  plt.plot(np.array(timeList)-timeList[0], np.array(rollList), 'g')
  plt.plot(np.array(timeList)-timeList[0], np.array(pitchList), 'r')

  plt.show()
  fileout.close()
  filemag.close()
  mobile.close()

