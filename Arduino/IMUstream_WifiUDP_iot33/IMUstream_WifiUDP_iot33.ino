
/*
  WiFi UDP Send and Receive String

 This sketch wait an UDP packet on localPort using the WiFi module.
 When a packet is received an Acknowledge packet is sent to the client on port remotePort

 created 30 December 2012
 by dlf (Metodo2 srl)

 */


#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <math.h>
#include <strings.h>
#include <avr/dtostrf.h>

int status = WL_IDLE_STATUS;
#include "arduino_secrets.h" 
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;            // your network key Index number (needed only for WEP)

unsigned int localPort = 2390;      // local port to listen on

char packetBuffer[256]; //buffer to hold incoming packet
char  ReplyBuffer[] = "acknowledged";       // a string to send back

WiFiUDP Udp;


/************************************************************
MPU9250_DMP_Quaternion
 Quaternion example for MPU-9250 DMP Arduino Library 
Jim Lindblom @ SparkFun Electronics
original creation date: November 23, 2016
https://github.com/sparkfun/SparkFun_MPU9250_DMP_Arduino_Library

The MPU-9250's digital motion processor (DMP) can calculate
four unit quaternions, which can be used to represent the
rotation of an object.

This exmaple demonstrates how to configure the DMP to 
calculate quaternions, and prints them out to the serial
monitor. It also calculates pitch, roll, and yaw from those
values.

Development environment specifics:
Arduino IDE 1.6.12
SparkFun 9DoF Razor IMU M0

Supported Platforms:
- ATSAMD21 (Arduino Zero, SparkFun SAMD21 Breakouts)
*************************************************************/
#include <SparkFunMPU9250-DMP.h>

#define SerialPort SerialUSB

MPU9250_DMP imu;
float Roll, Pitch, Yaw;
float gyroX, gyroY, gyroZ;

void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  delay(3000);
//  while (!Serial) {
//    ; // wait for serial port to connect. Needed for native USB port only
//  }

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    //Serial.print("Attempting to connect to SSID: ");
    //Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }
  //Serial.println("Connected to wifi");
  //printWifiStatus();

  //Serial.println("\nStarting connection to server...");
  // if you get a connection, report back via serial:
  Udp.begin(localPort);

    // Call imu.begin() to verify communication and initialize
  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      SerialPort.println("Unable to communicate with MPU-9250");
      SerialPort.println("Check connections, and try again.");
      SerialPort.println();
      delay(5000);
    }
  }
  //imu.setSensors(INV_XYZ_GYRO); // Enable gyroscope only
  imu.setGyroFSR(2000); // Set gyro to 2000 dps
  imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
               DMP_FEATURE_GYRO_CAL |  // Use gyro calibration
               DMP_FEATURE_SEND_CAL_GYRO, // Send gyro calibrated data
              100); // Set DMP FIFO rate to 100 Hz
  // DMP_FEATURE_LP_QUAT can also be used. It uses the 
  // accelerometer in low-power mode to estimate quat's.
  // DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive

}

void loop() {

  // if there's data available, read a packet
//  int packetSize = Udp.parsePacket();
//  if (packetSize) {
//    Serial.print("Received packet of size ");
//    Serial.println(packetSize);
//    Serial.print("From ");
//    IPAddress remoteIp = Udp.remoteIP();
//    Serial.print(remoteIp);
//    Serial.print(", port ");
//    Serial.println(Udp.remotePort());
//
//    // read the packet into packetBufffer
//    int len = Udp.read(packetBuffer, 255);
//    if (len > 0) {
//      packetBuffer[len] = 0;
//    }
//    Serial.println("Contents:");
//    Serial.println(packetBuffer);


    //Serial.println("Sending upd");
 // }
   // Check for new data in the FIFO
  if ( imu.fifoAvailable() )
  {
    // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
    if ( imu.dmpUpdateFifo() == INV_SUCCESS)
    {
      // computeEulerAngles can be used -- after updating the
      // quaternion values -- to estimate roll, pitch, and yaw
      printIMUData();
      // send a reply, to the IP address and port that sent us the packet we received
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      IPAddress IPaddress(192, 168, 43, 72);
      Udp.beginPacket(IPaddress, localPort);
      char buf[10];
      dtostrf(Roll*180/M_PI, 7, 2, buf);Udp.print("'Roll':");Udp.print(buf);
      dtostrf(Pitch*180/M_PI, 6, 2, buf);Udp.print(",'Pitch':");Udp.print(buf);
      dtostrf(Yaw*180/M_PI, 7, 2, buf);Udp.print(",'Yaw':");Udp.print(buf);
      dtostrf(gyroX, 8, 2, buf);Udp.print(",'GyroX':");Udp.print(buf);
      dtostrf(gyroY, 8, 2, buf);Udp.print(",'GyroY':");Udp.print(buf);
      dtostrf(gyroZ, 8, 2, buf);Udp.print(",'GyroZ':");Udp.print(buf);
      //Udp.print(String(Roll*180/M_PI, 2)+","+String(Pitch*180/M_PI, 2)+","+String(Yaw*180/M_PI, 2)+","+String(gyroX, 2)+"," + String(gyroY, 2) +"," +String(gyroZ, 2));
      Udp.endPacket();
    }
  }
}

void printIMUData(void)
{  
  // After calling dmpUpdateFifo() the ax, gx, mx, etc. values
  // are all updated.
  // Quaternion values are, by default, stored in Q30 long
  // format. calcQuat turns them into a float between -1 and 1
  float q_w = imu.calcQuat(imu.qw);
  float q_x = imu.calcQuat(imu.qx);
  float q_y = imu.calcQuat(imu.qy);
  float q_z = imu.calcQuat(imu.qz);

  gyroX = imu.calcGyro(imu.gx);
  gyroY = imu.calcGyro(imu.gy);
  gyroZ = imu.calcGyro(imu.gz);
  
//  SerialPort.println("Q: " + String(q_w, 4) + ", " +
//                    String(q_x, 4) + ", " + String(q_y, 4) + 
//                    ", " + String(q_z, 4));

 //  Inspired from https//github.com/amcmorl/motorlab/blob/master/rp3/rotations.py
 //  but two rotations were corrected

 // The sensor has to be mounted with x in the direction in which the wind is going(north) and z up when the kite is at zenith
 // This means x is backward, z up

   float Great_roll, Great_pitch, Small_yaw;
  // with reverse x and z axis to kite angles
  //'XYZ' convention
//  Great_roll  = atan2(2*(q_x*q_w-q_y*q_z), q_w*q_w-q_x*q_x - q_y*q_y+q_z*q_z);
//  Great_pitch = asin(2*(q_x*q_z+q_y*q_w));
//  Small_yaw   = atan2(2*(q_z*q_w-q_x*q_y), q_w*q_w+q_x*q_x - q_y*q_y-q_z*q_z);

  
  //'ZYX' aeronautical convention
  
  // Initially the sensor is in NED position (X forward is pointing north, z down is pointing down)

  Yaw  = -atan2(2*(q_x*q_y+q_z*q_w), q_w*q_w+q_x*q_x - q_y*q_y-q_z*q_z);
  Pitch = asin(2*(q_y*q_w - q_x*q_z));
  Roll   = -atan2(2*(q_x*q_w+q_z*q_y), q_w*q_w - q_x*q_x- q_y*q_y + q_z*q_z);




//
//  SerialPort.println("R/P/Y: " + String(Roll) + ", "
//            + String(Pitch) + ", " + String(Yaw));
//  SerialPort.println("GR/P/Y: " + String(Great_roll) + ", "
//            + String(Great_pitch) + ", " + String(Small_yaw));
//  SerialPort.println("Time: " + String(imu.time) + " ms");
//  SerialPort.println();

  char buf[10];
  
  dtostrf(Roll*180/M_PI, 7, 2, buf);SerialPort.print("R:");SerialPort.print(buf);
  dtostrf(Pitch*180/M_PI, 6, 2, buf);SerialPort.print(", P:");SerialPort.print(buf);
  dtostrf(Yaw*180/M_PI, 7, 2, buf);SerialPort.print(", Y:");SerialPort.print(buf);
  dtostrf(gyroX, 8, 2, buf);SerialPort.print(", GX:");SerialPort.print(buf);
  dtostrf(gyroY, 8, 2, buf);SerialPort.print(", GY:");SerialPort.print(buf);
  dtostrf(gyroZ, 8, 2, buf);SerialPort.print(", GZ:");SerialPort.println(buf);
}


void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
