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

void setup() 
{
  SerialPort.begin(115200);

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
  
  imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
               DMP_FEATURE_GYRO_CAL, // Use gyro calibration
              10); // Set DMP FIFO rate to 10 Hz
  // DMP_FEATURE_LP_QUAT can also be used. It uses the 
  // accelerometer in low-power mode to estimate quat's.
  // DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive
}

void loop() 
{
  // Check for new data in the FIFO
  if ( imu.fifoAvailable() )
  {
    // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
    if ( imu.dmpUpdateFifo() == INV_SUCCESS)
    {
      // computeEulerAngles can be used -- after updating the
      // quaternion values -- to estimate roll, pitch, and yaw
      printIMUData();
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
  
  SerialPort.println("Q: " + String(q_w, 4) + ", " +
                    String(q_x, 4) + ", " + String(q_y, 4) + 
                    ", " + String(q_z, 4));

 //  Inspired from https//github.com/amcmorl/motorlab/blob/master/rp3/rotations.py
 //  but two rotations were corrected

 // The sensor has to be mounted with x in the direction in which the wind is going(north) and z up when the kite is at zenith
 // This means x is backward, z up

   float Great_roll, Great_pitch, Small_yaw;
  // with reverse x and z axis to kite angles
  //'XYZ' convention
  Great_roll  = atan2(2*(q_x*q_w-q_y*q_z), q_w*q_w-q_x*q_x - q_y*q_y+q_z*q_z);
  Great_pitch = asin(2*(q_x*q_z+q_y*q_w));
  Small_yaw   = atan2(2*(q_z*q_w-q_x*q_y), q_w*q_w+q_x*q_x - q_y*q_y-q_z*q_z);

  
  //'ZYX' aeronautical convention
  float Roll, Pitch, Yaw;
  // Initially the sensor is in NED position (X forward is pointing north, z down is pointing down)

  Yaw  = -atan2(2*(q_x*q_y+q_z*q_w), q_w*q_w+q_x*q_x - q_y*q_y-q_z*q_z);
  Pitch = asin(2*(q_y*q_w - q_x*q_z));
  Roll   = -atan2(2*(q_x*q_w+q_z*q_y), q_w*q_w - q_x*q_x- q_y*q_y + q_z*q_z);




  SerialPort.println("R/P/Y: " + String(Roll) + ", "
            + String(Pitch) + ", " + String(Yaw));
  SerialPort.println("GR/P/Y: " + String(Great_roll) + ", "
            + String(Great_pitch) + ", " + String(Small_yaw));
  SerialPort.println("Time: " + String(imu.time) + " ms");
  SerialPort.println();
}
