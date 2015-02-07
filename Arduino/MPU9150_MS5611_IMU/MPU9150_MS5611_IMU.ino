/*****************************************************************
MPU9150_AHRS_directdata.ino
SFE_MPU9150 Library AHRS Data Fusion Example Code
Kris Winer for Sparkfun Electronics
Original Creation Date: April 8, 2014
https://github.com/sparkfun/MPU9150_Breakout

The MPU9150 is a versatile 9DOF sensor. It has a built-in
accelerometer, gyroscope, and magnetometer that
functions over I2C. It is very similar to the 6 DoF MPU6050 for which an extensive library has already been built.
Most of the function of the MPU9150 can utilize the MPU6050 library.

This Arduino sketch utilizes Jeff Rowberg's MPU6050 library to generate the basic sensor data
for use in two sensor fusion algorithms becoming increasingly popular with DIY quadcopter and robotics engineers. 
I have added and slightly modified Jeff's library here.

This simple sketch will demo the following:
* How to create a MPU6050 object, using a constructor (global variables section).
* How to use the initialize() function of the MPU6050 class.
* How to read the gyroscope, accelerometer, and magnetometer
  using the readAcceleration(), readRotation(), and readMag() functions and the
  gx, gy, gz, ax, ay, az, mx, my, and mz variables.
* How to calculate actual acceleration, rotation speed, magnetic
  field strength using the  specified ranges as described in the data sheet:
  http://dlnmh9ip6v2uc.cloudfront.net/datasheets/Sensors/IMU/PS-MPU-9150A.pdf
  and
  http://dlnmh9ip6v2uc.cloudfront.net/datasheets/Sensors/IMU/RM-MPU-9150A-00.pdf.
  
In addition, the sketch will demo:
* How to check for data updates using the data ready status register
* How to display output at a rate different from the sensor data update and fusion filter update rates
* How to specify the accelerometer and gyro sampling and bandwidth rates
* How to use the data from the MPU9150 to fuse the sensor data into a quaternion representation of the sensor frame
  orientation relative to a fixed Earth frame providing absolute orientation information for subsequent use.
* An example of how to use the quaternion data to generate standard aircraft orientation data in the form of
  Tait-Bryan angles representing the sensor yaw, pitch, and roll angles suitable for any vehicle stablization control application.

Hardware setup: This library supports communicating with the
MPU9150 over I2C. These are the only connections that need to be made:
	MPU9150 --------- Arduino
	 SCL ---------- SCL (A5 on older 'Duinos')
	 SDA ---------- SDA (A4 on older 'Duinos')
	 VDD ------------- 3.3V
	 GND ------------- GND

The LSM9DS0 has a maximum voltage of 3.5V. Make sure you power it
off the 3.3V rail! And either use level shifters between SCL
and SDA or just use a 3.3V Arduino Pro.	  

Development environment specifics:
	IDE: Arduino 1.0.5
	Hardware Platform: Arduino Pro 3.3V/8MHz
	MPU9150 Breakout Version: 1.0

This code is beerware. If you see me (or any other SparkFun 
employee) at the local, and you've found our code helpful, please 
buy us a round!

Distributed as-is; no warranty is given.
*****************************************************************/
/*
MS561101BA_altitude.pde - Computes altitude from sea level using pressure readings from the sensor.
The algorithm uses the Hypsometric formula as explained in http://keisan.casio.com/has10/SpecExec.cgi?path=06000000.Science%2F02100100.Earth%20science%2F12000300.Altitude%20from%20atmospheric%20pressure%2Fdefault.xml&charset=utf-8

Copyright (C) 2011 Fabio Varesano <fvaresano@yahoo.it>

Development of this code has been supported by the Department of Computer Science,
Universita' degli Studi di Torino, Italy within the Piemonte Project
http://www.piemonte.di.unito.it/


This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#define USE_MAVLINK //Uncomment this line to use mavlink
#define RAW_DATA //Uncomment this line to get raw data for sensor calibration
#ifdef USE_MAVLINK
  #include "MavlinkForArduino.h"        // Mavlink interface
#endif

#include <Wire.h>
#include "I2Cdev.h" //from https://github.com/sparkfun/MPU-9150_Breakout.git
#include "MPU6050_9Axis_MotionApps41.h"//from https://github.com/sparkfun/MPU-9150_Breakout.git
#include <MS561101BA.h>

#define OUTPUT_RATE_ms 50 // 20Hz

#define MOVAVG_SIZE 32

// Define corrections/offsets for magnetometer
#define LOCAL_DECLINATION_DEG -3 //at Nantes, France is around 3 degrees in 2014
#define MAG_X_OFFSET 0.0f
#define MAG_Y_OFFSET 0.0f
#define MAG_Z_OFFSET 0.0f
#define ACC_X_OFFSET 0.0f
#define ACC_Y_OFFSET 0.0f
#define ACC_Z_OFFSET 0.0f
#define GYRO_X_OFFSET 0.0f
#define GYRO_Y_OFFSET 0.0f
#define GYRO_Z_OFFSET 0.0f
#define GYRO_RANGE_DEGPS 250.0f
#define ACCEL_RANGE_G 2.0f
#define MAG_RANGE_mG 12290.0f// milliGauss (1229 microTesla per 2^12 bits, 10 mG per microTesla)
/*
#define MAG_X_OFFSET 123.0f
#define MAG_Y_OFFSET -47.0f
#define MAG_Z_OFFSET 119.0f
*/
// Declare device MPU6050 class
MPU6050 mpu(0x69);

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
#define GyroMeasError PI * (40.0f / 180.0f)       // gyroscope measurement error in rads/s (shown as 3 deg/s)
#define GyroMeasDrift PI * (0.0f / 180.0f)      // gyroscope measurement drift in rad/s/s (shown as 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
#define beta sqrt(3.0f / 4.0f) * GyroMeasError   // compute beta
#define zeta sqrt(3.0f / 4.0f) * GyroMeasDrift   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

int16_t a1, a2, a3, g1, g2, g3, m1, m2, m3;     // raw data arrays reading
uint16_t count = 0;  // used to control display output rate
uint16_t delt_t = 0; // used to control display output rate
uint16_t mcount = 0; // used to control display output rate
uint8_t MagRate;     // read rate for magnetometer data

float pitch, yaw, roll;
float deltat = 0.0f;        // integration interval for both filter schemes
uint64_t lastUpdate = 0; // used to calculate integration interval
uint64_t time_boot_us = 0;        // used to calculate integration interval

float ax_g, ay_g, az_g, gx_degps, gy_degps, gz_degps, mx, my, mz; // variables to hold latest sensor data values 
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method

uint8_t system_id = 100;
uint8_t component_id = 200;
uint32_t time_boot_ms;

//MS5611
MS561101BA baro = MS561101BA();
float movavg_buff[MOVAVG_SIZE];
int movavg_i=0;

const float sea_press = 1013.25;
float press, temperature, altitude;

short n_gyro_range = 0;
short n_gyro_range_old = 0;
short n_accel_range = 0;
short n_accel_range_old = 0;

void setup()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  Serial.begin(57600); // Start serial at 57600 bps

  delay(2000);            

  // initialize MPU6050 device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU9150 connection successful") : F("MPU9150 connection failed"));

// Set up the accelerometer, gyro, and magnetometer for data output

   mpu.setRate(7); // set gyro rate to 8 kHz/(1 + rate) shows 1 kHz, accelerometer ODR is fixed at 1 KHz

   MagRate = 100; // set magnetometer read rate in Hz; 10 to 100 (max) Hz are reasonable values

// Digital low pass filter configuration. 
// It also determines the internal sampling rate used by the device as shown in the table below.
// The accelerometer output rate is fixed at 1kHz. This means that for a Sample
// Rate greater than 1kHz, the same accelerometer sample may be output to the
// FIFO, DMP, and sensor registers more than once.
/*
 *          |   ACCELEROMETER    |           GYROSCOPE
 * DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
 * ---------+-----------+--------+-----------+--------+-------------
 * 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
 * 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
 * 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
 * 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
 * 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
 * 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
 * 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
 */
  mpu.setDLPFMode(4); // set bandwidth of both gyro and accelerometer to ~20 Hz

// Full-scale range of the gyro sensors:
// 0 = +/- 250 degrees/sec, 1 = +/- 500 degrees/sec, 2 = +/- 1000 degrees/sec, 3 = +/- 2000 degrees/sec
  mpu.setFullScaleGyroRange(0); // set gyro range to 250 degrees/sec

// Full-scale accelerometer range.
// The full-scale range of the accelerometer: 0 = +/- 2g, 1 = +/- 4g, 2 = +/- 8g, 3 = +/- 16g
  mpu.setFullScaleAccelRange(0); // set accelerometer to 2 g range

  mpu.setIntDataReadyEnabled(true); // enable data ready interrupt
  
  // Suppose that the CSB pin is connected to GND.
  // You'll have to check this on your breakout schematics
  baro.init(MS561101BA_ADDR_CSB_LOW); 
  delay(100);
  
  // populate movavg_buff before starting loop
  for(int i=0; i<MOVAVG_SIZE; i++) {
    movavg_buff[i] = baro.getPressure(MS561101BA_OSR_4096);
  }
}
/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read(); 
  }
}

void loop()
{
    temperature = baro.getTemperature(MS561101BA_OSR_4096);
    press = baro.getPressure(MS561101BA_OSR_4096);
    pushAvg(press);
    press = getAvg(movavg_buff, MOVAVG_SIZE);
    altitude = getAltitude(press, temperature);
    automaticGyroRangeSelection();
    automaticAccelRangeSelection();
    if(mpu.getIntDataReadyStatus() == 1) { // wait for data ready status register to update all data registers
            mcount++;
           // read the raw sensor data
            mpu.getAcceleration  ( &a1, &a2, &a3  );
            a2 = -a2;// Invert y to get NED classical convention
            a1 = a1 + ACC_X_OFFSET;
            a2 = a2 + ACC_Y_OFFSET;
            a3 = a3 + ACC_Z_OFFSET;
            ax_g = a1*ACCEL_RANGE_G*pow(2, n_accel_range)/pow(2, 15);
            ay_g = a2*ACCEL_RANGE_G*pow(2, n_accel_range)/pow(2, 15);
            az_g = a3*ACCEL_RANGE_G*pow(2, n_accel_range)/pow(2, 15); 

            mpu.getRotation  ( &g1, &g2, &g3  );
            g1 = -g1;
            g3 = -g3;
            g1 = g1 + GYRO_X_OFFSET;
            g2 = g2 + GYRO_Y_OFFSET;
            g3 = g3 + GYRO_Z_OFFSET;
            gx_degps = g1*GYRO_RANGE_DEGPS*pow(2, n_gyro_range)/pow(2, 15);
            gy_degps = g2*GYRO_RANGE_DEGPS*pow(2, n_gyro_range)/pow(2, 15);
            gz_degps = g3*GYRO_RANGE_DEGPS*pow(2, n_gyro_range)/pow(2, 15);
//  The gyros and accelerometers can in principle be calibrated in addition to any factory calibration but they are generally
//  pretty accurate. You can check the accelerometer by making sure the reading is +1 g in the positive direction for each axis.
//  The gyro should read zero for each axis when the sensor is at rest. Small or zero adjustment should be needed for these sensors.
//  The magnetometer is a different thing. Most magnetometers will be sensitive to circuit currents, computers, and 
//  other both man-made and natural sources of magnetic field. The rough way to calibrate the magnetometer is to record
//  the maximum and minimum readings (generally achieved at the North magnetic direction). The average of the sum divided by two
//  should provide a pretty good calibration offset. Don't forget that for the MPU9150, the magnetometer x- and y-axes are switched 
//  compared to the gyro and accelerometer!
            if (mcount > 1000/MagRate) {  // this is a poor man's way of setting the magnetometer read rate (see below) 
            mpu.getMag  ( &m1, &m2, &m3 );
            m2 = -m2;
            // Apply calibration offsets on raw measurements that correspond to your environment and magnetometer
            m1 = m1 + MAG_X_OFFSET;
            m2 = m2 + MAG_Y_OFFSET;
            m3 = m3 + MAG_Z_OFFSET;
            mx = m1*MAG_RANGE_mG/pow(2, 12); 
            my = m2*MAG_RANGE_mG/pow(2, 12); 
            mz = m3*MAG_RANGE_mG/pow(2, 12);
            mcount = 0;
            }           
         }
   
  time_boot_us = micros();
  time_boot_ms = millis();
  deltat = ((time_boot_us - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = time_boot_us;
  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
  // the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
  // We have to make some allowance for this orientationmismatch in feeding the output to the quaternion filter.
  // For the MPU-9150, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
  // in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
  // This is ok by aircraft orientation standards!  
  // Pass gyro rate as rad/s
   MadgwickQuaternionUpdate(ax_g, ay_g, az_g, gx_degps*PI/180.0f, gy_degps*PI/180.0f, gz_degps*PI/180.0f, my, mx, mz);
// MahonyQuaternionUpdate(ax_g, ay, az, gx_degps*PI/180.0f, gy_degps*PI/180.0f, gz_degps*PI/180.0f, my, mx, mz);

    // Serial print and/or display at output rate independent of data rates
    delt_t = millis() - count;
    if (delt_t > OUTPUT_RATE_ms) {

    // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
    // In this coordinate system, the positive z-axis is down toward Earth. 
    // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
    // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
    // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
    // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
    // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
    // applied in the correct order which for this configuration is yaw, pitch, and then roll.
    // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
      yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
      pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
      roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
      yaw = yaw + LOCAL_DECLINATION_DEG*PI/180.0f; // Declination is difference between magnetic and true north
  
#ifdef USE_MAVLINK
      // Initialize the required buffers 
      mavlink_message_t msg; 
      uint8_t buf[MAVLINK_MAX_PACKET_LEN];
      mavlink_msg_heartbeat_pack(system_id, component_id, &msg, MAV_TYPE_KITE, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
      uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
      Serial.write(buf, len);
      
      //static inline uint16_t mavlink_msg_attitude_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
      //  uint32_t time_boot_ms, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed)
      mavlink_msg_attitude_pack(system_id, component_id, &msg, time_boot_ms, roll, pitch, yaw, gx_degps*PI/180.0f, gy_degps*PI/180.0f, gz_degps*PI/180.0f);
      len = mavlink_msg_to_send_buffer(buf, &msg);
      Serial.write(buf, len);
      
  #ifdef RAW_DATA
      //static inline uint16_t mavlink_msg_highres_imu_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
      //						       uint64_t time_usec, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float xmag, float ymag, float zmag, float abs_pressure, float diff_pressure, float pressure_alt, float temperature, uint16_t fields_updated)
      mavlink_msg_highres_imu_pack(system_id, component_id, &msg, (uint64_t)time_boot_us, ax_g*9.81, ay_g*9.81, az_g*9.81, gx_degps*PI/180.0f, gy_degps*PI/180.0f, gz_degps*PI/180.0f, my, mx, mz, press, press, altitude, temperature, a1);
      len = mavlink_msg_to_send_buffer(buf, &msg);
      Serial.write(buf, len);
  #endif
      /*
      //mavlink_msg_scaled_imu_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
      // uint32_t time_boot_ms, int16_t xacc, int16_t yacc, int16_t zacc, int16_t xgyro, int16_t ygyro, int16_t zgyro, int16_t xmag, int16_t ymag, int16_t zmag)
      mavlink_msg_raw_imu_pack(system_id, component_id, &msg, time_boot_ms, a1, a2, a3, g1, g2, g3, m1, m2, m3);
      len = mavlink_msg_to_send_buffer(buf, &msg);
      Serial.write(buf, len);
      
      mavlink_msg_scaled_imu_pack(system_id, component_id, &msg, time_boot_ms, ax_g*1000, ay_g*1000, az_g*1000, gx_degps*PI/180.0f*1000, gy_degps*PI/180.0f*1000, gz_degps*PI/180.0f*1000, mx, my, mz);
      len = mavlink_msg_to_send_buffer(buf, &msg);
      Serial.write(buf, len);
      */
      
#else
      Serial.print("ax = "); Serial.print((int)1000*ax_g);  
      Serial.print(" ay = "); Serial.print((int)1000*ay_g); 
      Serial.print(" az = "); Serial.print((int)1000*az_g); Serial.println(" mg");
      Serial.print("gx = "); Serial.print( gx_degps, 2); 
      Serial.print(" gy = "); Serial.print( gy_degps, 2); 
      Serial.print(" gz = "); Serial.print( gz_degps, 2); Serial.println(" deg/s");
      Serial.print("mx = "); Serial.print( (int)mx ); 
      Serial.print(" my = "); Serial.print( (int)my ); 
      Serial.print(" mz = "); Serial.print( (int)mz ); Serial.println(" mG");
      
      Serial.print("q0 = "); Serial.print(q[0]);
      Serial.print(" qx = "); Serial.print(q[1]); 
      Serial.print(" qy = "); Serial.print(q[2]); 
      Serial.print(" qz = "); Serial.println(q[3]); 
      Serial.print("Yaw, Pitch, Roll: ");
      Serial.print(yaw*180.0f / PI, 2);
      Serial.print(", ");
      Serial.print(pitch*180.0f / PI, 2);
      Serial.print(", ");
      Serial.println(roll*180.0f / PI, 2);
      
      Serial.print("rate = "); Serial.print((float)1.0f/deltat, 2); Serial.println(" Hz");
      
      Serial.print(" temp: ");
      Serial.print(temperature);
      Serial.print(" degC pres: ");
      Serial.print(press);
      Serial.print(" mbar altitude: ");
      Serial.print(altitude);
      Serial.println(" m");
#endif
      
    
      // With these settings the filter is updating at a ~145 Hz rate using the Madgwick scheme and 
      // >200 Hz using the Mahony scheme even though the display refreshes at only 2 Hz.
      // The filter update rate is determined mostly by the mathematical steps in the respective algorithms, 
      // the processor speed (8 MHz for the 3.3V Pro Mini), and the magnetometer ODR:
      // an ODR of 10 Hz for the magnetometer produce the above rates, maximum magnetometer ODR of 100 Hz produces
      // filter update rates of 36 - 145 and ~38 Hz for the Madgwick and Mahony schemes, respectively. 
      // This is presumably because the magnetometer read takes longer than the gyro or accelerometer reads.
      // This filter update rate should be fast enough to maintain accurate platform orientation for 
      // stabilization control of a fast-moving robot or quadcopter. Compare to the update rate of 200 Hz
      // produced by the on-board Digital Motion Processor of Invensense's MPU6050 6 DoF and MPU9150 9DoF sensors.
      // The 3.3 V 8 MHz Pro Mini is doing pretty well!
  
      count = millis();
    }
}
void automaticGyroRangeSelection()
{
  if ((fabs(gx_degps)<0.2*GYRO_RANGE_DEGPS)&&(fabs(gy_degps)<0.2*GYRO_RANGE_DEGPS)&&(fabs(gz_degps)<0.2*GYRO_RANGE_DEGPS))
  {
    n_gyro_range = 0;
  }
  else   if ((fabs(gx_degps)<0.2*GYRO_RANGE_DEGPS*2)&&(fabs(gy_degps)<0.2*GYRO_RANGE_DEGPS*2)&&(fabs(gz_degps)<0.2*GYRO_RANGE_DEGPS*2))
  {
    n_gyro_range = 1;
  }
  else   if ((fabs(gx_degps)<0.2*GYRO_RANGE_DEGPS*4)&&(fabs(gy_degps)<0.2*GYRO_RANGE_DEGPS*4)&&(fabs(gz_degps)<0.2*GYRO_RANGE_DEGPS*4))
  {
    n_gyro_range = 2;
  }
  else
  {
    n_gyro_range = 3;
  }
  
  
  if (n_gyro_range!=n_gyro_range_old)
  {
    // Full-scale range of the gyro sensors:
    // 0 = +/- 250 degrees/sec, 1 = +/- 500 degrees/sec, 2 = +/- 1000 degrees/sec, 3 = +/- 2000 degrees/sec
    mpu.setFullScaleGyroRange(n_gyro_range); // set gyro range to 250 degrees/sec
    n_gyro_range_old = n_gyro_range;
  }
}
void automaticAccelRangeSelection()
{
  if ((fabs(ax_g)<0.2*ACCEL_RANGE_G)&&(fabs(ay_g)<0.2*ACCEL_RANGE_G)&&(fabs(az_g)<0.2*ACCEL_RANGE_G))
  {
    n_accel_range = 0;
  }
  else   if ((fabs(ax_g)<0.2*ACCEL_RANGE_G*2)&&(fabs(ay_g)<0.2*ACCEL_RANGE_G*2)&&(fabs(az_g)<0.2*ACCEL_RANGE_G*2))
  {
    n_accel_range = 1;
  }
  else   if ((fabs(ax_g)<0.2*ACCEL_RANGE_G*4)&&(fabs(ay_g)<0.2*ACCEL_RANGE_G*4)&&(fabs(az_g)<0.2*ACCEL_RANGE_G*4))
  {
    n_accel_range = 2;
  }
  else
  {
    n_accel_range = 3;
  }
  
  if (n_accel_range!=n_accel_range_old)
  {
  // Full-scale accelerometer range.
// The full-scale range of the accelerometer: 0 = +/- 2g, 1 = +/- 4g, 2 = +/- 8g, 3 = +/- 16g
    mpu.setFullScaleAccelRange(n_accel_range);
    n_accel_range_old = n_accel_range;
  }
}


float getAltitude(float press, float temp) {
  //return (1.0f - pow(press/101325.0f, 0.190295f)) * 4433000.0f;
  return ((pow((sea_press / press), 1/5.257) - 1.0) * (temp + 273.15)) / 0.0065;
}

void pushAvg(float val) {
  movavg_buff[movavg_i] = val;
  movavg_i = (movavg_i + 1) % MOVAVG_SIZE;
}

float getAvg(float * buff, int size) {
  float sum = 0.0;
  for(int i=0; i<size; i++) {
    sum += buff[i];
  }
  return sum / size;
}


// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
        void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
        {
            float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
            float norm;
            float hx, hy, _2bx, _2bz;
            float s1, s2, s3, s4;
            float qDot1, qDot2, qDot3, qDot4;

            // Auxiliary variables to avoid repeated arithmetic
            float _2q1mx;
            float _2q1my;
            float _2q1mz;
            float _2q2mx;
            float _4bx;
            float _4bz;
            float _2q1 = 2.0f * q1;
            float _2q2 = 2.0f * q2;
            float _2q3 = 2.0f * q3;
            float _2q4 = 2.0f * q4;
            float _2q1q3 = 2.0f * q1 * q3;
            float _2q3q4 = 2.0f * q3 * q4;
            float q1q1 = q1 * q1;
            float q1q2 = q1 * q2;
            float q1q3 = q1 * q3;
            float q1q4 = q1 * q4;
            float q2q2 = q2 * q2;
            float q2q3 = q2 * q3;
            float q2q4 = q2 * q4;
            float q3q3 = q3 * q3;
            float q3q4 = q3 * q4;
            float q4q4 = q4 * q4;

            // Normalise accelerometer measurement
            norm = sqrt(ax * ax + ay * ay + az * az);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f/norm;
            ax *= norm;
            ay *= norm;
            az *= norm;

            // Normalise magnetometer measurement
            norm = sqrt(mx * mx + my * my + mz * mz);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f/norm;
            mx *= norm;
            my *= norm;
            mz *= norm;

            // Reference direction of Earth's magnetic field
            _2q1mx = 2.0f * q1 * mx;
            _2q1my = 2.0f * q1 * my;
            _2q1mz = 2.0f * q1 * mz;
            _2q2mx = 2.0f * q2 * mx;
            hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
            hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
            _2bx = sqrt(hx * hx + hy * hy);
            _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
            _4bx = 2.0f * _2bx;
            _4bz = 2.0f * _2bz;

            // Gradient decent algorithm corrective step
            s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
            norm = 1.0f/norm;
            s1 *= norm;
            s2 *= norm;
            s3 *= norm;
            s4 *= norm;

            // Compute rate of change of quaternion
            qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
            qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
            qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
            qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

            // Integrate to yield quaternion
            q1 += qDot1 * deltat;
            q2 += qDot2 * deltat;
            q3 += qDot3 * deltat;
            q4 += qDot4 * deltat;
            norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
            norm = 1.0f/norm;
            q[0] = q1 * norm;
            q[1] = q2 * norm;
            q[2] = q3 * norm;
            q[3] = q4 * norm;

        }
  
  
  
 // Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and
 // measured ones. 
            void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
        {
            float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
            float norm;
            float hx, hy, bx, bz;
            float vx, vy, vz, wx, wy, wz;
            float ex, ey, ez;
            float pa, pb, pc;

            // Auxiliary variables to avoid repeated arithmetic
            float q1q1 = q1 * q1;
            float q1q2 = q1 * q2;
            float q1q3 = q1 * q3;
            float q1q4 = q1 * q4;
            float q2q2 = q2 * q2;
            float q2q3 = q2 * q3;
            float q2q4 = q2 * q4;
            float q3q3 = q3 * q3;
            float q3q4 = q3 * q4;
            float q4q4 = q4 * q4;   

            // Normalise accelerometer measurement
            norm = sqrt(ax * ax + ay * ay + az * az);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f / norm;        // use reciprocal for division
            ax *= norm;
            ay *= norm;
            az *= norm;

            // Normalise magnetometer measurement
            norm = sqrt(mx * mx + my * my + mz * mz);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f / norm;        // use reciprocal for division
            mx *= norm;
            my *= norm;
            mz *= norm;

            // Reference direction of Earth's magnetic field
            hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
            hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
            bx = sqrt((hx * hx) + (hy * hy));
            bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

            // Estimated direction of gravity and magnetic field
            vx = 2.0f * (q2q4 - q1q3);
            vy = 2.0f * (q1q2 + q3q4);
            vz = q1q1 - q2q2 - q3q3 + q4q4;
            wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
            wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
            wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);  

            // Error is cross product between estimated direction and measured direction of gravity
            ex = (ay * vz - az * vy) + (my * wz - mz * wy);
            ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
            ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
            if (Ki > 0.0f)
            {
                eInt[0] += ex;      // accumulate integral error
                eInt[1] += ey;
                eInt[2] += ez;
            }
            else
            {
                eInt[0] = 0.0f;     // prevent integral wind up
                eInt[1] = 0.0f;
                eInt[2] = 0.0f;
            }

            // Apply feedback terms
            gx = gx + Kp * ex + Ki * eInt[0];
            gy = gy + Kp * ey + Ki * eInt[1];
            gz = gz + Kp * ez + Ki * eInt[2];

            // Integrate rate of change of quaternion
            pa = q2;
            pb = q3;
            pc = q4;
            q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
            q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
            q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
            q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

            // Normalise quaternion
            norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
            norm = 1.0f / norm;
            q[0] = q1 * norm;
            q[1] = q2 * norm;
            q[2] = q3 * norm;
            q[3] = q4 * norm;
 
        }
        
        
      


