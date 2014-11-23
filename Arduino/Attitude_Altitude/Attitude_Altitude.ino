 // Note that MPU6050_6Axis_MotionApps20.h had to be modified to reduce fifo fill rate

//#include <RH_ASK.h>
#include <SPI.h> // Not actually used but needed to compile

//RH_ASK driver(4000, 11, 12);

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "HMC5883L.h"
//#include "MS561101BA.h"
#include "Wire.h"

#include <MsTimer2.h>

#define UPDATE_RATE 200
#define BAUDRATE 57600


#define MOVAVG_SIZE 32
float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
float qq[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float eInt[3] = {0.0f, 0.0f, 0.0f}; 
// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
#define GyroMeasError PI * (40.0f / 180.0f)      // gyroscope measurement error in rads/s (shown as 40 deg/s)
#define GyroMeasDrift PI * (2.0f / 180.0f)       // gyroscope measurement drift in rad/s/s (shown as 2.0 deg/s/s)
#define beta sqrt(3.0f / 4.0f) * GyroMeasError   // compute beta
#define zeta sqrt(3.0f / 4.0f) * GyroMeasDrift   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

float deltat = 0.0f;        // integration interval for both filter schemes

uint32_t lastUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;        // used to calculate integration interval



// class default I2C address is 0x1E
// specific I2C addresses may be passed as a parameter here
// this device only supports one I2C address (0x1E)
HMC5883L mag;
MPU6050 mpu(0x69); // <-- use for AD0 high
//MS561101BA baro = MS561101BA();

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL


///******* Acc+gyro
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// Magneto
int16_t mx16, my16, mz16;
float heading;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
int32_t aa[3];         // [x, y, z]            accel sensor measurements
int32_t rr[3];         // [x, y, z]            accel sensor measurements
//VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
//VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

///////********Baro
float movavg_buff[MOVAVG_SIZE];
int movavg_i=0;

const float sea_press = 1013.25;
float press, temp;
///////************


////*** Com

struct Attitude {
  int u;
  int v;
  int w;
  int x;
  int y;
  int z;
  int t;
  int h;
  int p;
};

Attitude attitude;
// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;  
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)    
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(BAUDRATE);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.
    
    /*
    Serial.println(F("Initializing Barometer..."));
    // Suppose that the CSB pin is connected to GND.
    // You'll have to check this on your breakout schematics
    baro.init(MS561101BA_ADDR_CSB_LOW); 
    delay(100);
    
    // populate movavg_buff before starting loop
    for(int i=0; i<MOVAVG_SIZE; i++) {
    movavg_buff[i] = baro.getPressure(MS561101BA_OSR_4096);
    }  */

    // initialize device
    Serial.println(F("Initializing Gyro+Acc..."));
    mpu.initialize();
    
    mag.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");

    while (Serial.available() && Serial.read()); // empty buffer

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    
    //Allow to get magnetometer data
    mpu.setI2CMasterModeEnabled(0);
    mpu.setI2CBypassEnabled(1);
    //if (!driver.init())
    //     Serial.println("init failed");
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
long last_time =0;

void loop() {

    
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
      
        // read raw heading measurements from device
        mag.getHeading(&mx16, &my16, &mz16);
        // To calculate heading in degrees. 0 degree indicates North
        heading = atan2(my16, mx16);
        if(heading < 0)
          heading += 2 * M_PI;
          
        /*    
        // other program behavior stuff here        
        float temperature = baro.getTemperature(MS561101BA_OSR_4096);
        if(temperature) { temp = temperature;   }
        attitude.t = temp*100;
        //Serial.print(" temp: "); Serial.print(temp);
        
        press = baro.getPressure(MS561101BA_OSR_4096);
        if(press!=NULL) {      pushAvg(press);      }
        press = getAvg(movavg_buff, MOVAVG_SIZE);
        attitude.p = press;
        //Serial.print(" degC pres: "); Serial.print(press);//
        
        float altitude = getAltitude(press, temp);
        attitude.h = altitude;*/
        //Serial.print(" mbar altitude: "); Serial.print(altitude); Serial.println(" m");
        
        if (millis() > last_time + UPDATE_RATE)
         { 
               Now = micros();
               deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
               lastUpdate = Now;
               MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  -mx,  -mz);
               sendData();
               last_time = millis();
         }
       
       // enf of other programm behavior
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            attitude.u = ypr[0] * 180/M_PI*100;
            attitude.v = ypr[1] * 180/M_PI*100;
            attitude.w = ypr[2] * 180/M_PI*100;
             
        mpu.dmpGetAccel(aa, fifoBuffer);
        ax = aa[0];
        ay = aa[1];
        az = aa[2];
        mpu.dmpGetGyro(rr, fifoBuffer);
        gx = rr[0];
        gy = rr[1];
        gz = rr[2];
        
        mx = mx16;
        my = my16;
        mz = mz16;

      
        Now = micros();
        deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
        lastUpdate = Now;
        //MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  -mx,  -mz);
           
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
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


void sendData()
{
  //Sending data to gbase
        char SensorMsg1[7];   
        /*itoa(int(heading * 180/M_PI), SensorMsg1,10);
        Serial.print("heading:\t");
        Serial.println(heading * 180/M_PI);
        */
        /*
        sprintf(SensorMsg1,"%c%d",'ax',ax);
        Serial.println(ax);
        sprintf(SensorMsg1,"%c%d",'ay',ay);
        Serial.println(ay);
        sprintf(SensorMsg1,"%c%d",'az',az);
        Serial.println(az);
        */
        /*
        sprintf(SensorMsg1,"%c%d",'mx',mx);
        Serial.println(SensorMsg1);
        sprintf(SensorMsg1,"%c%d",'my',my);
        Serial.println(SensorMsg1);
        sprintf(SensorMsg1,"%c%d",'mz',mz);
        Serial.println(SensorMsg1);
        */
        
        sprintf(SensorMsg1,"%c%d",'gx',gx);
        Serial.println(SensorMsg1);
        sprintf(SensorMsg1,"%c%d",'gy',gy);
        Serial.println(SensorMsg1);
        sprintf(SensorMsg1,"%c%d",'gz',gz);
        Serial.println(SensorMsg1);
        
        
        sprintf(SensorMsg1,"%c%d",'u',attitude.u);
        Serial.println(SensorMsg1);
        //driver.send((uint8_t *)SensorMsg1, strlen(msg));
        //driver.waitPacketSent();
        
        sprintf(SensorMsg1,"%c%d",'v',attitude.v);
        Serial.println(SensorMsg1);
        //driver.send((uint8_t *)SensorMsg1, strlen(msg));
        //driver.waitPacketSent();
        
        sprintf(SensorMsg1,"%c%d",'w',attitude.w);
        Serial.println(SensorMsg1);
        ///driver.send((uint8_t *)SensorMsg1, strlen(msg));
        //driver.waitPacketSent();
        
        //sprintf(SensorMsg1,"%c%d",'x',attitude.x);
        //Serial.println(SensorMsg1);
        //driver.send((uint8_t *)SensorMsg1, strlen(msg));
        //driver.waitPacketSent();
        
//        sprintf(SensorMsg1,"%c%d",'y',attitude.y);
//        Serial.println(SensorMsg1);
//        //driver.send((uint8_t *)SensorMsg1, strlen(msg));
//        //driver.waitPacketSent();
//        
//        sprintf(SensorMsg1,"%c%d",'z',attitude.z);
//        Serial.println(SensorMsg1);
        //driver.send((uint8_t *)SensorMsg1, strlen(msg));
        //driver.waitPacketSent();
        
        // Altitude
//        sprintf(SensorMsg1,"%c%d",'h',attitude.h);
//        Serial.println(SensorMsg1);
//        driver.send((uint8_t *)SensorMsg1, strlen(SensorMsg1));
//        driver.waitPacketSent();
        
        // Pressure
        //sprintf(SensorMsg1,"%c%d",'p',attitude.p);
        //Serial.println(SensorMsg1);
        //driver.send((uint8_t *)SensorMsg1, strlen(SensorMsg1));
        //driver.waitPacketSent();
        
        // Temperature
        //sprintf(SensorMsg1,"%c%d",'t',attitude.t);
        //Serial.println(SensorMsg1);
        //driver.send((uint8_t *)SensorMsg1, strlen(SensorMsg1));
        //driver.waitPacketSent();
        
        Serial.println();
  
}
