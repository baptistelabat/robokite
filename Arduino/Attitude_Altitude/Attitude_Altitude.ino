#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "HMC5883L.h"
#include "MS561101BA.h"
#include "Wire.h"

#include <MsTimer2.h>
#include <VirtualWire.h> 

#define UPDATE_RATE 500
#define BAUDRATE 57600


#define MOVAVG_SIZE 32

// class default I2C address is 0x1E
// specific I2C addresses may be passed as a parameter here
// this device only supports one I2C address (0x1E)
HMC5883L mag;
MPU6050 mpu(0x69); // <-- use for AD0 high
MS561101BA baro = MS561101BA();

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
#define OUTPUT_READABLE_WORLDACCEL


///******* Acc+gyro
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// Magneto
int16_t mx, my, mz;
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
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
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
    
    Serial.println(F("Initializing Barometer..."));
    // Suppose that the CSB pin is connected to GND.
    // You'll have to check this on your breakout schematics
    baro.init(MS561101BA_ADDR_CSB_LOW); 
    delay(100);
  
    // populate movavg_buff before starting loop
    for(int i=0; i<MOVAVG_SIZE; i++) {
    movavg_buff[i] = baro.getPressure(MS561101BA_OSR_4096);
    }  

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
        mag.getHeading(&mx, &my, &mz);
        // To calculate heading in degrees. 0 degree indicates North
        heading = atan2(my, mx);
        if(heading < 0)
          heading += 2 * M_PI;
            
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
        attitude.h = altitude;
        //Serial.print(" mbar altitude: "); Serial.print(altitude); Serial.println(" m");
        
        if (millis() > last_time +UPDATE_RATE)
         { sendData();
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
            /*Serial.print("ypr\t");  Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");  Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t"); Serial.println(ypr[2] * 180/M_PI);*/
                
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            attitude.x = aaWorld.x;
            attitude.y = aaWorld.y;
            attitude.z = aaWorld.z;
            /*Serial.print("aworld\t");  Serial.print(aaWorld.x);
            Serial.print("\t");  Serial.print(aaWorld.y);
            Serial.print("\t");  Serial.println(aaWorld.z);*/
           
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
        //itoa(attitude.u,SensorMsg1,10);
        Serial.print("heading:\t");
        Serial.println(heading * 180/M_PI);
        sprintf(SensorMsg1,"%c%d",'u',attitude.u);
        Serial.println(SensorMsg1);
        sprintf(SensorMsg1,"%c%d",'v',attitude.v);
        Serial.println(SensorMsg1);
        sprintf(SensorMsg1,"%c%d",'w',attitude.w);
        Serial.println(SensorMsg1);
        sprintf(SensorMsg1,"%c%d",'x',attitude.x);
        Serial.println(SensorMsg1);
        sprintf(SensorMsg1,"%c%d",'y',attitude.y);
        Serial.println(SensorMsg1);
        sprintf(SensorMsg1,"%c%d",'z',attitude.z);
        Serial.println(SensorMsg1);
        sprintf(SensorMsg1,"%c%d",'h',attitude.h);
        Serial.println(SensorMsg1);
        sprintf(SensorMsg1,"%c%d",'p',attitude.p);
        Serial.println(SensorMsg1);
        sprintf(SensorMsg1,"%c%d",'t',attitude.t);
        Serial.println(SensorMsg1);
        Serial.println();
}
