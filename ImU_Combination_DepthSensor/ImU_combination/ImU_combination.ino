/*****************************************************************
LSM9DS0_AHRS.ino
SFE_LSM9DS0 Library AHRS Data Fusion Example Code
Jim Lindblom @ SparkFun Electronics
Original Creation Date: February 18, 2014
https://github.com/sparkfun/LSM9DS0_Breakout

Modified by Kris Winer, April 4, 2014

The LSM9DS0 is a versatile 9DOF sensor. It has a built-in
accelerometer, gyroscope, and magnetometer. Very cool! Plus it
functions over either SPI or I2C.

This Arduino sketch utilizes Jim Lindblom's SFE_LSM9DS0 library to generate the basic sensor data
for use in two sensor fusion algorithms becoming increasingly popular with DIY quadcopter and robotics engineers. 

Like the original LSM9SD0_simple.ino sketch, it'll demo the following:
* How to create a LSM9DS0 object, using a constructor (global
  variables section).
* How to use the begin() function of the LSM9DS0 class.
* How to read the gyroscope, accelerometer, and magnetometer
  using the readGryo(), readAccel(), readMag() functions and the
  gx, gy, gz, ax, ay, az, mx, my, and mz variables.
* How to calculate actual acceleration, rotation speed, magnetic
  field strength using the calcAccel(), calcGyro() and calcMag()
  functions.
  
In addition, the sketch will demo:
* How to check for data updates using interrupts
* How to display output at a rate different from the sensor data update and fusion filter update rates
* How to specify the accelerometer anti-aliasing (low-pass) filter rate
* How to use the data from the LSM9DS0 to fuse the sensor data into a quaternion representation of the sensor frame
  orientation relative to a fixed Earth frame providing absolute orientation information for subsequent use.
* An example of how to use the quaternion data to generate standard aircraft orientation data in the form of
  Tait-Bryan angles representing the sensor yaw, pitch, and roll angles suitable for any vehicle stablization control application.

Hardware setup: This library supports communicating with the
LSM9DS0 over either I2C or SPI. If you're using I2C, these are
the only connections that need to be made:
  LSM9DS0 --------- Arduino
   SCL ---------- SCL (A5 on older 'Duinos')
   SDA ---------- SDA (A4 on older 'Duinos')
   VDD ------------- 3.3V
   GND ------------- GND
         DRDYG-------------4   (gyro data ready interrupt, can be any digital pin)
         INTX1-------------3   (accelerometer data ready interrupt, can be any digital pin)
         INTX2-------------2   (magnetometer data ready interrupt, can be any digital pin)
(CSG, CSXM, SDOG, and SDOXM should all be pulled high jumpers on 
  the breakout board will do this for you.)
  
 Note: The LSM9DS0 in the I2C mode uses the Arduino Wire library. 
 Because the sensor is not 5V tolerant, we are using a 3.3 V 8 MHz Pro Mini or a 3.3 V Teensy 3.1.
 We have disabled the internal pull-ups used by the Wire library in the Wire.h/twi.c utility file.
 We are also using the 400 kHz fast I2C mode by setting the TWI_FREQ  to 400000L /twi.h utility file.
  
If you're using SPI, here is an example hardware setup:
  LSM9DS0 --------- Arduino
          CSG -------------- 9
          CSXM ------------- 10
          SDOG ------------- 12
          SDOXM ------------ 12 (tied to SDOG)
          SCL -------------- 13
          SDA -------------- 11
          VDD -------------- 3.3V
          GND -------------- GND
  
The LSM9DS0 has a maximum voltage of 3.6V. Make sure you power it
off the 3.3V rail! And either use level shifters between SCL
and SDA or just use a 3.3V Arduino Pro.   

In addition, this sketch uses a Nokia 5110 48 x 84 pixel display which requires 
digital pins 5 - 9 described below. If using SPI you might need to press one of the A0 - A3 pins
into service as a digital input instead.

Development environment specifics:
  IDE: Arduino 1.0.5
  Hardware Platform: Arduino Pro 3.3V/8MHz
  LSM9DS0 Breakout Version: 1.0

This code is beerware. If you see me (or any other SparkFun 
employee) at the local, and you've found our code helpful, please 
buy us a round!

Distributed as-is; no warranty is given.
*****************************************************************/

#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <Wire.h>
#include "MS5837.h"
#include "SFE_LSM9DS0.h"

#define GyroMeasError PI * (40.0f / 180.0f)       // gyroscope measurement error in rads/s (shown as 3 deg/s)
#define GyroMeasDrift PI * (0.0f / 180.0f)      // gyroscope measurement drift in rad/s/s (shown as 0.0 deg/s/s)
#define beta sqrt(3.0f / 4.0f) * GyroMeasError   // compute beta
#define zeta sqrt(3.0f / 4.0f) * GyroMeasDrift   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

#define LSM9DS0_XM  0x1D 
#define LSM9DS0_G   0x6B 
LSM9DS0 dof(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);

const byte INT1XM = 53; // INT1XM tells us when accel data is ready
const byte INT2XM = 51; // INT2XM tells us when mag data is ready
const byte DRDYG  = 49; // DRDYG  tells us when gyro data is ready

//Initialization of the Servo's for Blue Robotics Motors 
Servo T1;     //right front
Servo T2;     //right back
Servo T3;     //left front 
Servo T4;     //left back
Servo T5;     //right front
Servo T6;     //right back
Servo T7;     //left front 
Servo T8;     //left back

MS5837 sensor;

//CHANGED PWM_Motors TO PWM_Motors_depth SINCE THERE ARE 2 DIFFERENT PWM CALCULATIONS
//ONE IS FOR DEPTH AND THE OTHER IS USED FOR MOTORS TO ROTATE TO PROPER NEW LOCATION
int PWM_Motors_Depth;
float dutyCycl_depth;
float assignedDepth;
float feetDepth_read;

//initializations for IMU
float pitch, yaw, roll, heading;
float deltat = 0.0f;        // integration interval for both filter schemes
uint32_t count = 0;         // used to control display output rate
uint32_t delt_t = 0;        // used to control display output rate
uint32_t lastUpdate = 0;    // used to calculate integration interval
uint32_t Now = 0;           // used to calculate integration interval

int i;
int PWM_Motors_orient;
float abias[3] = {0, 0, 0}, gbias[3] = {0, 0, 0};
float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method
float temperature;
float dutyCycl_orient;
float assignedYaw;

//Initialize ROS node
const float rotationUpperBound = 166.2;
const float rotationLowerBound = -193.8;
const float bottomDepth = 12;
int mControlDirection;
float mControlPower;
float mControlDistance;
float frontCamForwardDistance;
float frontCamHorizontalDistance;
float frontCamVerticalDistance;
float bottomCamForwardDistance;
float bottomCamHorizontalDistance;
float bottomCamVerticalDistance;
float centerTimer;
bool isGoingUp;
bool isGoingDown;
bool isTurningRight;
bool isTurningLeft;
bool keepTurningRight;
bool keepTurningLeft;
bool mControlMode1;
bool mControlMode2;
bool mControlMode3;
bool mControlMode4;
bool keepMovingForward;
bool keepMovingRight;
bool keepMovingBackward;
bool keepMovingLeft;

void setup()
{  
  //For servo motors on Pelican, pins 2-5 are for motors 1-4. PWM on these motors are 1100-1499 (counter
  //clockwise direction) and 1501-1900 (clockwise direction). Note that in code I use pins 6-8, this was used
  //for testing with leds. 
  feetDepth_read = 2; 
  assignedDepth = 2; 
  
  pinMode(INT1XM, INPUT);
  pinMode(INT2XM, INPUT);
  pinMode(DRDYG,  INPUT);
  pinMode(2, OUTPUT); //1 on motor  
  pinMode(3, OUTPUT); //2 on motor
  pinMode(4, OUTPUT); //3 on motor
  pinMode(5, OUTPUT); //4 on motor
  pinMode(6, OUTPUT); //5 on motor
  pinMode(7, OUTPUT); //6 on motor
  pinMode(8, OUTPUT); //7 on motor
  pinMode(9, OUTPUT); //8 on motor
  
 //Pelican Motors activation of motors (initialization of pins to servo motors)
  T1.attach(2); //right front servo
  T1.writeMicroseconds(1500);  
  T2.attach(3); //right back servo
  T2.writeMicroseconds(1500);  
  T3.attach(4); //back left servo
  T3.writeMicroseconds(1500);
  T4.attach(5); //front left servo
  T4.writeMicroseconds(1500);
  T5.attach(6); //front left servo
  T5.writeMicroseconds(1500);
  T6.attach(7); //front left servo
  T6.writeMicroseconds(1500);
  T7.attach(8); //front left servo
  T7.writeMicroseconds(1500);
  T8.attach(9); //front left servo
  T8.writeMicroseconds(1500);
  
  delay(1000);
  
  
  Wire.begin();
  Serial.begin(38400);


  initializeIMU();                    

  sensor.init();  
  sensor.setFluidDensity(997); // kg/m^3 (997 freshwater, 1029 for seawater)
  
}

void loop()
{
  gettingRawData();

  //Timer
  Now = micros();
  deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;
  // Sensors x- and y-axes are aligned but magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
  // This is ok by aircraft orientation standards!  
  // Pass gyro rate as rad/s
  MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, mx, my, mz);
  
    yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    pitch *= 180.0f / PI;
    yaw   *= 180.0f / PI; 
    yaw   -= 13.8; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    roll  *= 180.0f / PI;
  
  
  //Depth
  //Testing----------------------
  feetDepth_read =  sensor.depth() * 3.28;                                   //1 meter = 3.28 feet  
  dutyCycl_depth = (abs(assignedDepth - feetDepth_read)/ 13.0);              //function to get a percentage of assigned height to the feet read
  PWM_Motors_Depth = dutyCycl_depth * 400;                                   //PWM for motors are between 1500 - 1900; difference is 400 

  //Rotation
  //duty cycle and PWM calculation for orientation
  //dutyCycl_orient = degreeToTurn() / 360.0; //Warning: the return value from degreeToTurn is from 0 to 180
  //  PWM_Motors_orient = dutyCycl * 400;
  //****************************NEED TO CHECK WITH ERICK!!
  //PWM_Motors_orient = dutyCycl_orient * 400 / 2; //Maximum is 200

  //Apply on Motors
  heightControl();
  //rotationControl();
  //movementControl();
    
  count = millis(); 
  Serial.println(yaw);

  delay(10);   
}

void heightControl(){
  
  //Going down
  if (feetDepth_read < assignedDepth - 0.1){   
    goingDownward();
    
    //Testing--------------------------
   // feetDepth_read += 0.001;
    
  }  
  //Going up
  else if (feetDepth_read > assignedDepth + 0.1){ 
    goingUpward(); 
    
    //Testing---------------------------
    //feetDepth_read -= 0.001;
      
  } 
  //Staying
  else {   
    stayLeveling();
  }
  
}

void stayLeveling(){
  float high = 4;
  float low = 2;
  for (i = 0; (2 * i) < 90; i++){ //loop will start from 0 degrees -> 90 degrees 
    //rolled left (positive value)
    if((roll > 2*i) && (roll < (2*i + 2))){
      //Boost the left motors
      T1.writeMicroseconds(1500 + i*high);
      T2.writeMicroseconds(1500 - i*high);
      //Downgrade the right motors
      T3.writeMicroseconds(1500 + i*low);
      T4.writeMicroseconds(1500 + i*low);
    }
    //rolled right (negative value)
    if((roll < -1 *(2*i)) && (roll > -1 *(2*i + 2))){
      //Boost the right motors
      T3.writeMicroseconds(1500 - i*high);
      T4.writeMicroseconds(1500 + i*high);
      //Downgrade the left motors
      T1.writeMicroseconds(1500 - i*low);
      T2.writeMicroseconds(1500 + i*low);
    }
    //pitched back (positive value)
    if((pitch > 2*i) && (pitch < (2*i + 2))){
      //Boost the back motors
      T1.writeMicroseconds(1500 - i*high);
      T4.writeMicroseconds(1500 + i*high);
      //Downgrade the front motors
      T2.writeMicroseconds(1500 + i*low);
      T3.writeMicroseconds(1500 - i*low);  
    }
    //pitched forward (negative value)
    if((pitch < -1*( 2*i)) && (pitch > -1 *(2*i + 2))){
      //Boost the front motors
      T2.writeMicroseconds(1500 - i*high);
      T3.writeMicroseconds(1500 + i*high);
      //Downgrade the back motors
      T1.writeMicroseconds(1500 + i*low);
      T4.writeMicroseconds(1500 - i*low);
    }
  }
  
}

void goingUpward(){
  
  int levelPower = (400 - PWM_Motors_Depth) / 45;
  int reversedLevelPower = (PWM_Motors_Depth / 45) * (-1);

  for(i = 0; (2 * i) < 90; i++){
    //right
    if((roll > 2 * i) && (roll < (2 * i + 2))){
      //Boost the right motors
      T2.writeMicroseconds(1500 + PWM_Motors_Depth + i * levelPower);
      T3.writeMicroseconds(1500 - PWM_Motors_Depth - i * levelPower);
      //Downgrade the left motors
      T1.writeMicroseconds(1500 - PWM_Motors_Depth - i * reversedLevelPower);
      T4.writeMicroseconds(1500 + PWM_Motors_Depth + i * reversedLevelPower);
    }
    //left
    if((roll < -1 *( 2 * i)) && (roll > -1 * (2 * i + 2))){
      //Boost the left motors
      T1.writeMicroseconds(1500 - PWM_Motors_Depth - i * levelPower);
      T4.writeMicroseconds(1500 + PWM_Motors_Depth + i * levelPower);
      //Downgrade the right motors
      T2.writeMicroseconds(1500 + PWM_Motors_Depth + i * reversedLevelPower);
      T3.writeMicroseconds(1500 - PWM_Motors_Depth - i * reversedLevelPower);
    }
    //backward
    if((pitch > 2 * i) && (pitch < (2 * i + 2))){ 
      //Boost the back motors
      T3.writeMicroseconds(1500 - PWM_Motors_Depth - i * levelPower);
      T4.writeMicroseconds(1500 + PWM_Motors_Depth + i * levelPower);
      //Downgrade the front motors
      T1.writeMicroseconds(1500 - PWM_Motors_Depth - i * reversedLevelPower);
      T2.writeMicroseconds(1500 + PWM_Motors_Depth + i * reversedLevelPower);  
    }
    //forward
    if((pitch < -1 * (2 * i)) && (pitch > -1 * (2 * i + 2))){
      //Boost the front motors
      T1.writeMicroseconds(1500 - PWM_Motors_Depth - i * levelPower);
      T2.writeMicroseconds(1500 + PWM_Motors_Depth + i * levelPower);
      //Downgrade the back motors
      T3.writeMicroseconds(1500 - PWM_Motors_Depth - i * reversedLevelPower);
      T4.writeMicroseconds(1500 + PWM_Motors_Depth + i * reversedLevelPower);
    }
  }
    
}

void goingDownward(){
  
  PWM_Motors_Depth = -PWM_Motors_Depth;
  int levelPower = ((200 + PWM_Motors_Depth) / 45) * (-1);
  int reversedLevelPower = ((-1) * PWM_Motors_Depth) / 45;

  for (i = 0; (2 * i) < 90; i++){ //loop will start from 0 degrees -> 90 degrees 
    //right
    if((roll > 2*i) && (roll < (2*i + 2))){
      //Boost the left motors
      T1.writeMicroseconds(1500 + PWM_Motors_Depth - i * levelPower);
      T4.writeMicroseconds(1500 - PWM_Motors_Depth + i * levelPower);
      //Downgrade the right motors
      T2.writeMicroseconds(1500 - PWM_Motors_Depth + i * reversedLevelPower);
      T3.writeMicroseconds(1500 + PWM_Motors_Depth - i * reversedLevelPower);       
    }
    //left
    if((roll < -1 *(2*i)) && (roll > -1 *(2*i + 2))){
      //Boost the right motors
      T2.writeMicroseconds(1500 - PWM_Motors_Depth + i * levelPower);
      T3.writeMicroseconds(1500 + PWM_Motors_Depth - i * levelPower);
      //Downgrade the left motors
      T1.writeMicroseconds(1500 + PWM_Motors_Depth - i * reversedLevelPower);
      T4.writeMicroseconds(1500 - PWM_Motors_Depth + i * reversedLevelPower);     
    }
    //backward
    if((pitch > 2*i) && (pitch < (2*i + 2))){
      //Boost the front motors
      T1.writeMicroseconds(1500 + PWM_Motors_Depth - i * levelPower);
      T2.writeMicroseconds(1500 - PWM_Motors_Depth + i * levelPower); 
      //Downgrade the back motors
      T3.writeMicroseconds(1500 + PWM_Motors_Depth - i * reversedLevelPower);
      T4.writeMicroseconds(1500 - PWM_Motors_Depth + i * reversedLevelPower);       
    }
    //forward
    if((pitch < -1*( 2*i)) && (pitch > -1 *(2*i + 2))){
      //Boost the back motors
      T3.writeMicroseconds(1500 + PWM_Motors_Depth - i * levelPower);
      T4.writeMicroseconds(1500 - PWM_Motors_Depth + i * levelPower);
      //Downgrade the front motors
      T1.writeMicroseconds(1500 + PWM_Motors_Depth - i * reversedLevelPower);
      T2.writeMicroseconds(1500 - PWM_Motors_Depth + i * reversedLevelPower);      
    }
  }
    
}
void initializeIMU(){
  
  uint32_t status = dof.begin();
  delay(2000); 

  // Set data output ranges; choose lowest ranges for maximum resolution
  // Accelerometer scale can be: A_SCALE_2G, A_SCALE_4G, A_SCALE_6G, A_SCALE_8G, or A_SCALE_16G   
  dof.setAccelScale(dof.A_SCALE_2G);
  // Gyro scale can be:  G_SCALE__245, G_SCALE__500, or G_SCALE__2000DPS
  dof.setGyroScale(dof.G_SCALE_245DPS);
  // Magnetometer scale can be: M_SCALE_2GS, M_SCALE_4GS, M_SCALE_8GS, M_SCALE_12GS   
  dof.setMagScale(dof.M_SCALE_2GS);
  
  // Set output data rates  
  // Accelerometer output data rate (ODR) can be: A_ODR_3125 (3.225 Hz), A_ODR_625 (6.25 Hz), A_ODR_125 (12.5 Hz), A_ODR_25, A_ODR_50, 
  //                                              A_ODR_100,  A_ODR_200, A_ODR_400, A_ODR_800, A_ODR_1600 (1600 Hz)
  dof.setAccelODR(dof.A_ODR_200); // Set accelerometer update rate at 100 Hz
  // Accelerometer anti-aliasing filter rate can be 50, 194, 362, or 763 Hz
  // Anti-aliasing acts like a low-pass filter allowing oversampling of accelerometer and rejection of high-frequency spurious noise.
  // Strategy here is to effectively oversample accelerometer at 100 Hz and use a 50 Hz anti-aliasing (low-pass) filter frequency
  // to get a smooth ~150 Hz filter update rate
  dof.setAccelABW(dof.A_ABW_50); // Choose lowest filter setting for low noise
 
  // Gyro output data rates can be: 95 Hz (bandwidth 12.5 or 25 Hz), 190 Hz (bandwidth 12.5, 25, 50, or 70 Hz)
  //                                 380 Hz (bandwidth 20, 25, 50, 100 Hz), or 760 Hz (bandwidth 30, 35, 50, 100 Hz)
  dof.setGyroODR(dof.G_ODR_190_BW_125);  // Set gyro update rate to 190 Hz with the smallest bandwidth for low noise

  // Magnetometer output data rate can be: 3.125 (ODR_3125), 6.25 (ODR_625), 12.5 (ODR_125), 25, 50, or 100 Hz
  dof.setMagODR(dof.M_ODR_125); // Set magnetometer to update every 80 ms
    
  // Use the FIFO mode to average accelerometer and gyro readings to calculate the biases, which can then be removed from
  // all subsequent measurements.
  dof.calLSM9DS0(gbias, abias);
  
}
  
void gettingRawData(){
  
  if(digitalRead(DRDYG)) {  // When new gyro data is ready
  dof.readGyro();           // Read raw gyro data
    gx = dof.calcGyro(dof.gx) - gbias[0];   // Convert to degrees per seconds, remove gyro biases
    gy = dof.calcGyro(dof.gy) - gbias[1];
    gz = dof.calcGyro(dof.gz) - gbias[2];
  }
  
  if(digitalRead(INT1XM)) {  // When new accelerometer data is ready
    dof.readAccel();         // Read raw accelerometer data
    ax = dof.calcAccel(dof.ax) - abias[0];   // Convert to g's, remove accelerometer biases
    ay = dof.calcAccel(dof.ay) - abias[1];
    az = dof.calcAccel(dof.az) - abias[2];
  }
  
  if(digitalRead(INT2XM)) {  // When new magnetometer data is ready
    dof.readMag();           // Read raw magnetometer data
    mx = dof.calcMag(dof.mx);     // Convert to Gauss and correct for calibration
    my = dof.calcMag(dof.my);
    mz = dof.calcMag(dof.mz);
    
    dof.readTemp();
    temperature = 21.0 + (float) dof.temperature/8.; // slope is 8 LSB per degree C, just guessing at the intercept
  }
  
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
