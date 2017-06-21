#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <auv_cal_state_la_2017/HControl.h>
#include <Servo.h>
#include <Wire.h>
#include "MS5837.h"

//Initialization of the Servo's for Blue Robotics Motors 
Servo T1;     //right front
Servo T2;     //right back
Servo T3;     //left front 
Servo T4;     //left back

int  i; 
int  xGyro, yGyro, zGyro, temperature;
int angle_pitch_buffer, angle_roll_buffer;
char string[8];
long loop_timer;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
long xAccl, yAccl, zAccl, acc_total_vector;
float angle_pitch, angle_roll;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;
boolean set_gyro_angles;

MS5837 sensor;
int PWM_Motors;
float dutyCycl;
float assignedDepth;
float feetDepth_read;


//**********************
//Initialize ROS node
//**********************
float bottomDepth = 12;
bool isGoingUp;
bool isGoingDown;

ros::NodeHandle nh;
std_msgs::Float32 currentDepth;

//Testing------------------------
//std_msgs::Float32 ad;

auv_cal_state_la_2017::HControl hControlStatus;


// Publish to Topic height_control_status
// Description: The node will publish the current/received control 
//   status to master whenever it receives any command from topic 
//   height_control.

ros::Publisher hControlPublisher("height_control_status", &hControlStatus);

//Testing-------------------------------------
//ros::Publisher assignedDepthPublisher("assigned_depth", &ad);

// Publish to Topic current_depth
// Description: The node will publish the currentDepth to master 
//   for every loop.

ros::Publisher currentDepthPublisher("current_depth", &currentDepth);


// Subscribe from Topic height_control
// hControl contains two variables: (int)state, (float)depth
// state: the direction (0 = going down, 1 = staying, 2 = going up)
// depth: how far is the sub going
// example: going up 2 ft (state = 0, depth = 2) 
// Description: If depth is -1, it means either all the way up or 
//   all the way down. When state is 1, it either asks the sub to 
//   stay or interrupts the  previous command and force the sub to 
//   stay.Otherwise, the sub will not be interrupt until it finishes 
//   the current command.

void hControlCallback(const auv_cal_state_la_2017::HControl& hControl) {

  char depthChar[6];
  float depth = hControl.depth;  
  dtostrf(depth, 4, 2, depthChar);

  if(hControl.state == 0){  
    if(!isGoingUp && !isGoingDown){
      if(depth == -1 || depth + assignedDepth >= bottomDepth){
        assignedDepth = bottomDepth;
      }else {
        assignedDepth = assignedDepth + depth;
      }
      isGoingDown = true;
      nh.loginfo("Going down...");
      nh.loginfo(depthChar);
      nh.loginfo("ft...(-1 means infinite)");
      hControlStatus.state = 0;
      hControlStatus.depth = depth;
    }else{
      nh.loginfo("Sub is still running. Command abort.");
    }
  }
  else if(hControl.state == 1){
    assignedDepth = feetDepth_read;
    isGoingUp = false;
    isGoingDown = false;
    nh.loginfo("Staying...");
    hControlStatus.state = 1;
    hControlStatus.depth = depth;
  }
  else if(hControl.state == 2){
    if(!isGoingUp && !isGoingDown){
      if(depth == -1 || depth >= assignedDepth){
        assignedDepth = 0;
      }else {
        assignedDepth = assignedDepth - depth;
      } 
      isGoingUp = true;
      nh.loginfo("Going up...");
      nh.loginfo(depthChar);
      nh.loginfo("ft...(-1 means infinite)");
      hControlStatus.state = 2;
      hControlStatus.depth = depth;
    }else{
      nh.loginfo("Sub is still running.Command abort.");
    }
  }
  hControlPublisher.publish(&hControlStatus);
  
}

ros::Subscriber<auv_cal_state_la_2017::HControl> hControlSubscriber("height_control", &hControlCallback);


void setup() {
  
  //For servo motors on Pelican, pins 2-5 are for motors 1-4. PWM on these motors are 1100-1499 (counter
  //clockwise direction) and 1501-1900 (clockwise direction). Note that in code I use pins 6-8, this was used
  //for testing with leds. 
  pinMode(2, OUTPUT); //1 on motor  
  pinMode(3, OUTPUT); //2 on motor
  pinMode(4, OUTPUT); //3 on motor
  pinMode(5, OUTPUT); //4 on motor

 //Pelican Motors activation of motors (initialization of pins to servo motors)
  T1.attach(2); //right front servo
  T1.writeMicroseconds(1500);
  
  T2.attach(3); //right back servo
  T2.writeMicroseconds(1500);
  
  T3.attach(4); //back left servo
  T3.writeMicroseconds(1500);
  
  T4.attach(5); //front left servo
  T4.writeMicroseconds(1500);
  
  delay(1000);
  
  
  Wire.begin();
  Serial.begin(9600);

  
  //*************************
  //Initialize ROS variables
  //*************************
  isGoingUp = false;
  isGoingDown = false;
  
  //Testing------------------
  feetDepth_read = 0;
  
  assignedDepth = feetDepth_read;
  currentDepth.data = feetDepth_read;
  hControlStatus.state = 1;
  hControlStatus.depth = 0;
  
  nh.initNode();
  nh.subscribe(hControlSubscriber);
  nh.advertise(hControlPublisher);
  nh.advertise(currentDepthPublisher);

  //Testing-----------------------
//  nh.advertise(assignedDepthPublisher);

  nh.loginfo("Node initialized. Start gathering data for IMU...");
  //*************************

  setup_mpu_6050_registers();     //Function used to initialize I2C protocol to retrieve data from desired registers                    

  sensor.init();  
  sensor.setFluidDensity(997); // kg/m^3 (997 freshwater, 1029 for seawater)
  feetDepth_read = 0;
  
  for (i = 0; i < 2000; i++){ //Function to add up 2000 readings for X-Z Gyro
    read_mpu_6050_data();
    gyro_x_cal += xGyro;
    gyro_y_cal += yGyro;
    gyro_z_cal += zGyro;
    delay(3);
  }

  //Compute an average of the 2000 readings
  gyro_x_cal /= 2000;                                                  
  gyro_y_cal /= 2000;                                                  
  gyro_z_cal /= 2000;

  
  //*************************
  //Output in ROS: Data is ready.
  //*************************
  nh.loginfo("Data is ready.");
  nh.loginfo("Sub is staying. Waiting to receive data from master...");
//  hControlPublisher.publish(&hControlStatus);
//  currentDepthPublisher.publish(&currentDepth);
  //*************************

  Serial.println(gyro_x_cal);
  loop_timer = micros();                                               //Reset the loop timer

}

void loop() {


  IMUcomputation();

  //*******************************
  //React and leveling with corresponding state
  //*******************************

  //Testing----------------------
  //feetDepth_read =  sensor.depth() * 3.28;                             //1 meter = 3.28 feet
  
  dutyCycl = (abs(assignedDepth - feetDepth_read)/ 12.0);              //function to get a percentage of assigned height to the feet read
  PWM_Motors = dutyCycl * 350;                                         //PWM for motors are between 1500 - 1900; difference is 400 

  //going down
  if (feetDepth_read < assignedDepth){   
    isGoingUp = false;
    goingDownward();
    
    //Testing--------------------------
    feetDepth_read += 0.05;
    
  }
  //going up
  else if (feetDepth_read > assignedDepth){
    isGoingDown = false;   
    goingUpward(); 
    
    //Testing---------------------------
    feetDepth_read -= 0.05;
      
  }
  //staying
  else {   
    isGoingUp = false;
    isGoingDown = false;
    stayLeveling();
  }

  //Update and publish current depth value to master
  currentDepth.data = feetDepth_read;
  currentDepthPublisher.publish(&currentDepth);

  //Testing------------------------
//  ad.data = assignedDepth;
//  assignedDepthPublisher.publish(&ad);
  
  nh.spinOnce();

    
  Serial.println(angle_pitch_output);
  delay(100);
 
    
  while(micros() - loop_timer < 11000);                                 //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
  loop_timer = micros();       
 
  }


//*****************************
//Leveling while staying
//*****************************
void stayLeveling(){
  
  for (i = 0; (2 * i) < 90; i++){ //loop will start from 0 degrees -> 90 degrees 
    //right
    if((angle_roll_output > 2*i) && (angle_roll_output < (2*i + 2))){
      //Boost the right motors
      T2.writeMicroseconds(1500 + i*8);
      T3.writeMicroseconds(1500 - i*8);
      //Downgrade the left motors
      T1.writeMicroseconds(1500 + i*4);
      T4.writeMicroseconds(1500 - i*4);
    }
    //left
    if((angle_roll_output < -1 *(2*i)) && (angle_roll_output > -1 *(2*i + 2))){
      //Boost the left motors
      T1.writeMicroseconds(1500 - i*8);
      T4.writeMicroseconds(1500 + i*8);
      //Downgrade the right motors
      T2.writeMicroseconds(1500 - i*4);
      T3.writeMicroseconds(1500 + i*4);
    }
    //backward
    if((angle_pitch_output > 2*i) && (angle_pitch_output < (2*i + 2))){
      //Boost the back motors
      T3.writeMicroseconds(1500 - i*8);
      T4.writeMicroseconds(1500 + i*8);
      //Downgrade the front motors
      T1.writeMicroseconds(1500 + i*4);
      T2.writeMicroseconds(1500 - i*4);  
    }
    //forward
    if((angle_pitch_output < -1*( 2*i)) && (angle_pitch_output > -1 *(2*i + 2))){
      //Boost the front motors
      T1.writeMicroseconds(1500 - i*8);
      T2.writeMicroseconds(1500 + i*8);
      //Downgrade the back motors
      T3.writeMicroseconds(1500 + i*4);
      T4.writeMicroseconds(1500 - i*4);
    }
  }
  
}


//*****************************
//Going upward
//*****************************
void goingUpward(){
  
  int levelPower = (400 - PWM_Motors) / 45;
  int reversedLevelPower = (PWM_Motors / 45) * (-1);

  for(i = 0; (2 * i) < 90; i++){
    //right
    if((angle_roll_output > 2 * i) && (angle_roll_output < (2 * i + 2))){
      //Boost the right motors
      T2.writeMicroseconds(1500 + PWM_Motors + i * levelPower);
      T3.writeMicroseconds(1500 - PWM_Motors - i * levelPower);
      //Downgrade the left motors
      T1.writeMicroseconds(1500 - PWM_Motors - i * reversedLevelPower);
      T4.writeMicroseconds(1500 + PWM_Motors + i * reversedLevelPower);
    }
    //left
    if((angle_roll_output < -1 *( 2 * i)) && (angle_roll_output > -1 * (2 * i + 2))){
      //Boost the left motors
      T1.writeMicroseconds(1500 - PWM_Motors - i * levelPower);
      T4.writeMicroseconds(1500 + PWM_Motors + i * levelPower);
      //Downgrade the right motors
      T2.writeMicroseconds(1500 + PWM_Motors + i * reversedLevelPower);
      T3.writeMicroseconds(1500 - PWM_Motors - i * reversedLevelPower);
    }
    //backward
    if((angle_pitch_output > 2 * i) && (angle_pitch_output < (2 * i + 2))){ 
      //Boost the back motors
      T3.writeMicroseconds(1500 - PWM_Motors - i * levelPower);
      T4.writeMicroseconds(1500 + PWM_Motors + i * levelPower);
      //Downgrade the front motors
      T1.writeMicroseconds(1500 - PWM_Motors - i * reversedLevelPower);
      T2.writeMicroseconds(1500 + PWM_Motors + i * reversedLevelPower);  
    }
    //forward
    if((angle_pitch_output < -1 * (2 * i)) && (angle_pitch_output > -1 * (2 * i + 2))){
      //Boost the front motors
      T1.writeMicroseconds(1500 - PWM_Motors - i * levelPower);
      T2.writeMicroseconds(1500 + PWM_Motors + i * levelPower);
      //Downgrade the back motors
      T3.writeMicroseconds(1500 - PWM_Motors - i * reversedLevelPower);
      T4.writeMicroseconds(1500 + PWM_Motors + i * reversedLevelPower);
    }
  }
    
}

//*****************************
//Going downward
//*****************************
void goingDownward(){
  
  PWM_Motors = -PWM_Motors;
  int levelPower = ((400 + PWM_Motors) / 45) * (-1);
  int reversedLevelPower = ((-1) * PWM_Motors) / 45;

  for (i = 0; (2 * i) < 90; i++){ //loop will start from 0 degrees -> 90 degrees 
    //right
    if((angle_roll_output > 2*i) && (angle_roll_output < (2*i + 2))){
      //Boost the left motors
      T1.writeMicroseconds(1500 - PWM_Motors - i * levelPower);
      T4.writeMicroseconds(1500 + PWM_Motors + i * levelPower);
      //Downgrade the right motors
      T2.writeMicroseconds(1500 + PWM_Motors + i * reversedLevelPower);
      T3.writeMicroseconds(1500 - PWM_Motors - i * reversedLevelPower);       
    }
    //left
    if((angle_roll_output < -1 *(2*i)) && (angle_roll_output > -1 *(2*i + 2))){
      //Boost the right motors
      T2.writeMicroseconds(1500 + PWM_Motors + i * levelPower);
      T3.writeMicroseconds(1500 - PWM_Motors - i * levelPower);
      //Downgrade the left motors
      T1.writeMicroseconds(1500 - PWM_Motors - i * reversedLevelPower);
      T4.writeMicroseconds(1500 + PWM_Motors + i * reversedLevelPower);     
    }
    //backward
    if((angle_pitch_output > 2*i) && (angle_pitch_output < (2*i + 2))){
      //Boost the front motors
      T1.writeMicroseconds(1500 - PWM_Motors - i * levelPower);
      T2.writeMicroseconds(1500 + PWM_Motors + i * levelPower); 
      //Downgrade the back motors
      T3.writeMicroseconds(1500 - PWM_Motors - i * reversedLevelPower);
      T4.writeMicroseconds(1500 + PWM_Motors + i * reversedLevelPower);       
    }
    //forward
    if((angle_pitch_output < -1*( 2*i)) && (angle_pitch_output > -1 *(2*i + 2))){
      //Boost the back motors
      T3.writeMicroseconds(1500 - PWM_Motors - i * levelPower);
      T4.writeMicroseconds(1500 + PWM_Motors + i * levelPower);
      //Downgrade the front motors
      T1.writeMicroseconds(1500 - PWM_Motors - i * reversedLevelPower);
      T2.writeMicroseconds(1500 + PWM_Motors + i * reversedLevelPower);      
    }
  }
    
}


//*****************************
//IMU computation
//*****************************
void IMUcomputation(){
  
  read_mpu_6050_data();                                              //Read the raw data from the gyroscope
  
  xGyro -= gyro_x_cal;                                               //Calibration to set xGyro to be 0
  yGyro -= gyro_y_cal;                                               //Calibration to set yGyro to be 0
  zGyro -= gyro_z_cal;                                               //Calibration to set zGyro to be 0

  //Gyro angle calculations
  

  angle_pitch += xGyro * 0.000169;                                  //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_roll  += yGyro * 0.000169;                                  //Calculate the traveled roll angle and add this to the angle_roll variable
  
  angle_pitch += angle_roll  * sin(zGyro * 0.000002948);             //If the IMU has yawed transfer the roll angle to the pitch angle
  angle_roll  -= angle_pitch * sin(zGyro * 0.000002948);            //If the IMU has yawed transfer the pitch angle to the roll angle
   
   //Accelerometer angle calculations
  acc_total_vector = sqrt((xAccl*xAccl)+(yAccl*yAccl)+(zAccl*zAccl));  //Calculate the total accelerometer vector (Magnitude of the 3 dimensional vector)
 
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  angle_pitch_acc = asin((float)yAccl/acc_total_vector)* 57.296;       //Calculate the pitch angle
  angle_roll_acc = asin((float)xAccl/acc_total_vector)* -57.296;       //Calculate the roll angle
  
  //Place the MPU-6050 spirit level(flat) and note the values in the following two lines for calibration
  angle_pitch_acc -= -0.87;                                            //Accelerometer calibration value for pitch
  angle_roll_acc -= -3.8;                                              //Accelerometer calibration value for roll

  if(set_gyro_angles){                                                 //If the IMU is already started
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
  }
  else{                                                                //At first start
    angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle 
    angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle 
    set_gyro_angles = true;                                            //Set the IMU started flag
  }
  
  //To dampen the pitch and roll angles a complementary filter is used
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value

}

void read_mpu_6050_data(){

  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68,14);                                           //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);                                        //Wait until all the bytes are received
  xAccl = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_x variable
  yAccl = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_y variable
  zAccl = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_z variable
  temperature = Wire.read()<<8|Wire.read();                            //Add the low and high byte to the temperature variable
  xGyro = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the gyro_x variable
  yGyro = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the gyro_y variable
  zGyro = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the gyro_z variable

  
}

void setup_mpu_6050_registers(){
 
 //Activate the MPU-6050
  Wire.beginTransmission(0x68);                       //Start communicating with the MPU-6050
  Wire.write(0x6B);                                   //Send the requested starting register
  Wire.write(0x00);                                   //Set the requested starting register
  Wire.endTransmission();                             //End the transmission
  
  //Accelerometer Configuration
  Wire.beginTransmission (0x68);                     
  Wire.write(0x1C);                                   //Configuration of the Accelerometer
  Wire.write(0x10);                                   //Using FS_SEL = 2 (plus/minus 8g)
  Wire.endTransmission();                             //End I2C transmission
  
  //Gyroscope Configuration
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);                                   //Configuration of the Gyroscope
  Wire.write(0x08);                                   //Using FS_SEL = 1 (500 dps) && (65.5 LSB/ (dps)
  Wire.endTransmission();                             //End I2C transmission
 
}

