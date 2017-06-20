#include <ros.h>
#include <std_msgs/Int32.h>
#include <Servo.h>
#include <Wire.h>

//Initialization of the Servo's for Blue Robotics Motors 
Servo T1;     //right front
Servo T2;     //right back
Servo T3;     //left front 
Servo T4;     //left back

int  i; 
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
int  xGyro, yGyro, zGyro, temperature;
long xAccl, yAccl, zAccl, acc_total_vector;
long loop_timer;
float angle_pitch, angle_roll;
char string[8];
int angle_pitch_buffer, angle_roll_buffer;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;


//**********************
//Initialize ROS node
//**********************
bool staying;
bool goingUp;
bool goingDown;
bool changeState;
int currentState;

ros::NodeHandle nh;

void motorStateCallback(const std_msgs::Int32& state) {
  if(currentState != state.data){
    changeState = true;
  }
  if(changeState){
    currentState = state.data;
    changeState = false;
    if(state.data == 0){
      goingDown = true;
      staying = false;
      goingUp = false;
      nh.loginfo("Going down...");
    }
    else if(state.data == 1){
      goingDown = false;
      staying = true;
      goingUp = false;
      nh.loginfo("Staying...");
    }
    else if(state.data == 2){
      goingDown = false;
      staying = false;
      goingUp = true;
      nh.loginfo("Going up...");
    }
  }
}

ros::Subscriber<std_msgs::Int32> stateSubscriber("motor_state", &motorStateCallback);
//***********************


void setup() {
  
  //For servo motors on Pelican, pins 2-5 are for motors 1-4. PWM on these motors are 1100-1499 (counter
  //clockwise direction) and 1501-1900 (clockwise direction). Note that in code I use pins 6-8, this was used
  //for testing with leds. 
  pinMode(2, OUTPUT); //1 on motor  
  pinMode(3, OUTPUT); //2 on motor
  pinMode(4, OUTPUT); //3 on motor
  pinMode(5, OUTPUT); //4 on motor
  /*pinMode(6, OUTPUT); //1 on motor reverse 
  pinMode(7, OUTPUT); //2 on motor reverse
  pinMode(8, OUTPUT); //3 on motor reverse
  pinMode(9, OUTPUT);//4 on motor reverse*/

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
  staying = true; //Start with staying mode
  goingDown = false;
  goingUp = false;
  changeState = false;
  currentState = 1;

  nh.initNode();
  nh.subscribe(stateSubscriber);
  nh.loginfo("Node initialized. Start gathering data for IMU...");
  //*************************

  setup_mpu_6050_registers();     //Function used to initialize I2C protocol to retrieve data from desired registers                    
  
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
  nh.loginfo("Sub is staying. Waiting to receive data from other nodes...");
  //*************************

  Serial.println(gyro_x_cal);
  loop_timer = micros();                                               //Reset the loop timer

}

void loop() {
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
  angle_pitch_acc -= -0.87;                                             //Accelerometer calibration value for pitch
  angle_roll_acc -= -3.8;                                               //Accelerometer calibration value for roll

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


  //*******************************
  //React with corresponding state
  //*******************************
  if(goingDown){
    goingDownward();
  }
  else if(staying){
    stayLeveling();
  }
  else if(goingUp){
    goingUpward();
  }
  nh.spinOnce();
  //*******************************

    
  Serial.println(angle_pitch_output);
  delay(100); //******************change from 3 to 100, needs to confirm with Erick
 
    
  while(micros() - loop_timer < 11000);                                 //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
  loop_timer = micros();       
 
  }

//*****************************
//Leveling while staying
//*****************************
void stayLeveling(){
  //Note that for PWM on arduino, it ranges from 0 - 255. The loop only executing 45 times, dividing 255 by 45 yields 5. This will increment the PWM by 5 every 2 degrees. 
  //**************************
  //Warning::Data might be overwrited
  //**************************
  for (i = 0; (2 * i) < 90; i++){ //loop will start from 0 degrees -> 90 degrees 
    //right
    if((angle_roll_output > 2*i) && (angle_roll_output < (2*i + 2))){ //check the roll to the right (positive degree)
      //analogWrite(2, i*5);     //turn on right motors to 5 PWM ~ 2% duty cycle
      //analogWrite(3, i*5);
      //analogWrite(8, i*2);     //turn the left motors in reverse to 2 PWM ~ 1% duty cycle (half of duty cycle from right side)
      //analogWrite(9, i*2);
      T2.writeMicroseconds(1500 + i*8);
      T3.writeMicroseconds(1500 - i*8);
      T1.writeMicroseconds(1500 + i*4);
      T4.writeMicroseconds(1500 - i*4);
    }
    //left
    if((angle_roll_output < -1 *(2*i)) && (angle_roll_output > -1 *(2*i + 2))){//check the roll to the left (negative degree)
      /*analogWrite(4, i*5);       //turn on left motors to 5 PWM ~ 2% duty cycle
      analogWrite(5 ,i*5);
      analogWrite(6, i*2);       //turn the right motors in reverse to 2 PWM ~ 1% duty cycle (half of duty cycle from right side)
      analogWrite(7, i*2);*/
      T1.writeMicroseconds(1500 - i*8);
      T4.writeMicroseconds(1500 + i*8);
      T2.writeMicroseconds(1500 - i*4);
      T3.writeMicroseconds(1500 + i*4);
    }
    //backward
    if((angle_pitch_output > 2*i) && (angle_pitch_output < (2*i + 2))){
      /*analogWrite(3, i*5);       //turn on back motors to 5 PWM ~ 2% duty cycle
      analogWrite(5 ,i*5);
      analogWrite(6, i*2);       //turn the front motors in reverse to 2 PWM ~ 1% duty cycle (half of duty cycle from right side)
      analogWrite(8, i*2);  */   
      T3.writeMicroseconds(1500 - i*8);
      T4.writeMicroseconds(1500 + i*8);
      T1.writeMicroseconds(1500 + i*4);
      T2.writeMicroseconds(1500 - i*4);  
    }
    //forward
    if((angle_pitch_output < -1*( 2*i)) && (angle_pitch_output > -1 *(2*i + 2))){
      /*analogWrite(2, i*5);       //turn on front motors to 5 PWM ~ 2% duty cycle
      analogWrite(4 ,i*5);
      analogWrite(7, i*2);       //turn the back motors in reverse to 2 PWM ~ 1% duty cycle (half of duty cycle from right side)
      analogWrite(9, i*2);*/
      T1.writeMicroseconds(1500 - i*8);
      T2.writeMicroseconds(1500 + i*8);
      T3.writeMicroseconds(1500 + i*4);
      T4.writeMicroseconds(1500 - i*4);
    }
  }
}

//*****************************
//Going upward
//*****************************
void goingUpward(){
  //Initialize the power variables
  int power = 100;
  int levelPower = (400 - power) / 45;
  int reversedLevelPower = (power / 45) * (-1);

  //**************************
  //Warning::Data might be overwrited
  //**************************
  for (i = 0; (2 * i) < 90; i++){
    //right
    if((angle_roll_output > 2*i) && (angle_roll_output < (2*i + 2))){
      //Boost the right motors
      T2.writeMicroseconds(1500 + power + i * levelPower);
      T3.writeMicroseconds(1500 - power - i * levelPower);
      //Downgrade the left motors
      T1.writeMicroseconds(1500 - power - i * reversedLevelPower);
      T4.writeMicroseconds(1500 + power + i * reversedLevelPower);
    }
    //left
    if((angle_roll_output < -1 *(2*i)) && (angle_roll_output > -1 *(2*i + 2))){
      //Boost the left motors
      T1.writeMicroseconds(1500 - power - i * levelPower);
      T4.writeMicroseconds(1500 + power + i * levelPower);
      //Downgrade the right motors
      T2.writeMicroseconds(1500 + power + i * reversedLevelPower);
      T3.writeMicroseconds(1500 - power - i * reversedLevelPower);
    }
    //backward
    if((angle_pitch_output > 2*i) && (angle_pitch_output < (2*i + 2))){ 
      //Boost the back motors
      T3.writeMicroseconds(1500 - power - i * levelPower);
      T4.writeMicroseconds(1500 + power + i * levelPower);
      //Downgrade the front motors
      T1.writeMicroseconds(1500 - power - i * reversedLevelPower);
      T2.writeMicroseconds(1500 + power + i * reversedLevelPower);  
    }
    //forward
    if((angle_pitch_output < -1*( 2*i)) && (angle_pitch_output > -1 *(2*i + 2))){
      //Boost the front motors
      T1.writeMicroseconds(1500 - power - i * levelPower);
      T2.writeMicroseconds(1500 + power + i * levelPower);
      //Downgrade the back motors
      T3.writeMicroseconds(1500 - power - i * reversedLevelPower);
      T4.writeMicroseconds(1500 + power + i * reversedLevelPower);
    }
  }
}

//*****************************
//Going downward
//*****************************
void goingDownward(){
  //Initialize the power variables
  int power = -100;
  int levelPower = ((400 + power) / 45) * (-1);
  int reversedLevelPower = ((-1) * power) / 45;
   
  //**************************
  //Warning::Data might be overwrited
  //**************************
  for (i = 0; (2 * i) < 90; i++){ //loop will start from 0 degrees -> 90 degrees 
    //right
    if((angle_roll_output > 2*i) && (angle_roll_output < (2*i + 2))){
      //Boost the left motors
      T1.writeMicroseconds(1500 - power - i * levelPower);
      T4.writeMicroseconds(1500 + power + i * levelPower);
      //Downgrade the right motors
      T2.writeMicroseconds(1500 + power + i * reversedLevelPower);
      T3.writeMicroseconds(1500 - power - i * reversedLevelPower);       
    }
    //left
    if((angle_roll_output < -1 *(2*i)) && (angle_roll_output > -1 *(2*i + 2))){
      //Boost the right motors
      T2.writeMicroseconds(1500 + power + i * levelPower);
      T3.writeMicroseconds(1500 - power - i * levelPower);
      //Downgrade the left motors
      T1.writeMicroseconds(1500 - power - i * reversedLevelPower);
      T4.writeMicroseconds(1500 + power + i * reversedLevelPower);     
    }
    //backward
    if((angle_pitch_output > 2*i) && (angle_pitch_output < (2*i + 2))){
      //Boost the front motors
      T1.writeMicroseconds(1500 - power - i * levelPower);
      T2.writeMicroseconds(1500 + power + i * levelPower); 
      //Downgrade the back motors
      T3.writeMicroseconds(1500 - power - i * reversedLevelPower);
      T4.writeMicroseconds(1500 + power + i * reversedLevelPower);       
    }
    //forward
    if((angle_pitch_output < -1*( 2*i)) && (angle_pitch_output > -1 *(2*i + 2))){
      //Boost the back motors
      T3.writeMicroseconds(1500 - power - i * levelPower);
      T4.writeMicroseconds(1500 + power + i * levelPower);
      //Downgrade the front motors
      T1.writeMicroseconds(1500 - power - i * reversedLevelPower);
      T2.writeMicroseconds(1500 + power + i * reversedLevelPower);      
    }
  }
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

