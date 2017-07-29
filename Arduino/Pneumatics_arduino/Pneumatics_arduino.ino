#include <ros.h>
#include <std_msgs/Int32.h>

//Initializa ROS node
int pControlNum;
float pControlTime;
float pControlTimer;
bool pControlIsRunning;

ros::NodeHandle nh;
std_msgs::Int32 pControlStatus;

ros::Publisher pControlPublisher("pneumatics_control_status", &pControlStatus);

void pControlCallback(const std_msgs::Int32& pControl);
ros::Subscriber<std_msgs::Int32> pControlSubscriber("pneumatics_control", &pControlCallback);

void setup (){
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);
  pinMode(11,OUTPUT);
  pinMode(12,OUTPUT);
  pinMode(13,OUTPUT);

  pControlNum = 0;
  pControlTime = 1;
  pControlTimer = 0;
  pControlIsRunning = false;
  
  pControlStatus.data = 0;
  
  nh.initNode();
  nh.subscribe(pControlSubscriber);
  nh.advertise(pControlPublisher);

  nh.loginfo("pneumatics_control is ready...\n");
}

void loop (){
  pneumaticsControl();
  nh.spinOnce();
  delay(100);
}

void pControlCallback(const std_msgs::Int32& pControl){

  char numChar[6];
  int num = pControl.data;
  dtostrf(num, 4, 2, numChar);
  
  if(pControl.data == 0){
    if(pControlIsRunning){
      pControlNum = 0;
      pControlTimer = 0;
      pControlIsRunning = false;
      nh.loginfo("pneumatics_control is now cancelled\n");
    }
  }
  else if (pControl.data >= 8 && pControl.data <= 13){
    if(!pControlIsRunning){
      pControlNum = pControl.data;
      pControlIsRunning = true;
      pControlTimer = 0;
      if(pControl.data == 5)
        pControlTime = 3;
      else
        pControlTime = 1;
      char charBuf[5];
      nh.loginfo("pneumatics_control number");
      nh.loginfo(numChar);
      nh.loginfo("is now activated.\n");
    }
    else
      nh.loginfo("pneumatics_control is still running. Command abort.");
  }
  pControlStatus.data = pControl.data;
  pControlPublisher.publish(&pControlStatus);
  
}

//digitalWrite(4, HIGH); //solenoid 1
//digitalWrite(5, HIGH); //solenoid 2
//digitalWrite(6, HIGH); //solenoid 3 
//digitalWrite(7, HIGH); //solenoid 4
//digitalWrite(8, HIGH); //solenoid 5
//digitalWrite(9, HIGH); //solenoid 6

void pneumaticsControl(){
  if(pControlIsRunning){
    digitalWrite(pControlNum, HIGH);
    pControlTimer += 0.1;
    if(pControlTimer >= pControlTime){
      nh.loginfo("Mission completed.\n");
      pControlIsRunning = false;
      digitalWrite(pControlNum, LOW);
    }
  }
  else{
    pControlTimer = 0;
    pControlStatus.data = 0;
    pControlPublisher.publish(&pControlStatus);
  }
}
