#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "auv_cal_state_la_2017/HControl.h"
#include <sstream>

//height_control: (int) state, (float) depth
// state: going down (0), staying (1), going up (2)
// depth: nonstop moving (-1), moving distance (x)


// Task List
// 0: floating
// 1: emerging
// 2: submerging

//const int TASK_NUM = 3;
//bool taskAry[TASK_NUM];


//ROS variables
auv_cal_state_la_2017::HControl hControl;

//Subscriber callback functions
void currentDepthCallback(const std_msgs::Float32& currentDepth);
void hControlStatusCallback(const auv_cal_state_la_2017::HControl);

//Regular functions
void checkMotorNode();

//Regular variables
bool checkingCurrentDepth;
bool checkingHeightControl;
bool motorNodeIsReady;
bool allNodesAreReady;

int main(int argc, char **argv){
  
  //Initializing ROS variables  
  ros::init(argc, argv, "master");
  ros::NodeHandle node;
  ros::Subscriber currentDepthSubscriber = node.subscribe("current_depth", 100, currentDepthCallback);
  ros::Subscriber hControlStatusSubscriber = node.subscribe("height_control_status", 100, hControlStatusCallback);
  ros::Publisher hControlPublisher = node.advertise<auv_cal_state_la_2017::HControl>("height_control", 100);
  ros::Rate loop_rate(100);

  checkingCurrentDepth = false;
  checkingHeightControl = false;
  motorNodeIsReady = false;
  allNodesAreReady = false;

  ROS_INFO("Master starts running. Checking each topic...");

  //Checking nodes...
  while(ros::ok() && !allNodesAreReady){
     
    //Checking height_control...
    if(!checkingHeightControl){
      hControl.state = 1;
      hControl.depth = 0;
      hControlPublisher.publish(hControl);
    }
     
    ros::spinOnce();
    

    if(motorNodeIsReady){
      allNodesAreReady = true;
    }
    
    loop_rate.sleep();
  }

  //Executing...
  while(ros::ok()){
  
    //hControlPublisher.publish(hControl);
    ros::spinOnce();
    loop_rate.sleep();    

  }   

  return 0;

}

void currentDepthCallback(const std_msgs::Float32& currentDepth){
  //Checking current_depth...
  if(!checkingCurrentDepth){
    checkingCurrentDepth = true;
    ROS_INFO("current_depth is ready.");
    checkMotorNode();
  }
  else{
    //While running...
  }
}

void hControlStatusCallback(const auv_cal_state_la_2017::HControl hc){
  //Checking height_control...
  if(!checkingHeightControl){
    ROS_INFO("Checking height_control...");
    if(hc.state == 1 && hc.depth == 0){
      checkingHeightControl = true;
      ROS_INFO("height_control is ready.");
      checkMotorNode();
    }
    else{
      ROS_INFO("Cannot communicate with height_control...");
    }
  }else {
    //While running...
  }
}

void checkMotorNode(){
  if(checkingCurrentDepth && checkingHeightControl){
    motorNodeIsReady = true;
    ROS_INFO("Motor node is ready...");
  }
}
