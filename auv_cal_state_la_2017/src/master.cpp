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
//float currentTargetDepth;

//Checking variables
bool checkingCurrentDepth;
bool checkingHeightControl;
bool motorNodeIsReady;
bool allNodesAreReady;

//Task variables
bool task0Finished;
bool task1Finished;
bool task1ReceivedFromMotor;
bool task2Finished;
bool task2ReceivedFromMotor;

int main(int argc, char **argv){
  
  //Initializing ROS variables  
  ros::init(argc, argv, "master");
  ros::NodeHandle node;
  ros::Subscriber currentDepthSubscriber = node.subscribe("current_depth", 100, currentDepthCallback);
  ros::Subscriber hControlStatusSubscriber = node.subscribe("height_control_status", 100, hControlStatusCallback);
  ros::Publisher hControlPublisher = node.advertise<auv_cal_state_la_2017::HControl>("height_control", 100);
  ros::Rate loop_rate(100);

  //currentTargetDepth = 0;

  checkingCurrentDepth = false;
  checkingHeightControl = false;
  motorNodeIsReady = false;
  allNodesAreReady = false;

  task0Finished = false;
  task1Finished = false;
  task1ReceivedFromMotor = false;
  task2Finished = false;
  task2ReceivedFromMotor = false;


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
  

  //Task 0 - checking barometer (current_depth) to make sure the sub is under water
  while(ros::ok() && !task0Finished){
    ros::spinOnce();
    loop_rate.sleep();
  }

  
  //Task 1 - submerging down 8 ft (from 0 ft to 8 ft)
  while(ros::ok() && !task1Finished){
    //Sending command to motor node...
    if(!task1ReceivedFromMotor){
      hControl.state = 0;
      hControl.depth = 8;
      hControlPublisher.publish(hControl);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  //Task 2 - emerging up 5 ft (from 8 ft to 3 ft)
  while (ros::ok() && !task2Finished){
    //Sending command to motor node...
    if(!task2ReceivedFromMotor){
      hControl.state = 2;
      hControl.depth = 5;
      hControlPublisher.publish(hControl);
    }
  
    ros::spinOnce();
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
  //Task 0 - Submerge under water
  else if(!task0Finished){
    if(currentDepth.data > 0){
       task0Finished = true;
       ROS_INFO("Sub is now under water... Ready to get start...");
    }
  }
  //Task 1 - Submerging 8 ft
  else if(!task1Finished){
    if(currentDepth.data >= 8){
      task1Finished = true;
      ROS_INFO("Task 1 completed! Sub is now 8 ft below the water");
    }
  }
  //Task 2 - Emerging 5 ft
  else if(!task2Finished){
    if(currentDepth.data <= 3){
      task2Finished = true;
      ROS_INFO("Task 2 completed! Sub is now 3 ft below the water");
    }
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
  }
  //Task 0 - Submerge under water
  else if(!task0Finished){}
  //Task 1 - Submerging 8 ft
  else if(!task1Finished){
    ROS_INFO("Sending command to motor nodes - submerge 8 ft");
    if(hc.state == 0 && hc.depth == 8){
      task1ReceivedFromMotor = true;
      ROS_INFO("Task 1 message received - submerging...");
    }
  }
  //Task 2 - Emerging 5 ft
  else if(!task2Finished){
    ROS_INFO("Sending commend to motor nodes - emerge 5 ft");
    if(hc.state == 2 && hc.depth == 5){
      task2ReceivedFromMotor = true;
      ROS_INFO("Task 2 message received - emerging...");
    }
  }
}

void checkMotorNode(){
  if(checkingCurrentDepth && checkingHeightControl){
    motorNodeIsReady = true;
    ROS_INFO("Motor node is ready...");
  }
}
