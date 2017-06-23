#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "auv_cal_state_la_2017/HControl.h"
#include "auv_cal_state_la_2017/RControl.h"
#include <sstream>

// height_control: (int) state, (float) depth
// state: going down (0), staying (1), going up (2)
// depth: nonstop moving (-1), moving distance (x)

// rotation_control: (int)state, (float) rotation
// state: rotate left (0), staying (1), rotate right (2)
// rotation: nonstop rotating (-1), rotate degree (x)

// Task List
// Task 0: sub is under water
// Task 1: submerging 8 ft
// Task 2: rotate right 90 degrees
// Task 3: emerging 5ft

//ROS variables
auv_cal_state_la_2017::HControl hControl;
auv_cal_state_la_2017::RControl rControl;

//Subscriber callback functions
void currentDepthCallback(const std_msgs::Float32& currentDepth);
void hControlStatusCallback(const auv_cal_state_la_2017::HControl);
void currentRotationCallback(const std_msgs::Float32& currentRotation);
void rControlStatusCallback(const auv_cal_state_la_2017::RControl);

//Regular functions
void checkMotorNode();

//Regular variables
//float currentTargetDepth;
//float currentTargetRotation;

//Checking variables
bool checkingCurrentDepth;
bool checkingHeightControl;
bool checkingCurrentRotation;
bool checkingRotationControl;
bool motorNodeIsReady;
bool allNodesAreReady;

//Task variables
bool task0Finished;
bool task1Finished;
bool task1ReceivedFromHControl;
bool task2Finished;
bool task2ReceivedFromRControl;
bool task3Finished;
bool task3ReceivedFromHControl;
bool task4Finished;
bool task5Finished;
bool task6Finished;

int main(int argc, char **argv){
  
  //Initializing ROS variables  
  ros::init(argc, argv, "master");
  ros::NodeHandle node;
  ros::Subscriber currentDepthSubscriber = node.subscribe("current_depth", 100, currentDepthCallback);
  ros::Subscriber currentRotationSubscriber = node.subscribe("current_rotation", 100, currentRotationCallback);
  ros::Subscriber hControlStatusSubscriber = node.subscribe("height_control_status", 100, hControlStatusCallback);
  ros::Subscriber rControlStatusSubscriber = node.subscribe("rotation_control_status", 100, rControlStatusCallback);
  ros::Publisher hControlPublisher = node.advertise<auv_cal_state_la_2017::HControl>("height_control", 100);
  ros::Publisher rControlPublisher = node.advertise<auv_cal_state_la_2017::RControl>("rotation_control", 100);
  ros::Rate loop_rate(10);

  //currentTargetDepth = 0;
  //currentTargetRotation = 0;

  checkingCurrentDepth = false;
  checkingHeightControl = false;
  checkingCurrentRotation = false;
  checkingRotationControl = false;
  motorNodeIsReady = false;
  allNodesAreReady = false;

  task0Finished = false;
  task1Finished = false;
  task1ReceivedFromHControl = false;
  task2Finished = false;
  task2ReceivedFromRControl = false;
  task3Finished = false;
  task3ReceivedFromHControl = false;
  task4Finished = false;
  task5Finished = false;
  task6Finished = false;


  ROS_INFO("Master starts running. Checking each topic...");

  //Checking nodes...
  while(ros::ok() && !allNodesAreReady){
     
    //Checking height_control...
    if(!checkingHeightControl){
      hControl.state = 1;
      hControl.depth = 0;
      hControlPublisher.publish(hControl);
    }

    if(!checkingRotationControl){
      rControl.state = 1;
      rControl.rotation = 0;
      rControlPublisher.publish(rControl);
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

  
  //Task 1 - submerge 8 ft (from 0 ft to 8 ft)
  while(ros::ok() && !task1Finished){
    //Sending command to height_control...
    if(!task1ReceivedFromHControl){
      hControl.state = 0;
      hControl.depth = 8;
      hControlPublisher.publish(hControl);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  //Task 2 - rotate right 90 degrees (from 0 to 90)
  while (ros::ok() && !task2Finished){
    //Sending command to rotate_control...
    if(!task2ReceivedFromRControl){
      rControl.state = 2;
      rControl.rotation = 90;
      rControlPublisher.publish(rControl);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  //Task 3 - emerge 5 ft (from 8 ft to 3 ft)
  while (ros::ok() && !task3Finished){
    //Sending command to height_control...
    if(!task3ReceivedFromHControl){
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
  else if(!task1Finished){}
  //Task 2 - Rotate right 90 degrees
  else if(!task2Finished){}
  //Task 2 - Emerge 5 ft
  else if(!task3Finished){}
}

void currentRotationCallback(const std_msgs::Float32& currentRotation){
  //Checking current rotation...
  if(!checkingCurrentRotation){
    checkingCurrentRotation = true;
    ROS_INFO("current_rotation is ready.");
    checkMotorNode();
  }
  //Task 0 - Submerge under water
  else if(!task0Finished){}
  //Task 1 - Submerging 8 ft
  else if(!task1Finished){}
  //Task 2 - Rotate right 90 degrees
  else if(!task2Finished){}
  //Task 3 - Emerge 5 ft
  else if(!task3Finished){}
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
    if(!task1ReceivedFromHControl){
      ROS_INFO("Sending command to height_control - submerge 8 ft");
    }
    if(!task1ReceivedFromHControl && hc.state == 0 && hc.depth == 8){
      task1ReceivedFromHControl = true;
      ROS_INFO("height_control message received - submerging...");
    }
    if(task1ReceivedFromHControl && hc.state == 1 && hc.depth == 0){
      task1Finished = true;
      ROS_INFO("Task 1 completed! Sub is now 8 ft below the water.");
    }
  }
  //Task 2 - Rotate right 90 degrees
  else if(!task2Finished){}
  //Task 3 - Emerge 5 ft
  else if(!task3Finished){
    if(!task3ReceivedFromHControl){
      ROS_INFO("Sending command to height_control - emerge 5 ft");
    }
    if(!task3ReceivedFromHControl && hc.state == 2 && hc.depth == 5){
      task3ReceivedFromHControl = true;
      ROS_INFO("height_control message received - emerging...");
    }
    if(task3ReceivedFromHControl && hc.state == 1 && hc.depth == 0){
      task3Finished = true;
      ROS_INFO("Task 3 completed! Sub is now 3 ft below the water.");
    }
  }
}

void rControlStatusCallback(const auv_cal_state_la_2017::RControl rc){
  //Checking rotation_control...
  if(!checkingRotationControl){
    ROS_INFO("Checking rotation_control...");
    if(rc.state == 1 && rc.rotation == 0){
      checkingRotationControl = true;
      ROS_INFO("rotation_control is ready.");
      checkMotorNode();
    }
    else{
      ROS_INFO("Cannot communicate with rotation_control...");
    }
  }
  //Task 0 - Submerge under water
  else if(!task0Finished){}
  //Task 1 - Submerging 8 ft
  else if(!task1Finished){}
  //Task 2 - Rotate right 90 degrees
  else if(!task2Finished){
    if(!task2ReceivedFromRControl){
      ROS_INFO("Sending commend to rotation_control - rotate right 90 degrees");
    }
    if(!task2ReceivedFromRControl && rc.state == 2 && rc.rotation == 90){
      task2ReceivedFromRControl = true;
      ROS_INFO("rotation_control message received - rotating...");
    }
    if(task2ReceivedFromRControl && rc.state == 1 && rc.rotation == 0){
      task2Finished = true;
      ROS_INFO("Task 2 completed! Sub is now at rotation 90 degrees.");
    }
  }
  //Task 3 - Emerge 5 ft
  else if(!task3Finished){}
}

void checkMotorNode(){
  if(checkingCurrentDepth && 
     checkingHeightControl && 
     checkingCurrentRotation &&
     checkingRotationControl){
    motorNodeIsReady = true;
    ROS_INFO("Motor node is ready...");
  }
}
