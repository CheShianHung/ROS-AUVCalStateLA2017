#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "auv_cal_state_la_2017/HControl.h"
#include "auv_cal_state_la_2017/RControl.h"
#include "auv_cal_state_la_2017/MControl.h"
#include "auv_cal_state_la_2017/FrontCamDistance.h"
#include "auv_cal_state_la_2017/BottomCamDistance.h"
#include "auv_cal_state_la_2017/TargetInfo.h"
#include "auv_cal_state_la_2017/CVInfo.h"
#include <sstream>

// height_control: (int) state, (float) depth
// state: going down (0), staying (1), going up (2)
// depth: nonstop moving (-1), moving distance (x)

// rotation_control: (int) state, (float) rotation
// state: rotate left (0), staying (1), rotate right (2)
// rotation: nonstop rotating (-1), rotate degree (x)

// movement_control: (int) state, (int) mDirection, (float) power, (float) distance
// state: off (0), adjust with power (1), ajust with distance (2), 
//        centered with front camera (3), centered with bottom camera (4)
// mDirection: none (0), forward (1), right (2), backward (3), left(4)
// power: none (0), motor power (x)
// distance: distance away from the object (x)

// target_info: (int) state, (float) angle, (float) height, (int) direction
// state: not found (0), found (1)
// angle: angles to turn left (x < 0), none (0), angles to turn right (x > 0)
// height: depth to go down (x < 0), none (0), depth to go up (x > 0)
// direction: left (-1), none (0), right (1)

// cv_info: (int) cameraNumber, (int) taskNumber, (float) givenLength, 
//          (float) givenDistance, (int) givenColor, (int) givenShape
// cameraNumber: off (0), front camera (1), bottom camera (2)
// taskNumber: none (0), task # (x)
// givenLength: none (0), object length (x)
// givenDistance: none (0), object distance (x)
// givenColor: none (0), customize color # (x)
// givenShape: none (0), customize shape # (x)

// Task List
// Task 0: sub is under water
// Task 1: submerging 8 ft
// Task 2: rotate right 90 degrees
// Task 3: emerging 5ft

//ROS variables
auv_cal_state_la_2017::HControl hControl;
auv_cal_state_la_2017::RControl rControl;
auv_cal_state_la_2017::MControl mControl;
auv_cal_state_la_2017::CVInfo cvInfo;

//Subscriber callback functions
void currentDepthCallback(const std_msgs::Float32& currentDepth);
void currentRotationCallback(const std_msgs::Float32& currentRotation);
void hControlStatusCallback(const auv_cal_state_la_2017::HControl hc);
void rControlStatusCallback(const auv_cal_state_la_2017::RControl rc);
void mControlStatusCallback(const auv_cal_state_la_2017::MControl mc);
void frontCamDistanceCallback(const auv_cal_state_la_2017::FrontCamDistance fcd);
void bottomCamDistanceCallback(const auv_cal_state_la_2017::BottomCamDistance bcd);
void targetInfoCallback(const auv_cal_state_la_2017::TargetInfo ti);


//Regular functions
void checkMotorNode();
void checkCVNode();
void settingCVInfo(int cameraNum, int taskNum, int givenColor, int givenShape, float givenFloat, float givenDistance);

//Regular variables
//float currentTargetDepth;
//float currentTargetRotation;

//Checking variables
bool checkingCurrentDepth;
bool checkingHeightControl;
bool checkingCurrentRotation;
bool checkingRotationControl;
bool checkingMovementControl;
bool checkingFrontCamDistance;
bool checkingBottomCamDistance;
bool checkingTargetInfo;
bool motorNodeIsReady;
bool cvNodeIsReady;
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
bool task4ObjectFound;
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
  ros::Subscriber mControlStatusSubscriber = node.subscribe("movement_control_status", 100, mControlStatusCallback);
  ros::Subscriber frontCamDistanceSubscriber = node.subscribe("front_cam_distance", 100, frontCamDistanceCallback);
  ros::Subscriber bottomCamDistanceSubscriber = node.subscribe("bottom_cam_distance", 100, bottomCamDistanceCallback);
  ros::Subscriber targetInfoSubscriber = node.subscribe("target_info", 100, targetInfoCallback);
  ros::Publisher hControlPublisher = node.advertise<auv_cal_state_la_2017::HControl>("height_control", 100);
  ros::Publisher rControlPublisher = node.advertise<auv_cal_state_la_2017::RControl>("rotation_control", 100);
  ros::Publisher mControlPublisher = node.advertise<auv_cal_state_la_2017::MControl>("movement_control", 100);
  ros::Publisher cvInfoPublisher = node.advertise<auv_cal_state_la_2017::CVInfo>("cv_info", 100);
  ros::Rate loop_rate(10);
  
  //currentTargetDepth = 0;
  //currentTargetRotation = 0;

  checkingCurrentDepth = false;
  checkingHeightControl = false;
  checkingFrontCamDistance = false;
  checkingBottomCamDistance = false;
  checkingCurrentRotation = false;
  checkingRotationControl = false;
  checkingMovementControl = false;
  checkingTargetInfo = false;
  motorNodeIsReady = false;
  cvNodeIsReady = false;
  allNodesAreReady = false;
  
  //Task 0 - make sure the sub is submerged
  task0Finished = true;
  //Task 1 - submerge 8 ft
  task1Finished = true;
  task1ReceivedFromHControl = true;
  //Task 2 - rotate right 90 degrees
  task2Finished = true;
  task2ReceivedFromRControl = true;
  //Task 3 - emerge 5 ft
  task3Finished = true;
  task3ReceivedFromHControl = true;
  //Task 4 - cv test
  task4Finished = false;
  task4ObjectFound = false;
  task5Finished = false;
  task6Finished = false;


  ROS_INFO("Master starts running. Checking each topic...");

  //Checking nodes...
  while(ros::ok() && !allNodesAreReady){
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
    if(!checkingMovementControl){
      mControl.state = 0;
      mControl.mDirection = 0;
      mControl.power = 0;
      mControl.distance = 0;
      mControlPublisher.publish(mControl);
    }
    
    ros::spinOnce();
    
    if(motorNodeIsReady && cvNodeIsReady){
      allNodesAreReady = true;
      ROS_INFO("Sub is ready to rock and roll!!\n");
    }

    settingCVInfo(0,0,0,0,0,0);
    cvInfoPublisher.publish(cvInfo);
    loop_rate.sleep();
  }
  

  //Task 0 - checking barometer (current_depth) to make sure the sub is under water
  while(ros::ok() && !task0Finished){
    settingCVInfo(0,0,0,0,0,0);
    cvInfoPublisher.publish(cvInfo);
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
    settingCVInfo(0,0,0,0,0,0);
    cvInfoPublisher.publish(cvInfo);
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
    settingCVInfo(0,0,0,0,0,0);
    cvInfoPublisher.publish(cvInfo);
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
    settingCVInfo(0,0,0,0,0,0);
    cvInfoPublisher.publish(cvInfo);
    ros::spinOnce();
    loop_rate.sleep();
  }

  //Task 4 - cv test
  while (ros::ok() && !task4Finished){
    
    if(!task4ObjectFound){
      settingCVInfo(1,1,2,3,4,5);
    }
    else{
      settingCVInfo(0,0,0,0,0,0);
    }
    cvInfoPublisher.publish(cvInfo);
   
    loop_rate.sleep();
  }

  //Executing...
  while(ros::ok()){
  
    settingCVInfo(0,0,0,0,0,0);
    cvInfoPublisher.publish(cvInfo);
    ros::spinOnce();
    loop_rate.sleep();    

  }   

  return 0;

}

void settingCVInfo(int cameraNum, int taskNum, int givenColor, int givenShape, float givenLength, float givenDistance){
  cvInfo.cameraNumber = cameraNum;
  cvInfo.taskNumber = taskNum;
  cvInfo.givenColor = givenColor;
  cvInfo.givenShape = givenShape;
  cvInfo.givenLength = givenLength;
  cvInfo.givenDistance = givenDistance;
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
  //Task 3 - Emerge 5 ft
  else if(!task3Finished){}
  //Task 4 - cv test
  else if(!task4Finished){}
}

void currentRotationCallback(const std_msgs::Float32& currentRotation){
  //Checking current_rotation...
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
  //Task 4 - cv test
  else if(!task4Finished){}
}

void frontCamDistanceCallback(const auv_cal_state_la_2017::FrontCamDistance fcd){
  //Checking front_cam_distance...
  if(!checkingFrontCamDistance){
    ROS_INFO("Checking front_cam_distance...");
    if(fcd.frontCamForwardDistance == 0 &&
       fcd.frontCamHorizontalDistance == 0 &&
       fcd.frontCamVerticalDistance == 0){
      checkingFrontCamDistance = true;
      ROS_INFO("front_cam_distance is ready.");
      checkCVNode();
    }
  }
  //Task 0 - Submerge under water
  else if(!task0Finished){}
  //Task 1 - Submerging 8 ft
  else if(!task1Finished){}
  //Task 2 - Rotate right 90 degrees
  else if(!task2Finished){}
  //Task 3 - Emerge 5 ft
  else if(!task3Finished){}
  //Task 4 - cv test
  else if(!task4Finished){}
}

void bottomCamDistanceCallback(const auv_cal_state_la_2017::BottomCamDistance bcd){
  //Checking bottom_cam_distance...
  if(!checkingBottomCamDistance){
    ROS_INFO("Checking bottom_cam_distance...");
    if(bcd.bottomCamForwardDistance == 0 &&
       bcd.bottomCamHorizontalDistance == 0 &&
       bcd.bottomCamVerticalDistance == 0){
      checkingBottomCamDistance = true;
      ROS_INFO("bottom_cam_distance is ready.");
      checkCVNode();
    }
  }
  //Task 0 - Submerge under water
  else if(!task0Finished){}
  //Task 1 - Submerging 8 ft
  else if(!task1Finished){}
  //Task 2 - Rotate right 90 degrees
  else if(!task2Finished){}
  //Task 3 - Emerge 5 ft
  else if(!task3Finished){}
  //Task 4 - cv test
  else if(!task4Finished){}
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
  //Task 4 - cv test
  else if(!task4Finished){}
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
  //Tasl 4 - cv test
  else if(!task4Finished){}
}

void mControlStatusCallback(const auv_cal_state_la_2017::MControl mc){
  //Checking movement_control...
  if(!checkingMovementControl){
    ROS_INFO("Checking rotation_control...");
    if(mc.state == 0 && mc.mDirection == 0 && mc.power == 0 && mc.distance == 0){
      checkingMovementControl = true;
      ROS_INFO("movement_control is ready.");
      checkMotorNode();
    }
    else{
      ROS_INFO("Cannot communicate with movement_control...");
    }
  }
  //Task 0 - Submerge under water
  else if(!task0Finished){}
  //Task 1 - Submerge 8 ft
  else if(!task1Finished){}
  //Task 2 - Rotate right 90 degrees
  else if(!task2Finished){}
  //Task 3 - Emerge 5 ft
  else if(!task3Finished){}
  //Task 4 - cv test
  else if(!task4Finished){}
}

void targetInfoCallback(const auv_cal_state_la_2017::TargetInfo ti){
  //Checking target_info...
  if(!checkingTargetInfo){
    ROS_INFO("Checking target_info...");
    if(ti.state == 0 && ti.angle == 0 && ti.height == 0 && ti.direction == 0){
      checkingTargetInfo = true;
      ROS_INFO("target_info is ready.");
      checkCVNode();
    }
  }
  //Task 0 - Submerge under water
  else if(!task0Finished){}
  //Task 1 - Submerge 8 ft
  else if(!task1Finished){}
  //Task 2 - Rotate right 90 degrees
  else if(!task2Finished){}
  //Task 3 - Emerge 5 ft
  else if(!task3Finished){}
  //Task 4 - cv test
  else if(!task4Finished){}
}

void checkMotorNode(){
  if(checkingCurrentDepth && 
     checkingHeightControl && 
     checkingCurrentRotation &&
     checkingRotationControl &&
     checkingMovementControl){
    motorNodeIsReady = true;
    ROS_INFO("Motor node is ready...\n");
  }
}

void checkCVNode(){
  if(checkingFrontCamDistance &&
     checkingBottomCamDistance &&
     checkingTargetInfo){
    cvNodeIsReady = true;
    ROS_INFO("CV node is ready...\n");
  }
}
