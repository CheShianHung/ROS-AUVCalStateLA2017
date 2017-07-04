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
void resetBoolVariables();

//Regular variables
const float angleError = 5.0;
const float heightError = 0.2;
int directionToMove;
float angleToTurn;
float heightToMove;
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

//Communication variables
bool receivedFromRControl;
bool receivedFromHControl;
bool receivedFromMControl;
bool doneRotationControl;
bool doneHeightControl;
bool finishedRotationControl;
bool finishedHeightControl;
bool objectFound;

//Task variables
bool task0_submergeToWater;
bool task_submerge8Ft;
bool task_rotateRight90D;
bool task_emerge5Ft;
bool task_cv_getTargetInfo_1;
bool task_cv_beforeCenter_1;
bool task_cv_centering_1;

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

  directionToMove = 0;
  angleToTurn = 0;
  heightToMove = 0;

  resetBoolVariables();

  task0_submergeToWater = true;     //Task 0 - make sure the sub is submerged
  task_submerge8Ft = true;         //Task 1 - submerge 8 ft
  task_rotateRight90D = true;      //Task 2 - rotate right 90 degrees
  task_emerge5Ft = true;           //Task 3 - emerge 5 ft
  task_cv_getTargetInfo_1 = false;  //Task 4 - cv test 1 (get target_info)
  task_cv_beforeCenter_1 = false;   //Task 5 - cv test 2 (find object before center)
  task_cv_centering_1 = false;      //Task 6 - cv test 3 (centering the sub)


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
  while(ros::ok() && !task0_submergeToWater){
    settingCVInfo(0,0,0,0,0,0);
    cvInfoPublisher.publish(cvInfo);
    ros::spinOnce();
    loop_rate.sleep();
  }


  //Task =======================================================================
  while(ros::ok() && !task_submerge8Ft){
    //Sending command to height_control...
    if(!receivedFromHControl){
      hControl.state = 0;
      hControl.depth = 8;
      hControlPublisher.publish(hControl);
    }
    settingCVInfo(0,0,0,0,0,0);
    cvInfoPublisher.publish(cvInfo);
    ros::spinOnce();
    loop_rate.sleep();
  }

  resetBoolVariables();

  //Task =======================================================================
  while (ros::ok() && !task_rotateRight90D){
    //Sending command to rotate_control...
    if(!receivedFromRControl){
      rControl.state = 2;
      rControl.rotation = 90;
      rControlPublisher.publish(rControl);
    }
    settingCVInfo(0,0,0,0,0,0);
    cvInfoPublisher.publish(cvInfo);
    ros::spinOnce();
    loop_rate.sleep();
  }

  resetBoolVariables();

  //Task =======================================================================
  while (ros::ok() && !task_emerge5Ft){
    //Sending command to height_control...
    if(!receivedFromHControl){
      hControl.state = 2;
      hControl.depth = 5;
      hControlPublisher.publish(hControl);
    }
    settingCVInfo(0,0,0,0,0,0);
    cvInfoPublisher.publish(cvInfo);
    ros::spinOnce();
    loop_rate.sleep();
  }

  resetBoolVariables();

  //Task =======================================================================
  while(ros::ok() && !task_cv_getTargetInfo_1){
    objectFound = false;
    receivedFromRControl = false;
    receivedFromHControl = false;
    finishedRotationControl = false;
    finishedHeightControl = false;
    ROS_INFO("Finding Object...");
    while(ros::ok() && !objectFound){
      settingCVInfo(1,1,2,3,4,5);
      cvInfoPublisher.publish(cvInfo);
      ros::spinOnce();
      loop_rate.sleep();
    }
    ROS_INFO("Object found.");
    receivedFromRControl = false;
    receivedFromHControl = false;
    while(ros::ok() && (!receivedFromRControl || !receivedFromHControl)){
      if(!receivedFromRControl && !doneRotationControl){
        if(angleToTurn > angleError){
          rControl.state = 2;
          rControl.rotation = angleToTurn;
        }
        else if(angleToTurn < -angleError){
          rControl.state = 0;
          rControl.rotation = -angleToTurn;
        }
        else{
          doneRotationControl = true;
          finishedRotationControl = true;
          rControl.state = 1;
          rControl.rotation = 0;
        }
        rControlPublisher.publish(rControl);
      }
      if(!receivedFromHControl && !doneHeightControl){
        if(heightToMove > heightError){
          hControl.state = 2;
          hControl.depth = heightToMove;
        }
        else if(heightToMove < -heightToMove){
          hControl.state = 0;
          hControl.depth = -heightToMove;
        }
        else{
          doneHeightControl = true;
          finishedHeightControl = true;
          hControl.state = 1;
          hControl.depth = 0;
        }
        hControlPublisher.publish(hControl);
      }
      settingCVInfo(0,0,0,0,0,0);
      cvInfoPublisher.publish(cvInfo);
      ros::spinOnce();
      loop_rate.sleep();
    }
    ROS_INFO("Messages received by rotation and height. Sub starts moving...");
    while(!finishedRotationControl || !finishedHeightControl){
      settingCVInfo(0,0,0,0,0,0);
      cvInfoPublisher.publish(cvInfo);
      ros::spinOnce();
      loop_rate.sleep();
    }
    ROS_INFO("Sub is now in the right rotatio and depth.");
    if(doneRotationControl && doneHeightControl){
      task_cv_getTargetInfo_1 = true;
      ROS_INFO("Task 4 finished.\n");
    }
    //TESTING!!
    task_cv_getTargetInfo_1 = true;
  }

  resetBoolVariables();

  //Task =======================================================================
  while(ros::ok() && !task_cv_beforeCenter_1){
    ROS_INFO("Finding Object....");
    while(ros::ok() && !objectFound && directionToMove != 0){
      //Sending command to movement_control...
      if(!receivedFromMControl){
        ROS_INFO("sending to mControl");
        mControl.state = 1;
        if(directionToMove == 1)
          mControl.mDirection = 2;
        else if(directionToMove == -1)
          mControl.mDirection = 4;
        mControl.power = 50;
        mControl.distance = 0;
        mControlPublisher.publish(mControl);
      }
      settingCVInfo(1,1,2,3,4,5);
      cvInfoPublisher.publish(cvInfo);
      ros::spinOnce();
      loop_rate.sleep();
    }
    receivedFromMControl = false;
    ROS_INFO("Object found. Stopping the motor...");
    while(ros::ok() && !receivedFromMControl){
      //Sending command to movement_control...
      mControl.state = 0;
      mControl.mDirection = 0;
      mControl.mDirection = 0;
      mControl.power = 0;
      mControl.distance = 0;
      mControlPublisher.publish(mControl);
      settingCVInfo(1,1,2,3,4,5);
      cvInfoPublisher.publish(cvInfo);
      ros::spinOnce();
      loop_rate.sleep();
    }
    task_cv_beforeCenter_1 = true;
  }

  resetBoolVariables();

  //Task =======================================================================
  ROS_INFO("Centering the sub with front camera...");
  while(ros::ok() && !task_cv_centering_1){
    if(!receivedFromMControl){
      mControl.state = 3;
      mControl.mDirection = 0;
      mControl.mDirection = 0;
      mControl.power = 0;
      mControl.distance = 0;
      mControlPublisher.publish(mControl);
    }
    settingCVInfo(1,1,2,3,4,5);
    cvInfoPublisher.publish(cvInfo);
    ros::spinOnce();
    loop_rate.sleep();
  }

  resetBoolVariables();

  //Executing... ===============================================================
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

void resetBoolVariables(){
  receivedFromRControl = false;
  receivedFromHControl = false;
  receivedFromMControl = false;
  doneRotationControl = false;
  doneHeightControl = false;
  finishedRotationControl = false;
  finishedHeightControl = false;
  objectFound = false;
}

void currentDepthCallback(const std_msgs::Float32& currentDepth){
  if(!allNodesAreReady){
    if(!checkingCurrentDepth){
      checkingCurrentDepth = true;
      ROS_INFO("current_depth is ready.");
      checkMotorNode();
    }
  }
  else if(!task0_submergeToWater){
    if(currentDepth.data > 0){
       task0_submergeToWater = true;
       ROS_INFO("Sub is now under water... Ready to get start...");
    }
  }
  else if(!task_submerge8Ft){}
  else if(!task_rotateRight90D){}
  else if(!task_emerge5Ft){}
  else if(!task_cv_getTargetInfo_1){}
  else if(!task_cv_beforeCenter_1){}
  else if(!task_cv_centering_1){}
}

void currentRotationCallback(const std_msgs::Float32& currentRotation){
  if(!allNodesAreReady){
    if(!checkingCurrentRotation){
      checkingCurrentRotation = true;
      ROS_INFO("current_rotation is ready.");
      checkMotorNode();
    }
  }
  else if(!task0_submergeToWater){}
  else if(!task_submerge8Ft){}
  else if(!task_rotateRight90D){}
  else if(!task_emerge5Ft){}
  else if(!task_cv_getTargetInfo_1){}
  else if(!task_cv_beforeCenter_1){}
  else if(!task_cv_centering_1){}
}

void frontCamDistanceCallback(const auv_cal_state_la_2017::FrontCamDistance fcd){
  if(!allNodesAreReady){
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
  }
  else if(!task0_submergeToWater){}
  else if(!task_submerge8Ft){}
  else if(!task_rotateRight90D){}
  else if(!task_emerge5Ft){}
  else if(!task_cv_getTargetInfo_1){}
  else if(!task_cv_beforeCenter_1){}
  else if(!task_cv_centering_1){}
}

void bottomCamDistanceCallback(const auv_cal_state_la_2017::BottomCamDistance bcd){
  if(!allNodesAreReady){
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
  }
  else if(!task0_submergeToWater){}
  else if(!task_submerge8Ft){}
  else if(!task_rotateRight90D){}
  else if(!task_emerge5Ft){}
  else if(!task_cv_getTargetInfo_1){}
  else if(!task_cv_beforeCenter_1){}
  else if(!task_cv_centering_1){}
}


void hControlStatusCallback(const auv_cal_state_la_2017::HControl hc){
  if(!allNodesAreReady){
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
  }
  else if(!task0_submergeToWater){}
  else if(!task_submerge8Ft){
    if(!receivedFromHControl){
      ROS_INFO("Sending command to height_control - submerge 8 ft");
    }
    if(!receivedFromHControl && hc.state == 0 && hc.depth == 8){
      receivedFromHControl = true;
      ROS_INFO("height_control message received - submerging...");
    }
    if(receivedFromHControl && hc.state == 1 && hc.depth == 0){
      task_submerge8Ft = true;
      ROS_INFO("Task 1 completed! Sub is now 8 ft below the water.");
    }
  }
  else if(!task_rotateRight90D){}
  else if(!task_emerge5Ft){
    if(!receivedFromHControl){
      ROS_INFO("Sending command to height_control - emerge 5 ft");
    }
    if(!receivedFromHControl && hc.state == 2 && hc.depth == 5){
      receivedFromHControl = true;
      ROS_INFO("height_control message received - emerging...");
    }
    if(receivedFromHControl && hc.state == 1 && hc.depth == 0){
      task_emerge5Ft = true;
      ROS_INFO("Task 3 completed! Sub is now 3 ft below the water.");
    }
  }
  else if(!task_cv_getTargetInfo_1){
    if(!doneHeightControl){
      if(!receivedFromHControl && hc.state != 1 && hc.depth == hControl.depth){
        receivedFromHControl = true;
      }
      if(receivedFromHControl && !finishedHeightControl && hc.state == 1 && hc.depth == 0){
        finishedHeightControl = true;
      }
    }
  }
  else if(!task_cv_beforeCenter_1){}
  else if(!task_cv_centering_1){}
}

void rControlStatusCallback(const auv_cal_state_la_2017::RControl rc){
  if(!allNodesAreReady){
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
  }
  else if(!task0_submergeToWater){}
  else if(!task_submerge8Ft){}
  else if(!task_rotateRight90D){
    if(!receivedFromRControl){
      ROS_INFO("Sending commend to rotation_control - rotate right 90 degrees");
    }
    if(!receivedFromRControl && rc.state == 2 && rc.rotation == 90){
      receivedFromRControl = true;
      ROS_INFO("rotation_control message received - rotating...");
    }
    if(receivedFromRControl && rc.state == 1 && rc.rotation == 0){
      task_rotateRight90D = true;
      ROS_INFO("Task 2 completed! Sub is now at rotation 90 degrees.");
    }
  }
  else if(!task_emerge5Ft){}
  else if(!task_cv_getTargetInfo_1){
    if(!doneRotationControl){
      if(!receivedFromRControl && rc.state != 1 && rc.rotation == rControl.rotation){
        receivedFromRControl = true;
      }
      if(receivedFromRControl && !finishedRotationControl && rc.state == 1 && rc.rotation == 0){
        finishedRotationControl = true;
      }
    }
  }
  else if(!task_cv_beforeCenter_1){}
  else if(!task_cv_centering_1){}
}

void mControlStatusCallback(const auv_cal_state_la_2017::MControl mc){
  if(!allNodesAreReady){
    if(!checkingMovementControl){
      ROS_INFO("Checking movement_control...");
      if(mc.state == 0 && mc.mDirection == 0 && mc.power == 0 && mc.distance == 0){
        checkingMovementControl = true;
        ROS_INFO("movement_control is ready.");
        checkMotorNode();
      }
      else{
        ROS_INFO("Cannot communicate with movement_control...");
      }
    }
  }
  else if(!task0_submergeToWater){}
  else if(!task_submerge8Ft){}
  else if(!task_rotateRight90D){}
  else if(!task_emerge5Ft){}
  else if(!task_cv_getTargetInfo_1){}
  else if(!task_cv_beforeCenter_1){
    if(!receivedFromMControl){
      ROS_INFO("Sending command to movememt_control - move towards target");
    }
    if(!receivedFromMControl && mc.state == 1){
      receivedFromMControl = true;
      ROS_INFO("movement_control message received - moving...");
    }
    if(objectFound && !receivedFromMControl && mc.state == 0){
      receivedFromMControl = true;
      ROS_INFO("Sub is now stopped.");
    }
  }
  else if(!task_cv_centering_1){
    if(!receivedFromMControl){
      ROS_INFO("Sending command to movement_control - centering with front camera");
    }
    if(!receivedFromMControl && mc.state == 3){
      receivedFromMControl = true;
      ROS_INFO("movement_control message received - centering");
    }
    if(receivedFromMControl && mc.state == 0){
      task_cv_centering_1 = true;
      ROS_INFO("Finished centering.");
    }
  }
}

void targetInfoCallback(const auv_cal_state_la_2017::TargetInfo ti){
  if(!allNodesAreReady){
    if(!checkingTargetInfo){
      ROS_INFO("Checking target_info...");
      if(ti.state == 0 && ti.angle == 0 && ti.height == 0 && ti.direction == 0){
        checkingTargetInfo = true;
        ROS_INFO("target_info is ready.");
        checkCVNode();
      }
    }
  }
  else if(!task0_submergeToWater){}
  else if(!task_submerge8Ft){}
  else if(!task_rotateRight90D){}
  else if(!task_emerge5Ft){}
  else if(!task_cv_getTargetInfo_1){
    //Object found
    if(!objectFound && ti.state == 1){
      objectFound = true;
      directionToMove = ti.direction;
      angleToTurn = ti.angle;
      heightToMove = ti.height;
    }
  }
  else if(!task_cv_beforeCenter_1){
    //Object found
    if(!objectFound && ti.state == 1){
      objectFound = true;
    }
  }
  else if(!task_cv_centering_1){}
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
