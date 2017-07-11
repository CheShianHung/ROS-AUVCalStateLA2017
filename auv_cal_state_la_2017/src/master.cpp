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
//        centered with front camera (3), centered with bottom camera (4),
//        turn on motor with specific time (5)
// mDirection: none (0), forward (1), right (2), backward (3), left(4)
// power: none (0), motor power (x)
// distance: distance away from the object (x)
// time: time for the motor to turn on

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
float motorPower;
float motorRunningTime;
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
bool task_submergeXft;
bool task_turnOnMotors;
bool task_emergeXft;
bool task_emergeToTop;
bool task_rotateRightXD1;
bool task_rotateRightXD2;
bool task_rotateLeftXD1;
bool task_rotateLeftXD2;
bool task_keepRotatingRight;
bool task_keepRotatingLeft;
bool task_mode1Movement;
bool task_mode5Movement1;
bool task_mode5Movement2;

bool task_square_submergeXft;
bool task_square_mode5Movement1;
bool task_square_mode5Movement2;
bool task_square_rotateRightXD;
bool task_square_emergeToTop;

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
  cvNodeIsReady = true;
  allNodesAreReady = true;

  directionToMove = 0;
  angleToTurn = 0;
  heightToMove = 0;
  motorPower = 0;
  motorRunningTime = 0;

  task0_submergeToWater = true;
  task_turnOnMotors = false;
  task_submergeXft = true;
  task_emergeXft = true;
  task_emergeToTop = true;
  task_rotateRightXD1 = true;
  task_rotateRightXD2 = true;
  task_rotateLeftXD1 = true;
  task_rotateLeftXD2 = true;
  task_keepRotatingRight = true;
  task_keepRotatingLeft = true;
  task_mode1Movement = false;
  task_mode5Movement1 = false;
  task_mode5Movement2 = false;

  task_square_submergeXft = true;
  task_square_mode5Movement1 = true;
  task_square_mode5Movement2 = true;
  task_square_rotateRightXD = true;
  task_square_emergeToTop = true;

  task_cv_getTargetInfo_1 = true;
  task_cv_beforeCenter_1 = true;
  task_cv_centering_1 = true;


  // ROS_INFO("Master starts running. Checking each topic...");

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
      mControl.runningTime = 0;
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

  resetBoolVariables();

  ROS_INFO("Turning on motors...");
  //Task =======================================================================
  while(ros::ok() && !task_turnOnMotors){
    if(!receivedFromHControl){
      hControl.state = 4;
      hControl.depth = 9;
      hControlPublisher.publish(hControl);
    }
    settingCVInfo(0,0,0,0,0,0);
    cvInfoPublisher.publish(cvInfo);
    ros::spinOnce();
    loop_rate.sleep();
  }

  resetBoolVariables();

  //Task =======================================================================
  heightToMove = 8;
  while(ros::ok() && !task_submergeXft){
    if(!receivedFromHControl){
      hControl.state = 0;
      hControl.depth = heightToMove;
      hControlPublisher.publish(hControl);
    }
    settingCVInfo(0,0,0,0,0,0);
    cvInfoPublisher.publish(cvInfo);
    ros::spinOnce();
    loop_rate.sleep();
  }

  resetBoolVariables();

  //Task =======================================================================
  heightToMove = 5;
  while (ros::ok() && !task_emergeXft){
    if(!receivedFromHControl){
      hControl.state = 2;
      hControl.depth = heightToMove;
      hControlPublisher.publish(hControl);
    }
    settingCVInfo(0,0,0,0,0,0);
    cvInfoPublisher.publish(cvInfo);
    ros::spinOnce();
    loop_rate.sleep();
  }

  resetBoolVariables();

  //Task =======================================================================
  while (ros::ok() && !task_emergeToTop){
    if(!receivedFromHControl){
      hControl.state = 2;
      hControl.depth = -1;
      hControlPublisher.publish(hControl);
    }
    settingCVInfo(0,0,0,0,0,0);
    cvInfoPublisher.publish(cvInfo);
    ros::spinOnce();
    loop_rate.sleep();
  }

  resetBoolVariables();

  //Task =======================================================================
  angleToTurn = 179.9;
  while (ros::ok() && !task_rotateRightXD1){
    if(!receivedFromRControl){
      rControl.state = 2;
      rControl.rotation = angleToTurn;
      rControlPublisher.publish(rControl);
    }
    settingCVInfo(0,0,0,0,0,0);
    cvInfoPublisher.publish(cvInfo);
    ros::spinOnce();
    loop_rate.sleep();
  }

  resetBoolVariables();

  //Task =======================================================================
  angleToTurn = 179.9;
  while (ros::ok() && !task_rotateRightXD2){
    if(!receivedFromRControl){
      rControl.state = 2;
      rControl.rotation = angleToTurn;
      rControlPublisher.publish(rControl);
    }
    settingCVInfo(0,0,0,0,0,0);
    cvInfoPublisher.publish(cvInfo);
    ros::spinOnce();
    loop_rate.sleep();
  }

  resetBoolVariables();

  //Task =======================================================================
  angleToTurn = 179.9;
  while (ros::ok() && !task_rotateLeftXD1){
    if(!receivedFromRControl){
      rControl.state = 0;
      rControl.rotation = angleToTurn;
      rControlPublisher.publish(rControl);
    }
    settingCVInfo(0,0,0,0,0,0);
    cvInfoPublisher.publish(cvInfo);
    ros::spinOnce();
    loop_rate.sleep();
  }

  resetBoolVariables();

  //Task =======================================================================
  angleToTurn = 179.9;
  while (ros::ok() && !task_rotateLeftXD2){
    if(!receivedFromRControl){
      rControl.state = 0;
      rControl.rotation = angleToTurn;
      rControlPublisher.publish(rControl);
    }
    settingCVInfo(0,0,0,0,0,0);
    cvInfoPublisher.publish(cvInfo);
    ros::spinOnce();
    loop_rate.sleep();
  }

  resetBoolVariables();

  //Task =======================================================================
  while (ros::ok() && !task_keepRotatingRight){
    if(!receivedFromRControl){
      rControl.state = 2;
      rControl.rotation = -1;
      rControlPublisher.publish(rControl);
    }
    settingCVInfo(0,0,0,0,0,0);
    cvInfoPublisher.publish(cvInfo);
    ros::spinOnce();
    loop_rate.sleep();
  }

  resetBoolVariables();

  //Task =======================================================================
  while (ros::ok() && !task_keepRotatingLeft){
    if(!receivedFromRControl){
      rControl.state = 0;
      rControl.rotation = -1;
      rControlPublisher.publish(rControl);
    }
    settingCVInfo(0,0,0,0,0,0);
    cvInfoPublisher.publish(cvInfo);
    ros::spinOnce();
    loop_rate.sleep();
  }

  resetBoolVariables();

  //Task =======================================================================
  motorPower = 100;
  directionToMove = 4;
  while (ros::ok() && !task_mode1Movement){
    if(!receivedFromMControl){
      mControl.state = 1;
      mControl.mDirection = directionToMove;
      mControl.power = motorPower;
      mControl.distance = 0;
      mControl.runningTime = 0;
      mControlPublisher.publish(mControl);
    }
    settingCVInfo(0,0,0,0,0,0);
    cvInfoPublisher.publish(cvInfo);
    ros::spinOnce();
    loop_rate.sleep();
  }

  resetBoolVariables();

  //Task =======================================================================
  motorPower = 100;
  motorRunningTime = 8;
  directionToMove = 1;
  while (ros::ok() && !task_mode5Movement1){
    if(!receivedFromMControl){
      mControl.state = 5;
      mControl.mDirection = directionToMove;
      mControl.power = motorPower;
      mControl.distance = 0;
      mControl.runningTime = motorRunningTime;
      mControlPublisher.publish(mControl);
    }
    settingCVInfo(0,0,0,0,0,0);
    cvInfoPublisher.publish(cvInfo);
    ros::spinOnce();
    loop_rate.sleep();
  }

  resetBoolVariables();

  //Task =======================================================================
  motorPower = 100;
  motorRunningTime = 3;
  directionToMove = 3;
  while (ros::ok() && !task_mode5Movement2){
    if(!receivedFromMControl){
      mControl.state = 5;
      mControl.mDirection = directionToMove;
      mControl.power = motorPower;
      mControl.distance = 0;
      mControl.runningTime = motorRunningTime;
      mControlPublisher.publish(mControl);
    }
    settingCVInfo(0,0,0,0,0,0);
    cvInfoPublisher.publish(cvInfo);
    ros::spinOnce();
    loop_rate.sleep();
  }

  resetBoolVariables();

  ROS_INFO("Starting mission code - square");
  //Task =======================================================================
  heightToMove = 4;
  while(ros::ok() && !task_square_submergeXft){
    if(!receivedFromHControl){
      hControl.state = 0;
      hControl.depth = heightToMove;
      hControlPublisher.publish(hControl);
    }
    settingCVInfo(0,0,0,0,0,0);
    cvInfoPublisher.publish(cvInfo);
    ros::spinOnce();
    loop_rate.sleep();
  }

  resetBoolVariables();
  if(!task_square_mode5Movement1){
  for(int i = 0; i < 4; i++){

    //Task =======================================================================
    motorPower = 100;
    motorRunningTime = 8;
    directionToMove = 1;
    while (ros::ok() && !task_square_mode5Movement1){
      if(!receivedFromMControl){
        mControl.state = 5;
        mControl.mDirection = directionToMove;
        mControl.power = motorPower;
        mControl.distance = 0;
        mControl.runningTime = motorRunningTime;
        mControlPublisher.publish(mControl);
      }
      settingCVInfo(0,0,0,0,0,0);
      cvInfoPublisher.publish(cvInfo);
      ros::spinOnce();
      loop_rate.sleep();
    }

    resetBoolVariables();

    //Task =======================================================================
    motorPower = 100;
    motorRunningTime = 3;
    directionToMove = 3;
    while (ros::ok() && !task_square_mode5Movement2){
      if(!receivedFromMControl){
        mControl.state = 5;
        mControl.mDirection = directionToMove;
        mControl.power = motorPower;
        mControl.distance = 0;
        mControl.runningTime = motorRunningTime;
        mControlPublisher.publish(mControl);
      }
      settingCVInfo(0,0,0,0,0,0);
      cvInfoPublisher.publish(cvInfo);
      ros::spinOnce();
      loop_rate.sleep();
    }

    resetBoolVariables();

    //Task =======================================================================
    angleToTurn = 90;
    while (ros::ok() && !task_square_rotateRightXD){
      if(!receivedFromRControl){
        rControl.state = 2;
        rControl.rotation = angleToTurn;
        rControlPublisher.publish(rControl);
      }
      settingCVInfo(0,0,0,0,0,0);
      cvInfoPublisher.publish(cvInfo);
      ros::spinOnce();
      loop_rate.sleep();
    }

    resetBoolVariables();

    if(i != 3){
      task_square_mode5Movement1 = false;
      task_square_mode5Movement2 = false;
      task_square_rotateRightXD = false;
    }
  }
}

  //Task =======================================================================
  while (ros::ok() && !task_square_emergeToTop){
    if(!receivedFromHControl){
      hControl.state = 2;
      hControl.depth = -1;
      hControlPublisher.publish(hControl);
    }
    settingCVInfo(0,0,0,0,0,0);
    cvInfoPublisher.publish(cvInfo);
    ros::spinOnce();
    loop_rate.sleep();
  }
  ROS_INFO("Mission code -square finished.\n");

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
        mControl.runningTime = 0;
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
      mControl.power = 0;
      mControl.distance = 0;
      mControl.runningTime = 0;
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
  //ROS_INFO("Centering the sub with front camera...");
  while(ros::ok() && !task_cv_centering_1){
    if(!receivedFromMControl){
      mControl.state = 3;
      mControl.mDirection = 0;
      mControl.power = 0;
      mControl.distance = 0;
      mControl.runningTime = 0;
      mControlPublisher.publish(mControl);
    }
    settingCVInfo(1,1,2,3,4,5);
    cvInfoPublisher.publish(cvInfo);
    ros::spinOnce();
    loop_rate.sleep();
  }

  resetBoolVariables();

  //Executing... ===============================================================
  ROS_INFO("Missions completed!");
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
  else if(!task_turnOnMotors){}
  else if(!task_submergeXft){}
  else if(!task_emergeXft){}
  else if(!task_emergeToTop){}
  else if(!task_rotateRightXD1){}
  else if(!task_rotateRightXD2){}
  else if(!task_rotateLeftXD1){}
  else if(!task_rotateLeftXD2){}
  else if(!task_keepRotatingRight){}
  else if(!task_keepRotatingLeft){}
  else if(!task_mode1Movement){}
  else if(!task_mode5Movement1){}
  else if(!task_mode5Movement2){}
  else if(!task_square_submergeXft){}
  else if(!task_square_mode5Movement1){}
  else if(!task_square_mode5Movement2){}
  else if(!task_square_rotateRightXD){}
  else if(!task_square_emergeToTop){}
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
  else if(!task_turnOnMotors){}
  else if(!task_submergeXft){}
  else if(!task_emergeXft){}
  else if(!task_emergeToTop){}
  else if(!task_rotateRightXD1){}
  else if(!task_rotateRightXD2){}
  else if(!task_rotateLeftXD1){}
  else if(!task_rotateLeftXD2){}
  else if(!task_keepRotatingRight){}
  else if(!task_keepRotatingLeft){}
  else if(!task_mode1Movement){}
  else if(!task_mode5Movement1){}
  else if(!task_mode5Movement2){}
  else if(!task_square_submergeXft){}
  else if(!task_square_mode5Movement1){}
  else if(!task_square_mode5Movement2){}
  else if(!task_square_rotateRightXD){}
  else if(!task_square_emergeToTop){}
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
  else if(!task_turnOnMotors){}
  else if(!task_submergeXft){}
  else if(!task_emergeXft){}
  else if(!task_emergeToTop){}
  else if(!task_rotateRightXD1){}
  else if(!task_rotateRightXD2){}
  else if(!task_rotateLeftXD1){}
  else if(!task_rotateLeftXD2){}
  else if(!task_keepRotatingRight){}
  else if(!task_keepRotatingLeft){}
  else if(!task_mode1Movement){}
  else if(!task_mode5Movement1){}
  else if(!task_mode5Movement2){}
  else if(!task_square_submergeXft){}
  else if(!task_square_mode5Movement1){}
  else if(!task_square_mode5Movement2){}
  else if(!task_square_rotateRightXD){}
  else if(!task_square_emergeToTop){}
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
  else if(!task_turnOnMotors){}
  else if(!task_submergeXft){}
  else if(!task_emergeXft){}
  else if(!task_emergeToTop){}
  else if(!task_rotateRightXD1){}
  else if(!task_rotateRightXD2){}
  else if(!task_rotateLeftXD1){}
  else if(!task_rotateLeftXD2){}
  else if(!task_keepRotatingRight){}
  else if(!task_keepRotatingLeft){}
  else if(!task_mode1Movement){}
  else if(!task_mode5Movement1){}
  else if(!task_mode5Movement2){}
  else if(!task_square_submergeXft){}
  else if(!task_square_mode5Movement1){}
  else if(!task_square_mode5Movement2){}
  else if(!task_square_rotateRightXD){}
  else if(!task_square_emergeToTop){}
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
  else if(!task_turnOnMotors){
    if(!receivedFromHControl){
      ROS_INFO("Sending command to height_control - turn on the motors");
    }
    if(!receivedFromHControl && hc.state == 4 && hc.depth == 9){
      receivedFromHControl = true;
      ROS_INFO("height_control message received - motors are on");
    }
    if(receivedFromHControl && hc.state == 1 && hc.depth == 0){
      receivedFromHControl = false;
      task_turnOnMotors = true;
      ROS_INFO("Sub reached the initial assigned depth.\n");
    }
  }
  else if(!task_submergeXft){
    if(!receivedFromHControl){
      ROS_INFO("Sending command to height_control - submerge x ft");
    }
    if(!receivedFromHControl && hc.state == 0 && hc.depth == heightToMove){
      receivedFromHControl = true;
      ROS_INFO("height_control message received - submerging...");
    }
    if(receivedFromHControl && hc.state == 1 && hc.depth == 0){
      receivedFromHControl = false;
      task_submergeXft = true;
      ROS_INFO("Submerging completed.\n");
    }
  }
  else if(!task_emergeXft){
    if(!receivedFromHControl){
      ROS_INFO("Sending command to height_control - emerge x ft");
    }
    if(!receivedFromHControl && hc.state == 2 && hc.depth == heightToMove){
      receivedFromHControl = true;
      ROS_INFO("height_control message received - emerging...");
    }
    if(receivedFromHControl && hc.state == 1 && hc.depth == 0){
      receivedFromHControl = false;
      task_emergeXft = true;
      ROS_INFO("Emerging completed.\n");
    }
  }
  else if(!task_emergeToTop){
    if(!receivedFromHControl){
      ROS_INFO("Sending command to height_control - emerge to top");
    }
    if(!receivedFromHControl && hc.state == 2 && hc.depth == -1){
      receivedFromHControl = true;
      ROS_INFO("height_control message received - emerging...");
    }
    if(receivedFromHControl && hc.state == 1 && hc.depth == 0){
      receivedFromHControl = false;
      task_emergeToTop = true;
      ROS_INFO("Emerging completed.\n");
    }
  }
  else if(!task_rotateRightXD1){}
  else if(!task_rotateRightXD2){}
  else if(!task_rotateLeftXD1){}
  else if(!task_rotateLeftXD2){}
  else if(!task_keepRotatingRight){}
  else if(!task_keepRotatingLeft){}
  else if(!task_mode1Movement){}
  else if(!task_mode5Movement1){}
  else if(!task_mode5Movement2){}
  else if(!task_square_submergeXft){
    if(!receivedFromHControl){
      ROS_INFO("Sending command to height_control - submerge x ft");
    }
    if(!receivedFromHControl && hc.state == 0 && hc.depth == heightToMove){
      receivedFromHControl = true;
      ROS_INFO("height_control message received - submerging...");
    }
    if(receivedFromHControl && hc.state == 1 && hc.depth == 0){
      receivedFromHControl = false;
      task_square_submergeXft = true;
      ROS_INFO("Submerging completed.\n");
    }
  }
  else if(!task_square_mode5Movement1){}
  else if(!task_square_mode5Movement2){}
  else if(!task_square_rotateRightXD){}
  else if(!task_square_emergeToTop){
    if(!receivedFromHControl){
      ROS_INFO("Sending command to height_control - emerge to top");
    }
    if(!receivedFromHControl && hc.state == 2 && hc.depth == -1){
      receivedFromHControl = true;
      ROS_INFO("height_control message received - emerging...");
    }
    if(receivedFromHControl && hc.state == 1 && hc.depth == 0){
      receivedFromHControl = false;
      task_square_emergeToTop = true;
      ROS_INFO("Emerging completed.\n");
    }
  }
  else if(!task_cv_getTargetInfo_1){
    if(!doneHeightControl){
      if(!receivedFromHControl && hc.state != 1 && hc.depth == hControl.depth){
        receivedFromHControl = true;
      }
      if(receivedFromHControl && !finishedHeightControl && hc.state == 1 && hc.depth == 0){
        receivedFromHControl = false;
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
  else if(!task_turnOnMotors){}
  else if(!task_submergeXft){}
  else if(!task_emergeXft){}
  else if(!task_emergeToTop){}
  else if(!task_rotateRightXD1){
    if(!receivedFromRControl){
      ROS_INFO("Sending commend to rotation_control - rotate right x degrees");
    }
    if(!receivedFromRControl && rc.state == 2 && rc.rotation == angleToTurn){
      receivedFromRControl = true;
      ROS_INFO("rotation_control message received - rotating...");
    }
    if(receivedFromRControl && rc.state == 1 && rc.rotation == 0){
      receivedFromRControl = false;
      task_rotateRightXD1 = true;
      ROS_INFO("Rotating completed.\n");
    }
  }
  else if(!task_rotateRightXD2){
    if(!receivedFromRControl){
      ROS_INFO("Sending commend to rotation_control - rotate right x degrees");
    }
    if(!receivedFromRControl && rc.state == 2 && rc.rotation == angleToTurn){
      receivedFromRControl = true;
      ROS_INFO("rotation_control message received - rotating...");
    }
    if(receivedFromRControl && rc.state == 1 && rc.rotation == 0){
      receivedFromRControl = false;
      task_rotateRightXD2 = true;
      ROS_INFO("Rotating completed.\n");
    }
  }
  else if(!task_rotateLeftXD1){
    if(!receivedFromRControl){
      ROS_INFO("Sending commend to rotation_control - rotate left x degrees");
    }
    if(!receivedFromRControl && rc.state == 0 && rc.rotation == angleToTurn){
      receivedFromRControl = true;
      ROS_INFO("rotation_control message received - rotating...");
    }
    if(receivedFromRControl && rc.state == 1 && rc.rotation == 0){
      receivedFromRControl = false;
      task_rotateLeftXD1 = true;
      ROS_INFO("Rotating completed.\n");
    }
  }
  else if(!task_rotateLeftXD2){
    if(!receivedFromRControl){
      ROS_INFO("Sending commend to rotation_control - rotate left x degrees");
    }
    if(!receivedFromRControl && rc.state == 0 && rc.rotation == angleToTurn){
      receivedFromRControl = true;
      ROS_INFO("rotation_control message received - rotating...");
    }
    if(receivedFromRControl && rc.state == 1 && rc.rotation == 0){
      receivedFromRControl = false;
      task_rotateLeftXD2 = true;
      ROS_INFO("Rotating completed.\n");
    }
  }
  else if(!task_keepRotatingRight){
    if(!receivedFromRControl){
      ROS_INFO("Sending commend to rotation_control - keep rotating right");
    }
    if(!receivedFromRControl && rc.state == 2 && rc.rotation == -1){
      receivedFromRControl = true;
      ROS_INFO("rotation_control message received - rotating...");
    }
    //Testing-------------------------------------------------------
    if(receivedFromRControl && rc.state == 1 && rc.rotation == 0){
      receivedFromRControl = false;
      task_keepRotatingRight = true;
      ROS_INFO("Rotating completed.\n");
    }
  }
  else if(!task_keepRotatingLeft){
    if(!receivedFromRControl){
      ROS_INFO("Sending commend to rotation_control - keep rotating left");
    }
    if(!receivedFromRControl && rc.state == 0 && rc.rotation == -1){
      receivedFromRControl = true;
      ROS_INFO("rotation_control message received - rotating...");
    }
    //Testing-------------------------------------------------------
    if(receivedFromRControl && rc.state == 1 && rc.rotation == 0){
      receivedFromRControl = false;
      task_keepRotatingLeft = true;
      ROS_INFO("Rotating completed.\n");
    }
  }
  else if(!task_mode1Movement){}
  else if(!task_mode5Movement1){}
  else if(!task_mode5Movement2){}
  else if(!task_square_submergeXft){}
  else if(!task_square_mode5Movement1){}
  else if(!task_square_mode5Movement2){}
  else if(!task_square_rotateRightXD){
    if(!receivedFromRControl){
      ROS_INFO("Sending commend to rotation_control - rotate right x degrees");
    }
    if(!receivedFromRControl && rc.state == 2 && rc.rotation == angleToTurn){
      receivedFromRControl = true;
      ROS_INFO("rotation_control message received - rotating...");
    }
    if(receivedFromRControl && rc.state == 1 && rc.rotation == 0){
      receivedFromRControl = false;
      task_square_rotateRightXD = true;
      ROS_INFO("Rotating completed.\n");
    }
  }
  else if(!task_square_emergeToTop){}
  else if(!task_cv_getTargetInfo_1){
    if(!doneRotationControl){
      if(!receivedFromRControl && rc.state != 1 && rc.rotation == rControl.rotation){
        receivedFromRControl = true;
      }
      if(receivedFromRControl && !finishedRotationControl && rc.state == 1 && rc.rotation == 0){
        receivedFromRControl = false;
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
      if(mc.state == 0 && mc.mDirection == 0 && mc.power == 0 && mc.distance == 0 && mc.runningTime == 0){
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
  else if(!task_turnOnMotors){}
  else if(!task_submergeXft){}
  else if(!task_emergeXft){}
  else if(!task_emergeToTop){}
  else if(!task_rotateRightXD1){}
  else if(!task_rotateRightXD2){}
  else if(!task_rotateLeftXD1){}
  else if(!task_rotateLeftXD2){}
  else if(!task_keepRotatingRight){}
  else if(!task_keepRotatingLeft){}
  else if(!task_mode1Movement){
    if(!receivedFromMControl){
      ROS_INFO("Sending command to movememt_control - keep moving with fixed power");
    }
    if(!receivedFromMControl && mc.state == 1){
      receivedFromMControl = true;
      ROS_INFO("movement_control message received - moving...");
    }
    //Testing----------------------------------------------------
    if(receivedFromMControl && mc.state == 0){
      receivedFromMControl = false;
      task_mode1Movement = true;
      ROS_INFO("Movemment completed.\n");
    }
  }
  else if(!task_mode5Movement1){
    if(!receivedFromMControl){
      ROS_INFO("Sending command to movememt_control - keep moving with fixed power for a specific time");
    }
    if(!receivedFromMControl && mc.state == 5){
      receivedFromMControl = true;
      ROS_INFO("movement_control message received - moving...");
    }
    if(receivedFromMControl && mc.state == 0){
      receivedFromMControl = false;
      task_mode5Movement1 = true;
      ROS_INFO("Movemment completed.\n");
    }
  }
  else if(!task_mode5Movement2){
    if(!receivedFromMControl){
      ROS_INFO("Sending command to movememt_control - keep moving with fixed power for a specific time");
    }
    if(!receivedFromMControl && mc.state == 5){
      receivedFromMControl = true;
      ROS_INFO("movement_control message received - moving...");
    }
    if(receivedFromMControl && mc.state == 0){
      receivedFromMControl = false;
      task_mode5Movement2 = true;
      ROS_INFO("Movemment completed.\n");
    }
  }
  else if(!task_square_submergeXft){}
  else if(!task_square_mode5Movement1){
    if(!receivedFromMControl){
      ROS_INFO("Sending command to movememt_control - keep moving with fixed power for a specific time");
    }
    if(!receivedFromMControl && mc.state == 5){
      receivedFromMControl = true;
      ROS_INFO("movement_control message received - moving...");
    }
    if(receivedFromMControl && mc.state == 0){
      receivedFromMControl = false;
      task_square_mode5Movement1 = true;
      ROS_INFO("Movemment completed.\n");
    }
  }
  else if(!task_square_mode5Movement2){
    if(!receivedFromMControl){
      ROS_INFO("Sending command to movememt_control - keep moving with fixed power for a specific time");
    }
    if(!receivedFromMControl && mc.state == 5){
      receivedFromMControl = true;
      ROS_INFO("movement_control message received - moving...");
    }
    if(receivedFromMControl && mc.state == 0){
      receivedFromMControl = false;
      task_square_mode5Movement2 = true;
      ROS_INFO("Movemment completed.\n");
    }
  }
  else if(!task_square_rotateRightXD){}
  else if(!task_square_emergeToTop){}
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
      receivedFromMControl = false;
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
  else if(!task_turnOnMotors){}
  else if(!task_submergeXft){}
  else if(!task_emergeXft){}
  else if(!task_emergeToTop){}
  else if(!task_rotateRightXD1){}
  else if(!task_rotateRightXD2){}
  else if(!task_rotateLeftXD1){}
  else if(!task_rotateLeftXD2){}
  else if(!task_mode1Movement){}
  else if(!task_mode5Movement1){}
  else if(!task_mode5Movement2){}
  else if(!task_square_submergeXft){}
  else if(!task_square_mode5Movement1){}
  else if(!task_square_mode5Movement2){}
  else if(!task_square_rotateRightXD){}
  else if(!task_square_emergeToTop){}
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
