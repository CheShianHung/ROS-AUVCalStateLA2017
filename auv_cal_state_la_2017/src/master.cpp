#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "auv_cal_state_la_2017/HControl.h"
#include "auv_cal_state_la_2017/RControl.h"
#include "auv_cal_state_la_2017/MControl.h"
#include "auv_cal_state_la_2017/Rotation.h"
#include "auv_cal_state_la_2017/FrontCamDistance.h"
#include "auv_cal_state_la_2017/BottomCamDistance.h"
#include "auv_cal_state_la_2017/TargetInfo.h"
#include "auv_cal_state_la_2017/CVInfo.h"
#include "auv_cal_state_la_2017/Hydrophone.h"
#include <sstream>

// height_control: (int) state, (float) depth
// state: going down (0), staying (1), going up (2)
// depth: nonstop moving (-1), moving distance (x)

// rotation_control: (int) state, (float) rotation
// state: rotate left (0), staying (1), rotate right (2), rotate with fcd(3)
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
std_msgs::Int32 pControl;
auv_cal_state_la_2017::HControl hControl;
auv_cal_state_la_2017::RControl rControl;
auv_cal_state_la_2017::MControl mControl;
auv_cal_state_la_2017::CVInfo cvInfo;

//Subscriber callback functions
void currentDepthCallback(const std_msgs::Float32& currentDepth);
void currentRotationCallback(const auv_cal_state_la_2017::Rotation rotation);
void pControlStatusCallback(const std_msgs::Int32 pc);
void hControlStatusCallback(const auv_cal_state_la_2017::HControl hc);
void rControlStatusCallback(const auv_cal_state_la_2017::RControl rc);
void mControlStatusCallback(const auv_cal_state_la_2017::MControl mc);
void frontCamDistanceCallback(const auv_cal_state_la_2017::FrontCamDistance fcd);
void bottomCamDistanceCallback(const auv_cal_state_la_2017::BottomCamDistance bcd);
void targetInfoCallback(const auv_cal_state_la_2017::TargetInfo ti);
void hydrophoneCallback(const auv_cal_state_la_2017::Hydrophone hy);

//Regular functions
void checkMotorNode();
void checkCVNode();
void settingCVInfo(int cameraNum, int taskNum, int givenColor, int givenShape, float givenFloat, float givenDistance);
void resetVariables();
void breakBetweenTasks(int seconds);

//Regular variables
const float angleError = 5.0;
const float heightError = 0.2;
int directionToMove;
int pneumaticsNum;
int hydrophoneDirection;
int targetInfoCounter;
float depth;
float roll;
float pitch;
float yaw;
float cvTime;
float cvTimer;
float angleToTurn;
float heightToMove;
float motorPower;
float motorRunningTime;
float hydrophoneAngle;
float frontCamForwardDistance;
float frontCamHorizontalDistance;
float frontCamVerticalDistance;
float bottomCamForwardDistance;
float bottomCamHorizontalDistance;
float bottomCamVerticalDistance;

//float currentTargetDepth;
//float currentTargetRotation;

//Checking variables
bool checkingCurrentDepth;
bool checkingHeightControl;
bool checkingPneumaticsControl;
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
bool receivedFromPControl;
bool receivedFromRControl;
bool receivedFromHControl;
bool receivedFromMControl;
bool receivedFromCV;
bool doneRotationControl;
bool doneHeightControl;
bool finishedRotationControl;
bool finishedHeightControl;
bool objectFound;
bool findingObject;

//Task variables
bool task0_submergeToWater;
bool task_submergeXft;
bool task_turnOnMotors;
bool task_emergeXft;
bool task_emergeToTop;
bool task_rotateRightXd1;
bool task_rotateRightXd2;
bool task_rotateRightXd3;
bool task_rotateLeftXd1;
bool task_rotateLeftXd2;
bool task_rotateLeftXd3;
bool task_keepRotatingRight;
bool task_keepRotatingLeft;
bool task_submergeXft2;
bool task_mode1Movement;
bool task_mode5Movement1;
bool task_mode5Movement2;
bool task_pneumaticsControl1;
bool task_pneumaticsControl2;
bool task_pneumaticsControl3;
bool task_pneumaticsControl4;

bool task_gate1_submergeXft;
bool task_gate1_findGate;
bool task_gate1_changeAngle;
bool task_gate1_mode5Forward;
bool task_gate1_mode5Break;
bool task_gate1_emergeToTop;

bool task_square_submergeXft;
bool task_square_mode5Movement1;
bool task_square_mode5Movement2;
bool task_square_rotateRightXd;
bool task_square_emergeToTop;

bool task_cv_findingObject_testing;
bool task_cv_getDistance_testing;
bool task_cv_getTargetInfo_1;
bool task_cv_beforeCenter_1;
bool task_cv_centering_1;

bool task_hydrophone_finding;

int main(int argc, char **argv){

  //Initializing ROS variables
  ros::init(argc, argv, "master");
  ros::NodeHandle node;
  ros::Subscriber currentDepthSubscriber = node.subscribe("current_depth", 100, currentDepthCallback);
  ros::Subscriber currentRotationSubscriber = node.subscribe("current_rotation", 100, currentRotationCallback);
  ros::Subscriber pControlStatusSubscriber = node.subscribe("pneumatics_control_status", 100, pControlStatusCallback);
  ros::Subscriber hControlStatusSubscriber = node.subscribe("height_control_status", 100, hControlStatusCallback);
  ros::Subscriber rControlStatusSubscriber = node.subscribe("rotation_control_status", 100, rControlStatusCallback);
  ros::Subscriber mControlStatusSubscriber = node.subscribe("movement_control_status", 100, mControlStatusCallback);
  ros::Subscriber frontCamDistanceSubscriber = node.subscribe("front_cam_distance", 100, frontCamDistanceCallback);
  ros::Subscriber bottomCamDistanceSubscriber = node.subscribe("bottom_cam_distance", 100, bottomCamDistanceCallback);
  ros::Subscriber targetInfoSubscriber = node.subscribe("target_info", 100, targetInfoCallback);
  ros::Subscriber hydrophoneSubscriber = node.subscribe("hydrophone", 100, hydrophoneCallback);
  ros::Publisher pControlPublisher = node.advertise<std_msgs::Int32>("pneumatics_control", 100);
  ros::Publisher hControlPublisher = node.advertise<auv_cal_state_la_2017::HControl>("height_control", 100);
  ros::Publisher rControlPublisher = node.advertise<auv_cal_state_la_2017::RControl>("rotation_control", 100);
  ros::Publisher mControlPublisher = node.advertise<auv_cal_state_la_2017::MControl>("movement_control", 100);
  ros::Publisher cvInfoPublisher = node.advertise<auv_cal_state_la_2017::CVInfo>("cv_info", 100);
  ros::Rate loop_rate(10);

  resetVariables();

  checkingCurrentDepth = false;
  checkingHeightControl = false;
  checkingPneumaticsControl = false;
  checkingFrontCamDistance = false;
  checkingBottomCamDistance = false;
  checkingCurrentRotation = false;
  checkingRotationControl = false;
  checkingMovementControl = false;
  checkingTargetInfo = false;
  motorNodeIsReady = false;
  cvNodeIsReady = true;
  allNodesAreReady = true;

  depth = 0;
  roll = 0;
  pitch = 0;
  yaw = 0;
  cvTime = 0;
  cvTimer = 0;
  directionToMove = 0;
  pneumaticsNum = 0;
  angleToTurn = 0;
  heightToMove = 0;
  motorPower = 0;
  motorRunningTime = 0;
  hydrophoneDirection = 0;
  hydrophoneAngle = 0;
  targetInfoCounter = 0;
  frontCamForwardDistance = 0;
  frontCamHorizontalDistance = 0;
  frontCamVerticalDistance = 0;
  bottomCamForwardDistance = 0;
  bottomCamHorizontalDistance = 0;
  bottomCamVerticalDistance = 0;

  task0_submergeToWater = true;
  task_turnOnMotors = false;
  task_submergeXft = false;
  task_emergeXft = true;
  task_emergeToTop = true;
  task_rotateRightXd1 = false;
  task_rotateRightXd2 = true;
  task_rotateRightXd3 = true;
  task_rotateLeftXd1 = false;
  task_rotateLeftXd2 = true;
  task_rotateLeftXd3 = true;
  task_keepRotatingRight = true;
  task_keepRotatingLeft = true;
  task_submergeXft2 = true;
  task_mode1Movement = true;
  task_mode5Movement1 = false;
  task_mode5Movement2 = false;
  task_pneumaticsControl1 = true;
  task_pneumaticsControl2 = true;
  task_pneumaticsControl3 = true;
  task_pneumaticsControl4 = true;

  task_gate1_submergeXft = true;
  task_gate1_findGate = true;
  task_gate1_changeAngle = true;
  task_gate1_mode5Forward = true;
  task_gate1_mode5Break = true;
  task_gate1_emergeToTop = true;

  task_square_submergeXft = true;
  task_square_mode5Movement1 = true;
  task_square_mode5Movement2 = true;
  task_square_rotateRightXd = true;
  task_square_emergeToTop = true;

  task_cv_findingObject_testing = true;
  task_cv_getDistance_testing = false;
  task_cv_getTargetInfo_1 = true;
  task_cv_beforeCenter_1 = true;
  task_cv_centering_1 = true;

  task_hydrophone_finding = true;


  // ROS_INFO("Master starts running. Checking each topic...");
  //breakBetweenTasks(180);

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
    if(!checkingPneumaticsControl){
      pControl.data = 9;
      pControlPublisher.publish(pControl);
    }

    ros::spinOnce();

    if(motorNodeIsReady && cvNodeIsReady && checkingPneumaticsControl){
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

  resetVariables();
  // breakBetweenTasks(60);

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

  resetVariables();
  // breakBetweenTasks(15);

  //Task =======================================================================
  heightToMove = 2;
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

  resetVariables();
  // breakBetweenTasks(30);

  //Task =======================================================================
  heightToMove = 7;
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

  resetVariables();

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

  resetVariables();

  //Task =======================================================================
  angleToTurn = 20;
  while (ros::ok() && !task_rotateRightXd1){
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

  resetVariables();
  // breakBetweenTasks(30);

  //Task =======================================================================
  angleToTurn = 90;
  while (ros::ok() && !task_rotateRightXd2){
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

  resetVariables();
  // breakBetweenTasks(30);

  //Task =======================================================================
  angleToTurn = 180;
  while (ros::ok() && !task_rotateRightXd3){
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

  resetVariables();
  // breakBetweenTasks(30);

  //Task =======================================================================
  angleToTurn = 20;
  while (ros::ok() && !task_rotateLeftXd1){
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

  resetVariables();
  // breakBetweenTasks(15);

  //Task =======================================================================
  angleToTurn = 90;
  while (ros::ok() && !task_rotateLeftXd2){
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

  resetVariables();
  // breakBetweenTasks(30);

  //Task =======================================================================
  angleToTurn = 180;
  while (ros::ok() && !task_rotateLeftXd3){
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

  resetVariables();
  // breakBetweenTasks(30);

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

  resetVariables();

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

  resetVariables();

  //Task =======================================================================
  heightToMove = 8;
  while(ros::ok() && !task_submergeXft2){
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

  resetVariables();
  // breakBetweenTasks(60);

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

  resetVariables();

  //Task =======================================================================
  motorPower = 100;
  motorRunningTime = 3;
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

  resetVariables();

  //Task =======================================================================
  motorPower = 100;
  motorRunningTime = 1;
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

  resetVariables();

  //Task =======================================================================
  pneumaticsNum = 7;
  while (ros::ok() && !task_pneumaticsControl1){
    if(!receivedFromPControl){
      pControl.data = pneumaticsNum;
      pControlPublisher.publish(pControl);
    }
    settingCVInfo(0,0,0,0,0,0);
    cvInfoPublisher.publish(cvInfo);
    ros::spinOnce();
    loop_rate.sleep();
  }

  resetVariables();
  //breakBetweenTasks(5);

  //Task =======================================================================
  pneumaticsNum = 6;
  while (ros::ok() && !task_pneumaticsControl2){
    if(!receivedFromPControl){
      pControl.data = pneumaticsNum;
      pControlPublisher.publish(pControl);
    }
    settingCVInfo(0,0,0,0,0,0);
    cvInfoPublisher.publish(cvInfo);
    ros::spinOnce();
    loop_rate.sleep();
  }

  resetVariables();

  //Task =======================================================================
  pneumaticsNum = 5;
  while (ros::ok() && !task_pneumaticsControl3){
    if(!receivedFromPControl){
      pControl.data = pneumaticsNum;
      pControlPublisher.publish(pControl);
    }
    settingCVInfo(0,0,0,0,0,0);
    cvInfoPublisher.publish(cvInfo);
    ros::spinOnce();
    loop_rate.sleep();
  }

  resetVariables();
  //breakBetweenTasks(5);

  //Task =======================================================================
  pneumaticsNum = 4;
  while (ros::ok() && !task_pneumaticsControl4){
    if(!receivedFromPControl){
      pControl.data = pneumaticsNum;
      pControlPublisher.publish(pControl);
    }
    settingCVInfo(0,0,0,0,0,0);
    cvInfoPublisher.publish(cvInfo);
    ros::spinOnce();
    loop_rate.sleep();
  }

  resetVariables();

  //Task =======================================================================
  heightToMove = 4;
  while(ros::ok() && !task_gate1_submergeXft){
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

  resetVariables();

  if(task_gate1_findGate){
    ROS_INFO("Mission - Gate\n");
    for(int i = 0; i < 4; i++){

      ROS_INFO("Finding object...");
      //Task =======================================================================
      while (ros::ok() && !task_gate1_findGate){
        findingObject = true;
        settingCVInfo(1,1,2,0,49,60);
        cvInfoPublisher.publish(cvInfo);
        ros::spinOnce();
        loop_rate.sleep();
      }

      if(objectFound){
        resetVariables();

        ROS_INFO("Changing angle...");
        //Task =======================================================================
        while (ros::ok() && !task_gate1_changeAngle){
          if(receivedFromCV && !receivedFromRControl){
            rControl.state = 3;
            rControl.rotation = 0;
            rControlPublisher.publish(rControl);
          }
          settingCVInfo(1,1,2,0,49,60);
          cvInfoPublisher.publish(cvInfo);
          ros::spinOnce();
          loop_rate.sleep();
        }

        resetVariables();

        //Task =======================================================================
        motorPower = 250;
        motorRunningTime = 10;
        directionToMove = 1;
        while (ros::ok() && !task_gate1_mode5Forward){
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

        resetVariables();

        //Task =======================================================================
        motorPower = 250;
        motorRunningTime = 1;
        directionToMove = 3;
        while (ros::ok() && !task_gate1_mode5Break){
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

        resetVariables();

      }
      else{
        i = 4;
        ROS_INFO("Pass through the gate.\n");
        resetVariables();
      }
    }
  }

  //Task =======================================================================
  while (ros::ok() && !task_gate1_emergeToTop){
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

  resetVariables();


  //ROS_INFO("Starting mission code - square");
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

  resetVariables();
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

    resetVariables();

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

    resetVariables();

    //Task =======================================================================
    angleToTurn = 90;
    while (ros::ok() && !task_square_rotateRightXd){
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

    resetVariables();

    if(i != 3){
      task_square_mode5Movement1 = false;
      task_square_mode5Movement2 = false;
      task_square_rotateRightXd = false;
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
  //ROS_INFO("Mission code -square finished.\n");

  resetVariables();

  //settingCVInfo(cameraNum,taskNum,givenColor,givenShape,givenLength,givenDistance)
  //Task =======================================================================
  while (ros::ok() && !task_cv_findingObject_testing){
    ROS_INFO("Finding object...");
    while(ros::ok() && !objectFound){
      settingCVInfo(1,1,2,0,49,60);
      //settingCVInfo(1,2,2,0,49,60);
      //settingCVInfo(1,4,6,0,0,0);
      //settingCVInfo(1,4,7,0,0,0);
      cvInfoPublisher.publish(cvInfo);
      ros::spinOnce();
      loop_rate.sleep();
    }
    ROS_INFO("Mission code - object found.\n");
  }

  resetVariables();

  //settingCVInfo(cameraNum,taskNum,givenColor,givenShape,givenLength,givenDistance)
  //Task =======================================================================
  cvTime = 30;
  cvTimer = 0;
  if(!task_cv_getDistance_testing)
    ROS_INFO("Getting distance from computer vision.");
  while (ros::ok() && !task_cv_getDistance_testing){
    //settingCVInfo(1,1,2,0,49,60);
    settingCVInfo(1,2,2,0,49,60);
    //settingCVInfo(1,4,6,0,0,0);
    //settingCVInfo(1,4,7,0,0,0);
    cvInfoPublisher.publish(cvInfo);
    ros::spinOnce();
    loop_rate.sleep();
    cvTimer += 0.1;
    if(cvTimer >= cvTime){
      task_cv_getDistance_testing = true;
      ROS_INFO("Finish getting distance\n");
    } 
  }

  resetVariables();

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

  resetVariables();

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

  resetVariables();

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

  resetVariables();

  //Task =======================================================================
  while(ros::ok() && !task_hydrophone_finding){
    settingCVInfo(0,0,0,0,0,0);
    cvInfoPublisher.publish(cvInfo);
    ros::spinOnce();
    loop_rate.sleep();
  }

  resetVariables();

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



void breakBetweenTasks(int seconds){
  float t = 0;
  ros::Rate rate(10);
  ROS_INFO("breaking...");
  while(ros::ok() && t <= seconds){
    t += 0.1;
    rate.sleep();
  }
}



void resetVariables(){
  receivedFromPControl = false;
  receivedFromRControl = false;
  receivedFromHControl = false;
  receivedFromMControl = false;
  receivedFromCV = false;
  doneRotationControl = false;
  doneHeightControl = false;
  finishedRotationControl = false;
  finishedHeightControl = false;
  objectFound = false;
  findingObject = false;
  targetInfoCounter = 0;
  cvTime = 0;
  cvTimer = 0;
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
    if(currentDepth.data > 0.3){
       task0_submergeToWater = true;
       ROS_INFO("Sub is now under water... Ready to get start...");
    }
  }

  depth = currentDepth.data;
}



void currentRotationCallback(const auv_cal_state_la_2017::Rotation rotation){
  if(!allNodesAreReady){
    if(!checkingCurrentRotation){
      checkingCurrentRotation = true;
      ROS_INFO("current_rotation is ready.");
      checkMotorNode();
    }
  }

  roll = rotation.roll;
  pitch = rotation.pitch;
  yaw = rotation.yaw;
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

  frontCamForwardDistance = fcd.frontCamForwardDistance;
  frontCamHorizontalDistance = fcd.frontCamHorizontalDistance;
  frontCamVerticalDistance = fcd.frontCamVerticalDistance;
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

  bottomCamForwardDistance = bcd.bottomCamForwardDistance;
  bottomCamHorizontalDistance = bcd.bottomCamHorizontalDistance;
  bottomCamVerticalDistance = bcd.bottomCamVerticalDistance;
}



void pControlReceiveCheck(bool* currentTask, int pcData){
  if(!receivedFromPControl){
     ROS_INFO("Sending command to pneumatics_control - trigger #x");
  }
  if(!receivedFromPControl && pcData == pneumaticsNum){
    receivedFromPControl = true;
    ROS_INFO("pneumatics_control message received");
  }
  if(receivedFromPControl && pcData == 0){
    receivedFromPControl = false;
    *currentTask = true;
    ROS_INFO("Pneumatics #x triggered successfully.\n");
  }
}

void pControlStatusCallback(const std_msgs::Int32 pc){
  int pcData = pc.data;

  if(!allNodesAreReady){
    if(!checkingPneumaticsControl){
      ROS_INFO("Checking pneumatics_control...");
      if(pc.data == 0){
        checkingPneumaticsControl = true;
        ROS_INFO("pneumatics_control is ready.");
      }
    }
  }
  else if(!task0_submergeToWater){}
  else if(!task_turnOnMotors){}
  else if(!task_submergeXft){}
  else if(!task_emergeXft){}
  else if(!task_emergeToTop){}
  else if(!task_rotateRightXd1){}
  else if(!task_rotateRightXd2){}
  else if(!task_rotateRightXd3){}
  else if(!task_rotateLeftXd1){}
  else if(!task_rotateLeftXd2){}
  else if(!task_rotateLeftXd3){}
  else if(!task_keepRotatingRight){}
  else if(!task_keepRotatingLeft){}
  else if(!task_submergeXft2){}
  else if(!task_mode1Movement){}
  else if(!task_mode5Movement1){}
  else if(!task_mode5Movement2){}
  else if(!task_pneumaticsControl1){
    pControlReceiveCheck(&task_pneumaticsControl1, pcData);
  }
  else if(!task_pneumaticsControl2){
    pControlReceiveCheck(&task_pneumaticsControl2, pcData);
  }
  else if(!task_pneumaticsControl3){
    pControlReceiveCheck(&task_pneumaticsControl3, pcData);
  }
  else if(!task_pneumaticsControl4){
    pControlReceiveCheck(&task_pneumaticsControl4, pcData);
  }
  else if(!task_gate1_submergeXft){}
  else if(!task_gate1_findGate){}
  else if(!task_gate1_changeAngle){}
  else if(!task_gate1_mode5Forward){}
  else if(!task_gate1_mode5Break){}
  else if(!task_gate1_emergeToTop){}
  else if(!task_square_submergeXft){}
  else if(!task_square_mode5Movement1){}
  else if(!task_square_mode5Movement2){}
  else if(!task_square_rotateRightXd){}
  else if(!task_square_emergeToTop){}
  else if(!task_cv_findingObject_testing){}
  else if(!task_cv_getDistance_testing){}
  else if(!task_cv_getTargetInfo_1){}
  else if(!task_cv_beforeCenter_1){}
  else if(!task_cv_centering_1){}
  else if(!task_hydrophone_finding){}
}



void hControlReceiveCheck(int hState, float hDepth, bool* currentTask, int hcState, float hcDepth){
  if(!receivedFromHControl){
    if(hState == 0)
      ROS_INFO("Sending command to height_control - submerge x ft");
    if(hState == 2)
      ROS_INFO("Sending command to height_control - emerge x ft");
    if(hState == 4)
      ROS_INFO("Sending command to height_control - turn on the motors");
  }
  if(!receivedFromHControl && hcState == hState && hcDepth == hDepth){
    receivedFromHControl = true;
    if(hState == 0)
      ROS_INFO("height_control message received - submerging...");
    if(hState == 2)
      ROS_INFO("height_control message received - emerging...");
    if(hState == 4)
      ROS_INFO("height_control message received - motors are on");
  }
  if(receivedFromHControl && hcState == 1 && hcDepth == 0){
    receivedFromHControl = false;
    *currentTask = true;
    if(hState == 0)
      ROS_INFO("Submerging completed.\n");
    if(hState == 2)
      ROS_INFO("Emerging completed.\n");
    if(hState == 4)
      ROS_INFO("Sub reached the initial assigned depth.\n");
  }
}

void hControlStatusCallback(const auv_cal_state_la_2017::HControl hc){
  int hcState = hc.state;
  float hcDepth = hc.depth;

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
    hControlReceiveCheck(4,9,&task_turnOnMotors,hcState,hcDepth);
  }
  else if(!task_submergeXft){
    hControlReceiveCheck(0,heightToMove,&task_submergeXft,hcState,hcDepth);
  }
  else if(!task_emergeXft){
    hControlReceiveCheck(2,heightToMove,&task_emergeXft,hcState,hcDepth);
  }
  else if(!task_emergeToTop){
    hControlReceiveCheck(2,-1,&task_emergeToTop,hcState,hcDepth);
  }
  else if(!task_rotateRightXd1){}
  else if(!task_rotateRightXd2){}
  else if(!task_rotateRightXd3){}
  else if(!task_rotateLeftXd1){}
  else if(!task_rotateLeftXd2){}
  else if(!task_rotateLeftXd3){}
  else if(!task_keepRotatingRight){}
  else if(!task_keepRotatingLeft){}
  else if(!task_submergeXft2){
    hControlReceiveCheck(0,heightToMove,&task_submergeXft2,hcState,hcDepth);
  }
  else if(!task_mode1Movement){}
  else if(!task_mode5Movement1){}
  else if(!task_mode5Movement2){}
  else if(!task_pneumaticsControl1){}
  else if(!task_pneumaticsControl2){}
  else if(!task_pneumaticsControl3){}
  else if(!task_pneumaticsControl4){}
  else if(!task_gate1_submergeXft){
    hControlReceiveCheck(0,heightToMove,&task_gate1_submergeXft,hcState,hcDepth);
  }
  else if(!task_gate1_findGate){}
  else if(!task_gate1_changeAngle){}
  else if(!task_gate1_mode5Forward){}
  else if(!task_gate1_mode5Break){}
  else if(!task_gate1_emergeToTop){
    hControlReceiveCheck(2,-1,&task_gate1_emergeToTop,hcState,hcDepth);
  }
  else if(!task_square_submergeXft){
    hControlReceiveCheck(0,heightToMove,&task_square_submergeXft,hcState,hcDepth);
  }
  else if(!task_square_mode5Movement1){}
  else if(!task_square_mode5Movement2){}
  else if(!task_square_rotateRightXd){}
  else if(!task_square_emergeToTop){
    hControlReceiveCheck(2,-1,&task_square_emergeToTop,hcState,hcDepth);
  }
  else if(!task_cv_findingObject_testing){}
  else if(!task_cv_getDistance_testing){}
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
  else if(!task_hydrophone_finding){}
}



void rControlReceiveCheck(int rState, float rRotation, bool* currentTask, int rcState, float rcRotation){
  if(!receivedFromRControl){
    if(rState == 0 && rRotation != -1)
      ROS_INFO("Sending command to rotation_control - rotate left x degrees");
    if(rState == 0 && rRotation == -1)
      ROS_INFO("Sending command to rotation_control - keep rotating left");
    if(rState == 2 && rRotation != -1)
      ROS_INFO("Sending command to rotation_control - rotate right x degrees");
    if(rState == 2 && rRotation == -1)
      ROS_INFO("Sending command to rotation_control - keep rotating right");
    if(rState == 3)
      ROS_INFO("Sending commend to rotation_control - rotate according to fcd");
  }
  if(!receivedFromRControl && rcState == rState && rcRotation == rRotation){
    receivedFromRControl = true;
    ROS_INFO("rotation_control message received - rotating...");
  }
  if(receivedFromRControl && rcState == 1 && rcRotation == 0){
    receivedFromRControl = false;
    *currentTask = true;
    ROS_INFO("Rotating completed.\n");
  }
}

void rControlStatusCallback(const auv_cal_state_la_2017::RControl rc){
  int rcState = rc.state;
  float rcRotation = rc.rotation;
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
  else if(!task_rotateRightXd1){
    rControlReceiveCheck(2,angleToTurn,&task_rotateRightXd1,rcState,rcRotation);
  }
  else if(!task_rotateRightXd2){
    rControlReceiveCheck(2,angleToTurn,&task_rotateRightXd2,rcState,rcRotation);
  }
  else if(!task_rotateRightXd3){
    rControlReceiveCheck(2,angleToTurn,&task_rotateRightXd3,rcState,rcRotation);
  }
  else if(!task_rotateLeftXd1){
    rControlReceiveCheck(0,angleToTurn,&task_rotateLeftXd1,rcState,rcRotation);
  }
  else if(!task_rotateLeftXd2){
    rControlReceiveCheck(0,angleToTurn,&task_rotateLeftXd2,rcState,rcRotation);
  }
  else if(!task_rotateLeftXd3){
    rControlReceiveCheck(0,angleToTurn,&task_rotateLeftXd3,rcState,rcRotation);
  }
  else if(!task_keepRotatingRight){
    rControlReceiveCheck(2,-1,&task_keepRotatingRight,rcState,rcRotation);
  }
  else if(!task_keepRotatingLeft){
    rControlReceiveCheck(0,-1,&task_keepRotatingLeft,rcState,rcRotation);
  }
  else if(!task_submergeXft2){}
  else if(!task_mode1Movement){}
  else if(!task_mode5Movement1){}
  else if(!task_mode5Movement2){}
  else if(!task_pneumaticsControl1){}
  else if(!task_pneumaticsControl2){}
  else if(!task_pneumaticsControl3){}
  else if(!task_pneumaticsControl4){}
  else if(!task_gate1_submergeXft){}
  else if(!task_gate1_findGate){}
  else if(!task_gate1_changeAngle){
    rControlReceiveCheck(3,0,&task_gate1_changeAngle,rcState,rcRotation);
  }
  else if(!task_gate1_mode5Forward){}
  else if(!task_gate1_mode5Break){}
  else if(!task_gate1_emergeToTop){}
  else if(!task_square_submergeXft){}
  else if(!task_square_mode5Movement1){}
  else if(!task_square_mode5Movement2){}
  else if(!task_square_rotateRightXd){
    rControlReceiveCheck(2,angleToTurn,&task_square_rotateRightXd,rcState,rcRotation);  
  }
  else if(!task_square_emergeToTop){}
  else if(!task_cv_findingObject_testing){}
  else if(!task_cv_getDistance_testing){}
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
  else if(!task_hydrophone_finding){}
}



void mControlReceiveCheck(int mState, bool* currentTask, int mcState){
  if(!receivedFromMControl){
    if(mState == 1)
      ROS_INFO("Sending command to movememt_control - keep moving with fixed power");
    if(mState == 3)
      ROS_INFO("Sending command to movement_control - centering with front camera");
    if(mState == 5)
      ROS_INFO("Sending command to movement_control - keep moving for specific time");
  }
  if(!receivedFromMControl && mcState == mState){
    receivedFromMControl = true;
    ROS_INFO("movement_control message received - moving...");
  }
  if(receivedFromMControl && mcState == 0 && mState != 1){
    receivedFromMControl = false;
    *currentTask = true;
    ROS_INFO("Movemment completed.\n");
  }
}

void mControlStatusCallback(const auv_cal_state_la_2017::MControl mc){
  int mcState = mc.state;

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
  else if(!task_rotateRightXd1){}
  else if(!task_rotateRightXd2){}
  else if(!task_rotateRightXd3){}
  else if(!task_rotateLeftXd1){}
  else if(!task_rotateLeftXd2){}
  else if(!task_rotateLeftXd3){}
  else if(!task_keepRotatingRight){}
  else if(!task_keepRotatingLeft){}
  else if(!task_submergeXft2){}
  else if(!task_mode1Movement){
    mControlReceiveCheck(1,&task_mode1Movement,mcState);
    // if(!receivedFromMControl){
    //   ROS_INFO("Sending command to movememt_control - keep moving with fixed power");
    // }
    // if(!receivedFromMControl && mc.state == 1){
    //   receivedFromMControl = true;
    //   ROS_INFO("movement_control message received - moving...");
    // }
    // //Testing----------------------------------------------------
    // if(receivedFromMControl && mc.state == 0){
    //   receivedFromMControl = false;
    //   task_mode1Movement = true;
    //   ROS_INFO("Movemment completed.\n");
    // }
  }
  else if(!task_mode5Movement1){
    mControlReceiveCheck(5,&task_mode5Movement1,mcState);
    // if(!receivedFromMControl){
    //   ROS_INFO("Sending command to movememt_control - keep moving with fixed power for a specific time");
    // }
    // if(!receivedFromMControl && mc.state == 5){
    //   receivedFromMControl = true;
    //   ROS_INFO("movement_control message received - moving...");
    // }
    // if(receivedFromMControl && mc.state == 0){
    //   receivedFromMControl = false;
    //   task_mode5Movement1 = true;
    //   ROS_INFO("Movemment completed.\n");
    // }
  }
  else if(!task_mode5Movement2){
    mControlReceiveCheck(5,&task_mode5Movement2,mcState);
    // if(!receivedFromMControl){
    //   ROS_INFO("Sending command to movememt_control - keep moving with fixed power for a specific time");
    // }
    // if(!receivedFromMControl && mc.state == 5){
    //   receivedFromMControl = true;
    //   ROS_INFO("movement_control message received - moving...");
    // }
    // if(receivedFromMControl && mc.state == 0){
    //   receivedFromMControl = false;
    //   task_mode5Movement2 = true;
    //   ROS_INFO("Movemment completed.\n");
    // }
  }
  else if(!task_pneumaticsControl1){}
  else if(!task_pneumaticsControl2){}
  else if(!task_pneumaticsControl3){}
  else if(!task_pneumaticsControl4){}
  else if(!task_gate1_submergeXft){}
  else if(!task_gate1_findGate){}
  else if(!task_gate1_changeAngle){}
  else if(!task_gate1_mode5Forward){
    mControlReceiveCheck(5,&task_gate1_mode5Forward,mcState);
    // if(!receivedFromMControl){
    //   ROS_INFO("Sending command to movememt_control - keep moving with fixed power for a specific time");
    // }
    // if(!receivedFromMControl && mc.state == 5){
    //   receivedFromMControl = true;
    //   ROS_INFO("movement_control message received - moving...");
    // }
    // if(receivedFromMControl && mc.state == 0){
    //   receivedFromMControl = false;
    //   task_gate1_mode5Forward = true;
    //   ROS_INFO("Movemment completed.\n");
    // }
  }
  else if(!task_gate1_mode5Break){
    mControlReceiveCheck(5,&task_gate1_mode5Break,mcState);
    // if(!receivedFromMControl){
    //   ROS_INFO("Sending command to movememt_control - keep moving with fixed power for a specific time");
    // }
    // if(!receivedFromMControl && mc.state == 5){
    //   receivedFromMControl = true;
    //   ROS_INFO("movement_control message received - moving...");
    // }
    // if(receivedFromMControl && mc.state == 0){
    //   receivedFromMControl = false;
    //   task_gate1_mode5Break = true;
    //   ROS_INFO("Movemment completed.\n");
    // }
  }
  else if(!task_gate1_emergeToTop){}
  else if(!task_square_submergeXft){}
  else if(!task_square_mode5Movement1){
    mControlReceiveCheck(5,&task_square_mode5Movement1,mcState);
    // if(!receivedFromMControl){
    //   ROS_INFO("Sending command to movememt_control - keep moving with fixed power for a specific time");
    // }
    // if(!receivedFromMControl && mc.state == 5){
    //   receivedFromMControl = true;
    //   ROS_INFO("movement_control message received - moving...");
    // }
    // if(receivedFromMControl && mc.state == 0){
    //   receivedFromMControl = false;
    //   task_square_mode5Movement1 = true;
    //   ROS_INFO("Movemment completed.\n");
    // }
  }
  else if(!task_square_mode5Movement2){
    mControlReceiveCheck(5,&task_square_mode5Movement2,mcState);
    // if(!receivedFromMControl){
    //   ROS_INFO("Sending command to movememt_control - keep moving with fixed power for a specific time");
    // }
    // if(!receivedFromMControl && mc.state == 5){
    //   receivedFromMControl = true;
    //   ROS_INFO("movement_control message received - moving...");
    // }
    // if(receivedFromMControl && mc.state == 0){
    //   receivedFromMControl = false;
    //   task_square_mode5Movement2 = true;
    //   ROS_INFO("Movemment completed.\n");
    // }
  }
  else if(!task_square_rotateRightXd){}
  else if(!task_square_emergeToTop){}
  else if(!task_cv_findingObject_testing){}
  else if(!task_cv_getDistance_testing){}
  else if(!task_cv_getTargetInfo_1){}
  else if(!task_cv_beforeCenter_1){
    mControlReceiveCheck(5,&task_cv_beforeCenter_1,mcState);
    // if(!receivedFromMControl){
    //   ROS_INFO("Sending command to movememt_control - move towards target");
    // }
    // if(!receivedFromMControl && mc.state == 1){
    //   receivedFromMControl = true;
    //   ROS_INFO("movement_control message received - moving...");
    // }
    // if(objectFound && !receivedFromMControl && mc.state == 0){
    //   receivedFromMControl = true;
    //   ROS_INFO("Sub is now stopped.");
    // }
  }
  else if(!task_cv_centering_1){
    // rControlReceiveCheck(3,&task_cv_centering_1,mcState);
    // if(!receivedFromMControl){
    //   ROS_INFO("Sending command to movement_control - centering with front camera");
    // }
    // if(!receivedFromMControl && mc.state == 3){
    //   receivedFromMControl = true;
    //   ROS_INFO("movement_control message received - centering");
    // }
    // if(receivedFromMControl && mc.state == 0){
    //   receivedFromMControl = false;
    //   task_cv_centering_1 = true;
    //   ROS_INFO("Finished centering.");
    // }
  }
  else if(!task_hydrophone_finding){}
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
  else if(!task_rotateRightXd1){}
  else if(!task_rotateRightXd2){}
  else if(!task_rotateRightXd3){}
  else if(!task_rotateLeftXd1){}
  else if(!task_rotateLeftXd2){}
  else if(!task_rotateLeftXd3){}
  else if(!task_submergeXft2){}
  else if(!task_mode1Movement){}
  else if(!task_mode5Movement1){}
  else if(!task_mode5Movement2){}
  else if(!task_pneumaticsControl1){}
  else if(!task_pneumaticsControl2){}
  else if(!task_pneumaticsControl3){}
  else if(!task_pneumaticsControl4){}
  else if(!task_gate1_submergeXft){}
  else if(!task_gate1_findGate){
    //Object not found
    if(findingObject && ti.state == 0){
      targetInfoCounter++;
      if(targetInfoCounter >= 10){
        task_gate1_findGate = true;
        ROS_INFO("Object NOT found. The sub has been pasted through the gate.");
      }
    }
    //Object found
    if(!objectFound && ti.state == 1){
      ROS_INFO("Object found.");
      objectFound = true;
      task_gate1_findGate = true;
      directionToMove = ti.direction;
      angleToTurn = ti.angle;
      heightToMove = ti.height;
    }
  }
  else if(!task_gate1_changeAngle){}
  else if(!task_gate1_mode5Forward){}
  else if(!task_gate1_mode5Break){}
  else if(!task_gate1_emergeToTop){}
  else if(!task_square_submergeXft){}
  else if(!task_square_mode5Movement1){}
  else if(!task_square_mode5Movement2){}
  else if(!task_square_rotateRightXd){}
  else if(!task_square_emergeToTop){}
  else if(!task_cv_findingObject_testing){
    //Object found
    if(!objectFound && ti.state == 1){
      ROS_INFO("Object found.");
      objectFound = true;
      task_cv_findingObject_testing = true;
      directionToMove = ti.direction;
      angleToTurn = ti.angle;
      heightToMove = ti.height;
    }
  }
  else if(!task_cv_getDistance_testing){}
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
  else if(!task_hydrophone_finding){}
}



void hydrophoneCallback(const auv_cal_state_la_2017::Hydrophone hy){
  if(!allNodesAreReady){}
  else if(!task0_submergeToWater){}
  else if(!task_turnOnMotors){}
  else if(!task_submergeXft){}
  else if(!task_emergeXft){}
  else if(!task_emergeToTop){}
  else if(!task_rotateRightXd1){}
  else if(!task_rotateRightXd2){}
  else if(!task_rotateRightXd3){}
  else if(!task_rotateLeftXd1){}
  else if(!task_rotateLeftXd2){}
  else if(!task_rotateLeftXd3){}
  else if(!task_submergeXft2){}
  else if(!task_mode1Movement){}
  else if(!task_mode5Movement1){}
  else if(!task_mode5Movement2){}
  else if(!task_pneumaticsControl1){}
  else if(!task_pneumaticsControl2){}
  else if(!task_pneumaticsControl3){}
  else if(!task_pneumaticsControl4){}
  else if(!task_gate1_submergeXft){}
  else if(!task_gate1_findGate){}
  else if(!task_gate1_changeAngle){}
  else if(!task_gate1_mode5Forward){}
  else if(!task_gate1_mode5Break){}
  else if(!task_gate1_emergeToTop){}
  else if(!task_square_submergeXft){}
  else if(!task_square_mode5Movement1){}
  else if(!task_square_mode5Movement2){}
  else if(!task_square_rotateRightXd){}
  else if(!task_square_emergeToTop){}
  else if(!task_cv_findingObject_testing){}
  else if(!task_cv_getDistance_testing){}
  else if(!task_cv_getTargetInfo_1){}
  else if(!task_cv_beforeCenter_1){}
  else if(!task_cv_centering_1){}
  else if(!task_hydrophone_finding){
    ROS_INFO("Message received.");
    hydrophoneDirection = hy.direction;
    hydrophoneAngle = hy.angle;
  }
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
