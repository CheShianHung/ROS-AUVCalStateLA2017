#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include <sstream>

//Leveling State
// 0: going down
// 1: staying
// 2: going up

bool depthReceive;
std_msgs::Int32 motorState;

void depthCallback(const std_msgs::Float32& depth){
  depthReceive = true;
  if(depth.data < 4) {
    motorState.data = 0;
    ROS_INFO("Sub going down...");
  }
  else if(depth.data > 5) {
    motorState.data = 2;
    ROS_INFO("Sub going up...");
  }
  else {
    motorState.data = 1;
    ROS_INFO("Sub staying...");
  }
}

int main(int argc, char **argv){

  depthReceive = false;
  motorState.data = 1;
  
  ros::init(argc, argv, "master");
  ros::NodeHandle node;
  ros::Subscriber depthSubscriber = node.subscribe("depth", 100, depthCallback);
  ros::Publisher motorStatePublisher = node.advertise<std_msgs::Int32>("height_control", 100);
  ros::Rate loop_rate(100);
  
  ROS_INFO("Master starts running. Sub staying still...");
 
  while(ros::ok()){

    if(!depthReceive){
      motorState.data = 1;
    }

    motorStatePublisher.publish(motorState);
    ros::spinOnce();
    loop_rate.sleep();    

  }   

  return 0;

}
