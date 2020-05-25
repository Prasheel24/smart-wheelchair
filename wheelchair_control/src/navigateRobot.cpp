
#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>

#include "navigateRobot.hpp"

NavigateRobot::NavigateRobot() {
}

NavigateRobot::~NavigateRobot() {
}

void NavigateRobot::twistRobot(const geometry_msgs::TwistConstPtr &msg) {
  //  Set geometry messages to the robot
  double transVelocity = msg->linear.x;
  double rotVelocity = msg->angular.z;
  double velDiff = (0.143 * rotVelocity) / 2.0;
  double leftPower = (transVelocity + velDiff) / 0.0779;
  double rightPower = (transVelocity - velDiff) / 0.0779;
  //  check for individual node
  ROS_INFO_STREAM("\n Left wheel: " << leftPower
                  << ",  Right wheel: "<< rightPower << "\n");
}

int NavigateRobot::start(bool flag) {
  //  initialise node handle
  ros::NodeHandle nh;
  if (flag) {
    //  start a subcriber to the given topic
    ros::Subscriber sub =
        nh.subscribe("/wheelchair/mobile_base_controller/cmd_vel", 1000,
        &NavigateRobot::twistRobot, this);
    ros::Rate loop_rate(10);

    while (ros::ok()) {
      ros::spinOnce();
      loop_rate.sleep();
    }
  } 
  return 0;
}
