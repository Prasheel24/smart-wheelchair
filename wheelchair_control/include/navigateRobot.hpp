#ifndef INCLUDE_NAVIGATEROBOT_HPP_
#define INCLUDE_NAVIGATEROBOT_HPP_

#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>

class NavigateRobot {
 public:
  bool flag;

  NavigateRobot();

  ~NavigateRobot();

  void twistRobot(const geometry_msgs::TwistConstPtr &msg);

  int start(bool flag);
};

#endif  //  INCLUDE_NAVIGATEROBOT_HPP_
