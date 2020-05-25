#include <ros/ros.h>
#include "navigateRobot.hpp"

int main(int argc, char** argv) {
//  main node to start
ros::init(argc, argv, "wheelchair_listener");
NavigateRobot wheelchair;
int var = wheelchair.start(true);
return 0;
}
