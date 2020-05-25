cmake_minimum_required(VERSION 3.2.1)
project(wheelchair_control)

add_compile_options(-std=c++11)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS 
  nav_core 
  move_base_msgs
  base_local_planner
  roscpp
  std_msgs
  tf
  rostest
  roscpp
  rospy
  message_generation)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES cleaner_robot
   CATKIN_DEPENDS nav_core roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

find_package(roslaunch)
add_library(pathPlanner src/node.cpp src/rrtPlanner.cpp)

add_executable(rrt_move src/main.cpp src/navigateRobot.cpp)
add_dependencies(rrt_move ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rrt_move ${catkin_LIBRARIES})


