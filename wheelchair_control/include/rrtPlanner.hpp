#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Twist.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <set>
#include <string>
#include <vector>
#include <utility>
#include <random>
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"

#include "node.hpp"
#include <boost/foreach.hpp>

#ifndef INCLUDE_RRTPLANNER_HPP_
#define INCLUDE_RRTPLANNER_HPP_

namespace rrt_plugin {
class RRTPlanner : public nav_core::BaseGlobalPlanner {
 public:
    RRTPlanner();
    RRTPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    ros::NodeHandle ROSNodeHandle;

    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan);

    int width;
    int height;
    int mapSize;
    float resolution;
    bool* occupiedGridMap;

    float originX;
    float originY;
    bool initialized_;

    costmap_2d::Costmap2DROS* costmap_ros_;
    costmap_2d::Costmap2D* costmap_;

    void getXY(float& x, float& y);

    int cellIndexConversion(float x, float y);

    void coordinateConversion(int index, float& x, float& y);

    bool checkInsideMap(float x, float y);

    std::vector<int> planRRTpath(int startCell, int goalCell);

    bool checkStartGoal(int startCell, int goalCell);

    bool checkFree(int i, int j);

    bool checkFree(int CellID);

    std::pair<int, int> randomPointGenerate();

    int cellIndexCheck(int i, int j);

    int rowIDCheck(int index);

    int colIDCheck(int index);
};
};  // namespace rrt_planner
#endif  // INCLUDE_RRTPLANNER_HPP_"
