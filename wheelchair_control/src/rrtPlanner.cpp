#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>
#include "ros/console.h"
#include <vector>
#include <map>

#include <pluginlib/class_list_macros.h>
#include "rrtPlanner.hpp"
#include "node.hpp"

PLUGINLIB_EXPORT_CLASS(rrt_plugin::RRTPlanner, nav_core::BaseGlobalPlanner)

namespace rrt_plugin {
    ///< Constructors
    RRTPlanner::RRTPlanner() {}
    RRTPlanner::RRTPlanner(std::string name,
        costmap_2d::Costmap2DROS* costmap_ros) {
        initialize(name, costmap_ros);
    }


    void RRTPlanner::initialize(std::string name,
        costmap_2d::Costmap2DROS* costmap_ros) {
        if (!initialized_) {
            srand(time(NULL));
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();

            ros::NodeHandle private_nh("~/" + name);

            originX = costmap_->getOriginX();
            originY = costmap_->getOriginY();
            width = costmap_->getSizeInCellsX();
            height = costmap_->getSizeInCellsY();
            resolution = costmap_->getResolution();
            mapSize = width * height;

            occupiedGridMap = new bool[mapSize];
            for (unsigned int iy = 0; iy < height; iy++) {
                for (unsigned int ix = 0; ix < width; ix++) {
                    unsigned int cost =
                    static_cast<int>(costmap_->getCost(ix, iy));
                    if (cost == 0)
                        occupiedGridMap[iy * width + ix] = true;
                    else
                        occupiedGridMap[iy * width + ix] = false;
                }
            }
            ROS_INFO("rrt planner initialized successfully..!");
            initialized_ = true;
        } else {
            ROS_WARN("This planner is initialized...");
        }
    }


    bool RRTPlanner::makePlan(const geometry_msgs::PoseStamped& start,
        const geometry_msgs::PoseStamped& goal,
        std::vector<geometry_msgs::PoseStamped>& plan) {
        if (!initialized_) {
            ROS_ERROR("The planner is not initialized yet");
            return false;
        }

        ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f",
            start.pose.position.x, start.pose.position.y,
            goal.pose.position.x, goal.pose.position.y);

        plan.clear();

        if (goal.header.frame_id != costmap_ros_->getGlobalFrameID()) {
            ROS_ERROR("frame error, only %s frame, not in the %s frame.",
                       costmap_ros_->getGlobalFrameID().c_str(),
                       goal.header.frame_id.c_str());
            return false;
        }

        float startX = start.pose.position.x;
        float startY = start.pose.position.y;

        float goalX = goal.pose.position.x;
        float goalY = goal.pose.position.y;

        getXY(startX, startY);
        getXY(goalX, goalY);

        int startLoc;
        int goalLoc;
        if (checkInsideMap(startX, startY) && checkInsideMap(goalX, goalY)) {
            startLoc = cellIndexConversion(startX, startY);
            goalLoc = cellIndexConversion(goalX, goalY);
        } else {
            ROS_WARN("the start or goal is out of the map");
            return false;
        }

        if (checkStartGoal(startLoc, goalLoc)) {
            std::vector<int> bestPath;
            bestPath.clear();
            bestPath = planRRTpath(startLoc, goalLoc);

            if (bestPath.size() > 0) {
                for (int i = 0; i < bestPath.size(); i++) {
                    float x = 0.0;
                    float y = 0.0;

                    int index = bestPath[i];

                    coordinateConversion(index, x, y);

                    geometry_msgs::PoseStamped pose = goal;

                    pose.pose.position.x = x;
                    pose.pose.position.y = y;
                    pose.pose.position.z = 0.0;

                    pose.pose.orientation.x = 0.0;
                    pose.pose.orientation.y = 0.0;
                    pose.pose.orientation.z = 0.0;
                    pose.pose.orientation.w = 1.0;

                    plan.push_back(pose);
                }
                return true;
            } else {
                ROS_WARN("failed to find a path, choose aother goal");
                return false;
            }
        } else {
            ROS_WARN("Not valid start or goal");
            return false;
        }
    }


    void RRTPlanner::getXY(float& x, float& y) {
        x = x - originX;
        y = y - originY;
    }


    int RRTPlanner::cellIndexConversion(float x, float y) {
        int cellID;
        float newX = x / resolution;
        float newY = y / resolution;
        cellID = cellIndexCheck(newY, newX);
        return cellID;
    }

    void RRTPlanner::coordinateConversion(int index, float& x, float& y) {
        x = colIDCheck(index) * resolution;
        y = rowIDCheck(index) * resolution;

        x = x + originX;
        y = y + originY;
    }

    bool RRTPlanner::checkInsideMap(float x, float y) {
        bool isValid = true;
        if (x > (width * resolution) || y > (height * resolution))
            isValid = false;
        return isValid;
    }

    std::pair<int, int> RRTPlanner::randomPointGenerate() {
        thread_local unsigned int seed = time(NULL);
        int x = rand_r(&seed) % height;
        int y = rand_r(&seed) % width;
        return std::make_pair(x, y);
    }

    std::vector<int> RRTPlanner::planRRTpath(int startCell, int goalCell) {
        int totalSamples = 100000;
        int stepSize = 1;
        double probabilityCheckGoal = 0.1;


        node source, destination;
        source.setParentIdx(0);
        source.setIdx(1);
        source.setPosition(rowIDCheck(startCell), colIDCheck(startCell));
        destination.setPosition(rowIDCheck(goalCell), colIDCheck(goalCell));
        int id = 2;

        std::vector<node> frontierNodes = {source};
        std::map<int, node> exploredNodes = {{source.getIdx(), source}};

        for (int i = 0; i < totalSamples; i++) {

            double probability = (double) rand() / (RAND_MAX);
            std::pair<int, int> randomNode;
            if (probability < probabilityCheckGoal)
                randomNode = destination.getPosition();
            else
                randomNode = randomPointGenerate();

            double minDistance = 10000;
            int minID = 0;
            for (auto v : frontierNodes) {
                int xDifference = randomNode.first - v.getPosition().first;
                int yDifference = randomNode.second - v.getPosition().second;
                double currDistance = sqrt(xDifference * xDifference + yDifference * yDifference);
                if (currDistance < minDistance) {
                    minDistance = currDistance;
                    minID = v.getIdx();
                }
            }
            node nearestNode = exploredNodes[minID];

            int xDifference = randomNode.first - nearestNode.getPosition().first;
            int yDifference = randomNode.second - nearestNode.getPosition().second;
            int diff = sqrt(xDifference * xDifference + yDifference * yDifference);
            node newestNode;
            if (diff < stepSize) {  // go straight to randomNode
                newestNode.setPosition(randomNode.first, randomNode.second);
            } else {  // go a small step in randomNode direction
                int q_new_x = nearestNode.getPosition().first +
                              stepSize * xDifference / diff;
                int q_new_y = nearestNode.getPosition().second +
                              stepSize * yDifference / diff;
                newestNode.setPosition(q_new_x, q_new_y);
            }

            if (!checkFree(newestNode.getPosition().first, newestNode.getPosition().second))
                continue;

            newestNode.setParentIdx(nearestNode.getIdx());
            newestNode.setIdx(id); id++;
            frontierNodes.push_back(newestNode);
            exploredNodes[newestNode.getIdx()] = newestNode;

            if (newestNode.getPosition() == destination.getPosition()) {
                ROS_INFO("found the goal!!");
                break;
            }
        }

        std::stack<node> path;

        node v_tmp = frontierNodes.back();

        while (1) {
            path.push(v_tmp);
            int parent_idx = v_tmp.getParentIdx();
            if (parent_idx == 0) break;
            v_tmp = exploredNodes[parent_idx];
        }

        std::vector<int> bestPath;
        bestPath.clear();
        while (!path.empty()) {
            node nd = path.top(); path.pop();
            int array_position_index =
            cellIndexCheck(nd.getPosition().first, nd.getPosition().second);
            bestPath.push_back(array_position_index);
        }
        return bestPath;
    }

    bool RRTPlanner::checkStartGoal(int startCell, int goalCell) {
        return checkFree(startCell) && checkFree(goalCell);
    }

    bool  RRTPlanner::checkFree(int i, int j) {
        int CellID = cellIndexCheck(i, j);
        return occupiedGridMap[CellID];
    }

    bool RRTPlanner::checkFree(int CellID) {
        return occupiedGridMap[CellID];
    }

    int RRTPlanner::cellIndexCheck(int i, int j) {
        return (i * width) + j;
    }

    int RRTPlanner::rowIDCheck(int index) {
        return index / width;
    }

    int RRTPlanner::colIDCheck(int index) {
        return index % width;
    }
};  // namespace rrt_star_plugin