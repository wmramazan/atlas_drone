#ifndef GLOBALPLANNER_H
#define GLOBALPLANNER_H

// ROS Includes
#include <ros/ros.h>

// System Includes
#include <string>

// Octomap Includes
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <DroneNavigation/Costmap.h>

using namespace std;
using namespace ros;
using namespace octomap;
using namespace octomap_msgs;

class GlobalPlanner
{
public:
    GlobalPlanner(NodeHandle& nh);
    bool IsOccupied(Vec3Int index);
    uint GetMapSize();

private:
    void generate_global_costmap(const Octomap::ConstPtr& octomap);
    void octomap_callback(const Octomap::ConstPtr& octomap);

    Subscriber octomap_sub;

    OcTree* octree;
    OcTreeNode* octree_node;
    double occupancy_threshold;

    Costmap global_costmap;
    uint size;
    double resolution;
    uint radius;
};

#endif // GLOBALPLANNER_H
