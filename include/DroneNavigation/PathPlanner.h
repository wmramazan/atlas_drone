#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <ros/ros.h>
#include <boost/foreach.hpp>

#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <pcl_ros/point_cloud.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include "Costmap.h"
#include "Pathfinder.h"

using namespace ros;
using namespace octomap;
using namespace octomap_msgs;
using namespace nav_msgs;
using namespace geometry_msgs;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class PathPlanner
{

public:
    enum Mode { GROUND, VEHICLE };

    PathPlanner(NodeHandle& nh, Mode mode);

    void GenerateLocalCostmap(const PointCloud::ConstPtr& point_cloud);
    void GenerateGlobalCostmap(const Octomap::ConstPtr& octomap);
    Path* GeneratePath();
    Pose GetNextPathNode();
    void SetCurrentPose(Pose current_pose);
    void SetTargetPose(Pose target_pose);
    bool IsPathClear();

    Costmap* costmap;
    Path path;

private:
    NodeHandle* nh;
    int mode;

    void pointcloud_callback(const PointCloud::ConstPtr& msg);
    void octomap_callback(const Octomap::ConstPtr& octomap);
    void local_costmap_callback(const UInt8MultiArray::ConstPtr& msg);
    void global_costmap_callback(const UInt8MultiArray::ConstPtr& msg);

    Subscriber local_costmap_sub;
    Subscriber global_costmap_sub;
    Subscriber octomap_sub;
    Subscriber pointcloud_sub;

    Publisher local_costmap_pub;
    Publisher global_costmap_pub;
    Publisher path_pub;

    ColorOcTree* octree;
    ColorOcTreeNode* octree_node;
    double occupancy_threshold;

    uint size;
    double resolution;
    Costmap* local_costmap;
    Costmap* global_costmap;
    Pose current_pose;
    Pose target_pose;

    Pathfinder* pathfinder;

    std::string frame_id;

    std::string local_costmap_topic;
    std::string pointcloud_topic;
    std::string path_topic;

    std::string global_costmap_topic;
    std::string octomap_topic;

    uint radius;
};

#endif // PATHPLANNER_H
