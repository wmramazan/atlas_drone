#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <ros/ros.h>
#include <boost/foreach.hpp>

#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include "Costmap.h"
#include "Pathfinder.h"

using namespace ros;
using namespace nav_msgs;
using namespace visualization_msgs;
using namespace geometry_msgs;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::VoxelGrid<pcl::PointXYZ> VoxelGrid;

class PathPlanner
{

public:
    enum Mode
    {
        LOCAL_COSTMAP  = 0x01,
        GLOBAL_COSTMAP = 0x02,
        PATH           = 0x04
    };

    PathPlanner(NodeHandle& nh, Mode mode);

    void GenerateLocalCostmap(const PointCloud::ConstPtr& point_cloud);
    void GenerateGlobalCostmap(const MarkerArray::ConstPtr& marker_array);
    Path* GeneratePath();
    Pose GetNextPathNode();
    void SetCurrentPose(Pose current_pose);
    void SetTargetPose(Pose target_pose);

    Costmap* costmap;
    Path path;

private:
    NodeHandle* nh;
    Mode mode;

    void pointcloud_callback(const PointCloud::ConstPtr& msg);
    void occupied_cells_callback(const MarkerArray::ConstPtr& msg);
    void local_costmap_callback(const UInt8MultiArray::ConstPtr& msg);
    void global_costmap_callback(const UInt8MultiArray::ConstPtr& msg);

    Subscriber local_costmap_sub;
    Subscriber global_costmap_sub;
    Subscriber occupied_cells_sub;
    Subscriber pointcloud_sub;

    Publisher  local_costmap_pub;
    Publisher  global_costmap_pub;
    Publisher  path_pub;

    uint size;
    double resolution;
    double inflation_radius;
    Costmap* local_costmap;
    Costmap* global_costmap;
    Pose current_pose;
    Pose target_pose;

    Pathfinder* pathfinder;
    VoxelGrid voxel_grid;
    PointCloud* cloud_filtered;

    std::string frame_id;

    std::string local_costmap_topic;
    std::string pointcloud_topic;
    std::string path_topic;

    std::string global_costmap_topic;
    std::string occupied_cells_topic;

    uint radius;
    uint i, j, k;
    uint x, y, z;
};

#endif // PATHPLANNER_H
