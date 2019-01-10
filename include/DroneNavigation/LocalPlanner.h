#ifndef LOCALPLANNER_H
#define LOCALPLANNER_H

// ROS Includes
#include <ros/ros.h>

#include <string>

// PCL Includes
#include <pcl_ros/point_cloud.h>

#include <DroneNavigation/Costmap.h>

using namespace std;
using namespace ros;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class LocalPlanner
{
public:
    LocalPlanner(NodeHandle& nh);
    bool IsOccupied(Vec3Int index);
    bool IsPathClear(Path* path);
    uint GetMapSize();
    void SetOccupancy(Vec3Int index, uint8_t value);

private:
    void generate_local_costmap(const PointCloud::ConstPtr& point_cloud);
    void point_cloud_callback(const PointCloud::ConstPtr& msg);

    Subscriber point_cloud_sub;

    Costmap local_costmap;
    uint size;
    double resolution;
    uint inflation_radius;
};

#endif // LOCALPLANNER_H
