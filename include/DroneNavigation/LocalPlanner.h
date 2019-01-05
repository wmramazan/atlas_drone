#ifndef LOCALPLANNER_H
#define LOCALPLANNER_H

// ROS Includes
#include <ros/ros.h>

// PCL Includes
#include <pcl_ros/point_cloud.h>

#include <DroneNavigation/Costmap.h>

using namespace std;
using namespace ros;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class LocalPlanner
{
public:
    LocalPlanner(NodeHandle& nh, string drone_id);

private:
    void generate_local_costmap(const PointCloud::ConstPtr& point_cloud);
    void point_cloud_callback(const PointCloud::ConstPtr& msg);

    Subscriber point_cloud_sub;

    Costmap local_costmap;
    uint size;
    double resolution;
    uint radius;
};

#endif // LOCALPLANNER_H
