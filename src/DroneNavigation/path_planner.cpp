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

#include "DroneNavigation/Costmap.h"
#include "DroneNavigation/Pathfinder.h"

#include "DroneNavigation/Vec3.h"

using namespace ros;
using namespace octomap;
using namespace octomap_msgs;
using namespace nav_msgs;
using namespace geometry_msgs;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

Costmap* costmap;
Path path;

NodeHandle* nh;

Subscriber octomap_sub;
Subscriber pointcloud_sub;
Subscriber current_pose_sub;
Subscriber target_pose_sub;

Publisher path_pub;
Publisher global_costmap_pub;


OcTree* octree;
OcTreeNode* octree_node;
double occupancy_threshold;

uint size;
double resolution;
Costmap* local_costmap;
Costmap* global_costmap;
Pose current_pose;
Pose target_pose;
bool generate_path;

Pathfinder* pathfinder;

std::string frame_id;

std::string octomap_topic;
std::string pointcloud_topic;
std::string path_topic;

Vec3 local_position;

uint radius;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_planner");
    ros::NodeHandle nh("uav1");

    size = nh.param("/size", 600);
    resolution = nh.param("/resolution", 0.25);
    radius = nh.param("/inflation_radius", 5);

    nh.param<std::string>("/frame_id", frame_id, "world");

    nh.param<std::string>("/filtered_pointcloud_topic", pointcloud_topic, "pointcloud_filtered");
    nh.param<std::string>("/throttled_octomap_topic", octomap_topic, "/octomap_throttled");
    nh.param<std::string>("/drone_path_topic", path_topic, "drone_path");

    costmap = new Costmap(size, resolution);
    //local_costmap = new Costmap(size, resolution);
    global_costmap = new Costmap(size, resolution);
    pathfinder = new Pathfinder(costmap);

    path_pub = nh.advertise<Path>(path_topic, 5);
    global_costmap_pub = nh.advertise<std_msgs::UInt8MultiArray>(nh.param<std::string>("/global_costmap_topic", "global_costmap"), 1);

    while (ros::ok())
    {
        ros::spinOnce();

        ROS_INFO("%f - %f - %f", local_position.x, local_position.y, local_position.z);

        Vec3Int start;
        start.x = costmap->ToIndex(local_position.x);
        start.y = costmap->ToIndex(local_position.y);
        start.z = costmap->ToIndex(local_position.z);

        ROS_INFO("%d - %d - %d", start.x, start.y, start.z);
    }

    return 0;
}
