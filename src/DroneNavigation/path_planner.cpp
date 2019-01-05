#include <ros/ros.h>
#include <boost/foreach.hpp>

#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <std_srvs/Trigger.h>

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

ServiceServer is_path_clear_service;
ServiceServer generate_path_service;

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

void GenerateLocalCostmap(const PointCloud::ConstPtr& point_cloud)
{
    local_costmap->Clear();

    uint x, y, z;
    uint i, j, k;

    BOOST_FOREACH (const pcl::PointXYZ& pt, point_cloud->points)
    {   
        if (!isnan(pt.x))
        {
            x = local_costmap->ToIndex(pt.x);
            y = local_costmap->ToIndex(pt.y);
            z = local_costmap->ToIndex(pt.z);

            //ROS_INFO("%lf %lf %lf", pt.x, pt.y, pt.z);
            //ROS_INFO("%d %d %d", x, y, z);
            if (!local_costmap->Get(z, size - x, size - y))
            {
                for (i = x - radius; i <= x + radius; i++)
                {
                    for (j = y - radius; j <= y + radius; j++)
                    {
                        for (k = z - radius; k <= z + radius; k++)
                        {
                            local_costmap->Get(k, size - i, size - j) = 1;
                        }
                    }
                }
            }
        }
    }

    costmap->Merge(local_costmap);
    //local_costmap_pub.publish(local_costmap->data);
}

void GenerateGlobalCostmap(const Octomap::ConstPtr& octomap)
{
    costmap->Clear();
    global_costmap->Clear();

    octree = dynamic_cast<OcTree*>(fullMsgToMap(*octomap));
    occupancy_threshold = octree->getOccupancyThres();

    //ROS_INFO("%lf", occupancy_threshold);

    uint x, y, z;
    uint i, j, k;

    for (OcTree::leaf_iterator it = octree->begin_leafs(), end = octree->end_leafs(); it != end; ++it)
    {
        octree_node = octree->search(it.getKey());
        if (NULL != octree_node && octree_node->getOccupancy() > occupancy_threshold)
        {
            x = global_costmap->ToIndex(it.getX());
            y = global_costmap->ToIndex(it.getY());
            z = global_costmap->ToIndex(it.getZ());

            //ROS_INFO("octomap: %lf %lf %lf", it.getX(), it.getY(), it.getZ());
            //ROS_INFO("costmap: %d %d %d", x, y, z);

            // Inflation
            for (i = x - radius; i <= x + radius; i++)
            {
                for (j = y - radius; j <= y + radius; j++)
                {
                    for (k = z - radius; k <= z + radius; k++)
                    {
                        global_costmap->Get(i, j, k) = 1;
                    }
                }
            }
        }
    }

    costmap->Merge(global_costmap);
    global_costmap_pub.publish(global_costmap->data);
}

Path* GeneratePath()
{
    path.poses.clear();

    Vec3Int start;
    start.x = costmap->ToIndex(current_pose.position.x);
    start.y = costmap->ToIndex(current_pose.position.y);
    start.z = costmap->ToIndex(current_pose.position.z);

    ROS_INFO("Generating path from %d %d %d %lf %lf %lf", start.x, start.y, start.z, current_pose.position.x, current_pose.position.y, current_pose.position.z);

    Vec3Int end;
    end.x = costmap->ToIndex(target_pose.position.x);
    end.y = costmap->ToIndex(target_pose.position.y);
    end.z = costmap->ToIndex(target_pose.position.z);

    ROS_INFO("to %d %d %d %lf %lf %lf", end.x, end.y, end.z, target_pose.position.x, target_pose.position.y, target_pose.position.z);

    double current_timestamp = ros::Time::now().toSec();
    vector<Vec3Int> found_path = pathfinder->Find(start, end);
    ROS_INFO("Execution time: %lf s", ros::Time::now().toSec() - current_timestamp);
    if (found_path.size())
    {
        path.header.frame_id = frame_id;

        PoseStamped pose;

        for (Vec3Int coordinate : found_path)
        {
            pose.pose.position.x = costmap->ToPosition(coordinate.x);
            pose.pose.position.y = costmap->ToPosition(coordinate.y);
            pose.pose.position.z = costmap->ToPosition(coordinate.z);
            //ROS_INFO("Coordinate: %d %d %d  %lf %lf %lf", coordinate.x, coordinate.y, coordinate.z, pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
            path.poses.push_back(pose);
        }

        path_pub.publish(path);

        generate_path = false;
        return &path;
    }
    else
    {
        generate_path = false;
        return NULL;
    }
}

bool IsPathClear()
{
    return costmap->CanPathPass(&path);
}

Pose GetNextPathNode()
{
    // TODO: Move implementation from NavigationBehaviour
    return path.poses[0].pose;
}

void pointcloud_callback(const PointCloud::ConstPtr& msg)
{
    //ROS_INFO("pointCloudCallback");
    GenerateLocalCostmap(msg);
}

void octomap_callback(const Octomap::ConstPtr& msg)
{
    //ROS_INFO("octomapCallback");
    GenerateGlobalCostmap(msg);
}

void current_pose_callback(const PoseStamped::ConstPtr& msg)
{
    current_pose = msg->pose;
    local_position = Vec3(current_pose.position.x, current_pose.position.y, current_pose.position.z);
}

void target_pose_callback(const Pose::ConstPtr& msg)
{
    target_pose = *msg;
}

bool is_path_clear_service_callback(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response)
{
    response.success = IsPathClear();
    return true;
}

bool generate_path_service_callback(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response)
{
    generate_path = true;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_planner");
    ros::NodeHandle nh("uav1");

    size = nh.param("/size", 600);
    resolution = nh.param("/resolution", 0.25);
    radius = nh.param("/inflation_radius", 5);

    ROS_INFO("Path Planner: %f", resolution);

    nh.param<std::string>("/frame_id", frame_id, "world");

    nh.param<std::string>("/filtered_pointcloud_topic", pointcloud_topic, "pointcloud_filtered");
    nh.param<std::string>("/throttled_octomap_topic", octomap_topic, "/octomap_throttled");
    nh.param<std::string>("/drone_path_topic", path_topic, "drone_path");

    costmap = new Costmap(size, resolution);
    //local_costmap = new Costmap(size, resolution);
    global_costmap = new Costmap(size, resolution);
    pathfinder = new Pathfinder(costmap);

    octomap_sub         = nh.subscribe(octomap_topic, 5, &octomap_callback);
    //pointcloud_sub      = nh.subscribe(pointcloud_topic, 5, &pointcloud_callback);
    current_pose_sub    = nh.subscribe(nh.param<std::string>("/drone_position_topic", "mavros/local_position/pose"), 5, &current_pose_callback);
    target_pose_sub     = nh.subscribe(nh.param<std::string>("/target_pose_topic", "target_pose"), 5, &target_pose_callback);

    path_pub = nh.advertise<Path>(path_topic, 5);
    global_costmap_pub = nh.advertise<std_msgs::UInt8MultiArray>(nh.param<std::string>("/global_costmap_topic", "global_costmap"), 1);

    generate_path_service = nh.advertiseService(nh.param<std::string>("/generate_path_service", "/path_planner/generate_path"), &generate_path_service_callback);
    is_path_clear_service = nh.advertiseService(nh.param<std::string>("/is_path_clear_service", "/path_planner/is_path_clear"), &is_path_clear_service_callback);

    while (ros::ok())
    {
        ros::spinOnce();

        if (generate_path)
        {
            GeneratePath();
        }



        ROS_INFO("%f - %f - %f", local_position.x, local_position.y, local_position.z);

        Vec3Int start;
        start.x = costmap->ToIndex(local_position.x);
        start.y = costmap->ToIndex(local_position.y);
        start.z = costmap->ToIndex(local_position.z);

        ROS_INFO("%d - %d - %d", start.x, start.y, start.z);
    }

    return 0;
}
