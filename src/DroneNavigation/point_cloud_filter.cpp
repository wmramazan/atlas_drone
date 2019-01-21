#include "ros/ros.h"

#include <tf/transform_listener.h>

#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <string>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::VoxelGrid<pcl::PointXYZ> VoxelGrid;

ros::Publisher filtered_cloud_pub;
ros::Subscriber point_cloud_sub;

VoxelGrid voxel_grid;
PointCloud* cloud_filtered;
PointCloud* pc;
tf::TransformListener* listener;


using namespace std;

void pointCloudCallback(const PointCloud::ConstPtr& point_cloud)
{
    cloud_filtered->clear();
    pc->clear();

    voxel_grid.setInputCloud(point_cloud);
    voxel_grid.setLeafSize(0.1, 0.1, 0.1);
    voxel_grid.filter(*cloud_filtered);

    pcl_ros::transformPointCloud("/world", *cloud_filtered, *pc, *listener);

    filtered_cloud_pub.publish(*pc);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "point_cloud_filter");
  ros::NodeHandle nh("~");
  int id = nh.param("drone_id", 1);
  nh = ros::NodeHandle("uav" + to_string(id));

  ros::Duration(10.0).sleep();

  filtered_cloud_pub = nh.advertise<PointCloud>(nh.param<std::string>("/filtered_pointcloud_topic", "pointcloud_filtered"), 10);
  point_cloud_sub = nh.subscribe(nh.param<std::string>("/throttled_pointcloud_topic", "pointcloud_throttled"), 5, pointCloudCallback);

  listener = new tf::TransformListener();
  cloud_filtered = new PointCloud();
  pc = new PointCloud();

  ros::spin();

  return 0;
}
