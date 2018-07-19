#include "ros/ros.h"

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::VoxelGrid<pcl::PointXYZ> VoxelGrid;

ros::Publisher filtered_cloud_pub;
ros::Subscriber point_cloud_sub;

VoxelGrid voxel_grid;
PointCloud* cloud_filtered;

void pointCloudCallback(const PointCloud::ConstPtr& point_cloud)
{
    cloud_filtered->clear();

    voxel_grid.setInputCloud(point_cloud);
    voxel_grid.setLeafSize(0.1, 0.1, 0.1);
    voxel_grid.filter(*cloud_filtered);

    filtered_cloud_pub.publish(*cloud_filtered);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "point_cloud_filter");
  ros::NodeHandle nh("~");

  filtered_cloud_pub = nh.advertise<PointCloud>("filtered_cloud", 10);
  point_cloud_sub = nh.subscribe("/camera/depth/points", 5, pointCloudCallback);

  cloud_filtered = new PointCloud();

  ros::spin();

  return 0;
}
