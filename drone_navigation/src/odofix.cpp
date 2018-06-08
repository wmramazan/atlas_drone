#include <cmath>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <PublisherSubscriber.h>
//#include <boost/foreach.hpp>

#define TOPIC "/zed/odom"

geometry_msgs::Pose old_pose;
geometry_msgs::Pose pose;
nav_msgs::Odometry new_odom;
double difference, difference_threshold;

template<>
void PublisherSubscriber<nav_msgs::Odometry, nav_msgs::Odometry>::callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  pose = msg->pose.pose;
  ROS_INFO("Difference Threshold: %lf", difference_threshold);

  if(NULL != &old_pose) {
    //ROS_INFO("Pose: %lf", pose.position.x);
    //ROS_INFO("Old Pose: %lf", old_pose.position.x);
    difference = std::abs(pose.position.x - old_pose.position.x);
    if(difference > difference_threshold) {
      ROS_INFO("Odofix(x): %lf", difference);
      pose.position.x = old_pose.position.x;
    }

    difference = std::abs(pose.position.y - old_pose.position.y);
    if(difference > difference_threshold) {
      ROS_INFO("Odofix(y): %lf", difference);
      pose.position.y = old_pose.position.y;
    }

    difference = std::abs(pose.position.z - old_pose.position.z);
    if(difference > difference_threshold) {
      ROS_INFO("Odofix(z): %lf", difference);
      pose.position.z = old_pose.position.z;
    }
  }

  old_pose = geometry_msgs::Pose(pose);
  ROS_INFO("T: %lf", old_pose.position.x);
  new_odom = *msg;
  new_odom.pose.pose = pose;
  publisher.publish(new_odom);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odofix");
  PublisherSubscriber<nav_msgs::Odometry, nav_msgs::Odometry> odofix("/new_odom", TOPIC, 1000);
  odofix.nodeHandle.param("/odofix/difference_threshold", difference_threshold, 0.01);
  ROS_INFO("Difference Threshold: %lf", difference_threshold);
  ros::spin();
}
