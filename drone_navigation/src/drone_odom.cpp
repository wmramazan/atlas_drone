#include <cmath>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <drone_navigation/PublisherSubscriber.h>
#include <drone_navigation/Vec3.h>
//#include <boost/foreach.hpp>

#define TOPIC "/zed/odom"

geometry_msgs::Pose old_pose;
geometry_msgs::Pose drone_pose;
tf::Quaternion drone_rotation;
nav_msgs::Odometry new_odom;

bool initialized = false;

template<>
void PublisherSubscriber<nav_msgs::Odometry, nav_msgs::Odometry>::callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  if (initialized)
  {
      Vec3 deltaPos = Vec3(old_pose.position.x - msg->pose.pose.position.x,
                           old_pose.position.y - msg->pose.pose.position.y,
                           old_pose.position.z - msg->pose.pose.position.z);

      tf::Quaternion old_rotation;
      tf::quaternionMsgToTF(old_pose.orientation, old_rotation);

      tf::Quaternion new_rotation;
      tf::quaternionMsgToTF(msg->pose.pose.orientation, new_rotation);

      drone_rotation += new_rotation - old_rotation;

      tf::quaternionTFToMsg(drone_rotation, drone_pose.orientation);

      drone_pose.position.x -= deltaPos.x;
      drone_pose.position.y -= deltaPos.y;
      drone_pose.position.z -= deltaPos.z;

      new_odom = *msg;
      new_odom.pose.pose = drone_pose;
      publisher.publish(new_odom);
  }
  else
  {
      drone_pose = msg->pose.pose;
      tf::quaternionMsgToTF(drone_pose.orientation, drone_rotation);

      ROS_INFO("%lf %lf %lf %lf", drone_pose.orientation.x, drone_pose.orientation.y, drone_pose.orientation.z, drone_pose.orientation.w);

      initialized = true;
  }

  old_pose = msg->pose.pose;
}

bool resetOdom(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  drone_pose.position.x = 0;
  drone_pose.position.y = 0;
  drone_pose.position.z = 0;

  drone_rotation = tf::Quaternion(0, 0, 0, 0);

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "drone_odom");
  PublisherSubscriber<nav_msgs::Odometry, nav_msgs::Odometry> drone_odom("/drone_odom", TOPIC, 1000);
  ros::ServiceServer service = drone_odom.nodeHandle.advertiseService("resetOdom", &resetOdom);
  //drone_odom.nodeHandle.param("/drone_odom/difference_threshold", difference_threshold, 0.01);
  ros::spin();
}
