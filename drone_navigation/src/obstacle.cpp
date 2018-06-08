#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <PublisherSubscriber.h>
//#include <boost/foreach.hpp>

#define PUBLISHER_TOPIC "/drone/obstacle"
#define SUBSCRIBER_TOPIC "/camera/depth/points"
#define FREQUENCY 1

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
pcl::PointXYZ pt;
std_msgs::Bool msg_obstacle;
int counter = 0;
float min_x, min_y, min_z;

template<>
void PublisherSubscriber<std_msgs::Bool, PointCloud>::callback(const PointCloud::ConstPtr& msg)
{
  if(counter++ == FREQUENCY) {
    counter = 0;
    printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
    int i = 0;
    do {
      pt = msg->points[i++];
    } while(std::isnan(pt.x) || pt.x == 0);
    printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);

    min_x = 100.0;
    min_y = 100.0;
    min_z = 100.0;

    BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points) {
      if(pt.x < min_x)
        min_x = pt.x;
    }

    printf ("%f\n", min_x);

    msg_obstacle.data = true;
    publisher.publish(msg_obstacle);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "obstacle");
  PublisherSubscriber<std_msgs::Bool, PointCloud> obstacle(PUBLISHER_TOPIC, SUBSCRIBER_TOPIC, 1000);
  ros::spin();
}
