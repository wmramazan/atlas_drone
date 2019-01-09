#include "ros/ros.h"
#include <cstdlib>
#include "atlas_drone/VisualizerMessage.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_node");
    if (argc != 3)
    {
        ROS_INFO("usage: add_two_ints_client X Y");
        return 1;
    }

    ros::NodeHandle nh("uav1");
    ros::ServiceClient client = nh.serviceClient<atlas_drone::VisualizerMessage>("/visualizer/visualize_path");
    atlas_drone::VisualizerMessage srv;
    if (client.call(srv))
    {
        ROS_INFO("Success");
    }
    else
    {
        ROS_ERROR("Failed to call service add_two_ints");
        return 1;
    }

    return 0;
}
