#include "ros/ros.h"
#include <cstdlib>
#include "atlas_drone/VisualizerMessage.h"

bool test(atlas_drone::VisualizerMessageRequest& req,
          atlas_drone::VisualizerMessageResponse& res)
{
    ROS_INFO("VAY BABANIN KEMUGUNE");
}

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
    ros::ServiceServer server = nh.advertiseService("/test", &test);

    atlas_drone::VisualizerMessage srv;

    ros::Rate loop_rate(40);
    while(ros::ok())
    {
        ros::spinOnce();

        client.call(srv);

        loop_rate.sleep();
    }

    return 0;
}
