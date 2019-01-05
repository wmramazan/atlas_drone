#ifndef DRONENAVIGATION_H
#define DRONENAVIGATION_H

#include <ros/ros.h>
#include <string>

#include <geometry_msgs/Pose.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/String.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>

#include <DroneNavigation/PathPlanner.h>
#include <DroneNavigation/GlobalPlanner.h>

using namespace std;
using namespace ros;
using namespace std_msgs;
using namespace std_srvs;
using namespace visualization_msgs;

class DroneNavigation
{
public:
    DroneNavigation();

private:
    ros::NodeHandle nh;

    void marker_feedback_callback(const InteractiveMarkerFeedbackConstPtr &feedback);

    Subscriber feedback_marker_sub;

    Publisher navigation_target_pub;
    Publisher terminal_message_pub;

    ServiceClient go_to_target_service_client;
    ServiceClient generate_path_service_client;
    Trigger trigger_srv;

    GlobalPlanner* global_planner;

    PathPlanner* vehicle1_planner;
    PathPlanner* vehicle2_planner;
    PathPlanner* vehicle3_planner;
};

#endif // DRONENAVIGATION_H
