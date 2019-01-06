#ifndef PATHPLANNER_H
#define PATHPLANNER_H

// ROS Includes
#include <ros/ros.h>
#include <boost/foreach.hpp>

#include <std_msgs/String.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <std_srvs/Trigger.h>

#include "DroneNavigation/Vec3.h"
#include "DroneNavigation/GlobalPlanner.h"
#include "DroneNavigation/LocalPlanner.h"
#include "DroneNavigation/Costmap.h"
#include "DroneNavigation/Pathfinder.h"

#include <atlas_drone/MarkerService.h>

using namespace std;
using namespace ros;
using namespace std_srvs;
using namespace nav_msgs;
using namespace geometry_msgs;
using namespace visualization_msgs;
using namespace atlas_drone;

class PathPlanner
{

public:
    PathPlanner(NodeHandle& nh, GlobalPlanner* global_planner);

    void Update();
    void GeneratePath();
    void SetCurrentPose(Pose current_pose);
    void SetTargetPose(Pose target_pose);
    bool IsPathClear();

    GlobalPlanner* global_planner;
    LocalPlanner* local_planner;

private:
    bool is_path_clear_service_callback(TriggerRequest& request, TriggerResponse& response);
    bool generate_path_service_callback(TriggerRequest& request, TriggerResponse& response);

    void drone_position_callback(const PoseStamped::ConstPtr& msg);
    void marker_feedback_callback(const InteractiveMarkerFeedbackConstPtr &feedback);

    Subscriber drone_position_sub;
    Subscriber feedback_marker_sub;

    Publisher navigation_target_pub;
    Publisher terminal_message_pub;
    Publisher path_pub;

    Trigger trigger_srv;
    MarkerService marker_srv;

    ServiceClient go_to_target_service_client;
    ServiceClient path_marker_service_client;
    ServiceClient costmap_marker_service_client;

    ServiceServer request_path_clearance_service;
    ServiceServer request_path_service;



    Path path;

    Vec3 current_position;
    Vec3 target_position;

    Costmap* costmap;
    Pathfinder* pathfinder;
    bool generate_path;

    uint size;
    double resolution;

    string frame_id;
};

#endif // PATHPLANNER_H
