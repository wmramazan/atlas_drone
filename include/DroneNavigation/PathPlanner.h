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
#include <mavros_msgs/PositionTarget.h>

#include "DroneNavigation/Vec3.h"
#include "DroneNavigation/GlobalPlanner.h"
#include "DroneNavigation/LocalPlanner.h"
#include "DroneNavigation/Costmap.h"
#include "DroneNavigation/Pathfinder.h"
#include "atlas_drone/VisualizationMessage.h"

using namespace std;
using namespace ros;
using namespace std_srvs;
using namespace nav_msgs;
using namespace mavros_msgs;
using namespace geometry_msgs;
using namespace visualization_msgs;
using namespace atlas_drone;

struct VisualizationRequest
{
    int drone_id;
    int costmap_type;
    Vec3 origin;
};

class PathPlanner
{

public:
    PathPlanner(GlobalPlanner* global_planner, int drone_id, function<void (VisualizationMessage&)> visualization_request_callback);

    void Update();
    void GeneratePath();
    void SetCurrentPose(Pose current_pose);
    void SetTargetPose(Pose target_pose);
    bool IsPathClear();

    uint to_index(double value);
    double to_position(int value);
    bool is_occupied(Vec3Int index, int type);

    int drone_id;

    GlobalPlanner* global_planner;
    LocalPlanner* local_planner;

    Path path;
    Vec3 drone_start_position;
    Vec3 drone_target_position;

private:
    bool is_path_clear_service_callback(TriggerRequest& request, TriggerResponse& response);
    bool generate_path_service_callback(TriggerRequest& request, TriggerResponse& response);

    function<void(VisualizationMessage&)> visualization_request_callback;

    void drone_position_callback(const PoseStamped::ConstPtr& msg);
    void marker_feedback_callback(const InteractiveMarkerFeedbackConstPtr &feedback);
    void drone_target_callback(const PositionTarget::ConstPtr& msg);

    Subscriber drone_position_sub;
    Subscriber feedback_marker_sub;
    Subscriber drone_target_sub;

    Publisher navigation_target_pub;
    Publisher terminal_message_pub;
    Publisher path_pub;

    Trigger trigger_srv;

    ServiceClient go_to_target_service_client;

    ServiceServer request_path_clearance_service;
    ServiceServer request_path_service;

    Vec3 current_position;
    Vec3 target_position;

    Pathfinder* pathfinder;
    bool generate_path;
    bool go_to_target;

    int size;
    int half_size;
    double resolution;
    double half_resolution;

    int inflation_radius;

    string frame_id;
};

#endif // PATHPLANNER_H
