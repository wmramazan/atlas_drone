#ifndef PATHPLANNER_H
#define PATHPLANNER_H

// ROS Includes
#include <ros/ros.h>
#include <boost/foreach.hpp>

#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Trigger.h>

#include "DroneNavigation/Vec3.h"
#include "DroneNavigation/GlobalPlanner.h"
#include "DroneNavigation/LocalPlanner.h"
#include "DroneNavigation/Costmap.h"
#include "DroneNavigation/Pathfinder.h"

using namespace std;
using namespace ros;
using namespace std_srvs;
using namespace nav_msgs;
using namespace geometry_msgs;

class PathPlanner
{

public:
    PathPlanner(NodeHandle& nh, GlobalPlanner* global_planner);

    void GeneratePath(Path& path, Vec3 start_position, Vec3 target_position);
    Pose GetNextPathNode();
    void SetCurrentPose(Pose current_pose);
    void SetTargetPose(Pose target_pose);
    bool IsPathClear();

private:
    bool is_path_clear_service_callback(TriggerRequest& request, TriggerResponse& response);
    bool generate_path_service_callback(TriggerRequest& request, TriggerResponse& response);


    Publisher path_pub;

    ServiceServer request_path_clearence_service;
    ServiceServer request_path_service;

    GlobalPlanner* global_planner;
    LocalPlanner* local_planner;

    Path path;

    Costmap* costmap;
    Pathfinder* pathfinder;
    bool generate_path;

    uint size;
    double resolution;

    string frame_id;
};

#endif // PATHPLANNER_H
