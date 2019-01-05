#ifndef PATHPLANNER_H
#define PATHPLANNER_H

// ROS Includes
#include <ros/ros.h>
#include <boost/foreach.hpp>

#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include "DroneNavigation/Vec3.h"
#include "DroneNavigation/GlobalPlanner.h"
#include "DroneNavigation/LocalPlanner.h"
#include "DroneNavigation/Costmap.h"
#include "DroneNavigation/Pathfinder.h"

using namespace std;
using namespace ros;
using namespace nav_msgs;
using namespace geometry_msgs;

class PathPlanner
{

public:
    PathPlanner(NodeHandle& nh, GlobalPlanner* global_planner, string drone_id);

    void GeneratePath(Path& path, Vec3 start_position, Vec3 target_position);
    Pose GetNextPathNode();
    void SetCurrentPose(Pose current_pose);
    void SetTargetPose(Pose target_pose);
    bool IsPathClear();

private:
    Publisher path_pub;

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
