#ifndef DRONENAVIGATION_H
#define DRONENAVIGATION_H

#include <ros/ros.h>
#include <string>

#include <DroneNavigation/PathPlanner.h>
#include <DroneNavigation/GlobalPlanner.h>

#include <atlas_drone/MarkerService.h>

using namespace std;
using namespace ros;
using namespace atlas_drone;

class DroneNavigation
{
public:
    DroneNavigation();

private:
    ros::NodeHandle nh;

    GlobalPlanner* global_planner;

    PathPlanner* vehicle1_planner;
    PathPlanner* vehicle2_planner;
    PathPlanner* vehicle3_planner;

    bool pathMarkerServiceCallback(MarkerServiceRequest& request, MarkerServiceResponse& response);
    bool costmapMarkerServiceCallback(MarkerServiceRequest& request, MarkerServiceResponse& response);

    ServiceServer path_marker_service;
    ServiceServer costmap_marker_service;
};

#endif // DRONENAVIGATION_H
