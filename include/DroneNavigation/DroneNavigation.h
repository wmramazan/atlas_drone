#ifndef DRONENAVIGATION_H
#define DRONENAVIGATION_H

#include <ros/ros.h>
#include <string>

#include <DroneNavigation/PathPlanner.h>
#include <DroneNavigation/GlobalPlanner.h>
#include <DroneNavigation/NavigationVisualizer.h>

using namespace std;
using namespace ros;

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

    NavigationVisualizer* navigation_visualizer;
};

#endif // DRONENAVIGATION_H
