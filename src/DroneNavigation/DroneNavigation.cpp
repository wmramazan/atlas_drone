#include "DroneNavigation/DroneNavigation.h"

int main(int argc, char **argv)
{
    ROS_INFO("> Initializing Drone AI...");
    ros::init(argc, argv, "drone_ai");
    DroneNavigation drone_navigation;
    ROS_INFO("< Drone AI Terminated...");
    return 0;
}

DroneNavigation::DroneNavigation()
{
    nh = ros::NodeHandle();

    global_planner = new GlobalPlanner(nh);

    ros::NodeHandle uav1_nh(nh, "uav1");
    ros::NodeHandle uav2_nh(nh, "uav2");
    ros::NodeHandle uav3_nh(nh, "uav3");

    vehicle1_planner = new PathPlanner(uav1_nh, global_planner);
    vehicle2_planner = new PathPlanner(uav2_nh, global_planner);
    vehicle3_planner = new PathPlanner(uav3_nh, global_planner);

    path_marker_service    = nh.advertiseService(nh.param<string>("/path_marker_service", "/path_marker"), &DroneNavigation::pathMarkerServiceCallback, this);
    costmap_marker_service = nh.advertiseService(nh.param<string>("/costmap_marker_service", "/costmap_marker"), &DroneNavigation::costmapMarkerServiceCallback, this);

    ros::Rate loop_rate(40);
    while (ok())
    {
        spinOnce();
        loop_rate.sleep();
    }
}

bool DroneNavigation::pathMarkerServiceCallback(MarkerServiceRequest& request, MarkerServiceResponse& response)
{

}

bool DroneNavigation::costmapMarkerServiceCallback(MarkerServiceRequest& request, MarkerServiceResponse& response)
{

}



