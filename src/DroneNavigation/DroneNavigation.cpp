#include "DroneNavigation/DroneNavigation.h"

int main(int argc, char **argv)
{
    ROS_INFO("> Initializing Drone Navigation...");
    ros::init(argc, argv, "drone_navigation");
    DroneNavigation drone_navigation;
    ROS_INFO("< Drone Navigation Terminated...");
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

    navigation_visualizer = new NavigationVisualizer(nh, global_planner, vehicle1_planner->local_planner);

    path_marker_service    = nh.advertiseService(nh.param<string>("/path_marker_service", "/path_marker"), &DroneNavigation::path_marker_service_callback, this);
    costmap_marker_service = nh.advertiseService(nh.param<string>("/costmap_marker_service", "/costmap_marker"), &DroneNavigation::costmap_marker_service_callback, this);

    ros::Rate loop_rate(40);
    while (ok())
    {
        spinOnce();

        vehicle1_planner->Update();
        vehicle2_planner->Update();
        vehicle3_planner->Update();

        loop_rate.sleep();
    }
}

bool DroneNavigation::path_marker_service_callback(MarkerServiceRequest& request, MarkerServiceResponse& response)
{
    ROS_INFO("path_marker_service_callback");
    navigation_visualizer->PublishPathMarkers();
    return true;
}

bool DroneNavigation::costmap_marker_service_callback(MarkerServiceRequest& request, MarkerServiceResponse& response)
{

}



