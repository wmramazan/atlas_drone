#include "DroneNavigation/DroneNavigation.h"
#include "std_srvs/Trigger.h"

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
    NodeHandle nh = ros::NodeHandle();

    global_planner = new GlobalPlanner(nh);

    navigation_visualizer = new NavigationVisualizer(nh);

    vehicle1_planner = new PathPlanner(global_planner, 1, bind(&NavigationVisualizer::visualization_request_callback, navigation_visualizer, placeholders::_1));
    vehicle2_planner = new PathPlanner(global_planner, 2, bind(&NavigationVisualizer::visualization_request_callback, navigation_visualizer, placeholders::_1));
    vehicle3_planner = new PathPlanner(global_planner, 3, bind(&NavigationVisualizer::visualization_request_callback, navigation_visualizer, placeholders::_1));


    navigation_visualizer->AddPathPlanner(vehicle1_planner);
    navigation_visualizer->AddPathPlanner(vehicle2_planner);
    navigation_visualizer->AddPathPlanner(vehicle3_planner);

    navigation_visualizer->SwitchPathPlanner(0);

    ros::Rate loop_rate(80);
    while (ok())
    {
        spinOnce();

        vehicle1_planner->Update();
        vehicle2_planner->Update();
        vehicle3_planner->Update();

        loop_rate.sleep();
    }
}



