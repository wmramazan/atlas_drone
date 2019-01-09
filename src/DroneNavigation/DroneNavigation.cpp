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
    nh = ros::NodeHandle();

    global_planner = new GlobalPlanner(nh);

    vehicle1_planner = new PathPlanner(global_planner, 1);
    vehicle2_planner = new PathPlanner(global_planner, 2);
    vehicle3_planner = new PathPlanner(global_planner, 3);

    navigation_visualizer = new NavigationVisualizer(nh);
    navigation_visualizer->AddPathPlanner(vehicle1_planner);
    navigation_visualizer->AddPathPlanner(vehicle2_planner);
    navigation_visualizer->AddPathPlanner(vehicle3_planner);

    navigation_visualizer->SwitchPathPlanner(0);

    ros::Rate loop_rate(40);
    while (ok())
    {
        spinOnce();

        vehicle1_planner->Update();
        vehicle2_planner->Update();
        vehicle3_planner->Update();

        /*
        VisualizationRequest request;
        request.path_request = vehicle1_planner->request.path_request || vehicle2_planner->request.path_request || vehicle3_planner->request.path_request;

        if (vehicle1_planner->request.costmap_request)
        {
            request.costmap_request = true;
            request.costmap_type = vehicle1_planner->request.costmap_type;
            navigation_visualizer->SwitchPathPlanner(0);
        }
        else if (vehicle2_planner->request.costmap_request)
        {
            request.costmap_request = true;
            request.costmap_type = vehicle2_planner->request.costmap_type;
            navigation_visualizer->SwitchPathPlanner(1);
        }
        else if (vehicle3_planner->request.costmap_request)
        {
            request.costmap_request = true;
            request.costmap_type = vehicle3_planner->request.costmap_type;
            navigation_visualizer->SwitchPathPlanner(2);
        }

        navigation_visualizer->Update(request);
        */

        loop_rate.sleep();
    }
}

