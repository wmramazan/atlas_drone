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

    string feedback_marker_topic = nh.param<string>("/marker_feedback_topic", "marker/feedback");
    feedback_marker_sub     = nh.subscribe(feedback_marker_topic, 10, &DroneNavigation::marker_feedback_callback, this);

    navigation_target_pub   = nh.advertise<Pose>(nh.param<string>("/target_pose_topic", "target_pose"), 10);
    terminal_message_pub    = nh.advertise<std_msgs::String>(nh.param<string>("/terminal_message_topic", "drone_ai/terminal_message"), 1000);

    go_to_target_service_client     = nh.serviceClient<Trigger>(nh.param<string>("/go_to_target_service", "/drone_ai/go_to_target"));
    generate_path_service_client    = nh.serviceClient<Trigger>(nh.param<string>("/generate_path_service", "/path_planner/generate_path"));

    global_planner = new GlobalPlanner(nh);

    ros::NodeHandle uav1_nh(nh, "uav1");
    ros::NodeHandle uav2_nh(nh, "uav2");
    ros::NodeHandle uav3_nh(nh, "uav3");

    vehicle1_planner = new PathPlanner(uav1_nh, global_planner);
    vehicle2_planner = new PathPlanner(uav2_nh, global_planner);
    vehicle3_planner = new PathPlanner(uav3_nh, global_planner);

    ros::Rate loop_rate(40);
    while (ok())
    {
        spinOnce();
        loop_rate.sleep();
    }
}

void DroneNavigation::marker_feedback_callback(const InteractiveMarkerFeedbackConstPtr &feedback)
{
    switch (feedback->event_type)
    {
        case InteractiveMarkerFeedback::MENU_SELECT:
        {
            switch (feedback->menu_entry_id)
            {
                case 1: // Take off
                {
                    String message;
                    message.data = "init";
                    terminal_message_pub.publish(message);
                    break;
                }
                case 5: // Go to target
                {
                    navigation_target_pub.publish(feedback->pose);

                    if (go_to_target_service_client.call(trigger_srv))
                    {
                        ROS_INFO("Going to target...");
                    }
                    else
                    {
                        ROS_ERROR("Failed to call service go_to_target_service_client");
                    }
                    break;
                }
            }

            break;
        }
        case InteractiveMarkerFeedback::POSE_UPDATE:
        {
            //ROS_INFO("POSE_UPDATE");
            break;
        }
        case InteractiveMarkerFeedback::MOUSE_DOWN:
        {
            //ROS_INFO("MOUSE_DOWN");
            break;
        }
        case InteractiveMarkerFeedback::MOUSE_UP:
        {
            //ROS_INFO("MOUSE_UP");
            //findPath();
            //generate_costmap_marker_array(feedback->pose);
            break;
        }
    }
}
