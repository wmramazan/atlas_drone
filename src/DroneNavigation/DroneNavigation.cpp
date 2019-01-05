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

    global_planner = new GlobalPlanner(nh);

    vehicle1_planner = new PathPlanner(nh, global_planner, "uav1");
    vehicle2_planner = new PathPlanner(nh, global_planner, "uav2");
    vehicle3_planner = new PathPlanner(nh, global_planner, "uav3");

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
