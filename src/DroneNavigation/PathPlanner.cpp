#include "DroneNavigation/PathPlanner.h"

PathPlanner::PathPlanner(NodeHandle& nh, GlobalPlanner* global_planner)
{
    size = nh.param("/size", 600);
    resolution = nh.param("/resolution", 0.1);
    frame_id = nh.param<string>("/frame_id", "world");

    costmap = new Costmap(size, resolution);

    this->global_planner = global_planner;
    local_planner = new LocalPlanner(nh);

    pathfinder = new Pathfinder(global_planner, local_planner);

    string drone_position_topic = nh.param<string>("/drone_position_topic", "mavros/local_position/pose");
    drone_position_sub      = nh.subscribe(drone_position_topic, 10, &PathPlanner::drone_position_callback, this);

    string feedback_marker_topic = nh.param<string>("/marker_feedback_topic", "marker/feedback");
    feedback_marker_sub     = nh.subscribe(feedback_marker_topic, 10, &PathPlanner::marker_feedback_callback, this);

    string navigation_target_topic = nh.param<string>("/target_pose_topic", "target_pose");
    navigation_target_pub   = nh.advertise<Pose>(navigation_target_topic, 10);

    string terminal_message_topic = nh.param<string>("/terminal_message_topic", "drone_ai/terminal_message");
    terminal_message_pub    = nh.advertise<std_msgs::String>(terminal_message_topic, 100);

    string path_topic = nh.param<std::string>("/drone_path_topic", "drone_path");
    path_pub = nh.advertise<Path>(path_topic, 5);

    go_to_target_service_client     = nh.serviceClient<Trigger>(nh.param<string>("/go_to_target_service", "/drone_ai/go_to_target"));

    request_path_service            = nh.advertiseService(nh.param<std::string>("/generate_path_service", "path_planner/generate_path"), &PathPlanner::generate_path_service_callback, this);
    request_path_clearance_service  = nh.advertiseService(nh.param<std::string>("/is_path_clear_service", "path_planner/is_path_clear"), &PathPlanner::is_path_clear_service_callback, this);
}

void PathPlanner::Update()
{
    if (generate_path)
    {
        GeneratePath();
    }
}

void PathPlanner::GeneratePath()
{
    Path path;

    Vec3Int start;
    start.x = costmap->ToIndex(current_position.x);
    start.y = costmap->ToIndex(current_position.y);
    start.z = costmap->ToIndex(current_position.z);

    Vec3Int end;
    end.x = costmap->ToIndex(target_position.x);
    end.y = costmap->ToIndex(target_position.y);
    end.z = costmap->ToIndex(target_position.z);

    double current_timestamp = ros::Time::now().toSec();
    vector<Vec3Int> found_path = pathfinder->Find(start, end);
    ROS_INFO("Execution time: %lf s", ros::Time::now().toSec() - current_timestamp);
    if (found_path.size())
    {
        path.header.frame_id = frame_id;

        PoseStamped pose;

        for (Vec3Int coordinate : found_path)
        {
            pose.pose.position.x = costmap->ToPosition(coordinate.x);
            pose.pose.position.y = costmap->ToPosition(coordinate.y);
            pose.pose.position.z = costmap->ToPosition(coordinate.z);
            //ROS_INFO("Coordinate: %d %d %d  %lf %lf %lf", coordinate.x, coordinate.y, coordinate.z, pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
            path.poses.push_back(pose);
        }

        path_pub.publish(path);

        generate_path = false;
    }
    else
    {
        generate_path = false;
    }
}

bool PathPlanner::IsPathClear()
{
    return costmap->CanPathPass(&path);
}

Pose PathPlanner::GetNextPathNode()
{
    // TODO: Move implementation from NavigationBehaviour
    return path.poses[0].pose;
}

bool PathPlanner::is_path_clear_service_callback(TriggerRequest& request, TriggerResponse& response)
{
    response.success = IsPathClear();
    return true;
}

bool PathPlanner::generate_path_service_callback(TriggerRequest& request, TriggerResponse& response)
{
    generate_path = true;
    return true;
}

void PathPlanner::drone_position_callback(const PoseStamped::ConstPtr& msg)
{
    current_position = Vec3::FromPose(msg->pose);
}

void PathPlanner::marker_feedback_callback(const InteractiveMarkerFeedbackConstPtr &feedback)
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
            target_position = Vec3::FromPose(feedback->pose);
            navigation_target_pub.publish(feedback->pose);
            generate_path = true;
            break;
        }
    }
}

