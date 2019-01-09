#include "DroneNavigation/PathPlanner.h"

PathPlanner::PathPlanner(NodeHandle& nh, GlobalPlanner* global_planner, int drone_id)
{
    generate_path = false;
    go_to_target = false;

    string ns = "/uav" + to_string(drone_id) + "/";

    this->drone_start_position = Vec3(
                nh.param(ns + "start_position_x", 0),
                nh.param(ns + "start_position_y", 0),
                nh.param(ns + "start_position_z", 0)
    );

    this->drone_id = drone_id;

    size = nh.param("/size", 600);
    half_size = size / 2;

    resolution = nh.param("/resolution", 0.1);
    half_resolution = resolution / 2;

    frame_id = nh.param<string>("/frame_id", "world");

    this->global_planner = global_planner;
    local_planner = new LocalPlanner(nh, ns);

    pathfinder = new Pathfinder(global_planner, local_planner);

    drone_position_sub      = nh.subscribe(ns + nh.param<string>("/drone_position_topic", "mavros/local_position/pose"), 10, &PathPlanner::drone_position_callback, this);
    feedback_marker_sub     = nh.subscribe(ns + nh.param<string>("/marker_feedback_topic", "marker/feedback"), 10, &PathPlanner::marker_feedback_callback, this);
    drone_target_sub        = nh.subscribe(ns + "/mavros/setpoint_raw/local", 10, &PathPlanner::drone_target_callback, this);

    navigation_target_pub   = nh.advertise<Pose>(ns + nh.param<string>("/target_pose_topic", "target_pose"), 10);
    terminal_message_pub    = nh.advertise<String>(ns + nh.param<string>("/terminal_message_topic", "drone_ai/terminal_message"), 100);
    path_pub                = nh.advertise<Path>(ns + nh.param<string>("/drone_path_topic", "drone_path"), 5);

    go_to_target_service_client     = nh.serviceClient<Trigger>(ns + nh.param<string>("/go_to_target_service", "drone_ai/go_to_target"));
    visualize_path_service          = nh.serviceClient<VisualizerMessage>(nh.param<string>("/visualize_path_service", "visualizer/visualize_path"));
    visualize_costmap_service       = nh.serviceClient<VisualizerMessage>(nh.param<string>("/visualize_costmap_service", "visualizer/visualize_costmap"));

    request_path_service            = nh.advertiseService(ns + nh.param<string>("/generate_path_service", "path_planner/generate_path"), &PathPlanner::generate_path_service_callback, this);
    request_path_clearance_service  = nh.advertiseService(ns + nh.param<string>("/is_path_clear_service", "path_planner/is_path_clear"), &PathPlanner::is_path_clear_service_callback, this);
}

void PathPlanner::Update()
{
    if (generate_path)
    {
        GeneratePath();
        generate_path = false;
    }
}

void PathPlanner::GeneratePath()
{
    path.poses.clear();

    Vec3Int start;
    start.x = to_index(current_position.x);
    start.y = to_index(current_position.y);
    start.z = to_index(current_position.z);

    Vec3Int end;
    end.y = to_index(target_position.y);
    end.x = to_index(target_position.x);
    end.z = to_index(target_position.z);

    double current_timestamp = ros::Time::now().toSec();
    vector<Vec3Int> found_path = pathfinder->Find(start, end);
    ROS_INFO("Execution time: %lf s", ros::Time::now().toSec() - current_timestamp);
    if (found_path.size())
    {
        ROS_INFO("path_found");
        path.header.frame_id = frame_id;
        PoseStamped pose;

        for (Vec3Int coordinate : found_path)
        {
            pose.pose.position.x = to_position(coordinate.x);
            pose.pose.position.y = to_position(coordinate.y);
            pose.pose.position.z = to_position(coordinate.z);
            path.poses.push_back(pose);
        }

        path_pub.publish(path);
        VisualizerMessage message;
        ROS_INFO("visualize_path before");
        visualize_path_service.call(message);
        ROS_INFO("visualize_path after");
    }
    else
    {
        ROS_INFO("path_not_found");
    }
}

bool PathPlanner::IsPathClear()
{
    bool is_path_clear = local_planner->IsPathClear(&path);
    ROS_INFO("Path Clear: %d", is_path_clear);
    return is_path_clear;
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
    current_position = Vec3::FromPose(msg->pose) + drone_start_position;

    Vec3Int current_index(to_index(current_position.x), to_index(current_position.y), to_index(current_position.z));
    for (int i = current_index.x - 1; i <= current_index.x + 1; i++)
    {
        for (int j = current_index.y - 1; j <= current_index.y + 1; j++)
        {
            for (int k = current_index.z - 1; k <= current_index.z + 1; k++)
            {
                local_planner->SetOccupancy(Vec3Int(i, j, k), 0);
                global_planner->SetOccupancy(Vec3Int(i, j, k), 0);
            }
        }
    }
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
                    ROS_INFO("Take off");
                    terminal_message_pub.publish(message);
                    break;
                }

                case 5: // Go to target
                {
                    navigation_target_pub.publish(feedback->pose);
                    go_to_target_service_client.call(trigger_srv);
                    break;
                }

                case 7: // Local Costmap
                {
                    VisualizerMessage visualizerMessage;
                    visualizerMessage.request.drone_id = drone_id;
                    visualizerMessage.request.costmap_type = 1;
                    visualizerMessage.request.origin = feedback->pose.position;
                    visualize_costmap_service.call(visualizerMessage);
                }
                case 8: // Global Costmap
                {
                    VisualizerMessage visualizerMessage;
                    visualizerMessage.request.drone_id = drone_id;
                    visualizerMessage.request.costmap_type = 2;
                    visualizerMessage.request.origin = feedback->pose.position;
                    visualize_costmap_service.call(visualizerMessage);
                }
                case 9: // Merged Costmap
                {
                    VisualizerMessage visualizerMessage;
                    visualizerMessage.request.drone_id = drone_id;
                    visualizerMessage.request.costmap_type = 0;
                    visualizerMessage.request.origin = feedback->pose.position;
                    visualize_costmap_service.call(visualizerMessage);
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
            generate_path = true;
            break;
        }
    }
}

void  PathPlanner::drone_target_callback(const PositionTarget::ConstPtr& msg)
{
    drone_target_position = Vec3(msg->position.x, msg->position.y, msg->position.z) + drone_start_position;
}

uint PathPlanner::to_index(double value)
{
    uint result = (uint) (((value) / resolution) + half_size + 1);
    //ROS_INFO("toIndex: %lf %d", value, result);
    return result;
}

double PathPlanner::to_position(int value)
{
    double result = (double) (value - half_size) * resolution - half_resolution;
    //ROS_INFO("toPosition: %d %lf", value, result);
    return result;
}

bool PathPlanner::is_occupied(Vec3Int index, int type)
{
    switch  (type)
    {
        case 0:
            return global_planner->IsOccupied(index) || local_planner->IsOccupied(index);
        case 1:
            return local_planner->IsOccupied(index);
        case 2:
            return global_planner->IsOccupied(index);
    }
}
