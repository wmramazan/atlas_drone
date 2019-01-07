#include "DroneNavigation/PathPlanner.h"

PathPlanner::PathPlanner(NodeHandle& nh, GlobalPlanner* global_planner, string ns)
{
    generate_path = false;
    go_to_target = false;

    this->start_position = Vec3(
                nh.param(ns + "start_position_x", 0),
                nh.param(ns + "start_position_y", 0),
                nh.param(ns + "start_position_z", 0)
    );

    size = nh.param("/size", 600);
    resolution = nh.param("/resolution", 0.1);
    frame_id = nh.param<string>("/frame_id", "world");

    this->global_planner = global_planner;
    local_planner = new LocalPlanner(nh, ns);

    pathfinder = new Pathfinder(global_planner, local_planner);

    drone_position_sub      = nh.subscribe(ns + nh.param<string>("/drone_position_topic", "mavros/local_position/pose"), 10, &PathPlanner::drone_position_callback, this);
    feedback_marker_sub     = nh.subscribe(ns + nh.param<string>("/marker_feedback_topic", "marker/feedback"), 10, &PathPlanner::marker_feedback_callback, this);

    navigation_target_pub   = nh.advertise<Pose>(ns + nh.param<string>("/target_pose_topic", "target_pose"), 10);
    terminal_message_pub    = nh.advertise<String>(ns + nh.param<string>("/terminal_message_topic", "drone_ai/terminal_message"), 100);
    path_pub                = nh.advertise<Path>(ns + nh.param<string>("/drone_path_topic", "drone_path"), 5);

    go_to_target_service_client     = nh.serviceClient<Trigger>(ns + nh.param<string>("/go_to_target_service", "drone_ai/go_to_target"));

    request_path_service            = nh.advertiseService(ns + nh.param<string>("/generate_path_service", "path_planner/generate_path"), &PathPlanner::generate_path_service_callback, this);
    request_path_clearance_service  = nh.advertiseService(ns + nh.param<string>("/is_path_clear_service", "path_planner/is_path_clear"), &PathPlanner::is_path_clear_service_callback, this);
}

void PathPlanner::Update()
{
    request = VisualizationRequest();

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
        request.path_request = true;
    }
    else
    {

    }
}

bool PathPlanner::IsPathClear()
{
    return local_planner->IsPathClear(&path);
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
    current_position = Vec3::FromPose(msg->pose) + start_position;

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
                    request.costmap_request = true;
                    request.costmap_type = 1;
                    request.origin = Vec3::FromPose(feedback->pose);
                }
                case 8: // Global Costmap
                {
                    request.costmap_request = true;
                    request.costmap_type = 2;
                    request.origin = Vec3::FromPose(feedback->pose);
                }
                case 9: // Merged Costmap
                {
                    request.costmap_request = true;
                    request.costmap_type = 0;
                    request.origin = Vec3::FromPose(feedback->pose);
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

uint PathPlanner::to_index(double value)
{
    return (uint) (((value) / resolution) + size / 2 + 1);
}

double PathPlanner::to_position(int value)
{
    return (double) (value - size / 2) * resolution - resolution / 2;
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
