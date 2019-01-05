#include "DroneNavigation/PathPlanner.h"

PathPlanner::PathPlanner(NodeHandle& nh, GlobalPlanner* global_planner, string drone_id)
{
    size = nh.param("/size", 600);
    resolution = nh.param("/resolution", 0.1);

    frame_id = nh.param<string>("/frame_id", "world");

    string path_topic = nh.param<std::string>("/drone_path_topic", "drone_path");

    this->costmap = new Costmap(size, resolution);
    this->pathfinder = new Pathfinder(costmap);

    path_pub = nh.advertise<Path>(path_topic, 5);

    this->global_planner = global_planner;
    local_planner = new LocalPlanner(nh, drone_id);

    //generate_path_service = nh.advertiseService(nh.param<std::string>("/generate_path_service", "/path_planner/generate_path"), &generate_path_service_callback);
    //is_path_clear_service = nh.advertiseService(nh.param<std::string>("/is_path_clear_service", "/path_planner/is_path_clear"), &is_path_clear_service_callback);
}

void PathPlanner::GeneratePath(Path& path, Vec3 start_position, Vec3 target_position)
{
    path.poses.clear();

    Vec3Int start;
    start.x = costmap->ToIndex(start_position.x);
    start.y = costmap->ToIndex(start_position.y);
    start.z = costmap->ToIndex(start_position.z);

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

