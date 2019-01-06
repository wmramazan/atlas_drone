#include "DroneAI/AIBehaviour.h"
#include "DroneAI/DJIDrone.h"
#include "DroneAI/AITask.h"

#include <nav_msgs/Path.h>
#include "DroneNavigation/Vec3.h"

NavigationBehaviour::NavigationBehaviour(NodeHandle& nh)
{
    LOG("||-> Initializing Navigation Behaviour.");
    name = "Navigation Behaviour";

    path_sub = nh.subscribe(nh.param<string>("/drone_path_topic", "drone_path"), 1, &NavigationBehaviour::path_callback, this);
    generate_path_service_client = nh.serviceClient<Trigger>(nh.param<string>("/generate_path_service", "path_planner/generate_path"));
    is_path_clear_service_client = nh.serviceClient<Trigger>(nh.param<string>("/is_path_clear_service", "path_planner/is_path_clear"));
    LOG("||-< Navigation Behaviour Initialization Complete.");
}

void NavigationBehaviour::OnEnter()
{
    if (CurrentTask == nullptr)
    {
        //AddTask(new IdleTask(true));
    }
}

void NavigationBehaviour::Update()
{
    AIBehaviour::Update();

    if (!path_found)
    {
        return;
    }

    if (CurrentTask == nullptr)
    {
        Vec3 path_direction;
        uint target_node_index = calculate_next_node_index(0, path_direction);
        Pose target_pose = path.poses[target_node_index].pose;
        Vec3 target_position = Vec3::FromPoint(target_pose.position);
        Vec3 current_position = DRONE->GetPosition();
        double distance = current_position.Distance(target_position);

        if (distance > 0.1 && request_path_clearence())
        {
            LOG("||-> Adding task with pose: %f - %f - %f", target_pose.position.x, target_pose.position.y, target_pose.position.z);
            AddTask(new MoveTask(target_position, atan2(path_direction.y, path_direction.x), 100000));
        }
    }
    else
    {
        if (!request_path_clearence())
        {
            LOG("Path is not clear.");
            CurrentTask->Terminate();
        }
    }
}

void NavigationBehaviour::OnExit()
{

}

uint NavigationBehaviour::calculate_next_node_index(uint start_index, Vec3& path_direction)
{
    path_direction = Vec3::FromPose(path.poses[start_index + 1].pose) - Vec3::FromPose(path.poses[start_index].pose);

    Vec3 current_direction = path_direction;
    uint target_node_index = start_index;

    while (current_direction == path_direction && target_node_index - start_index < 20)
    {
        target_node_index++;
        Vec3 current_node   = Vec3::FromPose(path.poses[target_node_index].pose);
        Vec3 next_node      = Vec3::FromPose(path.poses[target_node_index + 1].pose);
        current_direction = next_node - current_node;
    }

    return target_node_index;
}

void NavigationBehaviour::on_task_added()
{
    if (CurrentTask == nullptr || CurrentTask->Name == "\"Initialization Task\"")
    {
        next_task();
    }
}

void NavigationBehaviour::task_complete_callback(AITaskResult &result)
{
    request_path();
    AIBehaviour::task_complete_callback(result);
}

void NavigationBehaviour::path_callback(const Path::ConstPtr &msg)
{
    //LOG("Recieving path");
    path = *msg;

    if (path.poses.size() > 1)
    {
        //LOG("Recieved path.");
        path_found = true;
        path_time = ros::Time::now();
    }
    else
    {
        //LOG("Recieved empty path.");
        path_found = false;
    }
}

bool NavigationBehaviour::request_path()
{
    //LOG("||-> Requesting path.");

    Trigger trigger;
    generate_path_service_client.call(trigger);

    if (trigger.response.success)
    {
        //LOG("||-< Request succesfull.");
        return true;
    }
    else
    {
        //LOG("||-< Request failed.");
        return false;
    }
}

bool NavigationBehaviour::request_path_clearence()
{
    //LOG("||-< Requesting path clearence");

    Trigger trigger;
    is_path_clear_service_client.call(trigger);

    if (trigger.response.success)
    {
        //LOG("||-< Request succesfull");
        return true;
    }
    else
    {
        //LOG("||-< Request failed");
        return false;
    }
}
