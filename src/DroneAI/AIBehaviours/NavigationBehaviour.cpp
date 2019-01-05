#include "DroneAI/AIBehaviour.h"
#include "DroneAI/DJIDrone.h"
#include "DroneAI/AITask.h"

#include <nav_msgs/Path.h>
#include "DroneNavigation/Vec3.h"

NavigationBehaviour::NavigationBehaviour(NodeHandle& nh)
{
    LOG("||-> Initializing Navigation Behaviour.");
    name = "Navigation Behaviour";

    navigation_target_sub   = nh.subscribe(nh.param<string>("/target_pose", "target_pose"),  1, &NavigationBehaviour::navigation_target_callback,    this);
    path_sub                = nh.subscribe(nh.param<string>("/drone_path_topic", "drone_path"),      1, &NavigationBehaviour::path_callback,                 this);

    generate_path_service_client = nh.serviceClient<Trigger>( nh.param<string>("/generate_path_service", "/path_planner/generate_path") );
    is_path_clear_service_client = nh.serviceClient<Trigger>( nh.param<string>("/is_path_clear_service", "/path_planner/is_path_clear") );
    target_pose_pub = nh.advertise<Pose>(nh.param<string>("/navigation_target_pose_topic", "navigation_target_pose"), 10);
    LOG("||-< Navigation Behaviour Initialization Complete.");
}

void NavigationBehaviour::Update()
{
    if (!path_found)
    {
        ROS_INFO("No path");
        return;
    }

    if (CurrentTask == NULL)
    {
        Vec3 current_position = DRONE->GetPosition();
        Vec3 direction = Vec3::FromPose(path.poses[1].pose) - Vec3::FromPose(path.poses[0].pose);

        //LOG("Direction = %f %f %f", direction.x, direction.y, direction.z);

        Vec3 currentDirection;

        int target_node_index = 1;
        do
        {
            Vec3 curP = Vec3::FromPoint(path.poses[target_node_index].pose.position);
            Vec3 nextP = Vec3::FromPoint(path.poses[target_node_index + 1].pose.position);
            currentDirection = nextP - curP;
            target_node_index++;
            ROS_INFO("%f %f %f from %f %f %f",  nextP.x, nextP.y, nextP.z, curP.x, curP.y, curP.z);
            ROS_INFO("Current Direction: %f %f %f index: %d", currentDirection.x, currentDirection.y, currentDirection.z, target_node_index);
        } while (currentDirection == direction && target_node_index < 20);

        Pose target_pose = path.poses[target_node_index - 1].pose;
        target_pose_pub.publish(target_pose);
        Vec3 target_position = Vec3::FromPoint(target_pose.position);

        ROS_INFO("Next position is from %f - %f - %f to %f - %f - %f", current_position.x, current_position.y, current_position.z, target_position.x, target_position.y, target_position.z);

        double distance = current_position.Distance(target_position);


        ROS_INFO("Distance is: %lf", distance);
        if (distance > 0.1f && request_path_clearence())
        {
            LOG("||-> Adding task with pose: %f - %f - %f", target_pose.position.x, target_pose.position.y, target_pose.position.z);
            AddTask(new MoveTask(target_position, atan2(direction.y, direction.x)));
        }
    }

    /*else
    {
        if (!request_path_clearence())
        {
            LOG("Path is not clear.");
            CurrentTask->Terminate();
        }
    }*/

    AIBehaviour::Update();
}

void NavigationBehaviour::task_complete_callback(AITaskResult &result)
{
    //generatePath();
    AIBehaviour::task_complete_callback(result);
}

void NavigationBehaviour::navigation_target_callback(const Pose::ConstPtr &msg)
{
    navigation_target = *msg;
    LOG("||-> Navigation target set to: %f - %f - %f", navigation_target.position.x, navigation_target.position.y, navigation_target.position.z);

    if (request_path())
    {
        LOG("||- Path found, should recieve soon.");
    }
    else
    {
        LOG("||- No path has found to navigation target.");
    }
}

void NavigationBehaviour::path_callback(const Path::ConstPtr &msg)
{
    LOG("Recieving path");
    path = *msg;

    if (path.poses.size() > 1)
    {
        LOG("Recieved path.");
        path_found = true;
        path_time = ros::Time::now();
    }
    else
    {
        LOG("Recieved empty path.");
        path_found = false;
    }
}

bool NavigationBehaviour::request_path()
{
    LOG("||-> Requesting path.");

    Trigger trigger;
    generate_path_service_client.call(trigger);

    if (trigger.response.success)
    {
        LOG("||-< Request succesfull.");
        return true;
    }
    else
    {
        LOG("||-< Request failed.");
        return false;
    }
}

bool NavigationBehaviour::request_path_clearence()
{
    LOG("||-< Requesting path clearence");

    Trigger trigger;
    is_path_clear_service_client.call(trigger);

    if (trigger.response.success)
    {
        LOG("||-< Request succesfull");
        return true;
    }
    else
    {
        LOG("||-< Request failed");
        return false;
    }
}
