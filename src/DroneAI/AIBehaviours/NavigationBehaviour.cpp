#include "DroneAI/AIBehaviour.h"
#include "DroneAI/DJIDrone.h"
#include "DroneAI/AITask.h"
#include "DroneNavigation/PathPlanner.h"
#include "DroneNavigation/Vec3.h"

NavigationBehaviour::NavigationBehaviour(NodeHandle& nh)
{
    LOG("||-> Initializing Navigation Behaviour.");
    name = "Navigation Behaviour";
    navigation_target_sub = nh.subscribe("/drone_navigation/navigation_target",  10, &NavigationBehaviour::navigation_target_callback,  this);
    pathPlanner = new PathPlanner(nh, PathPlanner::LOCAL_COSTMAP | PathPlanner::PATH);
    LOG("||-< Navigation Behaviour Initialization Complete.");
}

void NavigationBehaviour::Update()
{
    Pose pose;
    pose.position = DRONE->LocalPosition.point;
    pathPlanner->SetCurrentPose(pose);

    path = pathPlanner->GeneratePath();

    if (path != NULL)
    {
        PoseStamped next_pose = path->poses[1];

        Vec3 current_position = Vec3::FromPoint(DRONE->LocalPosition.point);
        Vec3 target_position = Vec3::FromPoint(next_pose.pose.position);

        ROS_INFO("Next delta is from %f - %f - %f to %f - %f - %f", current_position.x, current_position.y, current_position.z, target_position.x, target_position.y, target_position.z);

        double distance = current_position.Distance(target_position);

        ROS_INFO("Delta is: %lf", distance);
        if (distance > 0.1f && !on_task)
        {
            on_task = true;
            LOG("||-> Adding task with pose: %f - %f - %f", next_pose.pose.position.x, next_pose.pose.position.y, next_pose.pose.position.z);
            AddTask(new MoveTask(next_pose.pose));
        }
    }

    if (CurrentTask == NULL)
    {
        ROS_INFO("Navigation Behaviour: Null task exception.");
        next_task();
    }
    else
    {
        ROS_INFO("Updating %s.", CurrentTask->Name.c_str());
        CurrentTask->Update();
    }
}



void NavigationBehaviour::task_complete_callback(AITaskResult &result)
{
    on_task = false;
    AIBehaviour::task_complete_callback(result);
}

void NavigationBehaviour::navigation_target_callback(const Pose::ConstPtr &msg)
{
    navigation_target = *msg;
    pathPlanner->SetTargetPose(navigation_target);
    LOG("||-> Navigation target set to: %f - %f - %f", navigation_target.position.x, navigation_target.position.y, navigation_target.position.z);
}
