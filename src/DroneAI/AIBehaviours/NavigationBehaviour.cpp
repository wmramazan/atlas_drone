#include "DroneAI/AIBehaviour.h"
#include "DroneAI/DJIDrone.h"
#include "DroneAI/AITask.h"
#include "DroneNavigation/PathPlanner.h"

NavigationBehaviour::NavigationBehaviour(NodeHandle& nh)
{
    LOG("||-> Initializing Navigation Behaviour.");
    name = "Navigation Behaviour";
    navigation_target_sub = nh.subscribe("drone_navigation/navigation_target",  10, &NavigationBehaviour::navigation_target_callback,  this);
    pathPlanner = new PathPlanner(nh, PathPlanner::LOCAL_COSTMAP | PathPlanner::PATH);
    LOG("||-< Navigation Behaviour Initialization Complete.");
}

void NavigationBehaviour::Update()
{
    //pathPlanner->SetCurrentPose(DRONE->LocalPosition);
    //path = pathPlanner->GeneratePath();

    if (path.poses.size() > 0)
    {
        //Pose next_pose = path.poses[0];

    }

    if (CurrentTask == NULL)
    {
        ROS_INFO("Null task exception.");
        next_task();
    }
    else
    {
        ROS_INFO("Updating %s.", CurrentTask->name.c_str());
        CurrentTask->Update();
    }
}

void NavigationBehaviour::navigation_target_callback(const Pose::ConstPtr &msg)
{
    navigation_target = *msg;
    pathPlanner->SetTargetPose(navigation_target);
}
