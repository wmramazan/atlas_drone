#include "DroneAI/AIBehaviour.h"
#include "DroneAI/DJIDrone.h"
#include "DroneAI/AITask.h"
#include "DroneNavigation/PathPlanner.h"
#include "DroneNavigation/Vec3.h"

NavigationBehaviour::NavigationBehaviour(NodeHandle& nh)
{
    LOG("||-> Initializing Navigation Behaviour.");
    name = "Navigation Behaviour";
    navigation_target_sub = nh.subscribe("/drone_navigation/navigation_target",  1, &NavigationBehaviour::navigation_target_callback,  this);
    target_pose_pub = nh.advertise<Pose>("navigation_target_pose", 10);
    //pathPlanner = new PathPlanner(nh, PathPlanner::Mode::VEHICLE);
    LOG("||-< Navigation Behaviour Initialization Complete.");
}

void NavigationBehaviour::Update()
{
    if (path == NULL)
      return;

    if (CurrentTask == NULL)
    {
        Vec3 current_position = Vec3::FromPose(DRONE->LocalPosition.pose);
        Vec3 direction = Vec3::FromPose(path->poses[1].pose) - Vec3::FromPose(path->poses[0].pose);

        LOG("Direction = %f %f %f", direction.x, direction.y, direction.z);

        Vec3 currentDirection;

        int target_node_index = 1;
        do
        {
            Vec3 curP = Vec3::FromPoint(path->poses[target_node_index].pose.position);
            Vec3 nextP = Vec3::FromPoint(path->poses[target_node_index + 1].pose.position);
            currentDirection = nextP - curP;
            target_node_index++;
            ROS_INFO("%f %f %f from %f %f %f",  nextP.x, nextP.y, nextP.z, curP.x, curP.y, curP.z);
            ROS_INFO("Current Direction: %f %f %f index: %d", currentDirection.x, currentDirection.y, currentDirection.z, target_node_index);
        } while (currentDirection == direction && target_node_index < 20);

        Pose target_pose = path->poses[target_node_index - 1].pose;
        target_pose_pub.publish(target_pose);
        Vec3 target_position = Vec3::FromPoint(target_pose.position);

        ROS_INFO("Next position is from %f - %f - %f to %f - %f - %f", current_position.x, current_position.y, current_position.z, target_position.x, target_position.y, target_position.z);

        double distance = current_position.Distance(target_position);

        ROS_INFO("Distance is: %lf", distance);
        if (distance > 0.1f && pathPlanner->IsPathClear())
        {
            LOG("||-> Adding task with pose: %f - %f - %f", target_pose.position.x, target_pose.position.y, target_pose.position.z);
            AddTask(new MoveTask(target_pose));
        }
    }
    else
    {
        if (!pathPlanner->IsPathClear())
        {
            LOG("Path is not clear.");
            CurrentTask->Terminate();
        }
    }

    AIBehaviour::Update();
}

void NavigationBehaviour::task_complete_callback(AITaskResult &result)
{
    path = pathPlanner->GeneratePath();
    AIBehaviour::task_complete_callback(result);
}

void NavigationBehaviour::navigation_target_callback(const Pose::ConstPtr &msg)
{
    navigation_target = *msg;
    LOG("||-> Setting navigation target to: %f - %f - %f", navigation_target.position.x, navigation_target.position.y, navigation_target.position.z);
    Pose pose = DRONE->LocalPosition.pose;
    pathPlanner->SetCurrentPose(pose);
    pathPlanner->SetTargetPose(navigation_target);
    LOG("||-< Navigation target set.");

    path = pathPlanner->GeneratePath();
    if (path == NULL)
        LOG("||- No path has found to navigation target.");
    else
        LOG("||- Path found with size of %d", path->poses.size());
}
