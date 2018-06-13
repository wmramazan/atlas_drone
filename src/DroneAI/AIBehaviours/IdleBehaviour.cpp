#include "DroneAI/AIBehaviour.h"
#include "DroneAI/DJIDrone.h"
#include "DroneAI/AITask.h"

IdleBehaviour::IdleBehaviour(NodeHandle& nh)
{
    LOG("||-> Initializing Idle Behaviour.");
    name = "Idle Behaviour";
    LOG("||-< Idle Behaviour Initialization Complete.");
}

void IdleBehaviour::Update()
{
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
