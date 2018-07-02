#include "DroneAI/AIBehaviour.h"
#include "DroneAI/DJIDrone.h"
#include "DroneAI/AITask.h"

CommandedBehaviour::CommandedBehaviour(NodeHandle& nh)
{
    LOG("||-> Initializing Commanded Behaviour.");
    name = "Commanded Behaviour";
    LOG("||-< Commanded Behaviour Initialization Complete.");
}

void CommandedBehaviour::Update()
{
    if (CurrentTask == NULL)
    {
        ROS_INFO("Null task exception.");
        next_task();
    }
    else
    {
        ROS_INFO("Updating %s.", CurrentTask->Name.c_str());
        CurrentTask->Update();
    }
}

void CommandedBehaviour::task_complete_callback(AITaskResult &result)
{
    AIBehaviour::task_complete_callback(result);
}
