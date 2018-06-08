#include "DroneAI/AIBehaviour.h"
#include "DroneAI/DJIDrone.h"
#include "DroneAI/AITask.h"

CommandedBehaviour::CommandedBehaviour()
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
        ROS_INFO("Updating %s.", CurrentTask->name.c_str());
        CurrentTask->Update();
    }
}