#include "DroneAI/AIBehaviour.h"
#include "DroneAI/DJIDrone.h"
#include "DroneAI/AITask.h"

NavigationBehaviour::NavigationBehaviour()
{
    LOG("||-> Initializing Navigation Behaviour.");
    name = "Navigation Behaviour";
	this->nh = &nh;
    LOG("||-< Navigation Behaviour Initialization Complete.");
}

void NavigationBehaviour::Update()
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
