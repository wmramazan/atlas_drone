#include "DroneAI/AIBehaviour.h"
#include "DroneAI/DJIDrone.h"
#include "DroneAI/AITask.h"

IdleBehaviour::IdleBehaviour()
{
    LOG("||-> Initializing Idle Behaviour.");
    name = "Idle Behaviour";
/*
    AITakeOffTask* takeOffTask = new AITakeOffTask(std::bind(&AIIdleBehaviour::taskCompleteCallback, this, std::placeholders::_1));
    taskQueue.push(takeOffTask);

    AIMoveTarget move_target;
    move_target.x = 15;
    move_target.y = 0;
    move_target.z = 5;
    move_target.yaw = 0;
    AIMoveTask* moveTask = new AIMoveTask(std::bind(&AIIdleBehaviour::taskCompleteCallback, this, std::placeholders::_1), move_target);
    taskQueue.push(moveTask);

    AIHoverTask hoverTask = AIHoverTask(&taskCompleteCallback);
    taskQueue.push(hoverTask);

    AILandTask landTask = AILandTask(&taskCompleteCallback);
    taskQueue.push(landTask);

    next_task();
    */
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
