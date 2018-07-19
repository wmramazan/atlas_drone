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
    AIBehaviour::Update();
}

void CommandedBehaviour::task_complete_callback(AITaskResult &result)
{
    AIBehaviour::task_complete_callback(result);
}
