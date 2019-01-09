#include "DroneAI/AIBehaviour.h"
#include "DroneAI/DJIDrone.h"
#include "DroneAI/AITask.h"

IdleBehaviour::IdleBehaviour(NodeHandle& nh)
{
    LOG("||-> Initializing Idle Behaviour.");
    name = "Idle Behaviour";
    LOG("||-< Idle Behaviour Initialization Complete.");
}

void IdleBehaviour::OnEnter()
{

}

void IdleBehaviour::Update()
{
    AIBehaviour::Update();

    if (!DRONE->Ready())
    {
        if (CurrentTask == nullptr)
        {
            AddTask(new InitializationTask());
        }
    }
    else if (CurrentTask == nullptr && DRONE->GetPosition().z > 1.9)
    {
        AddTask(new IdleTask(true));
    }
}

void IdleBehaviour::OnExit()
{

}

void IdleBehaviour::task_complete_callback(AITaskResult &result)
{
    AIBehaviour::task_complete_callback(result);
}
