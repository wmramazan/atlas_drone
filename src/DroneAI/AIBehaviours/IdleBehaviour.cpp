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
    AIBehaviour::Update();

    if (!DRONE->Ready())
    {
        if (CurrentTask == NULL)
        {
            AddTask(new InitializationTask());
        }
    }
    else if (CurrentTask == NULL && DRONE->GetPosition().z > 1.9f)
    {
        AddTask(new IdleTask(true));
    }
}

void IdleBehaviour::task_complete_callback(AITaskResult &result)
{
    AIBehaviour::task_complete_callback(result);
}
