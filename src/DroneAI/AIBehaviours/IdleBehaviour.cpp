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

    if (!(DRONE->CurrentState.connected && DRONE->CurrentState.mode == "OFFBOARD" && DRONE->CurrentState.armed))
    {
        if (CurrentTask == NULL)
        {
            AddTask(new InitializationTask());
        }
    }
    else
    {
        if (CurrentTask == NULL)
        {
            AddTask(new IdleTask());
        }
    }
}

void IdleBehaviour::task_complete_callback(AITaskResult &result)
{
    AIBehaviour::task_complete_callback(result);
}
