#include "DroneAI/AITask.h"
#include "DroneAI/DJIDrone.h"

void IdleTask::Start()
{
    LOG("||-> Starting \"%s\" Task.", Name.c_str());
}

void IdleTask::Update()
{
    if (DRONE->CurrentState.connected && DRONE->CurrentState.mode == "OFFBOARD" && DRONE->CurrentState.armed)
    {
        if (DRONE->LocalPosition.pose.position.z < 1.9)
        {
            DRONE->Move(DRONE->LocalPosition.pose.position.x, DRONE->LocalPosition.pose.position.y, 2, 0);
        }
        else
        {
            DRONE->Move(DRONE->LocalPosition.pose.position.x, DRONE->LocalPosition.pose.position.y, DRONE->LocalPosition.pose.position.z, 0);
        }

    }
    else
    {

    }
}

void IdleTask::End()
{
    LOG("||-< Ending \"%s\" Task.", Name.c_str());
}
