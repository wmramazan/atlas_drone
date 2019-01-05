#include "DroneAI/AITask.h"
#include "DroneAI/DJIDrone.h"
#include "DroneAI/Math.h"

void IdleTask::Start()
{
    LOG("||-> Starting \"%s\" Task.", Name.c_str());
    target_position = DRONE->GetPosition();
    target_yaw = DRONE->GetYaw();
}

void IdleTask::Update()
{
    if (!DRONE->Ready())
    {
        Terminate();
        return;
    }

    double target_distance = (target_position - DRONE->GetPosition()).Magnitude();
    double rotation_diff = target_yaw - DRONE->GetYaw();

    if (target_distance > 0.25 || (abs(rotation_diff) > 1 * DEG2RAD))
    {
        DRONE->MoveTo(target_position, target_yaw);
    }
    else if (rotating)
    {
        target_yaw += 5 * DEG2RAD;
        target_yaw = Math::ClampAngle(target_yaw);
    }
}

void IdleTask::End()
{
    LOG("||-< Ending \"%s\" Task.", Name.c_str());
}
