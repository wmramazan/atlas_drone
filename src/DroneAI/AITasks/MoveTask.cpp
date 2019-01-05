#include "DroneAI/AITask.h"
#include "DroneAI/DJIDrone.h"

void MoveTask::Start()
{
    LOG("||-> Starting Move Task.");
    last_try_time = ros::Time::now();
}

void MoveTask::Update()
{
    if (!DRONE->Ready())
    {
        Terminate();
        return;
    }

    float target_distance = (target_position - DRONE->GetPosition()).Magnitude();
    double rotation_diff = target_yaw - DRONE->GetYaw();

    if (target_distance > 0.25f || (std::abs(rotation_diff) > 5 * DEG2RAD))
    {
        DRONE->MoveTo(target_position, target_yaw);
    }
    else
    {
        if (!time_set)
        {
            last_try_time = ros::Time::now();
            time_set = true;
        }

        if (time_set && ros::Time::now() - last_try_time > ros::Duration(3.0))
        {
            LOG("||- Move Task completed, terminating task.");
            task_completed = true;
            Terminate();
        }
    }
}

void MoveTask::End()
{
    LOG("||-< Ending Move Task.");
}
