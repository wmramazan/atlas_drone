#include "DroneAI/AITask.h"
#include "DroneAI/DJIDrone.h"

void MoveTask::Start()
{
    LOG("||-> Starting Move Task.");
    last_try_time = ros::Time::now();
}

void MoveTask::Update()
{
    float delta_x = move_target.x - DRONE->LocalPosition.point.x;
    float delta_y = move_target.y - DRONE->LocalPosition.point.y;
    float delta_z = move_target.z;

    if ((std::abs(delta_x) > 0.1) || (std::abs(delta_y) > 0.1) || (std::abs(delta_z - DRONE->LocalPosition.point.z) > 0.1))
    {
        LOG("||- Moving to target destination with offset: %f-%f-%f", delta_x, delta_y, delta_z);
        DRONE->Move(delta_x, delta_y, delta_z, move_target.yaw);
    }
    else
    {
        LOG("||- Move Task completed, terminating task.");
        terminate_task(true);
    }
}

void MoveTask::End()
{
    LOG("||-< Ending Move Task.");
}
