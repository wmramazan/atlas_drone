#include "DroneAI/AITask.h"
#include "DroneAI/DJIDrone.h"

void MoveTask::Start()
{
    LOG("||-> Starting Move Task.");
    last_try_time = ros::Time::now();
}

void MoveTask::Update()
{
    float delta_x = move_target.position.x - DRONE->LocalPosition.point.x;
    float delta_y = move_target.position.y - DRONE->LocalPosition.point.y;
    float delta_z = move_target.position.z;

    if ((std::abs(delta_x) > 0.1) || (std::abs(delta_y) > 0.1) || (std::abs(delta_z - DRONE->LocalPosition.point.z) > 0.1))
    {
        LOG("||- Moving to target destination with offset: %f-%f-%f", delta_x, delta_y, delta_z);
        tf::Quaternion q(move_target.orientation.x, move_target.orientation.y, move_target.orientation.z, move_target.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        DRONE->Move(delta_x, delta_y, delta_z, yaw);
    }
    else
    {
        LOG("||- Move Task completed, terminating task.");
        task_completed = true;
        Terminate();
    }
}

void MoveTask::End()
{
    LOG("||-< Ending Move Task.");
}
