#include "DroneAI/AITask.h"
#include "DroneAI/DJIDrone.h"

void MoveTask::Start()
{
    LOG("||-> Starting Move Task.");
    last_try_time = ros::Time::now();
}

void MoveTask::Update()
{
    Vec3 target_distance = Vec3(move_target.position.x - DRONE->LocalPosition.point.x,
                                move_target.position.y - DRONE->LocalPosition.point.y,
                                move_target.position.z);

    if ((std::abs(target_distance.x) > 0.25) || (std::abs(target_distance.y) > 0.25) || (std::abs(target_distance.z - DRONE->LocalPosition.point.z) > 0.25))
    {
        LOG("||- Moving to target destination with offset: %f-%f-%f", target_distance.x, target_distance.y, target_distance.z);
        tf::Quaternion q(move_target.orientation.x, move_target.orientation.y, move_target.orientation.z, move_target.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        Vec3 move_vector = target_distance.Normalized() * 0.5f;
        //DRONE->Move(move_vector.x, move_vector.y, move_target.position.z, 0);
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
