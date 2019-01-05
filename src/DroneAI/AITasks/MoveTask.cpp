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

    double target_position_distance = (target_position - DRONE->GetPosition()).Magnitude();
    double target_rotation_distance =  (target_rotation - DRONE->GetYaw());

    if (target_position_distance > 0.1 || (abs(target_rotation_distance) > 5 * DEG2RAD))
    {
        if (!moving_step)
        {
            Vec3 target_direction = (target_position - DRONE->GetPosition()).Normalized();
            step_position = (DRONE->GetPosition() + target_direction * min(move_speed, target_position_distance));
            step_rotation = target_rotation;

            DRONE->MoveTo(step_position, step_rotation);
            moving_step = true;
        }
        else
        {
            double step_position_distance = (step_position - DRONE->GetPosition()).Magnitude();
            double step_rotation_distance = step_rotation - DRONE->GetYaw();

            if (step_position_distance < move_speed * .25 && abs(step_rotation_distance) < 5 * DEG2RAD)
            {
                moving_step = false;
            }
        }
    }
    else
    {
        if (!time_set)
        {
            last_try_time = Time::now();
            time_set = true;
        }

        if (time_set && Time::now() - last_try_time > Duration(0.0))
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
