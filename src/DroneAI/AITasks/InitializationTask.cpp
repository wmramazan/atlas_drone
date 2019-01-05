#include "DroneAI/AITask.h"
#include "DroneAI/DJIDrone.h"
#include "DroneAI/Math.h"

void InitializationTask::Start()
{
    LOG("||-> Starting \"%s\" Task.", Name.c_str());
    last_try_time = ros::Time::now();
}

void InitializationTask::Update()
{
    if (ros::Time::now() - task_start_time > ros::Duration(30.0))
    {
        LOG("||-> Initialization failed in 30 seconds.");
        Terminate();
        return;
    }

    if (!DRONE->IsConnected())
    {
        LOG("||-> PX4 not connected, returning from initialization task.");
        return;
    }

    if (DRONE->GetMode() != "OFFBOARD" && ros::Time::now() - last_try_time > ros::Duration(1.0))
    {
        LOG("||-> Drone mode is not OFFBOARD");
        if (!set_mode())
        {
            LOG("||-> Drone mode failed to set OFFBOARD, trying again in 2 seconds.");
            return;
        }
    }

    if (DRONE->GetMode() == "OFFBOARD" && !DRONE->IsArmed() && ros::Time::now() - last_try_time > ros::Duration(1.0))
    {
        LOG("||-> Drone is not armed");
        if (!arm())
        {
            LOG("||-> Drone failed to arm, trying again in 2 seconds.");
            return;
        }
    }

    if (DRONE->Ready())
    {
        DRONE->MoveTo(0, 0, 2, 0);

        if (DRONE->GetPosition().z > 1.9)
        {
            if (!air_time_set)
            {
                in_air_time = ros::Time::now();
                air_time_set = true;
            }
        }
        else
        {
            air_time_set = false;
        }


        if (air_time_set && ros::Time::now() - in_air_time > ros::Duration(2.0))
        {
            LOG("||-> Initialization succesfull.");
            task_completed = true;
            Terminate();
            return;
        }
    }
}

void InitializationTask::End()
{
    LOG("||-< Ending \"%s\" Task.", Name.c_str());
}

bool InitializationTask::set_mode()
{
    last_try_time = ros::Time::now();
    bool mode_set = DRONE->SetMode("OFFBOARD");
    return mode_set && DRONE->GetMode() == "OFFBOARD";
}

bool InitializationTask::arm()
{
    last_try_time = ros::Time::now();
    bool armed = DRONE->RequestArming(ArmRequest::Arm);
    return armed && DRONE->IsArmed();
}
