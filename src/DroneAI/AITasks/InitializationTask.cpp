#include "DroneAI/AITask.h"
#include "DroneAI/DJIDrone.h"

void InitializationTask::Start()
{
    LOG("||-> Starting \"%s\" Task.", Name.c_str());
    task_state = INITIAL_STATE;
    last_try_time = ros::Time::now();
}

void InitializationTask::Update()
{
    if (ros::Time::now() - task_start_time > ros::Duration(30.0))
        Terminate();

    switch (task_state)
    {
        case INITIAL_STATE:
            ROS_INFO("initial state");
            if (DRONE->CurrentState.connected)
            {
                task_state = SETTING_MODE;
                LOG("||-> Entering \"Setting Mode\" State.");
            }
            break;
        case SETTING_MODE:
            ROS_INFO("setting mode");
            if (ros::Time::now() - last_try_time > ros::Duration(5.0))
                set_mode();
            else
                DRONE->Move(0.0f, 0.0f, 2.0f, 0.0f);
            break;
        case ARMING:
            ROS_INFO("arming mode");

            if (ros::Time::now() - last_try_time > ros::Duration(5.0))
                arm();
            break;
        case INITIALIZED:
            ROS_INFO("initialized mode");

            task_completed = true;
            Terminate();
            break;
    }
}

void InitializationTask::End()
{
    LOG("||-< Ending \"%s\" Task.", Name.c_str());
}

void InitializationTask::set_mode()
{
    if (DRONE->CurrentState.mode == "OFFBOARD")
    { 
        task_state = ARMING;
        return;
    }

    last_try_time = ros::Time::now();
    bool mode_set = DRONE->SetMode("OFFBOARD");
    if (mode_set)
    {
        task_state = ARMING;
        LOG("||-> Entering \"Arming Mode\" State.");
    }
}

void InitializationTask::arm()
{
    if (DRONE->CurrentState.mode != "OFFBOARD")
    {
        task_state = INITIAL_STATE;
        return;
    }

    if (DRONE->CurrentState.armed)
    { 
        task_state = INITIALIZED;
        return;
    }

    last_try_time = ros::Time::now();
    bool armed = DRONE->RequestArming(ArmRequest::Arm);
    if (task_state)
    {
        task_state = INITIALIZED;
        LOG("||-> Entering \"Initialized Mode\" State.");
    }
}
