#include "DroneAI/AITask.h"
#include "DroneAI/DJIDrone.h"

void InitializationTask::Start()
{
    LOG("||-> Starting \"%s\" Task.", Name.c_str());
    task_state = INITIAL_STATE;
    last_try_time = ros::Time::now();

    geometry_msgs::PoseStamped pose;

}

void InitializationTask::Update()
{
    if (ros::Time::now() - task_start_time > ros::Duration(30.0))
        Terminate();

    if (i < 100)
    {
        DRONE->Move(0, 0, 2, 0);
        i++;
    }
    else
    {
        if (!DRONE->CurrentState.connected)
        {
            LOG("||-> PX4 not connected, returning from initialization task.");
            return;
        }

        if (DRONE->CurrentState.mode != "OFFBOARD" && ros::Time::now() - last_try_time > ros::Duration(2.0))
        {
            LOG("||-> Drone mode is not OFFBOARD");
            set_mode();
        }

        if (DRONE->CurrentState.mode == "OFFBOARD" && !DRONE->CurrentState.armed && ros::Time::now() - last_try_time > ros::Duration(2.0))
        {
            LOG("||-> Drone is not armed");
            arm();
        }

        if (DRONE->CurrentState.connected && DRONE->CurrentState.mode == "OFFBOARD" && DRONE->CurrentState.armed)
        {
            DRONE->Move(0, 0, 2, 0);
            task_completed = true;
            Terminate();
            return;
        }

        DRONE->Move(DRONE->LocalPosition.pose.position.x, DRONE->LocalPosition.pose.position.y, DRONE->LocalPosition.pose.position.z, 0);
    }

    /*
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
    }*/
}

void InitializationTask::End()
{
    LOG("||-< Ending \"%s\" Task.", Name.c_str());
}

void InitializationTask::set_mode()
{
    last_try_time = ros::Time::now();
    bool mode_set = DRONE->SetMode("OFFBOARD");
    /*if (mode_set)
    {
        task_state = ARMING;
        LOG("||-> Entering \"Arming Mode\" State.");
    }*/
}

void InitializationTask::arm()
{
    last_try_time = ros::Time::now();
    bool armed = DRONE->RequestArming(ArmRequest::Arm);
    /*if (task_state)
    {
        task_state = INITIALIZED;
        LOG("||-> Entering \"Initialized Mode\" State.");
    }*/
}
