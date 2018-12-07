#include "DroneAI/AITask.h"
#include "DroneAI/DJIDrone.h"

void TakeOffTask::Start()
{
    LOG("||-> Starting Take Off Task.");
    last_try_time = ros::Time::now();

    if (DRONE->FlightStatus == FlightStatus::InAir)
    {
        LOG("||- Take Off Failed. Drone is already in air!");
        Terminate();
    }
    else
    {
        LOG("||- Requesting to take off.");
        request_takeoff();
    }
}

void TakeOffTask::Update()
{
    if (ros::Time::now() - task_start_time > ros::Duration(10.0))
        Terminate();

    if (DRONE->FlightStatus == FlightStatus::InAir)
    {
        static ros::Time in_air_time = ros::Time::now();

        if (ros::Time::now() - in_air_time > ros::Duration(5.0))
        {
            LOG("Take off succesfull, terminating task");
            task_completed = true;
            Terminate();
        }
    }
}

void TakeOffTask::End()
{
    LOG("||-< Ending Take Off Task.");
}

void TakeOffTask::start_engine()
{
    //TODO: Implement start engine sequence to state machine.
}

bool TakeOffTask::request_takeoff()
{
    last_try_time = ros::Time::now();
    //bool takeoff_request = DRONE->RequestTask(TaskRequest::TakeOff);
}
