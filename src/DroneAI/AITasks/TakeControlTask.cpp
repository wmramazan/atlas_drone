#include "DroneAI/AITask.h"
#include "DroneAI/DJIDrone.h"

void TakeControlTask::Start()
{
    LOG("||-> Starting Take Control Task.");
    control_state = NO_CONTROL;
    last_try_time = ros::Time::now();
}

void TakeControlTask::Update()
{
    switch (control_state)
    {
        case NO_CONTROL:
            take_control();
            break;
        case WAITING_TO_TRY_AGAIN:
            if (ros::Time::now() - task_start_time > ros::Duration(10.0))
              Terminate();
            else if (ros::Time::now() - last_try_time > ros::Duration(5.0))
              control_state = NO_CONTROL;
            break;
        case HAVE_CONTROL:
            task_completed = true;
            Terminate();
            break;
    }
}

void TakeControlTask::End()
{
    LOG("||-< Ending Take Control Task.");
}

void TakeControlTask::take_control()
{
    last_try_time = ros::Time::now();
    bool control_taken = DRONE->RequestControl(ControlRequest::TakeControl);
    control_state = control_taken ? HAVE_CONTROL : WAITING_TO_TRY_AGAIN;
}
