#include "DroneAI/AITask.h"
#include "DroneAI/DJIDrone.h"

void TakeOffTask::Start()
{
    LOG("||-> Starting Take Off Task.");
    //control_state = DJIDrone::Instance->FlightStatus;
    last_try_time = ros::Time::now();

    if (DRONE->FlightStatus == FlightStatus::InAir)
    {
        LOG("||- Take Off Failed. Drone is already in air!");
        terminate_task(false);
    }
    else
    {
        LOG("||- Requesting to take off.");
        request_takeoff();
    }
}

void TakeOffTask::Update()
{
    //ROS_INFO("---> Updating Take Off Task.");

    if (ros::Time::now() - task_start_time > ros::Duration(10.0))
        terminate_task(false);

    if (DRONE->FlightStatus == FlightStatus::InAir)
    {
        static ros::Time in_air_time = ros::Time::now();

        if (ros::Time::now() - in_air_time > ros::Duration(5.0))
        {
            LOG("Take off succesfull, terminating task");
            terminate_task(true);
            /*if (!moved)
            {
                ROS_INFO("-----> Moving.");
                //moved = true;
                DJIDrone::Instance->Move(1, 5, 0, 0);
            }
            else
            {
                ROS_INFO("<---- Terminating");
                terminate_task(true);
            }*/
        }
    }
    /*switch (control_state)
    {
        case NO_CONTROL:
          take_control();
          break;
        case WAITING_TO_TRY_AGAIN:
          if (ros::Time::now() - last_try_time > ros::Duration(5.0))
            control_state = NO_CONTROL;
          break;
        case HAVE_CONTROL:
          terminate_task();
          break;
    }*/
}

void TakeOffTask::End()
{
    LOG("||-< Ending Take Off Task.");
}

void TakeOffTask::start_engine()
{

}

bool TakeOffTask::request_takeoff()
{
    last_try_time = ros::Time::now();
    bool takeoff_request = DRONE->RequestTask(TaskRequest::TakeOff);

    /*if (takeoff_request)
    {
        ros::Duration(0.1).sleep();
        ros::spinOnce();


        // Step 1.1: Spin the motor
        while (DJIDrone::Instance->FlightStatus != FlightStatus::OnGround &&
               DJIDrone::Instance->DisplayMode != DisplayMode::EngineStart &&
               ros::Time::now() - start_time < ros::Duration(5))
        {
            ros::Duration(0.01).sleep();
            ros::spinOnce();
        }

        if(ros::Time::now() - start_time > ros::Duration(5))
        {
            ROS_ERROR("Takeoff failed. Motors are not spinnning.");
            return false;
        }
        else
        {
            start_time = ros::Time::now();
            ROS_INFO("Motor Spinning ...");
            ros::spinOnce();
        }


        // Step 1.2: Get in to the air
        while (DJIDrone::Instance->FlightStatus != FlightStatus::InAir &&
              (DJIDrone::Instance->DisplayMode != DisplayMode::AssistedTakeoff ||
               DJIDrone::Instance->DisplayMode != DisplayMode::AutoTakeoff) &&
               ros::Time::now() - start_time < ros::Duration(20))
        {
            ros::Duration(0.01).sleep();
            ros::spinOnce();
        }

        if(ros::Time::now() - start_time > ros::Duration(20))
        {
            ROS_ERROR("Takeoff failed. Aircraft is still on the ground, but the motors are spinning.");
            return false;
        }
        else
        {
            start_time = ros::Time::now();
            ROS_INFO("Ascending...");
            ros::spinOnce();
        }

        // Final check: Finished takeoff
        while ((DJIDrone::Instance->DisplayMode == DisplayMode::AssistedTakeoff ||
                DJIDrone::Instance->DisplayMode == DisplayMode::AutoTakeoff) &&
                ros::Time::now() - start_time < ros::Duration(20))
        {
            ros::Duration(0.01).sleep();
            ros::spinOnce();
        }

        if (DJIDrone::Instance->DisplayMode != DisplayMode::PGPS ||
            DJIDrone::Instance->DisplayMode != DisplayMode::Attitude)
        {
            ROS_INFO("Successful takeoff!");
            start_time = ros::Time::now();
        }
        else
        {
            ROS_ERROR("Takeoff finished, but the aircraft is in an unexpected mode. Please connect DJI GO.");
            return false;
        }
    }*/
}
