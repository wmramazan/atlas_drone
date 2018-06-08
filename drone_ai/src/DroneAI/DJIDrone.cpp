#include "DroneAI/DJIDrone.h"
#include "DroneAI/DroneAI.h"

DJIDrone::DJIDrone(ros::NodeHandle& nh)
{
    attitude_sub          = nh.subscribe("dji_sdk/attitude",          10, &DJIDrone::attitude_callback,         this);
    flightStatus_sub      = nh.subscribe("dji_sdk/flight_status",     10, &DJIDrone::flight_status_callback,    this);
    displayMode_sub       = nh.subscribe("dji_sdk/display_mode",      10, &DJIDrone::display_mode_callback,     this);
    localPosition_sub     = nh.subscribe("dji_sdk/local_position",    10, &DJIDrone::local_position_callback,   this);
    gps_sub               = nh.subscribe("dji_sdk/gps_position",      10, &DJIDrone::gps_position_callback,     this);
    gpsHealth_sub         = nh.subscribe("dji_sdk/gps_health",        10, &DJIDrone::gps_health_callback,       this);
    battery_state_sub      = nh.subscribe("dji_sdk/battery_state",     10, &DJIDrone::battery_state_callback,    this);


    sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
    drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>    ("dji_sdk/drone_task_control");
    query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>   ("dji_sdk/query_drone_version");
    set_local_pos_reference    = nh.serviceClient<dji_sdk::SetLocalPosRef>      ("dji_sdk/set_local_pos_ref");

    position_control_pub = nh.advertise<sensor_msgs::Joy>("/dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);

    dji_sdk::SetLocalPosRef localPosReferenceSetter;
    set_local_pos_reference.call(localPosReferenceSetter);
}

bool DJIDrone::RequestControl(ControlRequest request)
{
    if (request == ControlRequest::TakeControl)
        LOG("|||-> Attempting to take control.");
    else
        LOG("|||-> Attempting to release control.");

    dji_sdk::SDKControlAuthority authority;
    authority.request.control_enable = request;
    sdk_ctrl_authority_service.call(authority);

    if(authority.response.result)
    {
        LOG("|||-< Request successful!");
        return true;
    }
    else
    {
        LOG("|||-< Request failed!");
        return false;
    }
}

bool DJIDrone::RequestTask(TaskRequest request)
{
    string task;

    if (request == TaskRequest::TakeOff)
        task = "Take Off";
    else if (request == TaskRequest::Land)
        task = "Land";
    else if (request == TaskRequest::GoHome)
        task = "Go Home";

    LOG("|||-> Requesting to %s!", task.c_str());

    dji_sdk::DroneTaskControl droneTaskControl;
    droneTaskControl.request.task = request;
    drone_task_service.call(droneTaskControl);

    if (droneTaskControl.response.result)
    {
        LOG("|||-< %s request succesful!", task.c_str());
        return true;
    }
    else
    {
        LOG("|||-< %s request faield!", task.c_str());
        return false;
    }
}

void DJIDrone::Move(double x, double y, double z, double yaw)
{
    sensor_msgs::Joy controlPosYaw;
    controlPosYaw.axes.push_back(x);
    controlPosYaw.axes.push_back(y);
    controlPosYaw.axes.push_back(z);
    controlPosYaw.axes.push_back(yaw);
    position_control_pub.publish(controlPosYaw);
}
