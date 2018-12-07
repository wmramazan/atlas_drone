#include "DroneAI/DJIDrone.h"
#include "DroneAI/DroneAI.h"

DJIDrone::DJIDrone(ros::NodeHandle& nh)
{
    string position_topic;
    nh.param<string>("/position_topic", position_topic, "/drone_position");

    altitude_sub          = nh.subscribe("/navros/altitude",          10, &DJIDrone::altitude_callback,         this);
    //flightStatus_sub      = nh.subscribe("dji_sdk/flight_status",       10, &DJIDrone::flight_status_callback,    this);
    //displayMode_sub       = nh.subscribe("dji_sdk/display_mode",        10, &DJIDrone::display_mode_callback,     this);
    localPosition_sub     = nh.subscribe("/mavros/local_position/pose", 10, &DJIDrone::local_position_callback,   this);
    //gps_sub               = nh.subscribe("dji_sdk/gps_position",        10, &DJIDrone::gps_position_callback,     this);
    //gpsHealth_sub         = nh.subscribe("dji_sdk/gps_health",          10, &DJIDrone::gps_health_callback,       this);
    battery_state_sub     = nh.subscribe("/mavros/battery",             10, &DJIDrone::battery_state_callback,    this);
    current_state_sub     = nh.subscribe("/mavros/state",               10, &DJIDrone::current_state_callback,    this);

    arming_service          = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_service        = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    position_control_pub    = nh.advertise<sensor_msgs::Joy>("/dji_sdk/flight_control_setpoint_generic", 10);
    local_pos_pub           = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
}

bool DJIDrone::SetMode(string mode)
{
    LOG("|||-> Attempting to set mode \"%s\".", mode.c_str());

    mavros_msgs::SetMode offboard_set_mode;
    offboard_set_mode.request.custom_mode = mode.c_str();
    set_mode_service.call(offboard_set_mode);

    if (offboard_set_mode.response.mode_sent)
    {
        LOG("|||-< \"%s\" Mode set successful!", mode.c_str());
        return true;
    }
    else
    {
        LOG("|||-< \"%s\" Mode set failed!", mode.c_str());
        return false;
    }
}

bool DJIDrone::RequestArming(ArmRequest request)
{
    string task;
    bool request_value;

    if (request == ArmRequest::Arm)
    {
        task = "Arm";
        request_value = true;
    }
    else
    {
        task = "Disarm";
        request_value = false;
    }
        
    LOG("|||-> Attempting to \"%s\".", task.c_str());
    mavros_msgs::CommandBool arm_command;
    arm_command.request.value = request_value;
    arming_service.call(arm_command);

    if (arm_command.response.result)
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

void DJIDrone::Move(double x, double y, double z, double yaw)
{
    SetMode("OFFBOARD");

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;

    local_pos_pub.publish(pose);
}
