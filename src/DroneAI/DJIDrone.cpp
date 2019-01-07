#include "DroneAI/DJIDrone.h"
#include "DroneAI/DroneAI.h"

DJIDrone::DJIDrone(ros::NodeHandle& nh, Vec3 start_position)
{
    altitude_sub          = nh.subscribe("mavros/altitude",            10, &DJIDrone::altitude_callback,         this);
    localPosition_sub     = nh.subscribe("mavros/local_position/pose", 10, &DJIDrone::local_position_callback,   this);
    battery_state_sub     = nh.subscribe("mavros/battery",             10, &DJIDrone::battery_state_callback,    this);
    current_state_sub     = nh.subscribe("mavros/state",               10, &DJIDrone::current_state_callback,    this);

    arming_service          = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_service        = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    local_pos_pub           = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);

    target_position.type_mask = (PositionTarget::IGNORE_VX |
                                 PositionTarget::IGNORE_VY |
                                 PositionTarget::IGNORE_VZ |
                                 PositionTarget::IGNORE_AFX |
                                 PositionTarget::IGNORE_AFY |
                                 PositionTarget::IGNORE_AFZ |
                                 PositionTarget::IGNORE_YAW_RATE);

    target_position.coordinate_frame = PositionTarget::FRAME_LOCAL_NED;

    this->start_position = start_position;
}

bool DJIDrone::SetMode(string mode)
{
    LOG("|||-> Requesting to set mode \"%s\".", mode.c_str());

    mavros_msgs::SetMode offboard_set_mode;
    offboard_set_mode.request.custom_mode = mode.c_str();
    set_mode_service.call(offboard_set_mode);

    if (offboard_set_mode.response.mode_sent)
    {
        LOG("|||-< \"%s\" Mode set request successful!", mode.c_str());
        return true;
    }
    else
    {
        LOG("|||-< \"%s\" Mode set request failed!", mode.c_str());
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
        
    LOG("|||-> Requesting to \"%s\".", task.c_str());

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


void DJIDrone::Move()
{
    local_pos_pub.publish(target_position);
}

void DJIDrone::MoveTo(double x, double y, double z, double yaw)
{
    target_position.position.x = x - start_position.x;
    target_position.position.y = y - start_position.y;
    target_position.position.z = z - start_position.z;
    target_position.yaw = static_cast<float>(Math::ClampAngle(yaw));
}

void DJIDrone::MoveTo(Vec3 position, double yaw)
{
    target_position.position.x = position.x - start_position.x;
    target_position.position.y = position.y - start_position.y;
    target_position.position.z = position.z - start_position.z;
    target_position.yaw = static_cast<float>(Math::ClampAngle(yaw));
}

void DJIDrone::MoveBy(double x, double y, double z, double yaw)
{
    MoveTo(GetPosition().x + x,
           GetPosition().y + y,
           GetPosition().z + z,
           GetYaw() + yaw);
}

void DJIDrone::MoveBy(Vec3 position, double yaw)
{
    MoveTo(GetPosition() + position, GetYaw() + yaw);
}

Vec3 DJIDrone::GetPosition()
{
    return Vec3(local_position.pose.position.x, local_position.pose.position.y, local_position.pose.position.z);
}

geometry_msgs::Quaternion DJIDrone::GetRotation()
{
    return local_position.pose.orientation;
}

double DJIDrone::GetYaw()
{
    return Math::GetYaw(GetRotation());
}

Vec3 DJIDrone::GetTargetPosition()
{
    return Vec3(target_position.position.x, target_position.position.y, target_position.position.z);
}

double DJIDrone::GetTargetYaw()
{
    return static_cast<double>(target_position.yaw);
}

string DJIDrone::GetMode()
{
    return current_state.mode;
}

bool DJIDrone::IsConnected()
{
    return current_state.connected;
}

bool DJIDrone::IsArmed()
{
    return current_state.armed;
}

bool DJIDrone::IsGuided()
{
    return current_state.guided;
}

bool DJIDrone::Ready()
{
    return current_state.connected && current_state.mode == "OFFBOARD" && current_state.armed;
}


