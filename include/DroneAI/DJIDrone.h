#ifndef DJIDRONE_H
#define DJIDRONE_H

#include "DroneAI/DroneAI.h"

// System Includes
#include <string>

// ROS Includes
#include <ros/ros.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/UInt8.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/BatteryState.h>

// DJI SDK Includes
#include <dji_sdk/dji_sdk.h>
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/QueryDroneVersion.h>
#include <dji_sdk/SetLocalPosRef.h>

using namespace std;

enum ControlRequest
{
    ReleaseControl  = 0x0,
    TakeControl     = 0x1
};

enum TaskRequest
{
    GoHome          = dji_sdk::DroneTaskControl::Request::TASK_GOHOME,
    TakeOff         = dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF,
    Land            = dji_sdk::DroneTaskControl::Request::TASK_LAND
};

enum FlightStatus
{
    Stopped   = DJISDK::FlightStatus::STATUS_STOPPED,
    OnGround  = DJISDK::FlightStatus::STATUS_ON_GROUND,
    InAir     = DJISDK::FlightStatus::STATUS_IN_AIR
};

enum DisplayMode
{
    ManualControl       = DJISDK::DisplayMode::MODE_MANUAL_CTRL,
    Attitude            = DJISDK::DisplayMode::MODE_ATTITUDE,
    PGPS                = DJISDK::DisplayMode::MODE_P_GPS,
    HotpointMode        = DJISDK::DisplayMode::MODE_HOTPOINT_MODE,
    AssistedTakeoff     = DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF,
    AutoTakeoff         = DJISDK::DisplayMode::MODE_AUTO_TAKEOFF,
    AutoLanding         = DJISDK::DisplayMode::MODE_AUTO_LANDING,
    NaviGoHome          = DJISDK::DisplayMode::MODE_NAVI_GO_HOME,
    NaviSDKControl      = DJISDK::DisplayMode::MODE_NAVI_SDK_CTRL,
    ForceAutoLanding    = DJISDK::DisplayMode::MODE_FORCE_AUTO_LANDING,
    SearchMode          = DJISDK::DisplayMode::MODE_SEARCH_MODE,
    EngineStart         = DJISDK::DisplayMode::MODE_ENGINE_START
};

class DJIDrone
{
  // VARIABLES
  public:
    DJIDrone(ros::NodeHandle& nh);

    geometry_msgs::PointStamped LocalPosition;
    sensor_msgs::NavSatFix GPSPosition;
    sensor_msgs::BatteryState BatteryState;
    geometry_msgs::Quaternion CurrentAttitude;
    uint8_t FlightStatus  = -1;
    uint8_t DisplayMode   = -1;
    uint8_t GPSHealth     = -1;

  private:
    ros::Subscriber attitude_sub;
    ros::Subscriber flightStatus_sub;
    ros::Subscriber displayMode_sub;
    ros::Subscriber localPosition_sub;
    ros::Subscriber gps_sub;
    ros::Subscriber gpsHealth_sub;
    ros::Subscriber battery_state_sub;

    ros::Publisher position_control_pub;

    ros::ServiceClient sdk_ctrl_authority_service;
    ros::ServiceClient drone_task_service;
    ros::ServiceClient query_version_service;
    ros::ServiceClient set_local_pos_reference;

    uint8_t flag;

  // METHODS
  public:
    bool RequestControl(ControlRequest request);
    bool RequestTask(TaskRequest request);
    void Move(double x, double y, double z, double yaw);

  private:
    void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
    {
        CurrentAttitude = msg->quaternion;
    }

    void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
    {
        FlightStatus = msg->data;
    }

    void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg)
    {
        DisplayMode = msg->data;
    }

    void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
    {
        LocalPosition = *msg;
    }

    void gps_position_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
    {
        GPSPosition = *msg;
    }

    void gps_health_callback(const std_msgs::UInt8::ConstPtr& msg)
    {
        GPSHealth = msg->data;
    }

    void battery_state_callback(const sensor_msgs::BatteryState::ConstPtr& msg)
    {
        BatteryState = *msg;
    }
};

#endif // DJIDRONE_H
