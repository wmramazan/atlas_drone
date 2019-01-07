#ifndef DJIDRONE_H
#define DJIDRONE_H

#include "DroneAI/DroneAI.h"
#include "DroneAI/Math.h"
#include "DroneNavigation/Vec3.h"

// System Includes
#include <string>

// ROS Includes
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>

#include <std_msgs/UInt8.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/BatteryState.h>

#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Altitude.h>
#include <mavros/frame_tf.h>

// DJI SDK Includes
/*
#include <dji_sdk/dji_sdk.h>
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/QueryDroneVersion.h>
#include <dji_sdk/SetLocalPosRef.h>
*/

using namespace std;
using namespace geometry_msgs;
using namespace mavros_msgs;

enum ArmRequest
{
    Arm     = 0x0,
    Disarm  = 0x1
};

enum TaskRequest
{
    GoHome          /* = dji_sdk::DroneTaskControl::Request::TASK_GOHOME*/,
    TakeOff         /* = dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF*/,
    Land            /* = dji_sdk::DroneTaskControl::Request::TASK_LAND*/,
    Offboard
};

enum FlightStatus
{
    Stopped   /* = DJISDK::FlightStatus::STATUS_STOPPED*/,
    OnGround  /* = DJISDK::FlightStatus::STATUS_ON_GROUND*/,
    InAir     /* = DJISDK::FlightStatus::STATUS_IN_AIR*/
};

enum DisplayMode
{
    ManualControl       /* = DJISDK::DisplayMode::MODE_MANUAL_CTRL*/,
    Attitude            /* = DJISDK::DisplayMode::MODE_ATTITUDE*/,
    PGPS                /* = DJISDK::DisplayMode::MODE_P_GPS*/,
    HotpointMode        /* = DJISDK::DisplayMode::MODE_HOTPOINT_MODE*/,
    AssistedTakeoff     /* = DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF*/,
    AutoTakeoff         /* = DJISDK::DisplayMode::MODE_AUTO_TAKEOFF*/,
    AutoLanding         /* = DJISDK::DisplayMode::MODE_AUTO_LANDING*/,
    NaviGoHome          /* = DJISDK::DisplayMode::MODE_NAVI_GO_HOME*/,
    NaviSDKControl      /* = DJISDK::DisplayMode::MODE_NAVI_SDK_CTRL*/,
    ForceAutoLanding    /* = DJISDK::DisplayMode::MODE_FORCE_AUTO_LANDING*/,
    SearchMode          /* = DJISDK::DisplayMode::MODE_SEARCH_MODE*/,
    EngineStart         /* = DJISDK::DisplayMode::MODE_ENGINE_START*/
};

class DJIDrone
{
  // VARIABLES
  public:
    DJIDrone(ros::NodeHandle& nh, Vec3 start_position);

    Altitude CurrentAltitude;
    sensor_msgs::BatteryState BatteryState;

    uint8_t FlightStatus  = -1;
    uint8_t DisplayMode   = -1;
    uint8_t GPSHealth     = -1;

  private:
    ros::Subscriber altitude_sub;
    ros::Subscriber flightStatus_sub;
    ros::Subscriber displayMode_sub;
    ros::Subscriber localPosition_sub;
    ros::Subscriber gps_sub;
    ros::Subscriber gpsHealth_sub;
    ros::Subscriber battery_state_sub;
    ros::Subscriber current_state_sub;

    ros::Publisher position_control_pub;
    ros::Publisher local_pos_pub;

    ros::ServiceClient sdk_ctrl_authority_service;
    ros::ServiceClient drone_task_service;
    ros::ServiceClient query_version_service;
    ros::ServiceClient set_local_pos_reference;
    ros::ServiceClient arming_service;
    ros::ServiceClient set_mode_service;

    PoseStamped local_position;
    Vec3 start_position;
    sensor_msgs::NavSatFix gps_position;
    State current_state;

    PositionTarget target_position;

    uint8_t flag;

  // METHODS
  public:
    bool SetMode(string mode);
    bool RequestArming(ArmRequest request);
    void Move();

    void MoveTo(double x, double y, double z, double yaw);
    void MoveTo(Vec3 position, double yaw);

    void MoveBy(double x, double y, double z, double yaw);
    void MoveBy(Vec3 position, double yaw);

    Vec3 GetPosition();
    Quaternion GetRotation();
    double GetYaw();
    Vec3 GetTargetPosition();
    double GetTargetYaw();
    string GetMode();
    bool IsConnected();
    bool IsArmed();
    bool IsGuided();
    bool Ready();

  private:
    void altitude_callback(const Altitude::ConstPtr& msg)
    {
        CurrentAltitude = *msg;
    }

    void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
    {
        FlightStatus = msg->data;
    }

    void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg)
    {
        DisplayMode = msg->data;
    }

    void local_position_callback(const PoseStamped::ConstPtr& msg)
    {
        local_position = *msg;
        local_position.pose.position.x += start_position.x;
        local_position.pose.position.y += start_position.y;
        local_position.pose.position.z += start_position.z;
    }

    void gps_position_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
    {
        gps_position = *msg;
    }

    void gps_health_callback(const std_msgs::UInt8::ConstPtr& msg)
    {
        GPSHealth = msg->data;
    }

    void battery_state_callback(const sensor_msgs::BatteryState::ConstPtr& msg)
    {
        BatteryState = *msg;
    }

    void current_state_callback(const State::ConstPtr& msg)
    {
        current_state = *msg;
    }
};

#endif // DJIDRONE_H
