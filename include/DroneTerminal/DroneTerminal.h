#ifndef DRONE_TERMINAL_H
#define DRONE_TERMINAL_H

// System Includes
#include <iostream>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <vector>
#include <ctime>

// ROS Includes
#include <ros/ros.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/BatteryState.h>

// DJISDK Includes
//#include <dji_sdk/dji_sdk.h>
//#include <dji_sdk/SetLocalPosRef.h>

#include <atlas_drone/AIState.h>
#include <ncurses.h>

#define DISPLAY_SIZE 10

using namespace std;

class DisplayWindow;
class MessageWindow;
class InputWindow;

class DroneTerminal
{
  public:
    DroneTerminal();

    DisplayWindow* displayWindow;
    MessageWindow* messageWindow;
    InputWindow* inputWindow;
  private:
    void initialize_node();
    void initialize_terminal();
    void draw();

    ros::Subscriber attitude_sub;
    ros::Subscriber flight_status_sub;
    ros::Subscriber display_mode_sub;
    ros::Subscriber local_position_sub;
    ros::Subscriber gps_sub;
    ros::Subscriber gps_health_sub;
    ros::Subscriber battery_state_sub;
    ros::Subscriber drone_message_sub;
    ros::Subscriber ai_state_sub;

    ros::Publisher terminal_message_pub;

    ros::ServiceClient set_local_pos_reference;
};

class DisplayWindow
{
    public:
        DisplayWindow(DroneTerminal* terminal);
        void Draw();

        void FlightStatusCallback(const std_msgs::UInt8::ConstPtr& msg);
        void DisplayModeCallback(const std_msgs::UInt8::ConstPtr& msg);
        void BatteryStateCallback(const sensor_msgs::BatteryState::ConstPtr& msg);
        void LocalPositionCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
        void AttitudeCallback(const geometry_msgs::QuaternionStamped::ConstPtr& msg);
        void GPSPositionCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
        void GPSHealthCallback(const std_msgs::UInt8::ConstPtr& msg);
        void AIStateCallback(const atlas_drone::AIState::ConstPtr& msg);

    private:
        DroneTerminal* terminal;
        WINDOW* window;
        bool dirty;

        geometry_msgs::PointStamped local_position;
        geometry_msgs::Quaternion current_attitude;
        sensor_msgs::NavSatFix gps_position;
        sensor_msgs::BatteryState battery_state;
        int flight_status  = -1;
        int display_mode   = -1;
        int gps_health     = -1;
        string current_behaviour;
        string current_task;

        const string flight_status_error = "Flight Status Error";
        string flight_status_str[3]
        {
          "Stopped",                // 0
          "On Ground",              // 1
          "In Air"                  // 2
        };

        const string display_mode_error = "Display Mode Error";
        string display_mode_str[43]
        {
          "Manual Control",         // 0
          "Attitude",               // 1
          "Reserved",               // 2
          "Reserved",               // 3
          "Reserved",               // 4
          "Reserved",               // 5
          "P GPS",                  // 6
          "Reserved",               // 7
          "Reserved",               // 8
          "Hotpoint",               // 9
          "Assisted Takeoff",       // 10
          "Auto Takeoff",           // 11
          "Auto Landing",           // 12
          "Reserved",               // 13
          "Reserved",               // 14
          "Navigated Go Home",      // 15
          "Reserved",               // 16
          "Navigated SDK Control",  // 17
          "Reserved",               // 18
          "Reserved",               // 19
          "Reserved",               // 20
          "Reserved",               // 21
          "Reserved",               // 22
          "Reserved",               // 23
          "Reserved",               // 24
          "Reserved",               // 25
          "Reserved",               // 26
          "Reserved",               // 27
          "Reserved",               // 28
          "Reserved",               // 29
          "Reserved",               // 30
          "Reserved",               // 31
          "Reserved",               // 32
          "Force Auto Landing",     // 33
          "Reserved",               // 34
          "Reserved",               // 35
          "Reserved",               // 36
          "Reserved",               // 37
          "Reserved",               // 38
          "Reserved",               // 39
          "Search Mode",            // 40
          "Engine Start Mode",      // 41
          "Reserved"                // 42
        };
};

class MessageWindow
{
    public:
        MessageWindow(DroneTerminal* terminal);
        void Draw();
        void Print(bool, const char*);
        void ScrollUp();
        void ScrollDown();
        void Clear();

        void DroneMessageCallback(const std_msgs::String::ConstPtr& msg);

    private:
        DroneTerminal* terminal;
        WINDOW* window;
        bool dirty;

        vector<string> buffer;
        int size;
        int line;
};

class InputWindow
{
    public:
        InputWindow(DroneTerminal* terminal);
        void Draw();
        char* GetInput();

    private:
        DroneTerminal* terminal;
        WINDOW* window;
        bool dirty;

        char* buffer;
        char* last_input;
        int length;
        bool end;
};

#endif // DRONE_TERMINAL_H
