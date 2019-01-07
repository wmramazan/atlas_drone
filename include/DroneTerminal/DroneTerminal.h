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
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <sensor_msgs/BatteryState.h>

#include <mavros_msgs/State.h>
#include <mavros_msgs/Altitude.h>
#include <mavros_msgs/PositionTarget.h>

#include <atlas_drone/AIState.h>
#include <DroneNavigation/Vec3.h>
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

    Vec3 start_position;
    int id = 0;
  private:
    void initialize_node();
    void initialize_terminal();
    void draw();

    ros::Subscriber altitude_sub;
    ros::Subscriber local_position_sub;
    ros::Subscriber target_position_sub;
    ros::Subscriber battery_state_sub;
    ros::Subscriber current_state_sub;
    ros::Subscriber drone_message_sub;
    ros::Subscriber ai_state_sub;
    ros::Subscriber navigation_target_sub;

    ros::Publisher terminal_message_pub;
};

class DisplayWindow
{
    public:
        DisplayWindow(DroneTerminal* terminal);
        void Draw();

        void LocalPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void TargetPositionCallback(const mavros_msgs::PositionTarget::ConstPtr& msg);
        void BatteryStateCallback(const sensor_msgs::BatteryState::ConstPtr& msg);
        void AltitudeCallback(const mavros_msgs::Altitude::ConstPtr& msg);
        void CurrentStateCallback(const mavros_msgs::State::ConstPtr& msg);
        void AIStateCallback(const atlas_drone::AIState::ConstPtr& msg);
        void NavigationTargetCallback(const geometry_msgs::Pose::ConstPtr &msg);

    private:
        DroneTerminal* terminal;
        WINDOW* window;
        bool dirty;

        geometry_msgs::PoseStamped local_position;
        double local_yaw;
        mavros_msgs::PositionTarget target_position;
        geometry_msgs::Pose navigation_target;
        mavros_msgs::State current_state;
        sensor_msgs::BatteryState battery_state;
        mavros_msgs::Altitude current_altitude;
        string current_behaviour;
        string current_task;
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
