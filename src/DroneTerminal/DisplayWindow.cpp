#include "DroneTerminal/DroneTerminal.h"

DisplayWindow::DisplayWindow(DroneTerminal* terminal)
{
    this->terminal = terminal;
    window = newwin(DISPLAY_SIZE, COLS, 0, 0);
    dirty = true;
}

void DisplayWindow::Draw()
{
    if (!dirty)
        return;

    werase(window);
    wborder(window, ACS_VLINE, ACS_VLINE, ACS_HLINE, ACS_HLINE, ACS_ULCORNER, ACS_URCORNER, ACS_LTEE, ACS_RTEE);

    mvwprintw(window, 1, 1, "Flight Status\t: %d", flight_status);
    mvwprintw(window, 1, 1, "Flight Status\t: %d\t%s", flight_status, flight_status_str[flight_status].c_str());
    mvwprintw(window, 2, 1, "Display Mode\t: %d\t%s", display_mode, display_mode_str[display_mode].c_str());
    mvwprintw(window, 3, 1, "Battery State\t: %fV (%f\%)", battery_state.voltage, battery_state.percentage);
    mvwprintw(window, 4, 1, "GPS Health\t: %d", gps_health);
    mvwprintw(window, 5, 1, "GPS Position\t: %f - %f - %f", gps_position.altitude, gps_position.latitude, gps_position.longitude);
    mvwprintw(window, 6, 1, "Local Position\t: %f - %f - %f", local_position.point.x, local_position.point.y, local_position.point.z);
    mvwprintw(window, 7, 1, "Behaviour\t: %s", current_behaviour.c_str());
    mvwprintw(window, 8, 1, "Task\t: %s", current_task.c_str());

    wrefresh(window);

    dirty = false;
}

void DisplayWindow::AttitudeCallback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
    current_attitude = msg->quaternion;
    dirty = true;
}

void DisplayWindow::FlightStatusCallback(const std_msgs::UInt8::ConstPtr& msg)
{
    flight_status = msg->data;
    dirty = true;
}

void DisplayWindow::DisplayModeCallback(const std_msgs::UInt8::ConstPtr& msg)
{
    display_mode = msg->data;
    dirty = true;
}

void DisplayWindow::LocalPositionCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    local_position = *msg;
    dirty = true;
}

void DisplayWindow::GPSPositionCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    gps_position = *msg;
    dirty = true;
}

void DisplayWindow::GPSHealthCallback(const std_msgs::UInt8::ConstPtr& msg)
{
    gps_health = msg->data;
    dirty = true;
}

void DisplayWindow::AIStateCallback(const atlas_drone::AIState::ConstPtr& msg)
{
    current_behaviour = msg->current_behaviour;
    current_task = msg->current_task;
    dirty = true;
}

void DisplayWindow::BatteryStateCallback(const sensor_msgs::BatteryState::ConstPtr& msg)
{
    battery_state = *msg;
    dirty = true;
}
