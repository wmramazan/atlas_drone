#include "DroneTerminal/DroneTerminal.h"
#include "DroneAI/Math.h"

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

    mvwprintw(window, 1, 1, "Current Mode\t: %s", current_state.mode.c_str());
    mvwprintw(window, 2, 1, "Battery State\t: %.2fV (%.2f%%)",  static_cast<double>(battery_state.voltage),  static_cast<double>(battery_state.percentage));
    mvwprintw(window, 3, 1, "Connected\t: %s", current_state.connected ? "True" : "False");
    mvwprintw(window, 3, 40, "Armed\t: %s", current_state.armed ? "True" : "False");

    mvwprintw(window, 4, 1, "Local Pos\t: %.2f - %.2f - %.2f", local_position.pose.position.x, local_position.pose.position.y, local_position.pose.position.z);
    mvwprintw(window, 4, 40, "Yaw\t: %.2f", local_yaw  * RAD2DEG);

    mvwprintw(window, 5, 1, "Target Pos\t: %.2f - %.2f - %.2f", target_position.position.x, target_position.position.y, target_position.position.z);
    mvwprintw(window, 5, 40, "Yaw\t: %.2f", static_cast<double>(target_position.yaw) * RAD2DEG);
    mvwprintw(window, 5, 60 , "Rate:  %.2f", static_cast<double>(target_position.yaw_rate));

    mvwprintw(window, 6, 1, "Nav Target\t: %.2f - %.2f - %.2f", navigation_target.position.x, navigation_target.position.y, navigation_target.position.z);

    mvwprintw(window, 7, 1, "Behaviour\t: %s", current_behaviour.c_str());
    mvwprintw(window, 8, 1, "Task\t\t: %s", current_task.c_str());

    mvwprintw(window, 1, COLS - 4, "%d", terminal->id);

    wrefresh(window);

    dirty = false;
}

void DisplayWindow::AltitudeCallback(const mavros_msgs::Altitude::ConstPtr& msg)
{
    current_altitude = *msg;
    dirty = true;
}

void DisplayWindow::LocalPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    local_position = *msg;
    local_position.pose.position.x += terminal->start_position.x;
    local_position.pose.position.y += terminal->start_position.y;
    local_position.pose.position.z += terminal->start_position.z;
    local_yaw = Math::GetYaw(local_position.pose.orientation);
    dirty = true;
}

void DisplayWindow::TargetPositionCallback(const mavros_msgs::PositionTarget::ConstPtr &msg)
{
    target_position = *msg;
    target_position.position.x += terminal->start_position.x;
    target_position.position.y += terminal->start_position.y;
    target_position.position.z += terminal->start_position.z;
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

void DisplayWindow::CurrentStateCallback(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

void DisplayWindow::NavigationTargetCallback(const Pose::ConstPtr &msg)
{
    navigation_target = *msg;
}
