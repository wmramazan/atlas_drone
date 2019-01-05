#include "DroneTerminal/DroneTerminal.h"

//TODO_BD: Runtime Subcription

int main(int argc, char **argv)
{
    ros::init(argc, argv, "drone_terminal");
    DroneTerminal droneTerminal;
}

DroneTerminal::DroneTerminal()
{
    initialize_terminal();
    initialize_node();
}

void DroneTerminal::initialize_terminal()
{
    initscr();
    noecho();
    cbreak();
    refresh();
    nodelay(stdscr, true);
    keypad(stdscr, true);

    start_color();
    init_pair(1, COLOR_WHITE, COLOR_BLACK);
    init_pair(2, COLOR_MAGENTA, COLOR_BLACK);
    init_pair(3, COLOR_CYAN, COLOR_BLACK);

    displayWindow = new DisplayWindow(this);
    messageWindow = new MessageWindow(this);
    inputWindow = new InputWindow(this);

    draw();
}

void DroneTerminal::initialize_node()
{
    nh = ros::NodeHandle("uav" + to_string(nh.param("/drone_id", 1)));

    terminal_message_pub = nh.advertise<std_msgs::String>("drone_ai/terminal_message", 1000);

    altitude_sub        = nh.subscribe("mavros/altitude",               10, &DisplayWindow::AltitudeCallback,       displayWindow);
    local_position_sub  = nh.subscribe("mavros/local_position/pose",    10, &DisplayWindow::LocalPositionCallback,  displayWindow);
    target_position_sub = nh.subscribe("mavros/setpoint_raw/local",     10, &DisplayWindow::TargetPositionCallback, displayWindow);
    battery_state_sub   = nh.subscribe("mavros/battery",                10, &DisplayWindow::BatteryStateCallback,   displayWindow);
    current_state_sub   = nh.subscribe("mavros/state",                  10, &DisplayWindow::CurrentStateCallback,   displayWindow);
    ai_state_sub        = nh.subscribe("drone_ai/ai_state",             10, &DisplayWindow::AIStateCallback,        displayWindow);
    drone_message_sub   = nh.subscribe("drone_ai/drone_message",        10, &MessageWindow::DroneMessageCallback,   messageWindow);

    ros::Rate loop(50);
    while(ros::ok())
    {
        ros::spinOnce();
        loop.sleep();

        draw();

        char* input = inputWindow->GetInput();
        if (input != NULL)
        {
            if (strcmp(input, "clear") == 0)
                messageWindow->Clear();
            else
            {
                messageWindow->Print(true, input);
                std_msgs::String msg;
                msg.data = input;
                terminal_message_pub.publish(msg);
            }
        }
    }
}

void DroneTerminal::draw()
{
    displayWindow->Draw();
    messageWindow->Draw();
    inputWindow->Draw();
}
