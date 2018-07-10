#include "DroneTerminal/DroneTerminal.h"

///TODO_BD: Runtime Subcription

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
    ros::NodeHandle nh;
    ros::Time start_time = ros::Time::now();

    terminal_message_pub = nh.advertise<std_msgs::String>("drone_ai/terminal_message", 1000);

    attitude_sub      = nh.subscribe("dji_sdk/attitude",        10, &DisplayWindow::AttitudeCallback,      displayWindow);
    flight_status_sub  = nh.subscribe("dji_sdk/flight_status",  10, &DisplayWindow::FlightStatusCallback,  displayWindow);
    display_mode_sub   = nh.subscribe("dji_sdk/display_mode",   10, &DisplayWindow::DisplayModeCallback,   displayWindow);
    local_position_sub = nh.subscribe("drone_position",         10, &DisplayWindow::LocalPositionCallback, displayWindow);
    gps_sub           = nh.subscribe("dji_sdk/gps_position",    10, &DisplayWindow::GPSPositionCallback,   displayWindow);
    gps_health_sub     = nh.subscribe("dji_sdk/gps_health",     10, &DisplayWindow::GPSHealthCallback,     displayWindow);
    battery_state_sub = nh.subscribe("dji_sdk/battery_state",   10, &DisplayWindow::BatteryStateCallback,  displayWindow);
    ai_state_sub      = nh.subscribe("drone_ai/ai_state",       10, &DisplayWindow::AIStateCallback,       displayWindow);
    drone_message_sub = nh.subscribe("drone_ai/drone_message",  10, &MessageWindow::DroneMessageCallback,  messageWindow);

    set_local_pos_reference    = nh.serviceClient<dji_sdk::SetLocalPosRef> ("dji_sdk/set_local_pos_ref");

    dji_sdk::SetLocalPosRef localPosReferenceSetter;
    set_local_pos_reference.call(localPosReferenceSetter);

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
