#include "DroneTerminal/DroneTerminal.h"

MessageWindow::MessageWindow(DroneTerminal* terminal)
{
    this->terminal = terminal;
    size = LINES - DISPLAY_SIZE - 2;
    window = newwin(size + 2, COLS, DISPLAY_SIZE - 1, 0);
    dirty = true;
    line = 0;
}

void MessageWindow::Draw()
{
    if (!dirty)
        return;

    wclear(window);
    wborder(window, ACS_VLINE, ACS_VLINE, ACS_HLINE, ACS_HLINE, ACS_LTEE, ACS_RTEE, ACS_LTEE, ACS_RTEE);

    for (int i = 0; i < size; i++)
    {
        if (i + line < buffer.size())
        {
            const char* str = buffer[i + line].c_str();
            str[0] == 'G' ? wattron(window, COLOR_PAIR(2)) : wattron(window, COLOR_PAIR(3));
            mvwprintw(window, i + 1, 1, str);
            str[0] == 'G' ? wattroff(window, COLOR_PAIR(2)) : wattroff(window, COLOR_PAIR(3));
        }
        else
            break;
    }


    wrefresh(window);
    dirty = false;
}

void MessageWindow::Print(bool userMode, const  char* input)
{
    string message;
    if (userMode)
        message += "Ground";
    else
        message += "Atlas";

    time_t t = time(NULL);
    char mbstr[100];
    strftime(mbstr, 100, "%T", std::localtime(&t));
    string time_stamp = mbstr;

    message += "[" + time_stamp + "]: ";

    message += input;
    buffer.push_back(message);

    if (buffer.size() > size)
        line = buffer.size() - size;

    dirty = true;
}

void MessageWindow::ScrollUp()
{
    if (line > 0)
    {
        line--;
        dirty = true;
    }
}

void MessageWindow::ScrollDown()
{
    if (line + size < buffer.size())
    {
        line++;
        dirty = true;
    }
}

void MessageWindow::DroneMessageCallback(const std_msgs::String::ConstPtr& msg)
{
    Print(false, msg->data.c_str());
}

void MessageWindow::Clear()
{
    buffer.clear();
    line = 0;
    dirty = true;
    Draw();
}
