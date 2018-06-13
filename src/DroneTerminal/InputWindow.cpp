#include "DroneTerminal/DroneTerminal.h"

InputWindow::InputWindow(DroneTerminal* terminal)
{
    this->terminal = terminal;
    window = newwin(3, COLS, LINES - 2, 0);
    nodelay(window, true);
    keypad(window, true);
    dirty = true;

    length = 0;
    end = false;
    last_input = new char[100];
    buffer = new char[100];
}

void InputWindow::Draw()
{
    if(!dirty)
        return;

     wclear(window);
     wborder(window, ACS_LLCORNER, ACS_VLINE, ACS_HLINE, ' ', ACS_LTEE, ACS_RTEE, ' ', ' ');
     mvwaddch(window, 1, 1, ACS_RARROW);
     wrefresh(window);

     dirty = false;
}

char* InputWindow::GetInput()
{
    if (end)
    {
        buffer = new char[100];
        length = 0;
        end = false;
    }

    int input_char = mvwgetch(window, 1, length + 2);

    if (input_char != ERR)
    {
        if ((input_char == KEY_BACKSPACE || input_char == 127) && length > 0)
        {
            length--;
            buffer[length] = 0;
            mvwprintw(window, 1, length + 2, " ");
            wmove(window, 1, length + 2);
        }
        else if ((input_char == KEY_ENTER || input_char == 10) && length > 0)
        {
            buffer[length] = 0;
            strncpy(last_input, buffer, sizeof(buffer));
            length = 0;
            end = true;
            dirty = true;
        }
        else if (input_char == KEY_PPAGE)
        {
            terminal->messageWindow->ScrollUp();
        }
        else if (input_char == KEY_NPAGE)
        {
            terminal->messageWindow->ScrollDown();
        }
        else if (input_char == KEY_UP)
        {
            buffer = new char[100];
            strncpy(buffer, last_input, sizeof(last_input));
            length = strlen(last_input);
            dirty = true;
            Draw();
            mvwprintw(window, 1, 2, buffer);
        }
        else if ((input_char > 32 && input_char < 127) || input_char == ' ')
        {
            buffer[length] = input_char;
            mvwaddch(window, 1, length + 2, input_char);
            length++;
        }

        wrefresh(window);
    }

    if (end)
        return buffer;
    else
        return NULL;
}
