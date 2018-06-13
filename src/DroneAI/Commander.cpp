#include "DroneAI/Commander.h"
#include "DroneAI/AITask.h"
#include "DroneAI/BehaviourManager.h"

Commander::Commander(ros::NodeHandle& nh)
{
    terminal_message_sub  = nh.subscribe("drone_ai/terminal_message", 10, &Commander::terminal_message_callback, this);
    drone_message_pub = nh.advertise<std_msgs::String>("drone_ai/drone_message", 1000);
    ros::Duration(2.0).sleep();
}

void Commander::SendMessage(const char* fmt, ...)
{
      char buf[100];
      va_list vl;
      va_start(vl, fmt);

      vsnprintf(buf, sizeof(buf), fmt, vl);
      va_end(vl);


      std_msgs::String msg;
      msg.data = buf;

      drone_message_pub.publish(msg);
}

void Commander::terminal_message_callback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("Ground: %s", msg->data.c_str());
    Execute(msg->data);
}

void Commander::Execute(string message)
{
    transform(message.begin(), message.end(), message.begin(), ::tolower);
    vector<string> args;
    boost::split(args, message, boost::is_any_of(" "));

    /*if (validate(command))
    {
        if (command.name.compare("set_behaviour"))
        {

        }
        else if (command.name.compare("add_task"))
        {

        }
        else if (command.name.compare("add_task_to"))
        {

        }
    }*/
    if (args[0].compare("set_behaviour") == 0)
    {
        if (args.size() != 3)
        {
            LOG("Missing arguments: [set_behaviour] [behaviour] [force]");
            return;
        }

        if (args[1].compare("commanded") == 0)
        {
            if (args[2].compare("0"))
            {
                DRONEAI->behaviourManager->SetBehaviour("Commanded Behaviour", false);
            }
            else
            {
                DRONEAI->behaviourManager->SetBehaviour("Commanded Behaviour", true);
            }
        }
        else if (args[1].compare("idle") == 0)
        {
            DRONEAI->behaviourManager->SetBehaviour("Idle Behaviour", args[2].compare("0"));
        }
        else if (args[1].compare("navigation") == 0)
        {
            DRONEAI->behaviourManager->SetBehaviour("Navigation Behaviour", args[2].compare("0"));
        }
    }
    else if (args[0].compare("add_task") == 0)
    {
        if (args.size() == 2)
        {
            if (args[1].compare("take_control") == 0)
            {
                DRONEAI->behaviourManager->AddTaskToCurrentBehaviour(new TakeControlTask());
            }
            else if (args[1].compare("take_off") == 0)
            {
                DRONEAI->behaviourManager->AddTaskToCurrentBehaviour(new TakeOffTask());
            }
        }
        else if (args.size() == 5)
        {
            if (args[1].compare("move") == 0)
            {

                Pose moveTarget;
                moveTarget.position.x = stof(args[2]);
                moveTarget.position.y = stof(args[3]);
                moveTarget.position.z = stof(args[4]);

                DRONEAI->behaviourManager->AddTaskToCurrentBehaviour(new MoveTask(moveTarget));
            }
        }
    }

    //if (message.compare("take_control") == 0)
    //{
        //DRONEAI->behaviourManager->AddTaskToCurrentBehaviour(new TakeControlTask());
    //}
    //else if (message.compare("take_off") == 0)
    //{
    //    DRONEAI->behaviourManager->AddTaskToCurrentBehaviour(new TakeOffTask());
    //}
}

/*bool Commander::validate(vector<string> command)
{
    bool commandValid = false;
    Command validCommand;

    for(int i = 0; i < commands.size(); i++)
    {
        if (command[0].compare(commands[i].name) == 0)
        {
            commandValid = true;
            validCommand = commands[i];
            break;
        }
    }

    if (commandValid)
    {
        if (command.size() != validCommand.args.size())
        {
            string errorLog;
            for(int i = 0; i < validCommand.args.size(); i++)
              errorLog += "[" + validCommand.args[i] + "]";

            LOG("Wrong number of arguments. %s %s", validCommand.na,e.c_str(), errorLog.c_str());
            return false;
        }

        return true;
    }
    else
    {
        LOG("Command not recognized.");
        return false;
    }

    return false;
}*/
