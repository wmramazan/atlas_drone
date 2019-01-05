#ifndef AICOMMANDER_H
#define AICOMMANDER_H

#include <string>
#include <stdio.h>
#include <stdarg.h>
#include <algorithm>
#include <iostream>
#include <boost/algorithm/string.hpp>
#include <vector>


#include <ros/ros.h>

#include <std_msgs/String.h>

#include <DroneAI/Math.h>

using namespace std;

/*class Command;
struct Argument
{
    string name;
    vector<Command> args;
};

struct Command
{
    string name;
    vector<Argument> args;
};*/

class Commander
{
  public:
    Commander(ros::NodeHandle& nh);
    void SendMessage(const char* fmt, ...);
    void Execute(string command);

  private:
    void terminal_message_callback(const std_msgs::String::ConstPtr& msg);
    //bool validate(Command command);


  private:
    ros::Subscriber terminal_message_sub;
    ros::Publisher drone_message_pub;

    /*vector<Command> commands
    {
      {"set_behaviour", {
                          {"behaviour", {
                                          {"idle", {}},
                                          {"commanded", {}}
                                        }
                          },
                          {"forced",    {
                                          {"0", {}},
                                          {"1", {}}
                                        }
                          }
                        }
      },
      {"add_task",      {
                          {"task",      {
                                          {"take_control", {}},
                                          {"take_off", {}},
                                          {"move",  {
                                                      {"target",  {
                                                                    {"x", {}},
                                                                    {"y", {}},
                                                                    {"z", {}}
                                                                  }

                                                      }
                                                    }
                                          }
                                        }
                          }
                        }
      },
      {"add_task_to",   {
                          {"behaviour", {
                                          {"idle", {}},
                                          {"commanded", {}}
                                        }
                          },
                          {"task",      {
                                          {"take_control", {}},
                                          {"take_off", {}},
                                          {"move",  {
                                                      {"target",  {
                                                                    {"x", {}},
                                                                    {"y", {}},
                                                                    {"z", {}}
                                                                  }

                                                      }
                                                    }
                                          }
                                        }
                          }
                        }
      }
    };*/
};

#endif // AICOMMANDER_H
