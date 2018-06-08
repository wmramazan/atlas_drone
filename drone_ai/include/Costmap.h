#ifndef COSTMAP_H
#define COSTMAP_H

#include <vector>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/UInt8.h>

using namespace std;
using namespace std_msgs;

struct Costmap
{
    UInt8MultiArray data;

    Costmap(UInt8MultiArray& data)
    {
        this->data = data;
    }

    UInt8* Get(int x, int y, int z)
    {
        return 0;
    }
};

#endif // COSTMAP_H
