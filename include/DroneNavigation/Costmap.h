#ifndef COSTMAP_H
#define COSTMAP_H

#include <std_msgs/UInt8MultiArray.h>
#include "Vec3Int.h"

using namespace std_msgs;

class Costmap
{
public:
    Costmap(uint size, double resolution);
    Costmap(UInt8MultiArray& data, double resolution);

    uint8_t& Get(Vec3Int position);
    uint8_t& Get(uint x, uint y, uint z);
    uint8_t& Get(double x, double y, double z);
    int ToIndex(double value);
    double ToPosition(int value);
    void Merge(Costmap& costmap);
    void Clear();

    UInt8MultiArray data;
    uint size;
    uint size_square;
    uint size_cube;
    int origin;
    double offset;

private:
    double resolution;
};

#endif // COSTMAP_H
