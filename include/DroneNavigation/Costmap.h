#ifndef COSTMAP_H
#define COSTMAP_H

#define NUMBER_THREADS 8

#include <std_msgs/UInt8MultiArray.h>
#include <nav_msgs/Path.h>
#include "Vec3Int.h"
#include <boost/thread.hpp>
#include <limits>
#include <cmath>
#include <cassert>

#include <boost/thread/thread.hpp>
#include <ros/ros.h>

using namespace std_msgs;
using namespace nav_msgs;

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
    void Merge(Costmap* costmap);
    void Clear();
    bool CanPathPass(Path* path);

    UInt8MultiArray data;
    uint size;
    uint size_square;
    uint size_cube;
    int origin;
    double offset;

private:
    double resolution;

    class CostmapMergeJob
    {
          private:
              Costmap*  costmap;
              UInt8MultiArray* data;
              int       start;
              int       end;
         public:
              void Setup(UInt8MultiArray* data, Costmap* costmap, int start, int end)
              {
                  this->costmap = costmap;
                  this->start = start;
                  this->end = end;
                  this->data = data;
              }

              void Job()
              {
                  for (int i = start; i < end; i++)
                      data->data[i] |= costmap->data.data[i];
              }

              void operator()()
              {
                  Job();
              }
     };
};

#endif // COSTMAP_H
