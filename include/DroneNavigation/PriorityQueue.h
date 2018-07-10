#ifndef PRIORITYQUEUE_H
#define PRIORITYQUEUE_H

#include <vector>
#include <DroneNavigation/Pathfinder.h>

using namespace std;

class PriorityQueue
{
  public:
    PriorityQueue()
    {

    }
  private:
    vector<Pathfinder::Node*> queue;
};

#endif // PRIORITYQUEUE_H
