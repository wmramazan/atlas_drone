#ifndef AIBEHAVIOURMANAGER_H
#define AIBEHAVIOURMANAGER_H

#include "DroneAI/DroneAI.h"
#include "ros/ros.h"
#include <map>

class AIBehaviour;
class AITask;

using namespace std;
using namespace ros;

class BehaviourManager
{
  public:
    BehaviourManager(NodeHandle& nh);

    void Update();
    void SetBehaviour(AIBehaviour* newBehaviour, bool forceSet);
    void SetBehaviour(string behaviourName, bool forceSet);
    void AddTaskTo(AIBehaviour* behaviour, AITask* task);
    void AddTaskToCurrentBehaviour(AITask* task);

    AIBehaviour *currentBehaviour;

  private:
    map<string, AIBehaviour*> behaviourMap;
};

#endif // AIBEHAVIOURMANAGER_H
