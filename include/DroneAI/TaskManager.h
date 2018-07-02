#ifndef TASKMANAGER_H
#define TASKMANAGER_H

#include "AIBehaviour.h"

class TaskManager
{
public:
  TaskManager(AIBehaviour& behaviour);
private:
  AIBehaviour behaviour;
};

#endif // TASKMANAGER_H
