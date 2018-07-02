#include "DroneAI/TaskManager.h"

TaskManager::TaskManager(AIBehaviour& behaviour)
{
    this->behaviour = &behaviour;
}
