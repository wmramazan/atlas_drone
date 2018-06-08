#include "DroneAI/BehaviourManager.h"
#include "DroneAI/DJIDrone.h"
#include "DroneAI/AIBehaviour.h"

BehaviourManager::BehaviourManager()
{
    LOG("|-> Initializing Behaviour Manager.");
    CommandedBehaviour *commandedBehaviour = new CommandedBehaviour();
    IdleBehaviour *idleBehaviour = new IdleBehaviour();

    behaviourMap[commandedBehaviour->name] = commandedBehaviour;
    behaviourMap[idleBehaviour->name] = idleBehaviour;
    LOG("|-< Behaviour Manager Initialization Complete." );
}

void BehaviourManager::Update()
{
    if (currentBehaviour == NULL)
    {
        LOG("TODO: Null behaviour exception");
        //TODO: Null behaviour exception
    }
    else
    {
        //DJIDrone::Instance->SendMessage("-> Updating current behaviour.");
        currentBehaviour->Update();
    }
}

void BehaviourManager::SetBehaviour(string behaviourName, bool forceSet)
{
    if (forceSet)
    {
        LOG("||-> Force setting new behaviour : %s", behaviourName.c_str());
        currentBehaviour = behaviourMap[behaviourName];
        LOG("||-< Behaviour set.");
    }
    else
    {
        LOG("//TODO: Smooth behaviour transition");
        //TODO: Null behaviour exception
    }
}

void BehaviourManager::AddTaskToCurrentBehaviour(AITask* task)
{
    currentBehaviour->AddTask(task);
}

void BehaviourManager::AddTaskTo(AIBehaviour* behaviour, AITask* task)
{
    behaviour->AddTask(task);
}
