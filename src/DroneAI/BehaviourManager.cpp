#include "DroneAI/BehaviourManager.h"
#include "DroneAI/DJIDrone.h"
#include "DroneAI/AIBehaviour.h"

BehaviourManager::BehaviourManager(NodeHandle& nh)
{
    LOG("|-> Initializing Behaviour Manager.");
    CommandedBehaviour *commandedBehaviour = new CommandedBehaviour(nh);
    IdleBehaviour *idleBehaviour = new IdleBehaviour(nh);
    NavigationBehaviour *navigationBehaviour = new NavigationBehaviour(nh);

    behaviourMap[commandedBehaviour->name] = commandedBehaviour;
    behaviourMap[idleBehaviour->name] = idleBehaviour;
    behaviourMap[navigationBehaviour->name] = navigationBehaviour;
    LOG("|-< Behaviour Manager Initialization Complete." );
}

void BehaviourManager::Update()
{
    if (currentBehaviour != nullptr)
    {
        currentBehaviour->Update();
    }
    else
    {
        LOG("TODO: Null behaviour exception");
    }
}

void BehaviourManager::SetBehaviour(string behaviourName, bool forceSet)
{
    if (forceSet)
    {
        LOG("||-> Force setting new behaviour : %s", behaviourName.c_str());
        if (currentBehaviour != nullptr)
        {
            //currentBehaviour->OnExit();
        }

        currentBehaviour = behaviourMap[behaviourName];
        //currentBehaviour->OnEnter();
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
