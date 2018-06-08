#include "DroneAI/DroneAI.h"
#include "DroneAI/DJIDrone.h"
#include "DroneAI/BehaviourManager.h"
#include "DroneAI/AIBehaviour.h"
#include "DroneAI/AITask.h"
#include "drone_ai/AIState.h"

int main(int argc, char **argv)
{
    ROS_INFO("> Initializing Drone AI...");
    ros::init(argc, argv, "drone_ai");
    DroneAI drone_ai;
    ROS_INFO("< Drone AI Terminated...");
    return 0;
}

DroneAI* DroneAI::Instance = NULL;

DroneAI::DroneAI()
{
    if (Instance == NULL)
        Instance = this;

    ai_state_pub = nh.advertise<drone_ai::AIState>("/drone_ai/ai_state", 10);

    commander = new Commander(nh);
    Drone = new DJIDrone(nh);
    behaviourManager = new BehaviourManager();

    behaviourManager->SetBehaviour("Idle Behaviour", true);

    LOG("Drone AI Initialized. Beginning AI cycle.");

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        UpdateAI();
        loop_rate.sleep();
    }
}

void DroneAI::UpdateAI()
{
    ros::spinOnce();
    behaviourManager->Update();

    drone_ai::AIState aiState;

    if (behaviourManager->currentBehaviour != NULL)
    {
        aiState.current_behaviour = behaviourManager->currentBehaviour->name.c_str();

        if (behaviourManager->currentBehaviour->CurrentTask != NULL)
          aiState.current_task = behaviourManager->currentBehaviour->CurrentTask->name;
        else
          aiState.current_task = "NULL";
    }
    else
    {
        aiState.current_behaviour = "NULL";
        aiState.current_task = "NULL";
    }

    ai_state_pub.publish(aiState);
}
