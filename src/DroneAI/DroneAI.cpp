#include "DroneAI/DroneAI.h"
#include "DroneAI/DJIDrone.h"
#include "DroneAI/BehaviourManager.h"
#include "DroneAI/AIBehaviour.h"
#include "DroneAI/AITask.h"
#include "atlas_drone/AIState.h"

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

    NodeHandle nh("~");
    id = nh.param("drone_id", 1);
    ROS_INFO("Id: %d", id);

    nh = NodeHandle("uav" + to_string(id));
    Vec3 start_position = Vec3(nh.param("start_position_x", 0), nh.param("start_position_y", 0), nh.param("start_position_z", 0));

    ai_state_pub = nh.advertise<atlas_drone::AIState>(nh.param<string>("/ai_state_topic", "/drone_ai/ai_state"), 10);
    trigger_service = nh.advertiseService(nh.param<string>("/go_to_target_service", "/drone_ai/go_to_target"), &DroneAI::triggerServiceCallback, this);

    commander = new Commander(nh);
    Drone = new DJIDrone(nh, start_position);
    behaviourManager = new BehaviourManager(nh);

    behaviourManager->SetBehaviour("Commanded Behaviour", true);
    Duration(3.0).sleep();
    behaviourManager->SetBehaviour("Idle Behaviour", true);

    LOG("Drone AI Initialized. Beginning AI cycle.");

    ros::Rate loop_rate(40);
    while (ros::ok())
    {
        ros::spinOnce();
        UpdateAI();
        loop_rate.sleep();
    }
}

void DroneAI::UpdateAI()
{
    Drone->Move();
    behaviourManager->Update();

    atlas_drone::AIState aiState;

    if (behaviourManager->currentBehaviour != NULL)
    {
        aiState.current_behaviour = behaviourManager->currentBehaviour->name.c_str();

        if (behaviourManager->currentBehaviour->CurrentTask != NULL)
          aiState.current_task = behaviourManager->currentBehaviour->CurrentTask->Name;
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

bool DroneAI::triggerServiceCallback(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response)
{
    behaviourManager->SetBehaviour("Navigation Behaviour", true);

    response.success = true;
    response.message = "Following the path..";
    return true;
}
