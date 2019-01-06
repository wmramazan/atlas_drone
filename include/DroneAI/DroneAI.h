#ifndef DRONEAI_H
#define DRONEAI_H

#include <ros/ros.h>
#include <DroneAI/Commander.h>
#include <std_srvs/Trigger.h>
#include <DroneNavigation/Vec3.h>

#define LOG DroneAI::Instance->commander->SendMessage
#define DRONE DroneAI::Instance->Drone
#define DRONEAI DroneAI::Instance

using namespace ros;

class DJIDrone;
class Commander;
class BehaviourManager;
class AIBehaviour;
class AITask;

using namespace std;

class DroneAI
{
  public:
    DroneAI();
    void UpdateAI();

  public:
    static DroneAI* Instance;
    DJIDrone* Drone;
    Commander* commander;
    BehaviourManager* behaviourManager;

  private:
    NodeHandle nh;

    Publisher ai_state_pub;
    ServiceServer trigger_service;

    bool triggerServiceCallback(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response);
};

#endif // DRONEAI_H
