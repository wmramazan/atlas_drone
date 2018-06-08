#ifndef DRONEAI_H
#define DRONEAI_H

#include <ros/ros.h>
#include <DroneAI/Commander.h>

#define LOG DroneAI::Instance->commander->SendMessage
#define DRONE DroneAI::Instance->Drone
#define DRONEAI DroneAI::Instance

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
    ros::NodeHandle nh;

    ros::Publisher ai_state_pub;
};

#endif // DRONEAI_H
