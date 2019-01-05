#ifndef AITASK_H
#define AITASK_H

#include <DroneAI/DroneAI.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose.h>
#include <functional>
#include <DroneNavigation/Vec3.h>

using namespace std;
using namespace geometry_msgs;

struct AITaskResult
{
    string name;
    bool result;
    ros::Duration completation_time;
};

class AITask
{
  public:
    AITask()
    {
        task_start_time = ros::Time::now();
    }

    virtual void Start() = 0;
    virtual void Update() = 0;
    virtual void End() = 0;

    void SetCallback(function<void (AITaskResult &)> taskCompleteCallback)
    {
        this->taskCompleteCallback = taskCompleteCallback;
    }

    void Terminate()
    {
        AITaskResult taskResult;
        taskResult.name = Name;
        taskResult.result = task_completed;
        taskResult.completation_time = ros::Time::now() - task_start_time;
        taskCompleteCallback(taskResult);
    }

  public:
    string Name;

  protected:
    ros::Time task_start_time;
    function<void(AITaskResult&)> taskCompleteCallback;
    bool task_completed;
};

class InitializationTask : public AITask
{
  public:
    InitializationTask()
    {
        LOG("|-> \"Initializing Initialization\" Task.");
        Name = "\"Initialization Task\"";
        LOG("|-< \"Initialization Task\" Initialization Complete.");
    }

    virtual void Start();
    virtual void Update();
    virtual void End();

  private:
    ros::Time last_try_time;
    ros::Time in_air_time;
    bool air_time_set;

    bool set_mode();
    bool arm();
};

class IdleTask : public AITask
{
  public:
    IdleTask(bool rotating)
    {
        LOG("|-> Initializing \"Idle Task.\"");
        Name = "\"Idle Task\"";
        this->rotating = rotating;
        LOG("|-< \"Idle Task\" Initialization Complete.");
    }

    virtual void Start();
    virtual void Update();
    virtual void End();

  private:
    ros::Time last_try_time;
    bool rotating;

    Vec3 target_position;
    double target_yaw;
};

class TakeOffTask : public AITask
{
    enum AITakeOffTaskState
    {
      TAKEOFF_REQUEST       = 0,
      WAITING_TO_TRY_AGAIN  = 1,
      IN_AIR                = 2
    };

  public:
    TakeOffTask()
    {
        LOG("|-> Initializing Take Off Task.");
        Name = "Take Off Task";
        LOG("|-< Take Off Task Initialization Complete.");
    }

    virtual void Start();
    virtual void Update();
    virtual void End();

  private:
    ros::Time last_try_time;

    AITakeOffTaskState task_state;
    bool moved = false;
    bool isInAir = false;

    void start_engine();
    bool request_takeoff();
};

class MoveTask : public AITask
{
  public:
    MoveTask(Vec3 target_position, double target_yaw)
    {
        LOG("|-> Initializing Move Task.");
        Name = "Move Task";
        this->target_position = target_position;
        this->target_yaw = target_yaw;
        LOG("|-< Move Task Initialization Complete.");
    }

    virtual void Start();
    virtual void Update();
    virtual void End();

  private:
    ros::Time last_try_time;

    Pose move_target;
    bool moved = false;
    bool isInAir = false;
    bool time_set = false;

    Vec3 target_position;
    double target_yaw;
};

#endif // Task_H
