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
  enum InitializationTaskState
  {
      INITIAL_STATE         = 0,
      SETTING_MODE          = 1,
      ARMING                = 2,
      INITIALIZED           = 3
  };

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
    InitializationTaskState task_state;

    void set_mode();
    void arm();
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
    enum AIMoveTaskState
    {
      TAKEOFF_REQUEST       = 0,
      WAITING_TO_TRY_AGAIN  = 1,
      IN_AIR                = 2
    };

  public:
    MoveTask(Pose move_target)
    {
        LOG("|-> Initializing Move Task.");
        Name = "Move Task";
        this->move_target = move_target;
        LOG("|-< Move Task Initialization Complete.");
    }

    virtual void Start();
    virtual void Update();
    virtual void End();

  private:
    ros::Time last_try_time;
    AIMoveTaskState task_state;
    Pose move_target;
    bool moved = false;
    bool isInAir = false;
    bool time_set = false;
};

/*class AIHoverTask : AITask
{
  public:
    AIHoverTask(void* taskCallback(AITaskResult result));
};

class AILandTask : AITask
{
  public:
    AILandTask(void* taskCallback(AITaskResult result));
};

class AIMoveTask : AITask
{
  public:
    AIMoveTask(void* taskCallback(AITaskResult result));
};*/

#endif // Task_H
