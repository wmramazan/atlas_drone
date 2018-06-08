#ifndef AITASK_H
#define AITASK_H

#include <DroneAI/DroneAI.h>
#include <ros/ros.h>
#include <functional>

using namespace std;

struct AITaskResult
{
    bool result;
    ros::Duration completation_time;
};

struct AIMoveTarget
{
    float x;
    float y;
    float z;
    float yaw;
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

    string name;

  protected:
    ros::Time task_start_time;
    function<void(AITaskResult&)> taskCompleteCallback;

    void terminate_task(bool result)
    {
        AITaskResult taskResult;
        taskResult.result = result;
        taskResult.completation_time = ros::Time::now() - task_start_time;
        taskCompleteCallback(taskResult);
    }
};

class TakeControlTask : public AITask
{
  enum AITakeControlTaskState
  {
      NO_CONTROL            = 0,
      WAITING_TO_TRY_AGAIN  = 1,
      HAVE_CONTROL          = 2
  };

  public:
    TakeControlTask()
    {
        LOG("|-> Initializing Take Control Task.");
        name = "Take Control Task";
        LOG("|-< Take Control Task Initialization Complete.");
    }

    virtual void Start();
    virtual void Update();
    virtual void End();

  private:
    ros::Time last_try_time;
    AITakeControlTaskState control_state;

    void take_control();
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
        name = "Take-Off Task";
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
    MoveTask(AIMoveTarget& move_target)
    {
        LOG("|-> Initializing Move Task.");
        name = "Move Task";
        this->move_target.x = move_target.x;
        this->move_target.y = move_target.y;
        this->move_target.z = move_target.z;
        this->move_target.yaw = move_target.yaw;
        LOG("|-< Move Task Initialization Complete.");
    }

    virtual void Start();
    virtual void Update();
    virtual void End();

  private:
    ros::Time last_try_time;
    AIMoveTaskState task_state;
    AIMoveTarget move_target;
    bool moved = false;
    bool isInAir = false;
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
