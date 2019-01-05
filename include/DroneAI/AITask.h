#ifndef AITASK_H
#define AITASK_H

#include <DroneAI/DroneAI.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose.h>
#include <functional>
#include <DroneNavigation/Vec3.h>

using namespace std;
using namespace ros;
using namespace geometry_msgs;

struct AITaskResult
{
    string name;
    bool result;
    Duration completation_time;
};

class AITask
{
  public:
    AITask()
    {
        task_start_time = Time::now();
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
        taskResult.completation_time = Time::now() - task_start_time;
        taskCompleteCallback(taskResult);
    }

  public:
    string Name;

  protected:
    Time task_start_time;
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
    Time last_try_time;
    Time in_air_time;
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
    Time last_try_time;
    bool rotating;

    Vec3 target_position;
    double target_yaw;
};

class MoveTask : public AITask
{
  public:
    MoveTask(Vec3 target_position, double target_yaw, double move_speed = 1)
    {
        LOG("|-> Initializing Move Task.");
        Name = "Move Task";
        this->target_position = target_position;
        this->target_rotation = target_yaw;
        this->move_speed = move_speed;
        LOG("|-< Move Task Initialization Complete.");
    }

    virtual void Start();
    virtual void Update();
    virtual void End();

  private:
    Time last_try_time;

    bool time_set;
    Vec3 target_position;
    Vec3 step_position;
    double target_rotation;
    double step_rotation;
    double move_speed;
    double rotation_speed;
    bool moving_step;
};

#endif // Task_H
