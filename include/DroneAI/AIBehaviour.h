#ifndef AIBEHAVIOUR_H
#define AIBEHAVIOUR_H

#include "DroneAI/AITask.h"
#include "DroneAI/DroneAI.h"

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include <DroneNavigation/PathPlanner.h>

#include <queue>
#include <functional>
using namespace ros;
using namespace std;
using namespace geometry_msgs;

class AIBehaviour
{
  public:
    AIBehaviour() { }
    virtual void Update() = 0;

    virtual void AddTask(AITask* task)
    {
        task_queue.push(task);
    }

  protected:
    virtual void task_complete_callback(AITaskResult& result)
    {
        if (result.result)
            LOG("||- %s completed in %d seconds.", CurrentTask->Name.c_str(), result.completation_time.sec);
        else
            LOG("||- %s failed to complete in %d seconds.", CurrentTask->Name.c_str(), result.completation_time.sec);

        next_task();
    }

    void next_task()
    {
        if (CurrentTask != NULL)
            CurrentTask->End();

        if (task_queue.size() > 0)
        {
            CurrentTask = task_queue.front();
            task_queue.pop();
        }
        else
            CurrentTask = NULL;

        if (CurrentTask != NULL)
            CurrentTask->Start();
    }

  public:
    string name;
    AITask *CurrentTask = NULL;
  protected:
    queue<AITask*> task_queue;
};

class IdleBehaviour : public AIBehaviour
{
  public:
    IdleBehaviour(NodeHandle& nh);
    virtual void Update();
    virtual void AddTask(AITask* task)
    {
        task->SetCallback(bind(&IdleBehaviour::task_complete_callback, this, placeholders::_1));
        AIBehaviour::AddTask(task);
    }
  private:
    virtual void task_complete_callback(AITaskResult& result);
};

class CommandedBehaviour : public AIBehaviour
{
  public:
    CommandedBehaviour(NodeHandle& nh);
    virtual void Update();
    virtual void AddTask(AITask* task)
    {
        task->SetCallback(bind(&CommandedBehaviour::task_complete_callback, this, placeholders::_1));
        AIBehaviour::AddTask(task);
    }
  private:
    virtual void task_complete_callback(AITaskResult& result);
};

class NavigationBehaviour : public AIBehaviour
{
  public:
    NavigationBehaviour(NodeHandle& nh);
    virtual void Update();
    virtual void AddTask(AITask* task)
    {
        task->SetCallback(bind(&NavigationBehaviour::task_complete_callback, this, placeholders::_1));
        AIBehaviour::AddTask(task);
    }
  private:
    virtual void task_complete_callback(AITaskResult& result);
    void navigation_target_callback(const geometry_msgs::Pose::ConstPtr& msg);

  private:
    Subscriber navigation_target_sub;
    Pose navigation_target;
    Path* path;
    bool on_task;

    PathPlanner* pathPlanner;
};
#endif // AIBEHAVIOUR_H
