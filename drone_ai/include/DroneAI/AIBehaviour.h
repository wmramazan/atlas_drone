#ifndef AIBEHAVIOUR_H
#define AIBEHAVIOUR_H

#include "DroneAI/AITask.h"
#include "DroneAI/DroneAI.h"

#include <queue>
#include <functional>
using namespace std;

class AIBehaviour
{
  public:
    AIBehaviour() { }
    virtual void Update() = 0;

    void AddTask(AITask* task)
    {
        task_queue.push(task);
        task->SetCallback(bind(&AIBehaviour::task_complete_callback, this, placeholders::_1));
    }

  protected:
    void task_complete_callback(AITaskResult& result)
    {
        if (result.result)
            LOG("||- %s completed in %d seconds.", CurrentTask->name.c_str(), result.completation_time.sec);
        else
            LOG("||- %s failed to complete in %d seconds.", CurrentTask->name.c_str(), result.completation_time.sec);

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
    IdleBehaviour();
    virtual void Update();
};

class CommandedBehaviour : public AIBehaviour
{
  public:
    CommandedBehaviour();
    virtual void Update();
};

#endif // AIBEHAVIOUR_H
