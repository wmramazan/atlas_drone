#ifndef AIBEHAVIOUR_H
#define AIBEHAVIOUR_H

#include "DroneAI/AITask.h"
#include "DroneAI/DroneAI.h"

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <queue>
#include <functional>
using namespace ros;
using namespace std;
using namespace geometry_msgs;
using namespace nav_msgs;
using namespace std_srvs;

class AIBehaviour
{
  public:
    AIBehaviour()
    {

    }

    virtual void OnEnter()
    {

    }

    virtual void Update()
    {
        if (CurrentTask == NULL)
        {
            //ROS_INFO("%s : Null task exception.", name.c_str());
            on_handle_null_task_exception();
        }
        else
        {
            //ROS_INFO("Updating %s.", CurrentTask->Name.c_str());
            CurrentTask->Update();
        }
    }

    virtual void OnExit()
    {

    }

    virtual void AddTask(AITask* task)
    {
        task_queue.push(task);
    }

  protected:
    virtual void task_complete_callback(AITaskResult& result)
    {
        if (result.result)
            LOG("||- %s completed in %d seconds.", result.name.c_str(), result.completation_time.sec);
        else
            LOG("||- %s failed to complete in %d seconds.", result.name.c_str(), result.completation_time.sec);

        next_task();
    }

    virtual void on_task_added()
    {

    }

    virtual void on_handle_null_task_exception()
    {
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
    virtual void OnEnter();
    virtual void Update();
    virtual void OnExit();
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
    virtual void OnEnter();
    virtual void Update();
    virtual void OnExit();
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
    virtual void OnEnter();
    virtual void Update();
    virtual void OnExit();
    virtual void AddTask(AITask* task)
    {
        task->SetCallback(bind(&NavigationBehaviour::task_complete_callback, this, placeholders::_1));
        AIBehaviour::AddTask(task);
    }
  private:
    virtual void task_complete_callback(AITaskResult& result);
    virtual void on_task_added();

    void navigation_target_callback(const geometry_msgs::Pose::ConstPtr& msg);
    void path_callback(const Path::ConstPtr& msg);

    bool request_path();
    bool request_path_clearence();

  private:

    ServiceClient generate_path_service_client;
    ServiceClient is_path_clear_service_client;
    Subscriber navigation_target_sub;
    Subscriber path_sub;
    Publisher target_pose_pub;
    Pose navigation_target;

    Path path;
    bool path_found;
    ros::Time path_time;
};
#endif // AIBEHAVIOUR_H
