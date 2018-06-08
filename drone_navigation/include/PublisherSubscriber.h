#ifndef PUBLISHERSUBSCRIBER_H
#define PUBLISHERSUBSCRIBER_H

#include <ros/ros.h>
#include <string>

template<typename PublishT, typename SubscribeT>
class PublisherSubscriber
{
public:
  ros::NodeHandle nodeHandle;
  PublisherSubscriber() {}
  PublisherSubscriber(std::string publisherTopicName, std::string subscriberTopicName, int queueSize)
  {
    publisher = nodeHandle.advertise<PublishT>(publisherTopicName, queueSize);
    subscriber = nodeHandle.subscribe<SubscribeT>(subscriberTopicName, queueSize, &PublisherSubscriber::callback, this);
  }
  void callback(const typename SubscribeT::ConstPtr& message);
protected:
  ros::Publisher publisher;
  ros::Subscriber subscriber;
};

#endif // PUBLISHERSUBSCRIBER_H
