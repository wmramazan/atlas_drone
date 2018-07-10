#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PointStamped.h>

#define TARGET_FRAME "/base_link"
#define SOURCE_FRAME "/map"
#define POSITION_TOPIC "drone_position"

int main(int argc, char **argv)
{
    std::string target_frame;
    std::string source_frame;
    std::string position_topic;

    ros::Publisher drone_position_pub;
    geometry_msgs::PointStamped position;

    ros::init(argc, argv, "drone_tf");
    ros::NodeHandle nh;

    nh.param<std::string>("/child_frame_id", target_frame, TARGET_FRAME);
    nh.param<std::string>("/frame_id", source_frame, SOURCE_FRAME);
    nh.param<std::string>("/position_topic", position_topic, POSITION_TOPIC);

    drone_position_pub = nh.advertise<geometry_msgs::PointStamped>(position_topic, 10);

    tf::TransformListener listener;

    ros::Duration(1.0).sleep();

    ros::Rate rate(10);
    while (nh.ok())
    {
        tf::StampedTransform transform;
        try
        {
            listener.lookupTransform(target_frame, source_frame, ros::Time(0), transform);
        } catch (tf::TransformException &e)
        {
            ROS_ERROR("%s", e.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        position.header.frame_id = target_frame;
        position.header.stamp = ros::Time::now();
        position.point.x = transform.getOrigin().getX();
        position.point.y = transform.getOrigin().getY();
        position.point.z = transform.getOrigin().getZ();

        drone_position_pub.publish(position);

        rate.sleep();
    }
}
