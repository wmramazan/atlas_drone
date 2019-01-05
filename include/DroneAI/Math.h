#ifndef MATH_H
#define MATH_H

#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>

using namespace geometry_msgs;

#define PI 3.14159265359
#define DEG2RAD ((PI * 2) / 360)
#define RAD2DEG (360 / (PI * 2))

class Math
{
public:
    static double GetRPY(geometry_msgs::Quaternion orientation, double& roll, double& pitch, double& yaw)
    {
        tf::Quaternion quaternion(orientation.x, orientation.y, orientation.z, orientation.w);
        tf::Matrix3x3 m(quaternion);
        m.getRPY(roll, pitch, yaw);
    }

    static void SetRPY(geometry_msgs::Quaternion& orientation, double roll, double pitch, double yaw)
    {
        tf::Quaternion quaternion(orientation.x, orientation.y, orientation.z, orientation.w);
        tf::Matrix3x3 m(quaternion);
        m.setRPY(roll, pitch, yaw);
        m.getRotation(quaternion);

        orientation.x = quaternion.getX();
        orientation.y = quaternion.getY();
        orientation.z = quaternion.getZ();
        orientation.w = quaternion.getW();

        //tf::quaternionTFToMsg(quaternion, orientation);
    }

    static double GetRoll(Quaternion orientation)
    {
        double roll, pitch, yaw;
        GetRPY(orientation, roll, pitch, yaw);
        return roll;
    }

    static double GetPitch(Quaternion orientation)
    {
        double roll, pitch, yaw;
        GetRPY(orientation, roll, pitch, yaw);
        return pitch;
    }

    static double GetYaw(Quaternion orientation)
    {
        double roll, pitch, yaw;
        GetRPY(orientation, roll, pitch, yaw);
        return yaw;
    }

    static void SetYaw(Quaternion& orientation, double yaw_angle)
    {
        double roll, pitch, yaw;
        GetRPY(orientation, roll, pitch, yaw);
        SetRPY(orientation, roll, pitch, yaw_angle);
    }

    static double ClampAngle(double angle)
    {
        if (angle > PI)
        {
            angle -= 2 * PI;
        }
        else if (angle < -PI)
        {
            angle += 2 * PI;
        }

        return angle;
    }
};

#endif // MATH_H
