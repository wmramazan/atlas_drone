#ifndef VEC3_H
#define VEC3_H

#include <math.h>
#include <geometry_msgs/Point.h>

struct Vec3
{
    double x;
    double y;
    double z;

    Vec3()
    {
        x = 0;
        y = 0;
        z = 0;
    }

    Vec3(double x, double y, double z)
    {
        this->x = x;
        this->y = y;
        this->z = z;
    }

    void Set(double x, double y, double z)
    {
        this->x = x;
        this->y = y;
        this->z = z;
    }

    int Distance(const Vec3 &vector) const
    {
        return sqrt(pow(vector.x - x, 2) + pow(vector.y - y, 2) + pow(vector.z - z, 2));
    }

    Vec3 operator +(const Vec3 &vector) const
    {
        return Vec3(x + vector.x, y + vector.y, z + vector.z);
    }

    Vec3 operator -(const Vec3 &vector) const
    {
        return Vec3(x - vector.x, y - vector.y, z - vector.z);
    }

    bool operator ==(const Vec3 &vector) const
    {
        return x == vector.x && y == vector.y && z == vector.z;
    }

    bool operator !=(const Vec3 &vector) const
    {
        return x != vector.x || y != vector.y || z != vector.z;
    }

    static Vec3 FromPoint(const geometry_msgs::Point &point)
    {
        return Vec3(point.x, point.y, point.z);
    }
};

#endif // VEC3_H
