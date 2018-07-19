#ifndef VEC3_H
#define VEC3_H

#include <math.h>
#include <geometry_msgs/Point.h>
#include <cfloat>

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

    double Distance(const Vec3 &vector) const
    {
        float dx = vector.x - x;
        float dy = vector.y - y;
        float dz = vector.z - z;

        return sqrt(dx * dx + dy * dy + dz * dz);
    }

    double Magnitude() const
    {
        return sqrt(x * x + y * y + z * z);
    }

    Vec3 Normalized()
    {
        return *this / Magnitude();
    }

    Vec3 operator +(const Vec3 &vector) const
    {
        return Vec3(x + vector.x, y + vector.y, z + vector.z);
    }

    Vec3 operator -(const Vec3 &vector) const
    {
        return Vec3(x - vector.x, y - vector.y, z - vector.z);
    }

    Vec3 operator *(const double &value) const
    {
        return Vec3(x * value, y * value, z * value);
    }

    Vec3 operator /(const double &value) const
    {
        float div = 1 / value;
        return Vec3(x * div, y * div, z * div);
    }

    bool operator ==(const Vec3 &vector) const
    {
        return abs(x - vector.x) <= FLT_EPSILON &&
            abs(y - vector.y) <= FLT_EPSILON &&
            abs(z - vector.z) <= FLT_EPSILON;
    }

    bool operator !=(const Vec3 &vector) const
    {
        return abs(x - vector.x) > FLT_EPSILON ||
            abs(y - vector.y) > FLT_EPSILON ||
            abs(z - vector.z) > FLT_EPSILON;
    }

    static Vec3 FromPoint(const geometry_msgs::Point &point)
    {
        return Vec3(point.x, point.y, point.z);
    }
};

#endif // VEC3_H
