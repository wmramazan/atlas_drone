#ifndef VEC3_H
#define VEC3_H

struct Vec3
{
    int x;
    int y;
    int z;

    Vec3()
    {
        x = 0;
        y = 0;
        z = 0;
    }

    Vec3(int x, int y, int z)
    {
        this->x = x;
        this->y = y;
        this->z = z;
    }

    void Set(int x, int y, int z)
    {
        this->x = x;
        this->y = y;
        this->z = z;
    }

    int Distance(const Vec3 &vector) const
    {
        return abs(vector.x - x) + abs(vector.y - y) + abs(vector.z - z);
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
};

#endif // VEC3_H
