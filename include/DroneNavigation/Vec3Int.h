#ifndef VEC3INT_H
#define VEC3INT_H

struct Vec3Int
{
    int x;
    int y;
    int z;

    Vec3Int()
    {
        x = 0;
        y = 0;
        z = 0;
    }

    Vec3Int(int x, int y, int z)
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

    int Distance(const Vec3Int &vector) const
    {
        return abs(vector.x - x) + abs(vector.y - y) + abs(vector.z - z);
    }

    Vec3Int operator +(const Vec3Int &vector) const
    {
        return Vec3Int(x + vector.x, y + vector.y, z + vector.z);
    }

    Vec3Int operator -(const Vec3Int &vector) const
    {
        return Vec3Int(x - vector.x, y - vector.y, z - vector.z);
    }

    bool operator ==(const Vec3Int &vector) const
    {
        return x == vector.x && y == vector.y && z == vector.z;
    }

    bool operator !=(const Vec3Int &vector) const
    {
        return x != vector.x || y != vector.y || z != vector.z;
    }

    bool operator <(const Vec3Int &vector) const
    {
        if ((z < vector.z)                                      ) {return true;}
        if ((z == vector.z) && (y < vector.y)                   ) {return true;}
        if ((z == vector.z) && (y == vector.y) && (x < vector.x)) {return true;}

        return false;
    }
};

#endif // VEC3INT_H
