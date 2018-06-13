#include "DroneNavigation/Costmap.h"

Costmap::Costmap(uint size, double resolution)
{
    this->size = size;
    this->size_square = size * size;
    this->size_cube = size_square * size;
    this->origin = size / 2;
    this->resolution = resolution;
    this->offset = resolution / 2;

    data.layout.dim.resize(3);

    data.layout.dim[0].label = "depth";
    data.layout.dim[0].size = size;
    data.layout.dim[0].stride = size_cube;

    data.layout.dim[1].label = "height";
    data.layout.dim[1].size = size;
    data.layout.dim[1].stride = size_square;

    data.layout.dim[2].label = "width";
    data.layout.dim[2].size = size;
    data.layout.dim[2].stride = size;

    data.data.resize(size * size_square);
}

Costmap::Costmap(UInt8MultiArray& data, double resolution)
{
    this->data = data;
    this->resolution = resolution;

    this->size = this->data.layout.dim[0].size;
    this->size_square = size * size;
    this->size_cube = size_square * size;
    this->origin = size / 2;
}

uint8_t& Costmap::Get(Vec3Int position)
{
    return data.data[position.x + position.y * size + position.z * size_square];
}

uint8_t& Costmap::Get(uint x, uint y, uint z)
{
    return data.data[x + y * size + z * size_square];
}

uint8_t& Costmap::Get(double x, double y, double z)
{
    return data.data[ToIndex(x) + ToIndex(y) * size + ToIndex(z) * size_square];
}

int Costmap::ToIndex(double value)
{
    return ((int) ((value + offset) / resolution)) + origin;
}

double Costmap::ToPosition(int value)
{
    return (double) (value - origin) * resolution - offset;
}

void Costmap::Merge(Costmap& costmap)
{
    for (int i = 0; i < size_cube; i++)
        data.data[i] |= costmap.data.data[i];
}

void Costmap::Clear()
{
    data.data.clear();
    data.data.resize(size_cube);
}

