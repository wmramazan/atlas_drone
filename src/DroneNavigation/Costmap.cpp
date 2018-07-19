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

void Costmap::Merge(Costmap* costmap)
{
    bool ok = true;

    size_t i, j;

    // number of threads
    size_t number_threads = NUMBER_THREADS;

    // set of workers
    CostmapMergeJob jobs[NUMBER_THREADS];
    // threads for each worker
    boost::thread* bthread[NUMBER_THREADS];

    // Break the work up into sub work for each thread
    int n = size_cube / number_threads;

    jobs[0].Setup(&data, costmap, 0, n);

    for(i = 1; i < number_threads; i++)
    {
        jobs[i].Setup(&data, costmap, n * i, n * (i + 1));
        bthread[i] = new boost::thread(jobs[i]);
    }

    // do this threads protion of the work
    jobs[0]();

    // wait for other threads to finish
    for(i = 1; i < number_threads; i++)
    {     bthread[i]->join();
        delete bthread[i];
    }

    //for (int i = 0; i < size_cube; i++)
    //    data.data[i] |= costmap.data.data[i];
}


void Costmap::Clear()
{
    data.data.clear();
    data.data.resize(size_cube);
}

bool Costmap::CanPathPass(Path* path)
{
  for (int i = 0; i < path->poses.size(); i++)
  {
    Vec3Int position(ToIndex(path->poses[i].pose.position.x), ToIndex(path->poses[i].pose.position.y), ToIndex(path->poses[i].pose.position.z));
    if (Get(position) != 0)
      return false;
  }

  return true;
}

