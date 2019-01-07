#include "DroneNavigation/LocalPlanner.h"

LocalPlanner::LocalPlanner(NodeHandle& nh, string ns)
{
    size = nh.param("/size", 600);
    resolution = nh.param("/resolution", 0.1);
    radius = nh.param("/inflation_radius", 5);
    local_costmap = Costmap(size, resolution);

    point_cloud_sub = nh.subscribe(ns + nh.param<string>("/filtered_pointcloud_topic", "pointcloud_filtered"), 5, &LocalPlanner::point_cloud_callback, this);
}

bool LocalPlanner::IsOccupied(Vec3Int index)
{
    return local_costmap.Get(index) == 1;
}

bool LocalPlanner::IsPathClear(Path *path)
{
    return local_costmap.IsPathClear(path);
}

uint LocalPlanner::GetMapSize()
{
    return local_costmap.size;
}

void LocalPlanner::SetOccupancy(Vec3Int index, uint8_t value)
{
    local_costmap.Get(index) = value;
}

void LocalPlanner::generate_local_costmap(const PointCloud::ConstPtr& point_cloud)
{
    local_costmap.Clear();

    uint x, y, z;
    uint i, j, k;

    BOOST_FOREACH (const pcl::PointXYZ& pt, point_cloud->points)
    {
        if (!isnan(pt.x))
        {
            x = local_costmap.ToIndex(static_cast<double>(pt.x));
            y = local_costmap.ToIndex(static_cast<double>(pt.y));
            z = local_costmap.ToIndex(static_cast<double>(pt.z));

            // Inflation
            if (!local_costmap.Get(z, size - x, size - y))
            {
                for (i = x - radius; i <= x + radius; i++)
                {
                    for (j = y - radius; j <= y + radius; j++)
                    {
                        for (k = z - radius; k <= z + radius; k++)
                        {
                            SetOccupancy(Vec3Int(k, size-i, size-j), 1);
                        }
                    }
                }
            }
        }
    }
}

void LocalPlanner::point_cloud_callback(const PointCloud::ConstPtr& msg)
{
    generate_local_costmap(msg);
}
