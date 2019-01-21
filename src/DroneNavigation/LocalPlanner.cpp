#include "DroneNavigation/LocalPlanner.h"

LocalPlanner::LocalPlanner(NodeHandle& nh)
{
    size = nh.param("/size", 600);
    resolution = nh.param("/resolution", 0.1);
    inflation_radius = nh.param("/inflation_radius", 5);
    local_costmap = Costmap(size, resolution);

    point_cloud_sub = nh.subscribe(nh.param<string>("/filtered_pointcloud_topic", "pointcloud_filtered"), 5, &LocalPlanner::point_cloud_callback, this);
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
            if (!local_costmap.Get(x, y, z))
            {
                for (i = x - inflation_radius; i <= x + inflation_radius; i++)
                {
                    for (j = y - inflation_radius; j <= y + inflation_radius; j++)
                    {
                        for (k = z - inflation_radius; k <= z + inflation_radius; k++)
                        {
                            SetOccupancy(Vec3Int(i, j, k), 1);
                            //SetOccupancy(Vec3Int(k, size-i, size-j), 1);
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
