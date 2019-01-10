#include "DroneNavigation/GlobalPlanner.h"

GlobalPlanner::GlobalPlanner(NodeHandle& nh)
{
    size = nh.param("/size", 600);
    resolution = nh.param("/resolution", 0.1);
    inflation_radius = nh.param("/inflation_radius", 5);
    global_costmap = Costmap(size, resolution);

    octomap_sub = nh.subscribe(nh.param<string>("/throttled_octomap_topic", "/octomap_throttled"), 5, &GlobalPlanner::octomap_callback, this);
}

bool GlobalPlanner::IsOccupied(Vec3Int index)
{
    return global_costmap.Get(index) == 1;
}

bool GlobalPlanner::IsPathClear(Path *path)
{
    return global_costmap.IsPathClear(path);
}

uint GlobalPlanner::GetMapSize()
{
    return global_costmap.size;
}

void GlobalPlanner::SetOccupancy(Vec3Int index, uint8_t value)
{
    global_costmap.Get(index) = value;
}

void GlobalPlanner::generate_global_costmap(const Octomap::ConstPtr &octomap)
{
    global_costmap.Clear();

    octree = dynamic_cast<OcTree*>(fullMsgToMap(*octomap));
    occupancy_threshold = octree->getOccupancyThres();

    uint x, y, z;
    uint i, j, k;

    for (OcTree::leaf_iterator it = octree->begin_leafs(), end = octree->end_leafs(); it != end; ++it)
    {
        octree_node = octree->search(it.getKey());
        if (octree_node != nullptr && octree_node->getOccupancy() > occupancy_threshold)
        {
            x = global_costmap.ToIndex(it.getX());
            y = global_costmap.ToIndex(it.getY());
            z = global_costmap.ToIndex(it.getZ());

            // Inflation
            for (i = x - inflation_radius; i <= x + inflation_radius; i++)
            {
                for (j = y - inflation_radius; j <= y + inflation_radius; j++)
                {
                    for (k = z - inflation_radius; k <= z + inflation_radius; k++)
                    {
                        SetOccupancy(Vec3Int(i, j, k), 1);
                    }
                }
            }
        }
    }
}

void GlobalPlanner::octomap_callback(const Octomap::ConstPtr& msg)
{
    generate_global_costmap(msg);
}
