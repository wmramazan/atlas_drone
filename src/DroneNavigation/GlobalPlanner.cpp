#include "DroneNavigation/GlobalPlanner.h"

GlobalPlanner::GlobalPlanner(NodeHandle& nh)
{
    size = nh.param("/size", 600);
    resolution = nh.param("/resolution", 0.1);
    radius = nh.param("/inflation_radius", 5);
    global_costmap = Costmap(size, resolution);

    string octomap_topic = nh.param<std::string>("octomap_topic", "/rtabmap/octomap_full");
    octomap_sub = nh.subscribe(octomap_topic, 5, &GlobalPlanner::octomap_callback, this);
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
            for (i = x - radius; i <= x + radius; i++)
            {
                for (j = y - radius; j <= y + radius; j++)
                {
                    for (k = z - radius; k <= z + radius; k++)
                    {
                        global_costmap.Get(i, j, k) = 1;
                    }
                }
            }
        }
    }
}

void GlobalPlanner::octomap_callback(const Octomap::ConstPtr& msg)
{
    ROS_INFO("octomap_callback");
    generate_global_costmap(msg);
}
