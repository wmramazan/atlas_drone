#include "DroneNavigation/PathPlanner.h"
#include "ros/ros.h"

PathPlanner::PathPlanner(NodeHandle& nh, Mode mode)
{
    this->nh = &nh;
    this->mode = mode;

    this->size = nh.param("size", 600);
    this->resolution = nh.param("resolution", 0.1);
    this->radius = nh.param("inflation_radius", 5);

    nh.param<std::string>("frame_id", frame_id, "map");

    nh.param<std::string>("local_costmap_topic", local_costmap_topic ,"/local_costmap");
    nh.param<std::string>("pointcloud_topic", pointcloud_topic, "/point_cloud_filter/filtered_cloud");

    nh.param<std::string>("global_costmap_topic", global_costmap_topic, "/global_costmap");
    nh.param<std::string>("octomap_topic", octomap_topic, "/octomap_throttled");

    this->costmap = new Costmap(size, resolution);
    this->local_costmap = new Costmap(size, resolution);
    this->global_costmap = new Costmap(size, resolution);
    this->pathfinder = new Pathfinder(costmap);

    switch (mode)
    {
        case Mode::GROUND:
            global_costmap_pub  = nh.advertise<std_msgs::UInt8MultiArray>(global_costmap_topic, 10);
            local_costmap_sub  = nh.subscribe(local_costmap_topic, 5, &PathPlanner::local_costmap_callback, this);
            octomap_sub         = nh.subscribe(octomap_topic, 5, &PathPlanner::octomap_callback, this);
            nh.param<std::string>("ground_path_topic", path_topic, "ground_path");
            break;
        case Mode::VEHICLE:
            local_costmap_pub   = nh.advertise<std_msgs::UInt8MultiArray>(local_costmap_topic, 10);
            global_costmap_sub  = nh.subscribe(global_costmap_topic, 5, &PathPlanner::global_costmap_callback, this);
            pointcloud_sub      = nh.subscribe(pointcloud_topic, 5, &PathPlanner::pointcloud_callback, this);
            nh.param<std::string>("drone_path_topic", path_topic, "drone_path");
            break;
    }

    path_pub = nh.advertise<Path>(path_topic, 5);

}

void PathPlanner::GenerateLocalCostmap(const PointCloud::ConstPtr& point_cloud)
{
    local_costmap->Clear();

    uint x, y, z;
    uint i, j, k;

   BOOST_FOREACH (const pcl::PointXYZ& pt, point_cloud->points)
    {   
       if (!isnan(pt.x))
       {
           x = local_costmap->ToIndex(pt.x);
           y = local_costmap->ToIndex(pt.y);
           z = local_costmap->ToIndex(pt.z);

           //ROS_INFO("%lf %lf %lf", pt.x, pt.y, pt.z);
           //ROS_INFO("%d %d %d", x, y, z);
           if (!local_costmap->Get(z, size - x, size - y))
             for (i = x - radius; i <= x + radius; i++)
                 for (j = y - radius; j <= y + radius; j++)
                     for (k = z - radius; k <= z + radius; k++)
                         local_costmap->Get(k, size - i, size - j) = 1;
       }
    }

    costmap->Merge(local_costmap);
    local_costmap_pub.publish(local_costmap->data);
}

void PathPlanner::GenerateGlobalCostmap(const Octomap::ConstPtr& octomap)
{
    costmap->Clear();
    global_costmap->Clear();

    octree = dynamic_cast<OcTree*>(fullMsgToMap(*octomap));
    occupancy_threshold = octree->getOccupancyThres();

    //ROS_INFO("%lf", occupancy_threshold);

    uint x, y, z;
    uint i, j, k;

    for (OcTree::leaf_iterator it = octree->begin_leafs(), end = octree->end_leafs(); it != end; ++it)
    {
        octree_node = octree->search(it.getKey());
        if (NULL != octree_node && octree_node->getOccupancy() > occupancy_threshold)
        {
          x = global_costmap->ToIndex(it.getX());
          y = global_costmap->ToIndex(it.getY());
          z = global_costmap->ToIndex(it.getZ());

          //ROS_INFO("octomap: %lf %lf %lf", it.getX(), it.getY(), it.getZ());
          //ROS_INFO("costmap: %d %d %d", x, y, z);

          // Inflation
          for (i = x - radius; i <= x + radius; i++)
              for (j = y - radius; j <= y + radius; j++)
                  for (k = z - radius; k <= z + radius; k++)
                      global_costmap->Get(i, j, k) = 1;
        }

    }

    costmap->Merge(global_costmap);
    global_costmap_pub.publish(global_costmap->data);
}

Path* PathPlanner::GeneratePath()
{
    path.poses.clear();

    Vec3Int start;
    start.x = costmap->ToIndex(current_pose.position.x);
    start.y = costmap->ToIndex(current_pose.position.y);
    start.z = costmap->ToIndex(current_pose.position.z);

    ROS_INFO("Generating path from %d %d %d %lf %lf %lf", start.x, start.y, start.z, current_pose.position.x, current_pose.position.y, current_pose.position.z);

    Vec3Int end;
    end.x = costmap->ToIndex(target_pose.position.x);
    end.y = costmap->ToIndex(target_pose.position.y);
    end.z = costmap->ToIndex(target_pose.position.z);

    ROS_INFO("to %d %d %d %lf %lf %lf", end.x, end.y, end.z, target_pose.position.x, target_pose.position.y, target_pose.position.z);

    vector<Vec3Int> found_path = pathfinder->Find(start, end);
    if (found_path.size())
    {
        path.header.frame_id = frame_id;

        PoseStamped pose;

        for (Vec3Int coordinate : found_path)
        {
            pose.pose.position.x = costmap->ToPosition(coordinate.x);
            pose.pose.position.y = costmap->ToPosition(coordinate.y);
            pose.pose.position.z = costmap->ToPosition(coordinate.z);
            //ROS_INFO("Coordinate: %d %d %d  %lf %lf %lf", coordinate.x, coordinate.y, coordinate.z, pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
            path.poses.push_back(pose);
        }

        path_pub.publish(path);

        return &path;
    }
    else
        return NULL;
}

bool PathPlanner::IsPathClear()
{
    return costmap->CanPathPass(&path);
}

Pose PathPlanner::GetNextPathNode()
{
    // TODO: Move implementation from NavigationBehaviour
    return path.poses[0].pose;
}

void PathPlanner::SetCurrentPose(Pose current_pose)
{
    this->current_pose = current_pose;
}

void PathPlanner::SetTargetPose(Pose target_pose)
{
    this->target_pose = target_pose;
}

void PathPlanner::pointcloud_callback(const PointCloud::ConstPtr& msg)
{
    //ROS_INFO("pointCloudCallback");
    GenerateLocalCostmap(msg);
}

void PathPlanner::octomap_callback(const Octomap::ConstPtr& msg)
{
    //ROS_INFO("octomapCallback");
    GenerateGlobalCostmap(msg);
}

void PathPlanner::local_costmap_callback(const UInt8MultiArray::ConstPtr& msg)
{
    local_costmap->data = *msg;
    costmap->Merge(local_costmap);
}

void PathPlanner::global_costmap_callback(const UInt8MultiArray::ConstPtr& msg)
{
    costmap->Clear();
    global_costmap->data = *msg;
    costmap->Merge(global_costmap);
}


