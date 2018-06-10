#include "PathPlanner.h"

PathPlanner::PathPlanner(NodeHandle& nh, Mode mode)
{
    this->nh = &nh;
    this->size = nh.param("size", 200);
    this->resolution = nh.param("resolution", 0.1);
    this->inflation_radius = nh.param("inflation_radius", 0.5);
    this->radius = (int) (inflation_radius / resolution);

    this->costmap = new Costmap(size, resolution);
    this->local_costmap = new Costmap(size, resolution);
    this->global_costmap = new Costmap(size, resolution);
    this->pathfinder = new Pathfinder(costmap);

    if (mode | LOCAL_MODE)
    {
        local_costmap_pub   = nh.advertise<std_msgs::UInt8MultiArray>("drone_ai/local_costmap", 1000);
        pointcloud_sub      = nh.subscribe("/camera/depth/points", 5, &PathPlanner::pointcloud_callback, this);
    }
    else
    {
        local_costmap_sub   = nh.subscribe("drone_ai/local_costmap", 5, &PathPlanner::local_costmap_callback, this);
    }

    if (mode | GLOBAL_MODE)
    {
        global_costmap_pub   = nh.advertise<std_msgs::UInt8MultiArray>("drone_ai/global_costmap", 1000);
        occupied_cells_sub   = nh.subscribe("/occupied_cells_vis_array", 5, &PathPlanner::occupied_cells_callback, this);
    }
    else
    {
        global_costmap_sub   = nh.subscribe("drone_ai/global_costmap", 5, &PathPlanner::global_costmap_callback, this);
    }
}

void PathPlanner::GenerateLocalCostmap(const PointCloud::ConstPtr& point_cloud)
{
    local_costmap->Clear();
    cloud_filtered = new PointCloud();

    //Downsampling
    voxel_grid.setInputCloud(point_cloud);
    voxel_grid.setLeafSize(resolution, resolution, resolution);
    voxel_grid.filter(*cloud_filtered);

    BOOST_FOREACH (const pcl::PointXYZ& pt, cloud_filtered->points)
    {
        x = global_costmap->ToIndex(pt.x);
        y = global_costmap->ToIndex(pt.y);
        z = global_costmap->ToIndex(pt.z);

        for (i = x - radius; i < x + radius; i++)
            for (j = y - radius; j < y + radius; j++)
                for (k = z - radius; k < z + radius; k++)
                    local_costmap->Get(i, j, k) = 1;
    }

    delete cloud_filtered;

    local_costmap_pub.publish(local_costmap->data);
}

void PathPlanner::GenerateGlobalCostmap(const MarkerArray::ConstPtr& marker_array)
{
    global_costmap->Clear();

    BOOST_FOREACH (const Marker& marker, marker_array->markers)
    {
        BOOST_FOREACH(const Point& point, marker.points)
        {
            x = global_costmap->ToIndex(point.x);
            y = global_costmap->ToIndex(point.y);
            z = global_costmap->ToIndex(point.z);

            /*
            if (point.x == 1.35 && point.y == 0.05)
            {
              ROS_INFO("Occupied Cell: %lf %lf %lf", point.x, point.y, point.z);
              ROS_INFO("Occupied Cell: %d %d %d", x, y, z);
            }
            */

            global_costmap->Get(x, y, z) = 1;

            // Inflation
            for (i = x - radius; i < x + radius; i++)
                for (j = y - radius; j < y + radius; j++)
                    for (k = z - radius; k < z + radius; k++)
                        global_costmap->Get(i, j, k) = 1;
        }
    }

    global_costmap_pub.publish(global_costmap->data);
}

Path* PathPlanner::GeneratePath()
{
    path.poses.clear();

    costmap->Clear();

    costmap->Merge(*global_costmap);
    //costmap->Merge(*local_costmap);

    Vec3 start;
    start.x = costmap->origin + costmap->offset;
    start.y = costmap->origin + costmap->offset;
    start.z = costmap->origin + costmap->offset;

    Vec3 end;
    end.x = costmap->ToIndex(target_pose.position.x);
    end.y = costmap->ToIndex(target_pose.position.y);
    end.z = costmap->ToIndex(target_pose.position.z);

    vector<Vec3> found_path = pathfinder->Find(start, end);
    if (found_path.size())
    {
        path.poses.resize(found_path.size());
        path.header.frame_id = "map";

        PoseStamped pose;

        for (Vec3 coordinate : found_path)
        {
            pose.pose.position.x = costmap->ToPosition(coordinate.x);
            pose.pose.position.y = costmap->ToPosition(coordinate.y);
            pose.pose.position.z = costmap->ToPosition(coordinate.z);
            path.poses.push_back(pose);
        }

        //TODO: Publish path.
        return &path;
    }
    else
        return NULL;
}

Pose PathPlanner::GetNextPathNode()
{
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
    ROS_INFO("pointCloudCallback");
    GenerateLocalCostmap(msg);
}

void PathPlanner::occupied_cells_callback(const MarkerArray::ConstPtr& msg)
{
    ROS_INFO("occupiedCellsCallback");
    GenerateGlobalCostmap(msg);
}

void PathPlanner::local_costmap_callback(const UInt8MultiArray::ConstPtr& msg)
{
    local_costmap->data = *msg;
}

void PathPlanner::global_costmap_callback(const UInt8MultiArray::ConstPtr& msg)
{
    global_costmap->data = *msg;
}


