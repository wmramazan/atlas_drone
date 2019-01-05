#include "DroneNavigation/NavigationVisualizer.h"

NavigationVisualizer::NavigationVisualizer(NodeHandle& nh)
{
    size = nh.param("/size", 600);
    resolution = nh.param("/resolution", 0.1);
    radius = nh.param("/inflation_radius", 5);
    string frame_id = nh.param<std::string>("/frame_id", "world");

    vehicle_path_marker.header.frame_id = frame_id;
    vehicle_path_marker.header.stamp = Time();
    vehicle_path_marker.ns = "drone_path";
    vehicle_path_marker.type = Marker::SPHERE;
    vehicle_path_marker.action = Marker::ADD;

    vehicle_path_marker.scale.x = resolution / 2;
    vehicle_path_marker.scale.y = resolution / 2;
    vehicle_path_marker.scale.z = resolution / 2;
    vehicle_path_marker.color.a = 1.0;
    vehicle_path_marker.color.r = 1;
    vehicle_path_marker.color.g = 0;
    vehicle_path_marker.color.b = 0;

    costmap_marker.header.frame_id = frame_id;
    costmap_marker.header.stamp = Time();
    costmap_marker.ns = "costmap";
    costmap_marker.type = Marker::CUBE;
    costmap_marker.action = Marker::ADD;

    costmap_marker.scale.x = resolution / 2;
    costmap_marker.scale.y = resolution / 2;
    costmap_marker.scale.z = resolution / 2;
    costmap_marker.color.a = 0.1;
    costmap_marker.color.r = 1;
    costmap_marker.color.g = 1;
    costmap_marker.color.b = 0;

    delete_marker.header.frame_id = frame_id;
    delete_marker.header.stamp = Time();
    delete_marker.type = Marker::SPHERE;
    delete_marker.action = Marker::DELETEALL;

    id = 0;
}

void NavigationVisualizer::add_marker(MarkerType marker_type, Point position)
{
    Marker* marker;
    MarkerArray* marker_array;

    switch (marker_type)
    {
        case VEHICLE_PATH_MARKER:
            marker = &vehicle_path_marker;
            marker_array = &vehicle_path_marker_array;
            break;

        case COSTMAP_MARKER:
            marker = &costmap_marker;
            marker_array = &costmap_marker_array;
            break;

        case LOCAL_COSTMAP_MARKER:
            marker = &local_costmap_marker;
            marker_array = &local_costmap_marker_array;
            break;

        case GLOBAL_COSTMAP_MARKER:
            marker = &global_costmap_marker;
            marker_array = &global_costmap_marker_array;
            break;
    }

    marker->id = id++;
    marker->pose.position = position;
    marker_array->markers.push_back(*marker);
}

void NavigationVisualizer::generate_path_marker_array(Path path)
{
    dirty = true;
    vehicle_path_marker_array.markers.clear();

    for (int i = 0; i < path.poses.size(); i++)
    {
        add_marker(MarkerType::VEHICLE_PATH_MARKER, path.poses[i].pose.position);
    }
}

/*
void NavigationVisualizer::generate_costmap_marker_array(Pose origin)
{
    dirty = true;
    costmap_marker_array.markers.clear();
    costmap_marker_array.markers.push_back(delete_marker);

    Vec3Int origin_index = Vec3Int(
                global_costmap->ToIndex(origin.position.x),
                global_costmap->ToIndex(origin.position.y),
                global_costmap->ToIndex(origin.position.z)
                );

    Vec3Int neighbours[] =
    {
        {1, 0, 0},
        {-1, 0, 0},
        {0, 1, 0},
        {0, -1, 0},
        {0, 0, 1},
        {0, 0, -1}
    };

    for (int i = -radius; i <= radius; i++)
    {
        for (int j = -radius; j <= radius; j++)
        {
            for (int k = -radius; k <= radius; k++)
            {
                temp_vector = origin_index + Vec3Int(i, j, k);
                if (path_planner->costmap->Get(temp_vector))
                {
                    bool visibleCostmap = false;
                    bool visibleLocalCostmap = false;
                    bool visibleGlobalCostmap = false;

                    for  (int a = 0; a < 6; a++)
                    {
                        if (!visibleCostmap && !path_planner->costmap->Get(temp_vector + neighbours[a]))
                        {
                            visibleCostmap = true;
                        }

                        if (!visibleLocalCostmap && !path_planner->local_costmap->Get(temp_vector + neighbours[a]))
                        {
                            visibleLocalCostmap = true;
                        }

                        if (!visibleGlobalCostmap && !path_planner->global_costmap->Get(temp_vector + neighbours[a]))
                        {
                            visibleGlobalCostmap = true;
                        }

                        if (visibleCostmap && visibleLocalCostmap && visibleGlobalCostmap)
                        {
                            break;
                        }
                    }

                    if (visibleCostmap)
                    {
                        Point position;
                        position.x = path_planner->costmap->ToPosition(temp_vector.x);
                        position.y = path_planner->costmap->ToPosition(temp_vector.y);
                        position.z = path_planner->costmap->ToPosition(temp_vector.z);
                        add_marker(MarkerType::COSTMAP_MARKER, position);
                    }

                    if (visibleLocalCostmap)
                    {
                        Point position;
                        position.x = path_planner->local_costmap->ToPosition(temp_vector.x);
                        position.y = path_planner->local_costmap->ToPosition(temp_vector.y);
                        position.z = path_planner->local_costmap->ToPosition(temp_vector.z);
                        add_marker(MarkerType::LOCAL_COSTMAP_MARKER, position);
                    }

                    if (visibleGlobalCostmap)
                    {
                        Point position;
                        position.x = path_planner->global_costmap->ToPosition(temp_vector.x);
                        position.y = path_planner->global_costmap->ToPosition(temp_vector.y);
                        position.z = path_planner->global_costmap->ToPosition(temp_vector.z);
                        add_marker(MarkerType::GLOBAL_COSTMAP_MARKER, position);
                    }
                }
            }
        }
    }

    costmap_marker_array_pub.publish(costmap_marker_array);
}
*/
