#include "DroneNavigation/NavigationVisualizer.h"

NavigationVisualizer::NavigationVisualizer(NodeHandle& nh, GlobalPlanner* global_planner, LocalPlanner* local_planner)
{
    size = nh.param("/size", 600u);
    resolution = nh.param("/resolution", 0.1);
    radius = nh.param("/inflation_radius", 5u);
    string frame_id = nh.param<std::string>("/frame_id", "world");

    this->global_planner = global_planner;
    this->local_planner  = local_planner;

    vehicle_path_marker   = create_marker(frame_id, "drone_path",     Marker::SPHERE, Marker::ADD, Vec3(resolution / 2, resolution / 2, resolution / 2), 1.0f, Vec3(1, 0, 0));
    costmap_marker        = create_marker(frame_id, "costmap",        Marker::SPHERE, Marker::ADD, Vec3(resolution / 2, resolution / 2, resolution / 2), 0.1f, Vec3(1, 1, 0));
    local_costmap_marker  = create_marker(frame_id, "local_costmap",  Marker::SPHERE, Marker::ADD, Vec3(resolution / 2, resolution / 2, resolution / 2), 0.1f, Vec3(0, 1, 1));
    global_costmap_marker = create_marker(frame_id, "global_costmap", Marker::SPHERE, Marker::ADD, Vec3(resolution / 2, resolution / 2, resolution / 2), 0.1f, Vec3(1, 0, 1));
    delete_marker         = create_marker(frame_id, "delete",         Marker::SPHERE, Marker::DELETEALL, Vec3(0, 0, 0), 0, Vec3(0, 0, 0));

    id = 0;
}

Marker NavigationVisualizer::create_marker(string frame_id, string ns, int type, int action, Vec3 scale, float alpha, Vec3 color)
{
    Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time();
    marker.ns = ns;
    marker.type = type;
    marker.action = action;
    marker.scale.x = scale.x;
    marker.scale.y = scale.y;
    marker.scale.z = scale.z;
    marker.color.a = alpha;
    marker.color.r = static_cast<float>(color.x);
    marker.color.g = static_cast<float>(color.y);
    marker.color.b = static_cast<float>(color.z);
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

    for (uint i = 0; i < path.poses.size(); i++)
    {
        add_marker(MarkerType::VEHICLE_PATH_MARKER, path.poses[i].pose.position);
    }
}

void NavigationVisualizer::generate_costmap_marker_array(Pose origin)
{
    dirty = true;
    costmap_marker_array.markers.clear();
    costmap_marker_array.markers.push_back(delete_marker);

    Vec3Int origin_index = Vec3Int(to_index(origin.position.x), to_index(origin.position.y), to_index(origin.position.z));

    Vec3Int neighbours[] =
    {
        {1, 0, 0},
        {-1, 0, 0},
        {0, 1, 0},
        {0, -1, 0},
        {0, 0, 1},
        {0, 0, -1}
    };

    Vec3Int temp_vector;

    for (uint i = -radius; i <= radius; i++)
    {
        for (uint j = -radius; j <= radius; j++)
        {
            for (uint k = -radius; k <= radius; k++)
            {
                temp_vector = origin_index + Vec3Int(i, j, k);
                if (is_occupied(temp_vector))
                {
                    bool visibleCostmap = false;
                    bool visibleLocalCostmap = false;
                    bool visibleGlobalCostmap = false;

                    for  (int a = 0; a < 6; a++)
                    {
                        if (!visibleCostmap && !is_occupied(temp_vector + neighbours[a]))
                        {
                            visibleCostmap = true;
                        }

                        if (!visibleLocalCostmap && !local_planner->IsOccupied(temp_vector + neighbours[a]))
                        {
                            visibleLocalCostmap = true;
                        }

                        if (!visibleGlobalCostmap && !global_planner->IsOccupied(temp_vector + neighbours[a]))
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
                        position.x = to_position(temp_vector.x);
                        position.y = to_position(temp_vector.y);
                        position.z = to_position(temp_vector.z);
                        add_marker(MarkerType::COSTMAP_MARKER, position);
                    }

                    if (visibleLocalCostmap)
                    {
                        Point position;
                        position.x = to_position(temp_vector.x);
                        position.y = to_position(temp_vector.y);
                        position.z = to_position(temp_vector.z);
                        add_marker(MarkerType::LOCAL_COSTMAP_MARKER, position);
                    }

                    if (visibleGlobalCostmap)
                    {
                        Point position;
                        position.x = to_position(temp_vector.x);
                        position.y = to_position(temp_vector.y);
                        position.z = to_position(temp_vector.z);
                        add_marker(MarkerType::GLOBAL_COSTMAP_MARKER, position);
                    }
                }
            }
        }
    }

    costmap_marker_array_pub.publish(costmap_marker_array);
}
