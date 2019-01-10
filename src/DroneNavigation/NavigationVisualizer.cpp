#include "DroneNavigation/NavigationVisualizer.h"

NavigationVisualizer::NavigationVisualizer(NodeHandle& nh)
{
    size = nh.param("/size", 600);
    resolution = nh.param("/resolution", 0.1);
    costmap_radius = nh.param("/costmap_radius", 5);
    string frame_id = nh.param<string>("/frame_id", "world");

    vehicle_path_marker   = create_marker(frame_id, "drone_path",     Marker::SPHERE, Marker::ADD, Vec3(resolution / 2, resolution / 2, resolution / 2), 1.0f, Vec3(1, 0, 0));
    vehicle_target_marker = create_marker(frame_id, "drone_target",   Marker::SPHERE, Marker::ADD, Vec3(resolution / 2, resolution / 2, resolution / 2), 1.0f, Vec3(0, 0, 0));
    costmap_marker        = create_marker(frame_id, "costmap",        Marker::CUBE, Marker::ADD, Vec3(resolution, resolution, resolution), 0.5f, Vec3(1, 1, 0));
    local_costmap_marker  = create_marker(frame_id, "local_costmap",  Marker::CUBE, Marker::ADD, Vec3(resolution, resolution, resolution), 0.5f, Vec3(0, 1, 1));
    global_costmap_marker = create_marker(frame_id, "global_costmap", Marker::CUBE, Marker::ADD, Vec3(resolution, resolution, resolution), 0.5f, Vec3(1, 0, 1));
    delete_marker         = create_marker(frame_id, "delete",         Marker::SPHERE, Marker::DELETEALL, Vec3(0, 0, 0), 0, Vec3(0, 0, 0));

    id = 0;
    path_marker_request = 0;
    costmap_marker_request = 0;

    costmap_marker_array_pub = nh.advertise<MarkerArray>(nh.param<string>("/costmap_marker_array_topic" , "/costmap_markers"), 1);

    vehicle_path_marker_array_pub   = nh.advertise<MarkerArray>("drone_path_marker_array", 1);
    vehicle_target_marker_array_pub = nh.advertise<MarkerArray>("drone_target_marker_array", 1);
}

void NavigationVisualizer::PublishTargetMarkers()
{
    vehicle_target_marker_array.markers.clear();
    vehicle_target_marker_array.markers.push_back(delete_marker);

    for (uint k = 0; k < path_planners.size(); k++)
    {
        Point target_position;
        target_position.x = path_planners[k]->drone_target_position.x;
        target_position.y = path_planners[k]->drone_target_position.y;
        target_position.z = path_planners[k]->drone_target_position.z;

        add_marker(MarkerType::VEHICLE_TARGET_MARKER, target_position);
    }

    vehicle_target_marker_array_pub.publish(vehicle_target_marker_array);
}

void NavigationVisualizer::PublishPathMarkers()
{
    vehicle_path_marker_array.markers.clear();
    vehicle_path_marker_array.markers.push_back(delete_marker);

    for (uint k = 0; k < path_planners.size(); k++)
    {
        for (uint i = 0; i < path_planners[k]->path.poses.size(); i++)
        {
            add_marker(MarkerType::VEHICLE_PATH_MARKER, path_planners[k]->path.poses[i].pose.position);
        }
    }

    vehicle_path_marker_array_pub.publish(vehicle_path_marker_array);
}

void NavigationVisualizer::PublishCostmapMarkers(Vec3 origin, MarkerType costmap_type)
{
    //ROS_INFO("PublishCostmapMarkers");
    costmap_marker_array.markers.clear();
    costmap_marker_array.markers.push_back(delete_marker);

    Vec3Int origin_index = Vec3Int(path_planner->to_index(origin.x), path_planner->to_index(origin.y), path_planner->to_index(origin.z));
    //ROS_INFO("origin_index: %d %d %d", origin_index.x, origin_index.y, origin_index.z);

    Vec3Int temp_vector;

    for (int i = -costmap_radius; i <= costmap_radius; i++)
    {
        for (int j = -costmap_radius; j <= costmap_radius; j++)
        {
            for (int k = -costmap_radius; k <= costmap_radius; k++)
            {
                temp_vector = origin_index + Vec3Int(i, j, k);
                //ROS_INFO("temp_vector: %d %d %d", temp_vector.x, temp_vector.y, temp_vector.z);
                if (path_planner->is_occupied(temp_vector, costmap_type))
                {
                    bool visible = false;

                    for  (int a = 0; a < 6; a++)
                    {
                        if (!path_planner->is_occupied(temp_vector + neighbours[a], costmap_type))
                        {
                            visible = true;
                            break;
                        }
                    }

                    if (visible)
                    {
                        Point position;
                        position.x = path_planner->to_position(temp_vector.x);
                        position.y = path_planner->to_position(temp_vector.y);
                        position.z = path_planner->to_position(temp_vector.z);
                        add_marker(costmap_type, position);
                    }
                }
            }
        }
    }

    costmap_marker_array_pub.publish(costmap_marker_array);
}

void NavigationVisualizer::AddPathPlanner(PathPlanner* path_planner)
{
    path_planners.push_back(path_planner);
}

void NavigationVisualizer::SwitchPathPlanner(uint index)
{
    path_planner = path_planners[index];
    global_planner = path_planner->global_planner;
    local_planner = path_planner->local_planner;
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

    return marker;
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

        case VEHICLE_TARGET_MARKER:
            marker = &vehicle_target_marker;
            marker_array = &vehicle_target_marker_array;
            break;

        case COSTMAP_MARKER:
            marker = &costmap_marker;
            marker_array = &costmap_marker_array;
            break;

        case LOCAL_COSTMAP_MARKER:
            marker = &local_costmap_marker;
            marker_array = &costmap_marker_array;
            break;

        case GLOBAL_COSTMAP_MARKER:
            marker = &global_costmap_marker;
            marker_array = &costmap_marker_array;
            break;
    }

    marker->id = id++;
    marker->pose.position = position;
    marker_array->markers.push_back(*marker);
}

void NavigationVisualizer::visualization_request_callback(VisualizationMessage& request)
{
    if (request.request.marker_type > 2)
    {
        PublishPathMarkers();
        PublishTargetMarkers();
    }
    else
    {
        SwitchPathPlanner(request.request.drone_id - 1);
        PublishCostmapMarkers(Vec3::FromPoint(request.request.origin), MarkerType(request.request.marker_type));
    }
}
