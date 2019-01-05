#ifndef NAVIGATIONVISUALIZER_H
#define NAVIGATIONVISUALIZER_H

#include <ros/ros.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <DroneNavigation/GlobalPlanner.h>
#include <DroneNavigation/LocalPlanner.h>
#include <DroneNavigation/Vec3.h>
#include <DroneNavigation/Vec3Int.h>

using namespace std;
using namespace ros;
using namespace visualization_msgs;
using namespace geometry_msgs;
using namespace nav_msgs;

enum MarkerType
{
    COSTMAP_MARKER          = 0,
    LOCAL_COSTMAP_MARKER    = 1,
    GLOBAL_COSTMAP_MARKER   = 2,
    VEHICLE_PATH_MARKER     = 3
};

class NavigationVisualizer
{
public:
    NavigationVisualizer(NodeHandle& nh, GlobalPlanner* global_planner, LocalPlanner* local_planner);
    void PublishCostmapMarkers(Vec3 origin, MarkerType type);

private:
    Marker create_marker(string frame_id, string ns, int type, int action, Vec3 scale, float alpha, Vec3 color);
    void add_marker(MarkerType marker_type, Point position);
    void generate_path_marker_array(Path path);

    uint to_index(double value)
    {
        return (uint) (((value) / resolution) + size / 2 + 1);
    }

    double to_position(int value)
    {
        return (double) (value - size / 2) * resolution - resolution / 2;
    }

    bool is_occupied(Vec3Int index, int type)
    {
        switch  (type)
        {
            case 0:
                return global_planner->IsOccupied(index) || local_planner->IsOccupied(index);
            case 1:
                return local_planner->IsOccupied(index);
            case 2:
                return global_planner->IsOccupied(index);
        }
    }

    Subscriber drone_1_path_sub;
    Subscriber drone_2_path_sub;
    Subscriber drone_3_path_sub;

    Publisher vehicle_path_marker_array_pub;
    Publisher costmap_marker_array_pub;
    Publisher global_costmap_marker_array_pub;
    Publisher local_costmap_marker_array_pub;

    MarkerArray marker_array;
    MarkerArray vehicle_path_marker_array;
    MarkerArray costmap_marker_array;
    MarkerArray local_costmap_marker_array;
    MarkerArray global_costmap_marker_array;

    Marker vehicle_path_marker;
    Marker costmap_marker;
    Marker local_costmap_marker;
    Marker global_costmap_marker;
    Marker delete_marker;

    Path drone_1_path;
    Path drone_2_path;
    Path drone_3_path;

    GlobalPlanner* global_planner;
    LocalPlanner*  local_planner;

    uint size;
    double resolution;
    uint radius;

    int id;
    bool dirty;
};

#endif // NAVIGATIONVISUALIZER_H
