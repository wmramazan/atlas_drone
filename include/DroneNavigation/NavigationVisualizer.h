#ifndef NAVIGATIONVISUALIZER_H
#define NAVIGATIONVISUALIZER_H

#include <ros/ros.h>

#include <vector>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <DroneNavigation/PathPlanner.h>
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
    NavigationVisualizer(NodeHandle& nh);

    void PublishPathMarkers();
    void PublishCostmapMarkers(Vec3 origin, int path_planner, MarkerType costmap_type);
    void AddPathPlanner(PathPlanner* path_planner);
    void SwitchPathPlanner(uint index);

private:
    Marker create_marker(string frame_id, string ns, int type, int action, Vec3 scale, float alpha, Vec3 color);
    void add_marker(MarkerType marker_type, Point position);
    void drone_1_path_callback(const PathConstPtr &path);

    uint to_index(double value);
    double to_position(int value);
    bool is_occupied(Vec3Int index, int type);

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

    vector<PathPlanner*> path_planners;
    GlobalPlanner* global_planner;
    LocalPlanner*  local_planner;

    uint size;
    double resolution;
    uint radius;

    int id;
    bool dirty;
};

#endif // NAVIGATIONVISUALIZER_H
