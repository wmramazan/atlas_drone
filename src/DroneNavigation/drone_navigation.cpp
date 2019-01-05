#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>

#include <nav_msgs/Path.h>

#include <std_msgs/String.h>
#include <std_msgs/UInt8MultiArray.h>

#include "DroneNavigation/Vec3.h"
#include "DroneNavigation/Vec3Int.h"

#include "DroneNavigation/Costmap.h"
#include "DroneNavigation/PathPlanner.h"

using namespace std;
using namespace ros;
using namespace geometry_msgs;
using namespace nav_msgs;
using namespace visualization_msgs;



Pose marker_pose;
Pose start_pose;



Costmap* global_costmap;

Vec3Int goal_vector, temp_vector;

std::string frame_id;
double resolution;
double costmap_radius;
int radius;



PathPlanner* path_planner;

/*void findPath()
{
    if (!generate_path_service_client.call(trigger_srv))
    {
        ROS_INFO("The path couldn't find.");
    }
}*/

/*
void dronePathCallback(const Path::ConstPtr &path)
{
    ROS_INFO("Path: %d points", path->poses.size());
    generate_path_marker_array(*path);
}*/



/*
void draw_paths()
{
    if (dirty)
    {
        marker_array.markers.clear();
        marker_array.markers.push_back(delete_marker);

        for (Marker marker : vehicle_path_marker_array.markers)
        {
            marker_array.markers.push_back(marker);
        }

        path_marker_array_pub.publish(marker_array);

        dirty = false;
    }
}
*/

int main(int argc, char **argv)
{
    init(argc, argv, "drone_navigation");
    NodeHandle nh("uav1");

    nh.param<std::string>("/frame_id", frame_id, "world");
    nh.param<double>("/resolution", resolution, 0.1);
    nh.param<double>("/costmap_radius", costmap_radius, 1);


    radius = costmap_radius / resolution;

    global_costmap = new Costmap(nh.param("/size", 600), resolution);

    marker_pose.orientation.w = 1.0;

    return 0;
}
