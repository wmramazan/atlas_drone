#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <std_srvs/Trigger.h>

#include "DroneNavigation/Vec3.h"
#include "DroneNavigation/Vec3Int.h"

#define FRAME_ID "world"
#define RESOLUTION 0.25
#define COSTMAP_RADIUS 5
#define FEEDBACK_TOPIC "/uav1_marker/feedback"
#define PATH_MARKER_ARRAY_TOPIC "path_markers"
#define COSTMAP_MARKER_ARRAY_TOPIC "costmap_markers"
#define DRONE_PATH_TOPIC "/path_planner/drone_path"
#define DRONE_POSITION_TOPIC "/mavros/local_position/pose"
#define TARGET_POSE_TOPIC "/target_pose"
#define GO_TO_TARGET_SERVICE "/drone_ai/go_to_target"
#define GENERATE_PATH_SERVICE "/path_planner/generate_path"

using namespace ros;
using namespace std_srvs;
using namespace geometry_msgs;
using namespace nav_msgs;
using namespace visualization_msgs;

Subscriber drone_marker_sub;
Subscriber drone_path_sub;
Subscriber drone_position_sub;
Publisher path_marker_array_pub;
Publisher costmap_marker_array_pub;
Publisher target_pose_pub;
ServiceClient go_to_target_service_client;
ServiceClient generate_path_service_client;

Trigger trigger_srv;
Pose marker_pose;
Pose start_pose;

MarkerArray marker_array;
MarkerArray ground_path_marker_array;
MarkerArray vehicle_path_marker_array;
MarkerArray costmap_marker_array;

Marker ground_path_marker;
Marker drone_path_marker;
Marker costmap_marker;
Marker delete_marker;

Vec3Int goal_vector, temp_vector;

std::string frame_id;
double resolution;
double costmap_radius;
int radius;

int id;
bool dirty;

enum MarkerType {
  VEHICLE_PATH_MARKER,
  GROUND_PATH_MARKER,
  COSTMAP_MARKER
};

void add_marker(MarkerType marker_type, Point position)
{
    Marker* marker;
    MarkerArray* marker_array;

    switch (marker_type)
    {
        case VEHICLE_PATH_MARKER:
            marker = &drone_path_marker;
            marker_array = &vehicle_path_marker_array;
            break;

        case GROUND_PATH_MARKER:
            marker = &ground_path_marker;
            marker_array = &ground_path_marker_array;
            break;

        case COSTMAP_MARKER:
            marker = &costmap_marker;
            marker_array = &costmap_marker_array;
            break;
    }

    marker->id = id++;
    marker->pose.position = position;
    marker_array->markers.push_back(*marker);
}

void generate_path_marker_array(Path path, bool is_vehicle)
{
    dirty = true;
    is_vehicle ? vehicle_path_marker_array.markers.clear() : ground_path_marker_array.markers.clear();

    for (int i = 0; i < path.poses.size(); i++)
    {
        add_marker(is_vehicle ? MarkerType::VEHICLE_PATH_MARKER : MarkerType::GROUND_PATH_MARKER, path.poses[i].pose.position);
    }
}

/*
void generate_costmap_marker_array(Pose origin)
{
    dirty = true;
    costmap_marker_array.markers.clear();
    costmap_marker_array.markers.push_back(delete_marker);

    Vec3Int origin_index = Vec3Int(
                path_planner->costmap->ToIndex(origin.position.x),
                path_planner->costmap->ToIndex(origin.position.y),
                path_planner->costmap->ToIndex(origin.position.z)
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
                  bool visible = false;
                  for  (int a = 0; a < 6; a++)
                  {
                     if (!path_planner->costmap->Get(temp_vector + neighbours[a]))
                     {
                         visible = true;
                         break;
                     }
                  }

                  if (visible)
                  {
                    Point position;
                    position.x = path_planner->costmap->ToPosition(temp_vector.x);
                    position.y = path_planner->costmap->ToPosition(temp_vector.y);
                    position.z = path_planner->costmap->ToPosition(temp_vector.z);
                    add_marker(MarkerType::COSTMAP_MARKER, position);
                  }
                }
            }
        }
    }

    costmap_marker_array_pub.publish(costmap_marker_array);
}
*/

void findPath()
{
    if (!generate_path_service_client.call(trigger_srv))
    {
        ROS_INFO("The path couldn't find.");
    }
}

void dronePathCallback(const Path::ConstPtr &path)
{
    ROS_INFO("Path: %d points", path->poses.size());
    generate_path_marker_array(*path, false);
}

void markerFeedbackCallback(const InteractiveMarkerFeedbackConstPtr &feedback)
{
    switch ( feedback->event_type )
    {
        case InteractiveMarkerFeedback::MENU_SELECT:
            ROS_INFO_STREAM( "menu item " << feedback->menu_entry_id << " clicked." );

            if (feedback->menu_entry_id == 5)
            {
                if (go_to_target_service_client.call(trigger_srv))
                {
                    ROS_INFO("Going to target..");
                }
                else
                {
                    ROS_ERROR("Failed to call service go_to_target_service_client");
                }
            }
            break;

        case InteractiveMarkerFeedback::POSE_UPDATE:
            //ROS_INFO_STREAM( "pose update." );
            break;

        case InteractiveMarkerFeedback::MOUSE_DOWN:
            ROS_INFO_STREAM( "mouse down." );
            break;

        case InteractiveMarkerFeedback::MOUSE_UP:
            ROS_INFO_STREAM( "mouse up." );

            target_pose_pub.publish(feedback->pose);
            //findPath();
            //generate_costmap_marker_array(feedback->pose);
            break;
    }
}

void draw_paths()
{
    if (dirty)
    {
        marker_array.markers.clear();
        marker_array.markers.push_back(delete_marker);

        for (Marker marker : vehicle_path_marker_array.markers)
            marker_array.markers.push_back(marker);

        for (Marker marker : ground_path_marker_array.markers)
            marker_array.markers.push_back(marker);

        path_marker_array_pub.publish(marker_array);

        dirty = false;
    }
}

int main(int argc, char **argv)
{
    init(argc, argv, "drone_navigation");
    NodeHandle nh("~");

    nh.param<std::string>("/frame_id", frame_id, FRAME_ID);
    nh.param<double>("/resolution", resolution, RESOLUTION);
    nh.param<double>("/costmap_radius", costmap_radius, COSTMAP_RADIUS);

    drone_path_sub = nh.subscribe<Path>( DRONE_PATH_TOPIC, 10, dronePathCallback );
    drone_marker_sub = nh.subscribe<InteractiveMarkerFeedback>( FEEDBACK_TOPIC, 10, markerFeedbackCallback );
    path_marker_array_pub = nh.advertise<MarkerArray>( PATH_MARKER_ARRAY_TOPIC, 1 );
    costmap_marker_array_pub = nh.advertise<MarkerArray>( COSTMAP_MARKER_ARRAY_TOPIC, 1 );
    target_pose_pub = nh.advertise<Pose> ( TARGET_POSE_TOPIC, 10 );

    go_to_target_service_client = nh.serviceClient<Trigger>( GO_TO_TARGET_SERVICE );
    generate_path_service_client = nh.serviceClient<Trigger>( GENERATE_PATH_SERVICE );

    id = 0;
    radius = costmap_radius / resolution;

    ground_path_marker.header.frame_id = frame_id;
    ground_path_marker.header.stamp = Time();
    ground_path_marker.ns = "ground_path";
    ground_path_marker.type = Marker::SPHERE;
    ground_path_marker.action = Marker::ADD;

    ground_path_marker.scale.x = resolution / 2;
    ground_path_marker.scale.y = resolution / 2;
    ground_path_marker.scale.z = resolution / 2;
    ground_path_marker.color.a = 1.0;
    ground_path_marker.color.r = 1;
    ground_path_marker.color.g = 0;
    ground_path_marker.color.b = 0;

    drone_path_marker.header.frame_id = frame_id;
    drone_path_marker.header.stamp = Time();
    drone_path_marker.ns = "drone_path";
    drone_path_marker.type = Marker::SPHERE;
    drone_path_marker.action = Marker::ADD;

    drone_path_marker.scale.x = resolution / 2;
    drone_path_marker.scale.y = resolution / 2;
    drone_path_marker.scale.z = resolution / 2;
    drone_path_marker.color.a = 1.0;
    drone_path_marker.color.r = 0;
    drone_path_marker.color.g = 0;
    drone_path_marker.color.b = 1;

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

    marker_pose.orientation.w = 1.0;

    while (ok())
    {
        spinOnce();
        draw_paths();
    }

    return 0;
}
