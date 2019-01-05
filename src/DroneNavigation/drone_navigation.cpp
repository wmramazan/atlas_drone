#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8MultiArray.h>

#include "DroneNavigation/Vec3.h"
#include "DroneNavigation/Vec3Int.h"

#include "DroneNavigation/Costmap.h"

using namespace std;
using namespace ros;
using namespace std_srvs;
using namespace geometry_msgs;
using namespace nav_msgs;
using namespace visualization_msgs;

Subscriber drone_marker_sub;
Subscriber drone_path_sub;
Subscriber drone_position_sub;
Subscriber global_costmap_sub;

Publisher path_marker_array_pub;
Publisher costmap_marker_array_pub;
Publisher target_pose_pub;
Publisher terminal_message_pub;

ServiceClient go_to_target_service_client;
ServiceClient generate_path_service_client;

Trigger trigger_srv;
Pose marker_pose;
Pose start_pose;

MarkerArray marker_array;
MarkerArray vehicle_path_marker_array;
MarkerArray costmap_marker_array;

Marker ground_path_marker;
Marker drone_path_marker;
Marker costmap_marker;
Marker delete_marker;

Costmap* global_costmap;

Vec3Int goal_vector, temp_vector;

std::string frame_id;
double resolution;
double costmap_radius;
int radius;

int id;
bool dirty;

enum MarkerType {
  VEHICLE_PATH_MARKER,
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

        case COSTMAP_MARKER:
            marker = &costmap_marker;
            marker_array = &costmap_marker_array;
            break;
    }

    marker->id = id++;
    marker->pose.position = position;
    marker_array->markers.push_back(*marker);
}

void generate_path_marker_array(Path path)
{
    dirty = true;
    vehicle_path_marker_array.markers.clear();

    for (int i = 0; i < path.poses.size(); i++)
    {
        add_marker(MarkerType::VEHICLE_PATH_MARKER, path.poses[i].pose.position);
    }
}

void generate_costmap_marker_array(Pose origin)
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
                if (global_costmap->Get(temp_vector))
                {
                  bool visible = false;
                  for  (int a = 0; a < 6; a++)
                  {
                     if (!global_costmap->Get(temp_vector + neighbours[a]))
                     {
                         visible = true;
                         break;
                     }
                  }

                  if (visible)
                  {
                    Point position;
                    position.x = global_costmap->ToPosition(temp_vector.x);
                    position.y = global_costmap->ToPosition(temp_vector.y);
                    position.z = global_costmap->ToPosition(temp_vector.z);
                    add_marker(MarkerType::COSTMAP_MARKER, position);
                  }
                }
            }
        }
    }

    costmap_marker_array_pub.publish(costmap_marker_array);
}

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
    generate_path_marker_array(*path);
}

void markerFeedbackCallback(const InteractiveMarkerFeedbackConstPtr &feedback)
{
    switch ( feedback->event_type )
    {
        case InteractiveMarkerFeedback::MENU_SELECT:
            ROS_INFO_STREAM( "menu item " << feedback->menu_entry_id << " clicked." );

            switch ( feedback->menu_entry_id )
            {

                case 1: // Take off
                {
                    std_msgs::String message;
                    message.data = "init";
                    terminal_message_pub.publish(message);
                    break;
                }
                case 5: // Go to target
                {
                    if (go_to_target_service_client.call(trigger_srv))
                    {
                        ROS_INFO("Going to target..");
                    }
                    else
                    {
                        ROS_ERROR("Failed to call service go_to_target_service_client");
                    }
                    break;
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
            generate_costmap_marker_array(feedback->pose);
            break;
    }
}

void global_costmap_callback(const std_msgs::UInt8MultiArray::ConstPtr& msg)
{
    global_costmap->data = *msg;
}

void draw_paths()
{
    if (dirty)
    {
        marker_array.markers.clear();
        marker_array.markers.push_back(delete_marker);

        for (Marker marker : vehicle_path_marker_array.markers)
            marker_array.markers.push_back(marker);

        path_marker_array_pub.publish(marker_array);

        dirty = false;
    }
}

int main(int argc, char **argv)
{
    init(argc, argv, "drone_navigation");
    NodeHandle nh("uav1");

    nh.param<std::string>("/frame_id", frame_id, "world");
    nh.param<double>("/resolution", resolution, 0.1);
    nh.param<double>("/costmap_radius", costmap_radius, 1);

    ROS_INFO("Drone Navigation: %f", resolution);

    drone_path_sub = nh.subscribe<Path>( nh.param<string>("/drone_path_topic", "drone_path"), 10, dronePathCallback );
    drone_marker_sub = nh.subscribe<InteractiveMarkerFeedback>( nh.param<string>("/marker_feedback_topic", "marker/feedback"), 10, markerFeedbackCallback );
    global_costmap_sub = nh.subscribe<std_msgs::UInt8MultiArray>(nh.param<string>("/global_costmap_topic", "global_costmap"), 1, global_costmap_callback);

    path_marker_array_pub = nh.advertise<MarkerArray>( nh.param<string>("/path_marker_array_topic", "path_markers"), 1 );
    costmap_marker_array_pub = nh.advertise<MarkerArray>( nh.param<string>("/costmap_marker_array_topic", "costmap_markers"), 1 );
    target_pose_pub = nh.advertise<Pose> ( nh.param<string>("/target_pose_topic", "target_pose"), 10 );
    terminal_message_pub  = nh.advertise<std_msgs::String>(nh.param<string>("/terminal_message_topic", "drone_ai/terminal_message"), 10);

    go_to_target_service_client = nh.serviceClient<Trigger>( nh.param<string>("/go_to_target_service", "/drone_ai/go_to_target") );
    generate_path_service_client = nh.serviceClient<Trigger>( nh.param<string>("/generate_path_service", "/path_planner/generate_path") );

    id = 0;
    radius = costmap_radius / resolution;

    global_costmap = new Costmap(nh.param("/size", 600), resolution);

    drone_path_marker.header.frame_id = frame_id;
    drone_path_marker.header.stamp = Time();
    drone_path_marker.ns = "drone_path";
    drone_path_marker.type = Marker::SPHERE;
    drone_path_marker.action = Marker::ADD;

    drone_path_marker.scale.x = resolution / 2;
    drone_path_marker.scale.y = resolution / 2;
    drone_path_marker.scale.z = resolution / 2;
    drone_path_marker.color.a = 1.0;
    drone_path_marker.color.r = 1;
    drone_path_marker.color.g = 0;
    drone_path_marker.color.b = 0;

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
