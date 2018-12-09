#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <std_srvs/Trigger.h>

#include "DroneNavigation/PathPlanner.h"
#include "DroneNavigation/Vec3.h"

#define FRAME_ID "world"
#define RESOLUTION 0.1
#define COSTMAP_RADIUS 5
#define FEEDBACK_TOPIC "/drone_marker/feedback"
#define PATH_MARKER_ARRAY_TOPIC "path_markers"
#define COSTMAP_MARKER_ARRAY_TOPIC "costmap_markers"
#define DRONE_PATH_TOPIC "/drone_path"
#define DRONE_POSITION_TOPIC "/mavros/local_position/pose"
#define NAVIGATION_TARGET_TOPIC "navigation_target"
#define SERVICE_NAME "/drone_ai/go_to_target"

ros::Subscriber drone_marker_sub;
ros::Subscriber drone_path_sub;
ros::Subscriber drone_position_sub;
ros::Publisher path_marker_array_pub;
ros::Publisher costmap_marker_array_pub;
ros::Publisher navigation_pose_pub;
ros::ServiceClient trigger_service_client;

std_srvs::Trigger trigger_srv;
geometry_msgs::Pose marker_pose;
geometry_msgs::Pose start_pose;
nav_msgs::Path* path;

visualization_msgs::MarkerArray marker_array;
visualization_msgs::MarkerArray ground_path_marker_array;
visualization_msgs::MarkerArray vehicle_path_marker_array;
visualization_msgs::MarkerArray costmap_marker_array;

visualization_msgs::Marker ground_path_marker;
visualization_msgs::Marker drone_path_marker;
visualization_msgs::Marker costmap_marker;
visualization_msgs::Marker delete_marker;

PathPlanner* path_planner;
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
    visualization_msgs::Marker* marker;
    visualization_msgs::MarkerArray* marker_array;

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

void findPath(Pose start_pose, Pose target_pose)
{
    path_planner->SetCurrentPose(start_pose);
    path_planner->SetTargetPose(target_pose);

    path = path_planner->GeneratePath();

    if (path != NULL)
    {
        generate_path_marker_array(*path, false);
    }
    else
    {
        ROS_INFO("The path couldn't find.");
    }
}

void dronePathCallback(const nav_msgs::PathConstPtr &path)
{
    generate_path_marker_array(*path, true);
}

void dronePositionCallback(const geometry_msgs::PoseStampedConstPtr &pose)
{
    start_pose.position.x = pose->pose.position.x;
    start_pose.position.y = pose->pose.position.y;
    start_pose.position.z = pose->pose.position.z;
}

void markerFeedbackCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    switch ( feedback->event_type )
    {
        case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
            ROS_INFO_STREAM( "menu item " << feedback->menu_entry_id << " clicked." );

            if (feedback->menu_entry_id == 5)
            {
                if (trigger_service_client.call(trigger_srv))
                {
                    ROS_INFO("Going to target..");
                }
                else
                {
                    ROS_ERROR("Failed to call service go_to_target");
                }
            }
            break;

        case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
            //ROS_INFO_STREAM( "pose update." );
            break;

        case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
            ROS_INFO_STREAM( "mouse down." );
            break;

        case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
            ROS_INFO_STREAM( "mouse up." );

            navigation_pose_pub.publish(feedback->pose);
            findPath(start_pose, feedback->pose);
            generate_costmap_marker_array(feedback->pose);
            break;
    }

}

void draw_paths()
{
    if (dirty)
    {
        marker_array.markers.clear();
        marker_array.markers.push_back(delete_marker);

        for (visualization_msgs::Marker marker : vehicle_path_marker_array.markers)
            marker_array.markers.push_back(marker);

        for (visualization_msgs::Marker marker : ground_path_marker_array.markers)
            marker_array.markers.push_back(marker);

        path_marker_array_pub.publish(marker_array);

        dirty = false;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "drone_navigation");
    ros::NodeHandle nh("~");

    nh.param<std::string>("/frame_id", frame_id, FRAME_ID);
    nh.param<double>("/resolution", resolution, RESOLUTION);
    nh.param<double>("/costmap_radius", costmap_radius, COSTMAP_RADIUS);

    drone_path_sub = nh.subscribe<nav_msgs::Path>( DRONE_PATH_TOPIC, 10, dronePathCallback );
    drone_marker_sub = nh.subscribe<visualization_msgs::InteractiveMarkerFeedback>( FEEDBACK_TOPIC, 10, markerFeedbackCallback );
    drone_position_sub = nh.subscribe<geometry_msgs::PoseStamped>( DRONE_POSITION_TOPIC, 10, dronePositionCallback );
    path_marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>( PATH_MARKER_ARRAY_TOPIC, 1 );
    costmap_marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>( COSTMAP_MARKER_ARRAY_TOPIC, 1 );
    navigation_pose_pub = nh.advertise<geometry_msgs::Pose> ( NAVIGATION_TARGET_TOPIC, 10 );

    trigger_service_client = nh.serviceClient<std_srvs::Trigger>( SERVICE_NAME );

    id = 0;
    radius = costmap_radius / resolution;

    ground_path_marker.header.frame_id = frame_id;
    ground_path_marker.header.stamp = ros::Time();
    ground_path_marker.ns = "ground_path";
    ground_path_marker.type = visualization_msgs::Marker::SPHERE;
    ground_path_marker.action = visualization_msgs::Marker::ADD;

    ground_path_marker.scale.x = resolution / 2;
    ground_path_marker.scale.y = resolution / 2;
    ground_path_marker.scale.z = resolution / 2;
    ground_path_marker.color.a = 1.0;
    ground_path_marker.color.r = 1;
    ground_path_marker.color.g = 0;
    ground_path_marker.color.b = 0;

    drone_path_marker.header.frame_id = frame_id;
    drone_path_marker.header.stamp = ros::Time();
    drone_path_marker.ns = "drone_path";
    drone_path_marker.type = visualization_msgs::Marker::SPHERE;
    drone_path_marker.action = visualization_msgs::Marker::ADD;

    drone_path_marker.scale.x = resolution / 2;
    drone_path_marker.scale.y = resolution / 2;
    drone_path_marker.scale.z = resolution / 2;
    drone_path_marker.color.a = 1.0;
    drone_path_marker.color.r = 0;
    drone_path_marker.color.g = 0;
    drone_path_marker.color.b = 1;

    costmap_marker.header.frame_id = frame_id;
    costmap_marker.header.stamp = ros::Time();
    costmap_marker.ns = "costmap";
    costmap_marker.type = visualization_msgs::Marker::CUBE;
    costmap_marker.action = visualization_msgs::Marker::ADD;

    costmap_marker.scale.x = resolution / 2;
    costmap_marker.scale.y = resolution / 2;
    costmap_marker.scale.z = resolution / 2;
    costmap_marker.color.a = 0.1;
    costmap_marker.color.r = 1;
    costmap_marker.color.g = 1;
    costmap_marker.color.b = 0;

    delete_marker.header.frame_id = frame_id;
    delete_marker.header.stamp = ros::Time();
    delete_marker.type = visualization_msgs::Marker::SPHERE;
    delete_marker.action = visualization_msgs::Marker::DELETEALL;

    marker_pose.orientation.w = 1.0;

    path_planner = new PathPlanner(nh, PathPlanner::Mode::GROUND);

    while (ros::ok())
    {
        ros::spinOnce();
        draw_paths();
    }

    return 0;
}
