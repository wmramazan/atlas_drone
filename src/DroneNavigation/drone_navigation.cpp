#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <std_srvs/Trigger.h>

#include "DroneNavigation/PathPlanner.h"

#define FRAME_ID "map"
#define RESOLUTION 0.1
#define COSTMAP_RADIUS 2.0
#define FEEDBACK_TOPIC "/drone_marker/feedback"
#define MARKER_ARRAY_TOPIC "markers"
#define DRONE_PATH_TOPIC "/drone_path"
#define SERVICE_NAME "/drone_ai/go_to_target"
#define DURATION 0.5

ros::Subscriber drone_marker_sub;
ros::Subscriber drone_path_sub;
ros::Publisher marker_array_pub;
ros::Publisher navigation_pose_pub;
ros::ServiceClient trigger_service_client;

std_srvs::Trigger trigger_srv;
geometry_msgs::Pose marker_pose;
geometry_msgs::Pose start_pose;
geometry_msgs::Pose goal_pose;
nav_msgs::Path* path;
visualization_msgs::MarkerArray marker_array;
visualization_msgs::Marker drone_path_marker;
visualization_msgs::Marker ground_path_marker;
visualization_msgs::Marker costmap_marker;
visualization_msgs::Marker delete_marker;

ros::Time last_pose_update;
ros::Duration duration(DURATION);

PathPlanner* path_planner;
Vec3Int goal_vector, temp_vector;

std::string frame_id;
double resolution;
double costmap_radius;
int radius;

int id;
int i, j, k;

void addDronePathMarker()
{
    drone_path_marker.id = id++;
    drone_path_marker.pose = marker_pose;
    marker_array.markers.push_back(drone_path_marker);
}

void addGroundPathMarker()
{
    ground_path_marker.id = id++;
    ground_path_marker.pose = marker_pose;
    marker_array.markers.push_back(ground_path_marker);
}

void addCostmapMarker()
{
    costmap_marker.id = id++;
    costmap_marker.pose = marker_pose;
    marker_array.markers.push_back(costmap_marker);
}

void findPath()
{
    marker_array.markers.clear();
    marker_array.markers.push_back(delete_marker);

    //TODO: Read transform of drone instead of start pose.
    path_planner->SetCurrentPose(start_pose);
    path_planner->SetTargetPose(goal_pose);

    path = path_planner->GeneratePath();

    if (path != NULL)
    {
        ROS_INFO("Path Size: %lu", path->poses.size());

        for (i = 0; i < path->poses.size(); i++)
        {
            marker_pose.position.x = path->poses[i].pose.position.x;
            marker_pose.position.y = path->poses[i].pose.position.y;
            marker_pose.position.z = path->poses[i].pose.position.z;

            addGroundPathMarker();
        }

        marker_array_pub.publish(marker_array);
    }
    else
    {
        ROS_INFO("The path couldn't find.");
    }
}

void dronePathCallback(const nav_msgs::PathConstPtr &path)
{
    for (i = 0; i < path->poses.size(); i++)
    {
        marker_pose.position.x = path->poses[i].pose.position.x;
        marker_pose.position.y = path->poses[i].pose.position.y;
        marker_pose.position.z = path->poses[i].pose.position.z;

        addDronePathMarker();
    }
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
            if (ros::Time::now() - last_pose_update > duration)
            {
                last_pose_update = ros::Time::now();
                findPath();
            }

            goal_pose = feedback->pose;
            break;

        case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
            ROS_INFO_STREAM( "mouse down." );
            break;

        case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
            ROS_INFO_STREAM( "mouse up." );
            //ROS_INFO("Goal Pose: %lf %lf %lf", goal_pose.position.x, goal_pose.position.y, goal_pose.position.z);

            navigation_pose_pub.publish(goal_pose);

            findPath();

            goal_vector = Vec3Int(
                        path_planner->costmap->ToIndex(goal_pose.position.x),
                        path_planner->costmap->ToIndex(goal_pose.position.y),
                        path_planner->costmap->ToIndex(goal_pose.position.z)
                        );

            /*
            j = 0;
            for (i = 0; i < path_planner->costmap->size_cube; i++)
            {
                if (path_planner->costmap->data.data[i])
                   j++;
            }
            ROS_INFO("Occupied cells: %d", j);
            */

            for (i = -radius; i < radius; i++)
            {
                for (j = -radius; j < radius; j++)
                {
                    for (k = -radius; k < radius; k++)
                    {
                        temp_vector = goal_vector + Vec3Int(i, j, k);
                        if (path_planner->costmap->Get(temp_vector))
                        {
                            marker_pose.position.x = path_planner->costmap->ToPosition(temp_vector.x);
                            marker_pose.position.y = path_planner->costmap->ToPosition(temp_vector.y);
                            marker_pose.position.z = path_planner->costmap->ToPosition(temp_vector.z);
                            ROS_INFO("Marker Pose: %lf %lf %lf", marker_pose.position.x, marker_pose.position.y, marker_pose.position.z);
                            addCostmapMarker();
                        }

                    }
                }
            }

            marker_array_pub.publish(marker_array);
            break;
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
    marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>( MARKER_ARRAY_TOPIC, 1 );
    navigation_pose_pub = nh.advertise<geometry_msgs::Pose> ( "navigation_target", 10 );

    trigger_service_client = nh.serviceClient<std_srvs::Trigger>( SERVICE_NAME );

    id = 0;
    radius = costmap_radius / resolution;

    ground_path_marker.header.frame_id = frame_id;
    ground_path_marker.header.stamp = ros::Time();
    ground_path_marker.ns = "path";
    ground_path_marker.type = visualization_msgs::Marker::SPHERE;
    ground_path_marker.action = visualization_msgs::Marker::ADD;

    ground_path_marker.scale.x = resolution;
    ground_path_marker.scale.y = resolution;
    ground_path_marker.scale.z = resolution;
    ground_path_marker.color.a = 1.0;
    ground_path_marker.color.r = 1;
    ground_path_marker.color.g = 0;
    ground_path_marker.color.b = 0;

    drone_path_marker = ground_path_marker;
    drone_path_marker.color.r = 0;
    drone_path_marker.color.b = 1;

    costmap_marker.header.frame_id = frame_id;
    costmap_marker.header.stamp = ros::Time();
    costmap_marker.ns = "costmap";
    costmap_marker.type = visualization_msgs::Marker::CUBE;
    costmap_marker.action = visualization_msgs::Marker::ADD;

    costmap_marker.scale.x = resolution;
    costmap_marker.scale.y = resolution;
    costmap_marker.scale.z = resolution;
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
    last_pose_update = ros::Time::now();

    ros::spin();

    return 0;
}
