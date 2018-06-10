#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>

#include "PathPlanner.h"

#define FRAME_ID "map"
#define RESOLUTION 0.1
#define COSTMAP_RADIUS 3
#define FEEDBACK_TOPIC "/drone_marker/feedback"
#define MARKER_ARRAY_TOPIC "markers"

ros::Subscriber drone_marker_sub;
ros::Publisher marker_array_pub;

geometry_msgs::Pose marker_pose;
geometry_msgs::Pose start_pose;
geometry_msgs::Pose goal_pose;
nav_msgs::Path* path;
visualization_msgs::MarkerArray marker_array;
visualization_msgs::Marker path_marker;
visualization_msgs::Marker costmap_marker;
visualization_msgs::Marker delete_marker;

PathPlanner* path_planner;
Vec3 goal_vector, temp_vector;

std::string frame_id;
double resolution;
int costmap_radius;

int id;
int i, j, k;

void addPathMarker()
{
    path_marker.id = id++;
    path_marker.pose = marker_pose;
    marker_array.markers.push_back(path_marker);
}

void addCostmapMarker()
{
    costmap_marker.id = id++;
    costmap_marker.pose = marker_pose;
    marker_array.markers.push_back(costmap_marker);
}

void markerFeedbackCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{

    switch ( feedback->event_type )
    {
        case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
            ROS_INFO_STREAM( "menu item " << feedback->menu_entry_id << " clicked." );
            break;

        case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
            //ROS_INFO_STREAM( "pose update." );
            goal_pose = feedback->pose;
            break;

        case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
            ROS_INFO_STREAM( "mouse down." );
            break;

        case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
            ROS_INFO_STREAM( "mouse up." );
            //ROS_INFO("Goal Pose: %lf %lf %lf", goal_pose.position.x, goal_pose.position.y, goal_pose.position.z);

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

                    addPathMarker();
                }
            }
            else
            {
                ROS_INFO("The path couldn't find.");
            }

            goal_vector = Vec3(
                        path_planner->costmap->ToIndex(goal_pose.position.x),
                        path_planner->costmap->ToIndex(goal_pose.position.y),
                        path_planner->costmap->ToIndex(goal_pose.position.z)
                        );

            for (i = -costmap_radius; i < costmap_radius; i++)
            {
                for (j = -costmap_radius; j < costmap_radius; j++)
                {
                    for (k = -costmap_radius; k < costmap_radius; k++)
                    {
                        temp_vector = goal_vector + Vec3(i, j, k);
                        if (path_planner->costmap->Get(temp_vector))
                        {
                            marker_pose.position.x = path_planner->costmap->ToPosition(temp_vector.x);
                            marker_pose.position.y = path_planner->costmap->ToPosition(temp_vector.y);
                            marker_pose.position.z = path_planner->costmap->ToPosition(temp_vector.z);
                            //ROS_INFO("Marker Pose: %lf %lf %lf", marker_pose.position.x, marker_pose.position.y, marker_pose.position.z);
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
    ros::init(argc, argv, "nav_visualization");
    ros::NodeHandle nh("~");

    nh.param<std::string>("/frame_id", frame_id, FRAME_ID);
    nh.param<double>("/resolution", resolution, RESOLUTION);
    nh.param<int>("/costmap_radius", costmap_radius, COSTMAP_RADIUS);

    drone_marker_sub = nh.subscribe<visualization_msgs::InteractiveMarkerFeedback>( FEEDBACK_TOPIC, 10, markerFeedbackCallback );
    marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>( MARKER_ARRAY_TOPIC, 1 );

    id = 0;

    path_marker.header.frame_id = frame_id;
    path_marker.header.stamp = ros::Time();
    path_marker.ns = "path";
    path_marker.type = visualization_msgs::Marker::SPHERE;
    path_marker.action = visualization_msgs::Marker::ADD;

    path_marker.scale.x = resolution;
    path_marker.scale.y = resolution;
    path_marker.scale.z = resolution;
    path_marker.color.a = 1.0;
    path_marker.color.r = 1;
    path_marker.color.g = 0;
    path_marker.color.b = 0;

    costmap_marker.header.frame_id = frame_id;
    costmap_marker.header.stamp = ros::Time();
    costmap_marker.ns = "costmap";
    costmap_marker.type = visualization_msgs::Marker::CUBE;
    costmap_marker.action = visualization_msgs::Marker::ADD;

    costmap_marker.scale.x = resolution;
    costmap_marker.scale.y = resolution;
    costmap_marker.scale.z = resolution;
    costmap_marker.color.a = 0.5;
    costmap_marker.color.r = 1;
    costmap_marker.color.g = 1;
    costmap_marker.color.b = 0;

    delete_marker.header.frame_id = frame_id;
    delete_marker.header.stamp = ros::Time();
    delete_marker.type = visualization_msgs::Marker::SPHERE;
    delete_marker.action = visualization_msgs::Marker::DELETEALL;

    marker_pose.orientation.w = 1.0;

    path_planner = new PathPlanner(nh, PathPlanner::GLOBAL_MODE);

    ros::spin();

    return 0;
}
